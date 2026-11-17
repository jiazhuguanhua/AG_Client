#include <ag_client/client.hpp>
#include <slicer_air_ground_msgs/Command.h>
#include <std_msgs/String.h>
#include <ros/callback_queue.h>
#include <json/json.h>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <openssl/sha.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ag_client {
namespace {
using Command = slicer_air_ground_msgs::Command;

bool finiteNumber(const Json::Value& value) {
    return value.isNumeric() && std::isfinite(value.asDouble());
}

bool unsignedInteger(const Json::Value& value) {
    return (value.type() == Json::intValue || value.type() == Json::uintValue) && value.isUInt64();
}

bool text(const std::string& value) {
    if (value.empty() || value.size() > 256) return false;
    bool nonspace = false;
    for (unsigned char c : value) {
        if (c < 32 || c == 127) return false;
        nonspace = nonspace || c != ' ';
    }
    return nonspace;
}

std::string uuid() { return boost::uuids::to_string(boost::uuids::random_generator()()); }

std::string coordinates(const Target& target) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(9) << target.latitude << ", " << target.longitude;
    return stream.str();
}

bool freshStamp(const Json::Value& value, double now, double age) {
    return finiteNumber(value) && value.asDouble() > 0 &&
           now >= value.asDouble() && now-value.asDouble() <= age;
}

bool parseState(const std::string& data, Json::Value& state) {
    if (data.size() > 1024*1024) return false;
    Json::CharReaderBuilder builder;
    builder["rejectDupKeys"] = true;
    builder["failIfExtra"] = true;
    builder["allowSpecialFloats"] = false;
    builder["allowComments"] = false;
    builder["collectComments"] = false;
    std::string error;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(data.data(), data.data()+data.size(), &state, &error) || !state.isObject()) return false;
    return unsignedInteger(state["protocol_version"]) && state["protocol_version"].asUInt64() == 1 &&
           state["server_id"].isString() && text(state["server_id"].asString()) &&
           (state["last_sequence"].isNull() || unsignedInteger(state["last_sequence"])) &&
           state["phase"].isString() &&
           (state["task_id"].isNull() || state["task_id"].isString()) &&
           finiteNumber(state["updated_stamp"]) &&
           (state["last_command"].isNull() || state["last_command"].isObject());
}

bool executionReady(const Json::Value& state, double now, double age) {
    return freshStamp(state["execution_updated_stamp"], now, age) &&
           state["datum"] == "WGS84" && state["physical_reference"] == "base_link_horizontal" &&
           state["startup_checked"].isBool() && state["startup_checked"].asBool() &&
           state["runtime_phase"] != "WAITING_NAVIGATION" && state["phase"] != "CANCELING" &&
           state["cancellation_pending"].isBool() && !state["cancellation_pending"].asBool();
}

bool confirmedCompletion(const Json::Value& result, const std::string& task) {
    if (!result.isObject() || result["target"]["task_id"] != task) return false;
    const auto& action = result["move_base"]["state"];
    const auto& stop = result["stop"];
    const auto& geo = result["geographic"];
    if (!unsignedInteger(action) || action.asUInt64() != 3 || stop["confirmed"] != true ||
        geo["confirmed"] != true || geo["qualified"] != true) return false;
    const auto& residual = geo["horizontal_residual_m"];
    const auto& tolerance = geo["tolerance_m"];
    const auto& start = geo["stopped_window"]["start_stamp"];
    const auto& end = geo["stopped_window"]["end_stamp"];
    const auto& stamp = geo["stamp"];
    return finiteNumber(residual) && finiteNumber(tolerance) && residual.asDouble() >= 0 &&
           residual.asDouble() <= tolerance.asDouble() && finiteNumber(start) && finiteNumber(end) &&
           finiteNumber(stamp) && start.asDouble() <= stamp.asDouble() && stamp.asDouble() <= end.asDouble() &&
           stop["window_start_stamp"] == start && stop["window_end_stamp"] == end;
}

std::string lockPath(const std::string& topic) {
    const char* master = std::getenv("ROS_MASTER_URI");
    const std::string identity = std::string(master ? master : "") + "\n" + topic;
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(identity.data()), identity.size(), digest);
    std::ostringstream stream;
    stream << "/tmp/ag_client-" << getuid() << "-" << std::hex << std::setfill('0');
    for (unsigned char byte : digest) stream << std::setw(2) << static_cast<unsigned>(byte);
    return stream.str()+".lock";
}
}  // namespace

class Client::Impl {
    struct Request {
        Target target;
        Command command;
        ros::SteadyTime queued, sent;
        bool published{false}, accepted{false}, executing{false}, unknown_logged{false};
        std::string last_phase;
        Json::Value old_error;
    };
    Options options_;
    ros::CallbackQueue queue_;
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    ros::WallTimer timer_;
    std::mutex mutex_;
    Json::Value state_;
    ros::SteadyTime received_;
    bool state_valid_{false};
    std::unique_ptr<Request> request_;
    std::set<std::pair<std::string, std::string>> submitted_;
    std::string lock_path_;
    int lock_fd_{-1};

    void unlock() {
        if (lock_fd_ >= 0) { close(lock_fd_); lock_fd_ = -1; }
    }

    bool exclusive() {
        lock_fd_ = open(lock_path_.c_str(), O_CREAT | O_RDWR | O_NOFOLLOW, 0600);
        if (lock_fd_ < 0 || flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) { unlock(); return false; }
        return true;
    }

    void finish() { unlock(); request_.reset(); }

    void unknown(const std::string& reason) {
        if (request_->unknown_logged) return;
        request_->unknown_logged = true;
        unlock();
        ROS_ERROR_STREAM_NAMED("ag_client", "[AG_Client] UGV 请求结果未知【" << request_->target.label
            << "】：" << reason << "；不自动重发（request_id=" << request_->command.request_id << "）");
    }

    void onState(const std_msgs::String::ConstPtr& message) {
        std::lock_guard<std::mutex> guard(mutex_);
        try {
            state_valid_ = parseState(message->data, state_);
        } catch (const std::exception&) { state_valid_ = false; }
        received_ = ros::SteadyTime::now();
    }

    void observe() {
        auto& r = *request_;
        if (state_["server_id"] != r.command.server_id) {
            unknown("车端已重启，旧任务结果不可确认");
            finish();
            return;
        }
        const auto& receipt = state_["last_command"];
        const bool matches = receipt.isObject() && receipt["request_id"] == r.command.request_id &&
            receipt["task_id"] == r.command.task_id && receipt["operation"] == r.command.operation &&
            unsignedInteger(receipt["sequence"]) && receipt["sequence"].asUInt64() == r.command.sequence &&
            receipt["accepted"].isBool();
        if (matches && !r.accepted) {
            if (!receipt["accepted"].asBool()) {
                ROS_ERROR_STREAM_NAMED("ag_client", "[AG_Client] UGV 拒绝任务【" << r.target.label
                    << "】：" << receipt.get("reason", "").asString());
                finish();
                return;
            }
            r.accepted = true;
            unlock();
            ROS_INFO_STREAM_NAMED("ag_client", "[AG_Client] UGV 已收到【" << r.target.label
                << "】，任务已受理：经纬度 " << coordinates(r.target) << "（task_id=" << r.command.task_id << "）");
        }
        const auto& error = state_["transport_error"];
        if (!r.accepted && error.isObject() && error != r.old_error && error["request_id"] == r.command.request_id)
            unknown(error.get("reason", "transport error").asString());

        const auto& result = state_["last_task_result"];
        if (result.isObject() && result["task_id"] == r.command.task_id) {
            const std::string status = result.get("status", "UNKNOWN").asString();
            if (status == "COMPLETED") {
                if (confirmedCompletion(result["result"], r.command.task_id)) {
                    ROS_INFO_STREAM_NAMED("ag_client", "[AG_Client] UGV 任务完成！【" << r.target.label
                        << "】：经纬度 " << coordinates(r.target) << "（task_id=" << r.command.task_id << "）");
                } else {
                    ROS_ERROR_STREAM_NAMED("ag_client", "[AG_Client] UGV 报告完成，但到达证据不完整【"
                        << r.target.label << "】；未确认任务完成");
                }
                finish();
                return;
            }
            if (status == "REPLACED" || status == "CANCELED") {
                ROS_WARN_STREAM_NAMED("ag_client", "[AG_Client] UGV 任务结束【" << r.target.label << "】：" << status);
                finish();
                return;
            }
        }
        if (state_["task_id"] != r.command.task_id) {
            if (r.accepted) { unknown("当前任务已变化，快照没有该任务的结果"); finish(); }
            return;
        }
        const std::string phase = state_["phase"].asString();
        if (phase == "EXECUTING" && !r.executing) {
            r.executing = true;
            ROS_INFO_STREAM_NAMED("ag_client", "[AG_Client] UGV 开始执行任务【" << r.target.label
                << "】：经纬度 " << coordinates(r.target) << "（task_id=" << r.command.task_id << "）");
        }
        const std::string phase_reason = phase+":"+state_.get("reason", "").asString();
        if (phase_reason != r.last_phase && (phase == "BLOCKED" || phase == "PAUSED" || phase == "PAUSING")) {
            ROS_WARN_STREAM_NAMED("ag_client", "[AG_Client] UGV 任务暂未执行/已暂停【" << r.target.label
                << "】：" << phase_reason << "；不会自动解除 hold");
        }
        r.last_phase = phase_reason;
    }

    void tickUnsafe() {
        if (!request_) return;
        const auto now = ros::SteadyTime::now();
        const double ros_now = ros::Time::now().toSec();
        const bool fresh = state_valid_ && !received_.isZero() &&
            (now-received_).toSec() <= options_.state_max_age &&
            freshStamp(state_["updated_stamp"], ros_now, options_.state_max_age);
        if (request_->published) {
            if (fresh) observe();
            if (request_ && !request_->accepted && (now-request_->sent).toSec() > options_.receipt_timeout)
                unknown("等待受理回执超时，命令可能已执行");
            return;
        }
        if ((now-request_->queued).toSec() > options_.state_timeout+options_.connect_timeout) {
            ROS_ERROR_STREAM_NAMED("ag_client", "[AG_Client] 未发送【" << request_->target.label
                << "】：UGV 状态/执行快照/订阅连接未就绪；不自动重试");
            finish();
            return;
        }
        if (!fresh || !executionReady(state_, ros_now, options_.execution_max_age) || publisher_.getNumSubscribers() == 0)
            return;
        if (!exclusive()) {
            ROS_ERROR_NAMED("ag_client", "[AG_Client] 未发送：同一 master/topic 有其他命令出口或本地锁不可用");
            finish();
            return;
        }
        const auto sequence = state_["last_sequence"].isNull() ? 0 : state_["last_sequence"].asUInt64();
        if (sequence == std::numeric_limits<uint64_t>::max()) {
            ROS_ERROR_NAMED("ag_client", "[AG_Client] 未发送：UGV 序号已耗尽");
            finish();
            return;
        }
        auto& r = *request_;
        r.command.protocol_version = 1;
        r.command.server_id = state_["server_id"].asString();
        r.command.sequence = sequence+1;
        r.command.request_id = uuid();
        r.command.task_id = uuid();
        r.command.operation = Command::SET_TASK;
        r.command.final_goal.latitude = r.target.latitude;
        r.command.final_goal.longitude = r.target.longitude;
        // Only the measured ground target XY is sent. No aircraft height/yaw.
        r.command.final_goal.has_altitude = false;
        r.command.final_goal.has_yaw = false;
        r.old_error = state_["transport_error"];
        r.sent = now;
        r.published = true;
        try {
            publisher_.publish(r.command);
            ROS_INFO_STREAM_NAMED("ag_client", "[AG_Client] 已发送【" << r.target.label << "】：经纬度 "
                << coordinates(r.target) << " 到 UGV！（request_id=" << r.command.request_id << "）");
        } catch (const std::exception& error) { unknown(error.what()); }
    }

    void tick(const ros::WallTimerEvent&) {
        std::lock_guard<std::mutex> guard(mutex_);
        try { tickUnsafe(); }
        catch (const std::exception& error) {
            state_valid_ = false;
            ROS_ERROR_STREAM_THROTTLE_NAMED(5.0, "ag_client", "[AG_Client] 无效反馈/本地异常：" << error.what());
            if (request_ && request_->published) unknown("反馈字段或发送过程异常，未确认结果");
            else if (request_) finish();
        }
    }

public:
    explicit Impl(const Options& options) : options_(options), spinner_(1, &queue_) {
        for (double value : {options.state_timeout, options.state_max_age, options.execution_max_age,
                             options.connect_timeout, options.receipt_timeout})
            if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("AG_Client timeouts must be finite and positive");
        nh_.setCallbackQueue(&queue_);
        publisher_ = nh_.advertise<Command>(options.command_topic, 1, false);
        subscriber_ = nh_.subscribe(options.state_topic, 1, &Impl::onState, this);
        lock_path_ = lockPath(nh_.resolveName(options.command_topic));
        timer_ = nh_.createWallTimer(ros::WallDuration(.05), &Impl::tick, this);
        spinner_.start();
    }

    ~Impl() {
        timer_.stop();
        subscriber_.shutdown();
        spinner_.stop();
        publisher_.shutdown();
        unlock();
    }

    bool submit(const Target& target, std::string* reason) {
        auto reject = [reason](const std::string& why) { if (reason) *reason = why; return false; };
        if (!text(target.mission_id) || !text(target.label) || !std::isfinite(target.latitude) ||
            !std::isfinite(target.longitude) || std::abs(target.latitude) > 90 || std::abs(target.longitude) > 180)
            return reject("invalid mission/QR label or ground WGS84 coordinate");
        std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
        if (!guard.owns_lock()) return reject("client callback is busy");
        const auto key = std::make_pair(target.mission_id, target.label);
        if (submitted_.count(key)) return true;
        if (request_) return reject("another UGV task is still being observed");
        request_.reset(new Request());
        request_->target = target;
        request_->queued = ros::SteadyTime::now();
        submitted_.insert(key);
        if (reason) reason->clear();
        return true;
    }
};

Client::Client(const Options& options) : impl_(new Impl(options)) {}
Client::~Client() = default;
bool Client::submitTarget(const Target& target, std::string* reason) { return impl_->submit(target, reason); }

}  // namespace ag_client
