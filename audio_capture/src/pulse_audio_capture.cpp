#include <pulse/error.h>
#include <pulse/pulseaudio.h>

#include <boost/thread.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <vector>

#include "audio_common_msgs/msg/audio_data.hpp"
#include "audio_common_msgs/msg/audio_data_stamped.hpp"
#include "audio_common_msgs/msg/audio_info.hpp"

namespace audio_capture
{

class PulseAudioCaptureNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  PulseAudioCaptureNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("audio_capture_node", options),
    _mainloop(nullptr),
    _context(nullptr),
    _stream(nullptr),
    _chunk_size_bytes(0),
    updater_(this),
    _desired_rate(-1.0)
  {
    this->declare_parameter<std::string>("sample_format", "S16LE");

    this->declare_parameter<int>("channels", 1);
    this->declare_parameter<int>("sample_rate", 16000);
    this->declare_parameter<int>("bitrate", 192);
    this->declare_parameter<double>("desired_rate", 100.0);
    this->declare_parameter<std::string>("source", "");

    this->declare_parameter<double>("diagnostic_tolerance", 0.1);

    this->get_parameter("desired_rate", _desired_rate);

    auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    _pub_info = this->create_publisher<audio_common_msgs::msg::AudioInfo>("audio_info", info_qos);

    rclcpp::Publisher<audio_common_msgs::msg::AudioDataStamped>::SharedPtr pub_stamped =
      this->create_publisher<audio_common_msgs::msg::AudioDataStamped>("audio_stamped", 10);

    auto tolerance = this->get_parameter("diagnostic_tolerance").as_double();

    _last_publish_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    updater_.setHardwareID("microphone");
    _diagnosed_pub_stamped =
      std::make_shared<diagnostic_updater::DiagnosedPublisher<audio_common_msgs::msg::AudioDataStamped>>(
        pub_stamped, updater_, diagnostic_updater::FrequencyStatusParam(&_desired_rate, &_desired_rate, tolerance, 10),
        diagnostic_updater::TimeStampStatusParam());

    _auto_recovery_timer = create_wall_timer(std::chrono::duration<double>(1.0), [this] { autoRecoveryTrigger(); });
  }

  ~PulseAudioCaptureNode()
  {
    cleanupPulse();
  }

  void publishInfo()
  {
    audio_common_msgs::msg::AudioInfo info_msg;
    info_msg.channels = _channels;
    info_msg.sample_rate = _sample_rate;
    info_msg.sample_format = _sample_format;
    info_msg.bitrate = _bitrate;
    info_msg.coding_format = "raw";
    _pub_info->publish(info_msg);
  }

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  auto on_configure(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn override
  {
    this->get_parameter("sample_format", _sample_format);
    this->get_parameter("channels", _channels);
    this->get_parameter("sample_rate", _sample_rate);
    this->get_parameter("bitrate", _bitrate);

    if (_sample_format == "S16LE") {
      _sample_spec.format = PA_SAMPLE_S16LE;
    } else if (_sample_format == "S32LE") {
      _sample_spec.format = PA_SAMPLE_S32LE;
    } else if (_sample_format == "U8") {
      _sample_spec.format = PA_SAMPLE_U8;
    } else if (_sample_format == "F32LE") {
      _sample_spec.format = PA_SAMPLE_FLOAT32LE;
    } else if (_sample_format == "S24LE") {
      _sample_spec.format = PA_SAMPLE_S24LE;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported sample format: %s", _sample_format.c_str());
      return CallbackReturn::FAILURE;
    }

    _sample_spec.rate = static_cast<uint32_t>(_sample_rate);
    _sample_spec.channels = static_cast<uint8_t>(_channels);

    return CallbackReturn::SUCCESS;
  }

  auto on_activate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn override
  {
    std::string source;
    this->get_parameter("source", source);

    const auto frame_size = pa_frame_size(&_sample_spec);
    auto frames_per_chunk = static_cast<std::size_t>(std::round(static_cast<double>(_sample_spec.rate) / _desired_rate));
    if (frames_per_chunk == 0) {
      frames_per_chunk = 1;
    }
    _chunk_size_bytes = frames_per_chunk * frame_size;

    if (!initPulse(source)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize PulseAudio stream.");
      return CallbackReturn::FAILURE;
    }

    _timer_info = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(5), [this] { publishInfo(); });
    publishInfo();

    return CallbackReturn::SUCCESS;
  }

  auto on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn override
  {
    cleanupPulse();
    RCLCPP_INFO(this->get_logger(), "PulseAudioCaptureNode deactivated, stream closed.");
    return CallbackReturn::SUCCESS;
  }

  auto on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn override
  {
    _last_publish_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    return CallbackReturn::SUCCESS;
  }

  auto on_error(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn override
  {
    cleanupPulse();
    return CallbackReturn::SUCCESS;
  }

  void autoRecoveryTrigger()
  {
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    } else if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      if (get_clock()->now() - _last_publish_time_ > rclcpp::Duration::from_seconds(1.0)) {
        RCLCPP_WARN(this->get_logger(), "No audio data published for 1 second, deactivating.");
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }
    } else if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      if (_last_publish_time_ != rclcpp::Time(0, 0, this->get_clock()->get_clock_type())) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
      } else {
        RCLCPP_INFO(this->get_logger(), "Reactivating.");
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      }
    }
  }

  void publish(const audio_common_msgs::msg::AudioDataStamped & msg) { _diagnosed_pub_stamped->publish(msg); }

private:
  rclcpp::Publisher<audio_common_msgs::msg::AudioInfo>::SharedPtr _pub_info;

  rclcpp::TimerBase::SharedPtr _timer_info;
  rclcpp::TimerBase::SharedPtr _auto_recovery_timer;

  pa_sample_spec _sample_spec{};
  pa_threaded_mainloop * _mainloop;
  pa_context * _context;
  pa_stream * _stream;

  std::size_t _chunk_size_bytes;
  std::vector<uint8_t> _accum_buffer;
  std::mutex _buffer_mutex;

  int _bitrate{}, _channels{}, _sample_rate{};
  std::string _sample_format;

  diagnostic_updater::Updater updater_;
  double _desired_rate;
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<audio_common_msgs::msg::AudioDataStamped>>
    _diagnosed_pub_stamped;
  rclcpp::Time _last_publish_time_;

  static void contextStateCB(pa_context * c, void * userdata)
  {
    auto * node = static_cast<PulseAudioCaptureNode *>(userdata);
    node->onContextStateChanged(c);
  }

  static void streamStateCB(pa_stream * s, void * userdata)
  {
    auto * node = static_cast<PulseAudioCaptureNode *>(userdata);
    node->onStreamStateChanged(s);
  }

  static void streamReadCB(pa_stream * s, std::size_t length, void * userdata)
  {
    auto * node = static_cast<PulseAudioCaptureNode *>(userdata);
    node->onStreamRead(s, length);
  }

  void onContextStateChanged(pa_context * c)
  {
    pa_context_state_t state = pa_context_get_state(c);
    if (!PA_CONTEXT_IS_GOOD(state) && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "PulseAudio context state error: %d", state);
    }
    if (_mainloop != nullptr) {
      pa_threaded_mainloop_signal(_mainloop, 0);
    }
  }

  void onStreamStateChanged(pa_stream * s)
  {
    pa_stream_state_t state = pa_stream_get_state(s);
    if (!PA_STREAM_IS_GOOD(state) && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "PulseAudio stream state error: %d", state);
    }
    if (_mainloop != nullptr) {
      pa_threaded_mainloop_signal(_mainloop, 0);
    }
  }

  void onStreamRead(pa_stream * s, std::size_t /*length*/)
  {
    const void * data = nullptr;
    std::size_t bytes = 0;

    if (pa_stream_peek(s, &data, &bytes) < 0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1.0, "pa_stream_peek() failed");
      return;
    }

    if (data == nullptr || bytes == 0) {
      pa_stream_drop(s);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(_buffer_mutex);
      const uint8_t * src = static_cast<const uint8_t *>(data);
      _accum_buffer.insert(_accum_buffer.end(), src, src + bytes);

      while (_accum_buffer.size() >= _chunk_size_bytes && _chunk_size_bytes > 0 && rclcpp::ok()) {
        audio_common_msgs::msg::AudioData msg;
        msg.data.assign(_accum_buffer.begin(), _accum_buffer.begin() + static_cast<std::ptrdiff_t>(_chunk_size_bytes));

        _accum_buffer.erase(
          _accum_buffer.begin(), _accum_buffer.begin() + static_cast<std::ptrdiff_t>(_chunk_size_bytes));

        audio_common_msgs::msg::AudioDataStamped stamped_msg;
        stamped_msg.audio.data = msg.data;
        stamped_msg.header.stamp = this->now();
        publish(stamped_msg);

        _last_publish_time_ = this->now();
      }
    }

    pa_stream_drop(s);
  }

  bool initPulse(const std::string & source)
  {
    // Cleanup any existing PulseAudio resources
    cleanupPulse();

    _mainloop = pa_threaded_mainloop_new();
    if (_mainloop == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create PulseAudio mainloop");
      return false;
    }

    pa_mainloop_api * api = pa_threaded_mainloop_get_api(_mainloop);
    _context = pa_context_new(api, "PulseAudioCaptureNode");
    if (_context == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create PulseAudio context");
      cleanupPulse();
      return false;
    }

    pa_context_set_state_callback(_context, &PulseAudioCaptureNode::contextStateCB, this);

    pa_threaded_mainloop_lock(_mainloop);

    if (pa_context_connect(_context, nullptr, PA_CONTEXT_NOFLAGS, nullptr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "pa_context_connect() failed: %s", pa_strerror(pa_context_errno(_context)));
      pa_threaded_mainloop_unlock(_mainloop);
      cleanupPulse();
      return false;
    }

    // Start the mainloop
    if (pa_threaded_mainloop_start(_mainloop) < 0) {
      RCLCPP_ERROR(this->get_logger(), "pa_threaded_mainloop_start() failed");
      pa_threaded_mainloop_unlock(_mainloop);
      cleanupPulse();
      return false;
    }

    // Wait for the context to be ready
    while (rclcpp::ok()) {
      pa_context_state_t state = pa_context_get_state(_context);
      if (state == PA_CONTEXT_READY) {
        break;
      }
      if (!PA_CONTEXT_IS_GOOD(state)) {
        RCLCPP_ERROR(this->get_logger(), "PulseAudio context error state: %d", state);
        pa_threaded_mainloop_unlock(_mainloop);
        cleanupPulse();
        return false;
      }
      pa_threaded_mainloop_wait(_mainloop);
    }

    // Create a new recording stream
    _stream = pa_stream_new(_context, "record", &_sample_spec, nullptr);
    if (_stream == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "pa_stream_new() failed: %s", pa_strerror(pa_context_errno(_context)));
      pa_threaded_mainloop_unlock(_mainloop);
      cleanupPulse();
      return false;
    }

    pa_stream_set_state_callback(_stream, &PulseAudioCaptureNode::streamStateCB, this);
    pa_stream_set_read_callback(_stream, &PulseAudioCaptureNode::streamReadCB, this);

    pa_buffer_attr buffer_attr;
    buffer_attr.maxlength = static_cast<uint32_t>(-1);
    buffer_attr.tlength = static_cast<uint32_t>(-1);
    buffer_attr.prebuf = static_cast<uint32_t>(-1);
    buffer_attr.minreq = static_cast<uint32_t>(-1);
    buffer_attr.fragsize = static_cast<uint32_t>(_chunk_size_bytes);

    int flags = PA_STREAM_ADJUST_LATENCY;

    if (
      pa_stream_connect_record(
        _stream, source.empty() ? nullptr : source.c_str(), &buffer_attr, static_cast<pa_stream_flags_t>(flags)) < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "pa_stream_connect_record() failed: %s", pa_strerror(pa_context_errno(_context)));
      pa_threaded_mainloop_unlock(_mainloop);
      cleanupPulse();
      return false;
    }

    // Wait for the stream to be ready
    while (rclcpp::ok()) {
      pa_stream_state_t sstate = pa_stream_get_state(_stream);
      if (sstate == PA_STREAM_READY) {
        break;
      }
      if (!PA_STREAM_IS_GOOD(sstate)) {
        RCLCPP_ERROR(this->get_logger(), "PulseAudio stream error state: %d", sstate);
        pa_threaded_mainloop_unlock(_mainloop);
        cleanupPulse();
        return false;
      }
      pa_threaded_mainloop_wait(_mainloop);
    }

    pa_threaded_mainloop_unlock(_mainloop);

    return true;
  }

  void cleanupPulse()
  {
    if (_mainloop != nullptr) {
      pa_threaded_mainloop_lock(_mainloop);
    }

    if (_stream != nullptr) {
      pa_stream_disconnect(_stream);
      pa_stream_unref(_stream);
      _stream = nullptr;
    }

    if (_context != nullptr) {
      pa_context_disconnect(_context);
      pa_context_unref(_context);
      _context = nullptr;
    }

    if (_mainloop != nullptr) {
      pa_threaded_mainloop_unlock(_mainloop);
      pa_threaded_mainloop_stop(_mainloop);
      pa_threaded_mainloop_free(_mainloop);
      _mainloop = nullptr;
    }

    std::lock_guard<std::mutex> lock(_buffer_mutex);
    _accum_buffer.clear();
  }
};

}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::PulseAudioCaptureNode)
