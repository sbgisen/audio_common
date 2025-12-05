/*********************************************************************
 * Copyright (c) 2025 SoftBank Corp.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 ********************************************************************/

#include <pulse/error.h>
#include <pulse/simple.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "audio_common_msgs/msg/audio_data.hpp"
#include "audio_common_msgs/msg/audio_data_stamped.hpp"
#include "audio_common_msgs/msg/audio_info.hpp"

namespace audio_capture
{
class PulseAudioCaptureNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  PulseAudioCaptureNode(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("audio_capture_node", options), updater_(this), _desired_rate(-1.0)
  {
    this->declare_parameter<std::string>("sample_format", "S16LE");

    this->declare_parameter<int>("channels", 1);
    this->declare_parameter<int>("sample_rate", 16000);
    this->declare_parameter<int>("bitrate", 192);
    this->declare_parameter<double>("desired_rate", 100.0);
    this->declare_parameter<std::string>("source", "");

    this->declare_parameter<double>("diagnostic_tolerance", 0.1);

    this->get_parameter("desired_rate", _desired_rate);

    _pub = this->create_publisher<audio_common_msgs::msg::AudioData>("audio", 10);
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

    _auto_recovery_timer =
      create_wall_timer(std::chrono::duration<double>(1.0), [this] { autoRecoveryTrigger(); });
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

  ~PulseAudioCaptureNode()
  {
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
  auto on_activate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn override {
    int error;
    std::string source;
    this->get_parameter("source", source);
    _stream = pa_simple_new(
      nullptr,
      "pulse_capture_libpulse_node",
      PA_STREAM_RECORD,
      source.empty() ? nullptr : source.c_str(),
      "record",
      &_sample_spec,
      nullptr,
      nullptr,
      &error);
    if (!_stream) {
      RCLCPP_ERROR(this->get_logger(), "pa_simple_new() failed: %s", pa_strerror(error));
      return CallbackReturn::FAILURE;
    }

    _timer_info = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(5), [this] { publishInfo(); });
    publishInfo();
    _record_timer = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / _desired_rate), [this]() {
        const size_t buffer_size = pa_frame_size(&_sample_spec) * _sample_spec.rate / _desired_rate;
        std::vector<uint8_t> buffer(buffer_size);
        int error;
        if (pa_simple_read(_stream, buffer.data(), buffer.size(), &error) < 0) {
          RCLCPP_ERROR(this->get_logger(), "pa_simple_read() failed: %s", pa_strerror(error));
          trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
          return;
        }
        audio_common_msgs::msg::AudioData msg;
        msg.data = buffer;
        publish(msg);

        audio_common_msgs::msg::AudioDataStamped stamped_msg;
        stamped_msg.audio.data = msg.data;
        stamped_msg.header.stamp = this->now();
        _diagnosed_pub_stamped->publish(stamped_msg);

        _last_publish_time_ = this->now();
      });
    return CallbackReturn::SUCCESS;
  }
  auto on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn override
  {
    if (_stream) {
      pa_simple_free(_stream);
      _stream = nullptr;
    }
    RCLCPP_INFO(this->get_logger(), "PortAudioCaptureNode deactivated, stream closed.");
    return CallbackReturn::SUCCESS;
  }
  auto on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn override {
    _last_publish_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    return CallbackReturn::SUCCESS;
  }
  auto on_error(const rclcpp_lifecycle::State & /*previous_state*/) -> CallbackReturn override {
    if (_stream) {
      pa_simple_free(_stream);
      _stream = nullptr;
    }
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

  void publish(const audio_common_msgs::msg::AudioData & msg) { _pub->publish(msg); }

  void publishStamped(const audio_common_msgs::msg::AudioDataStamped & msg) { _diagnosed_pub_stamped->publish(msg); }

private:

  rclcpp::Publisher<audio_common_msgs::msg::AudioData>::SharedPtr _pub;
  rclcpp::Publisher<audio_common_msgs::msg::AudioInfo>::SharedPtr _pub_info;

  rclcpp::TimerBase::SharedPtr _timer_info;
  rclcpp::TimerBase::SharedPtr _record_timer;
  rclcpp::TimerBase::SharedPtr _auto_recovery_timer;

  pa_sample_spec _sample_spec;
  pa_simple * _stream;
  int _bitrate, _channels, _sample_rate;
  std::string _sample_format;

  diagnostic_updater::Updater updater_;
  double _desired_rate;
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<audio_common_msgs::msg::AudioDataStamped>>
    _diagnosed_pub_stamped;
  rclcpp::Time _last_publish_time_;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::PulseAudioCaptureNode)