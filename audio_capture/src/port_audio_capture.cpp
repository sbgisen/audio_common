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

#include <portaudio.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "audio_common_msgs/msg/audio_data.hpp"
#include "audio_common_msgs/msg/audio_data_stamped.hpp"
#include "audio_common_msgs/msg/audio_info.hpp"

namespace audio_capture
{
class PortAudioCaptureNode : public rclcpp::Node
{
public:
  PortAudioCaptureNode(const rclcpp::NodeOptions & options)
  : Node("audio_capture_node", options), updater_(this), _desired_rate(-1.0)
  {
    PaError err = Pa_Initialize();
    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "PortAudio error: %s", Pa_GetErrorText(err));
      exitOnMainThread(1);
    }

    this->declare_parameter<std::string>("sample_format", "S16LE");
    this->get_parameter("sample_format", _sample_format);

    this->declare_parameter<int>("channels", 1);
    this->declare_parameter<int>("sample_rate", 16000);
    this->declare_parameter<int>("bitrate", 192);
    this->get_parameter("channels", _channels);
    this->get_parameter("sample_rate", _sample_rate);
    this->get_parameter("bitrate", _bitrate);

    _pub = this->create_publisher<audio_common_msgs::msg::AudioData>("audio", 10);
    auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    _pub_info = this->create_publisher<audio_common_msgs::msg::AudioInfo>("audio_info", info_qos);

    rclcpp::Publisher<audio_common_msgs::msg::AudioDataStamped>::SharedPtr pub_stamped =
      this->create_publisher<audio_common_msgs::msg::AudioDataStamped>("audio_stamped", 10);

    this->declare_parameter<double>("diagnostic_tolerance", 0.1);
    auto tolerance = this->get_parameter("diagnostic_tolerance").as_double();

    updater_.setHardwareID("microphone");
    _diagnosed_pub_stamped =
      std::make_shared<diagnostic_updater::DiagnosedPublisher<audio_common_msgs::msg::AudioDataStamped>>(
        pub_stamped, updater_, diagnostic_updater::FrequencyStatusParam(&_desired_rate, &_desired_rate, tolerance, 10),
        diagnostic_updater::TimeStampStatusParam());

    _stream = nullptr;
    openStream();

    _gst_thread = boost::thread(boost::bind(&PortAudioCaptureNode::captureLoop, this));

    _timer_info = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(5), [this] { publishInfo(); });
    publishInfo();
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

  ~PortAudioCaptureNode()
  {
    if (_stream) {
      Pa_StopStream(_stream);
      Pa_CloseStream(_stream);
    }
    Pa_Terminate();
  }

  void exitOnMainThread(int code) { exit(code); }

  void publish(const audio_common_msgs::msg::AudioData & msg) { _pub->publish(msg); }

  void publishStamped(const audio_common_msgs::msg::AudioDataStamped & msg) { _diagnosed_pub_stamped->publish(msg); }

private:
  void openStream()
  {
    PaStreamParameters inputParameters;
    inputParameters.device = Pa_GetDefaultInputDevice();
    inputParameters.channelCount = _channels;
    if (_sample_format == "S16LE") {
      inputParameters.sampleFormat = paInt16;
    } else if (_sample_format == "S32LE") {
      inputParameters.sampleFormat = paInt32;
    } else if (_sample_format == "U8") {
      inputParameters.sampleFormat = paUInt8;
    } else if (_sample_format == "S8") {
      inputParameters.sampleFormat = paInt8;
    } else if (_sample_format == "F32LE") {
      inputParameters.sampleFormat = paFloat32;
    } else if (_sample_format == "S24LE") {
      inputParameters.sampleFormat = paInt24;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported sample format: %s", _sample_format.c_str());
      exitOnMainThread(1);
    }
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = nullptr;

    unsigned long framesPerBuffer = _sample_rate / 100.0 * _channels;

    PaError err = Pa_OpenStream(
      &_stream, &inputParameters,
      nullptr,
      _sample_rate, framesPerBuffer, paClipOff, &PortAudioCaptureNode::paCallback, this);

    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "PortAudio error: %s", Pa_GetErrorText(err));
      exitOnMainThread(1);
    }

    err = Pa_StartStream(_stream);
    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "PortAudio error: %s", Pa_GetErrorText(err));
      exitOnMainThread(1);
    }
  }

  static int paCallback(
    const void * inputBuffer, void * outputBuffer, unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo * timeInfo, PaStreamCallbackFlags statusFlags, void * userData)
  {
    PortAudioCaptureNode * server = reinterpret_cast<PortAudioCaptureNode *>(userData);
    const int16_t * in = reinterpret_cast<const int16_t *>(inputBuffer);

    audio_common_msgs::msg::AudioData msg;
    audio_common_msgs::msg::AudioDataStamped stamped_msg;

    msg.data.resize(framesPerBuffer * sizeof(int16_t));
    memcpy(&msg.data[0], in, msg.data.size());

    stamped_msg.header.stamp = server->now();
    stamped_msg.audio = msg;

    server->publish(msg);
    server->publishStamped(stamped_msg);

    return paContinue;
  }

  void captureLoop()
  {
    while (rclcpp::ok()) {
      Pa_Sleep(100);
    }
  }

  rclcpp::Publisher<audio_common_msgs::msg::AudioData>::SharedPtr _pub;
  rclcpp::Publisher<audio_common_msgs::msg::AudioInfo>::SharedPtr _pub_info;

  rclcpp::TimerBase::SharedPtr _timer_info;

  boost::thread _gst_thread;

  PaStream * _stream;
  int _bitrate, _channels, _sample_rate;
  std::string _sample_format;

  diagnostic_updater::Updater updater_;
  double _desired_rate;
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<audio_common_msgs::msg::AudioDataStamped>>
    _diagnosed_pub_stamped;
};
}  // namespace audio_capture

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::PortAudioCaptureNode)