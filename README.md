# ROS audio\_common Package

Packages in this stack:

audio_common_msgs: Message definitions for audio transport.

sound_play: A package to play sound files and synthesize speech.

audio_capture: Provides code to capture audio from a microphone and transport it to a destination for playback.

audio_play: Receives audio messages from an audio_capture node. Outputs the messages to the local speakers.

audio_captureとaudio_play、2つは、`libgstreamer-plugins-base1.0-dev` と ` gstreamer1.0`　をapt でinstallすること
とエラーが出る。

```
Errors     << audio_capture:cmake /home/gisen/ros/logs/audio_capture/build.cmake.001.log
CMake Error at /usr/share/cmake-3.5/Modules/FindPkgConfig.cmake:367 (message):
  A required package was not found
```


# Support

Please ask support questions on [ROS Answers](http://answers.ros.org/questions/).

# Building from source

On ROS Indigo or Jade, the `indigo-devel` branch is recommended.

On ROS Kinetic, the `master` branch is recommended.

# Development, Branch and Release Policy

This package is not under active development, but is accepting pull requests for bug fixes and new features. (Development may be done for serious bug fixes; pending maintainer time).

The `sound_play`, `groovy-devel` and `hydro-devel` branches are from previous ROS releases and are frozen; no new pull requests will be accepted on these branches.

The `indigo-devel` branch is the stable branch; only bug fixes are accepted on this branch. Periodic releases are done from `indigo-devel` into ROS Indigo and ROS Jade, with version numbers in the 0.2.x range.

The `master` branch is currently considered the development branch, and is released into ROS Kinetic with version numbers in the 0.3.x range. `master` is accepting new, non-breaking features and bug fixes.

Large, breaking changes such as changes to dependencies or the package API will be considered, but they will probably be staged into a development branch for release into the next major release of ROS (ROS L)
