# ROS audio\_common Package

Packages in this stack:

audio_capture: Provides code to capture audio from a microphone and transport it to a destination for playback.

audio_play: Receives audio messages from an audio_capture node. Outputs the messages to the local speakers.

現状上記、2つは、コンパイル時にエラーが出る。

```
Errors     << audio_capture:cmake /home/gisen/ros/logs/audio_capture/build.cmake.001.log
CMake Error at /usr/share/cmake-3.5/Modules/FindPkgConfig.cmake:367 (message):
  A required package was not found
Call Stack (most recent call first):
  /usr/share/cmake-3.5/Modules/FindPkgConfig.cmake:532 (_pkg_check_modules_internal)
  CMakeLists.txt:8 (pkg_check_modules)


cd /home/gisen/ros/build/audio_capture; catkin build --get-env audio_capture | catkin env -si  /usr/bin/cmake /home/gisen/ros/src/audio_common/audio_capture --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/gisen/ros/devel/.private/audio_capture -DCMAKE_INSTALL_PREFIX=/home/gisen/ros/install; cd -
...............................................................................
Failed     << audio_capture:cmake              [ Exited with code 1 ]          
Failed    <<< audio_capture                    [ 0.8 seconds ]                 
_______________________________________________________________________________
Errors     << audio_play:cmake /home/gisen/ros/logs/audio_play/build.cmake.001.log
CMake Error at /usr/share/cmake-3.5/Modules/FindPkgConfig.cmake:367 (message):
  A required package was not found
Call Stack (most recent call first):
  /usr/share/cmake-3.5/Modules/FindPkgConfig.cmake:532 (_pkg_check_modules_internal)
  CMakeLists.txt:8 (pkg_check_modules)


cd /home/gisen/ros/build/audio_play; catkin build --get-env audio_play | catkin env -si  /usr/bin/cmake /home/gisen/ros/src/audio_common/audio_play --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/home/gisen/ros/devel/.private/audio_play -DCMAKE_INSTALL_PREFIX=/home/gisen/ros/install; cd -
...............................................................................
Failed     << audio_play:cmake                 [ Exited with code 1 ]          
Failed    <<< audio_play                       [ 0.8 seconds ]       

```

audio_common_msgs: Message definitions for audio transport.

sound_play: A package to play sound files and synthesize speech.


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
