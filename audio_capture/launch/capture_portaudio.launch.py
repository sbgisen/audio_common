from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    _bitrate = LaunchConfiguration('bitrate')
    _channels = LaunchConfiguration('channels')
    _sample_rate = LaunchConfiguration('sample_rate')
    _desired_rate = LaunchConfiguration('desired_rate')
    _sample_format = LaunchConfiguration('sample_format')
    _ns = LaunchConfiguration('ns')
    _audio_topic = LaunchConfiguration('audio_topic')

    _format_launch_arg = DeclareLaunchArgument('format', default_value='mp3')
    _bitrate_launch_arg = DeclareLaunchArgument('bitrate', default_value='128')
    _channels_launch_arg = DeclareLaunchArgument('channels', default_value='1')
    _sample_rate_launch_arg = DeclareLaunchArgument('sample_rate', default_value='16000')
    _desired_rate_launch_arg = DeclareLaunchArgument('desired_rate', default_value='100.0')
    _sample_format_launch_arg = DeclareLaunchArgument('sample_format', default_value='S16LE')
    _ns_launch_arg = DeclareLaunchArgument('ns', default_value='audio')
    _audio_topic_launch_arg = DeclareLaunchArgument('audio_topic', default_value='audio')

    _audio_capture_node = LifecycleNode(
        package='audio_capture',
        name='audio_capture',
        executable='audio_capture_portaudio_node',
        namespace=_ns,
        autostart=True,
        remappings=[
            ('audio', _audio_topic),
        ],
        parameters=[{
            'bitrate': _bitrate,
            'channels': _channels,
            'sample_rate': _sample_rate,
            'sample_format': _sample_format,
            'desired_rate': _desired_rate,
        }],
    )

    return LaunchDescription([
        _format_launch_arg,
        _bitrate_launch_arg,
        _channels_launch_arg,
        _sample_rate_launch_arg,
        _desired_rate_launch_arg,
        _sample_format_launch_arg,
        _ns_launch_arg,
        _audio_topic_launch_arg,
        _audio_capture_node,
    ])
