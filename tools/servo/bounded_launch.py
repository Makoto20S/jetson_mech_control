"""ROS launch entry with screen-only logging, configured before logger creation."""
import sys
if __name__ == '__main__':
    sys.path.pop(0)


def main():
    import launch.logging
    from launch.logging.handlers import NullHandler
    # Humble LaunchConfig supports a factory; redirection alone leaves unbounded files.
    launch.logging.launch_config.log_handler_factory = lambda *args, **kwargs: NullHandler()
    from ros2launch.api import launch_a_launch_file
    return launch_a_launch_file(launch_file_path=sys.argv[1],
                               launch_file_arguments=sys.argv[2:], noninteractive=True)


if __name__ == '__main__':
    raise SystemExit(main())
