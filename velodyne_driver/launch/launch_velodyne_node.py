import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

from launch_ros import get_default_launch_description
import launch_ros.actions


def main(argv=sys.argv[1:]):
    """Main."""
    ld = LaunchDescription([
        launch_ros.actions.Node(
            package='velodyne_driver', node_executable='velodyne_node', output='screen', parameters=['cfg/config.yaml'])
    ])

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    # ls = LaunchService(debug=True)
    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description(prefix_output_with_name=False))
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
