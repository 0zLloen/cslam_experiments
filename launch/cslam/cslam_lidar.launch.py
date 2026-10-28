import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)

    loop_detection_node = Node(package='cslam',
                               executable='loop_closure_detection_node.py',
                               name='cslam_loop_closure_detection',
                               parameters=[
                                   LaunchConfiguration('config'), {
                                       "robot_id": LaunchConfiguration('robot_id'),
                                       "max_nb_robots": LaunchConfiguration('max_nb_robots'),
                                   }
                               ],
                               namespace=LaunchConfiguration('namespace'))

    map_manager_node = Node(package='cslam',
                            executable='lidar_handler_node.py',
                            name='cslam_map_manager',
                            parameters=[
                                LaunchConfiguration('config'), {
                                    "robot_id": LaunchConfiguration('robot_id'),
                                    "max_nb_robots": LaunchConfiguration('max_nb_robots'),
                                }
                            ],
                            remappings=[
                                ('cslam/local_descriptors_request',
                                 '/r' + robot_id + '/cslam/local_descriptors_request'),
                            ],
                            prefix=LaunchConfiguration('launch_prefix_cslam'),
                            namespace=LaunchConfiguration('namespace'))

    # The decentralized PGO backend HARDCODES /r{id}/cslam/... for its
    # cross-robot topics (decentralized_pgo.cpp), but we run each robot in a
    # ROBOT-NAME namespace (/alpha, /bob, ...) rather than /r{id}. So every
    # relative cslam/<topic> subscription resolves to /alpha/cslam/<topic> while
    # the matching publisher on another robot targets /r{id}/cslam/<topic> --
    # they never connect. We must remap each such relative topic to the absolute
    # /r{id} name the publishers use. Missing any one silently breaks that leg:
    #   * heartbeat            -> without it NeighborMonitor sees no neighbors
    #   * get_pose_graph       -> the OPTIMIZER's pose-graph REQUEST never reaches
    #                             neighbors; check_received_pose_graphs() never
    #                             passes, so the optimizer parks in state 3
    #                             (WAITING_FOR_NEIGHBORS_POSEGRAPHS) forever and
    #                             NO distributed optimisation ever runs.
    #   * optimized_estimates  -> even once optimisation runs, the RESULT never
    #                             reaches each robot, so current_pose_estimates_
    #                             (and thus viz/pose_graph + the merged map) stays
    #                             pure odometry.
    # 2026-07-15: get_pose_graph + optimized_estimates were previously missing;
    # diagnostic (SLAM_LOG_OPTIMIZER_STATE) showed alpha stuck at state 3 and the
    # other robots at state 1, with 0 fusion. (local_descriptors_request is
    # remapped on the map_manager node above.)
    pose_graph_manager_node = Node(package='cslam',
                                   executable='pose_graph_manager',
                                   name='cslam_pose_graph_manager',
                                   parameters=[
                                       LaunchConfiguration('config'), {
                                           "robot_id": LaunchConfiguration('robot_id'),
                                           "max_nb_robots": LaunchConfiguration('max_nb_robots'),
                                           "evaluation.enable_simulated_rendezvous": LaunchConfiguration('enable_simulated_rendezvous'),
                                           "evaluation.rendezvous_schedule_file": LaunchConfiguration('rendezvous_schedule_file'),
                                       }
                                   ],
                                   remappings=[
                                       ('cslam/heartbeat',
                                        '/r' + robot_id + '/cslam/heartbeat'),
                                       ('cslam/get_pose_graph',
                                        '/r' + robot_id + '/cslam/get_pose_graph'),
                                       ('cslam/optimized_estimates',
                                        '/r' + robot_id + '/cslam/optimized_estimates'),
                                   ],
                                   prefix=LaunchConfiguration('launch_prefix_cslam'),
                                   namespace=LaunchConfiguration('namespace'))

    return [
        loop_detection_node,
        map_manager_node,
        pose_graph_manager_node,
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/r0', description=''),
        DeclareLaunchArgument('robot_id', default_value='0', description=''),
        DeclareLaunchArgument('max_nb_robots', default_value='1', description=''),
        DeclareLaunchArgument('config_path',
                              default_value=os.path.join(
                                  get_package_share_directory('cslam_experiments'),
                                  'config/'),
                              description=''),
        DeclareLaunchArgument('config_file',
                              default_value='kitti_lidar.yaml',
                              description=''),
        DeclareLaunchArgument('config',
                              default_value=[
                                  LaunchConfiguration('config_path'),
                                  LaunchConfiguration('config_file')
                              ],
                              description=''),
        DeclareLaunchArgument('launch_prefix_cslam', default_value='', description=''),
        DeclareLaunchArgument('enable_simulated_rendezvous', default_value='false', description=''),
        DeclareLaunchArgument('rendezvous_schedule_file', default_value='', description=''),
        DeclareLaunchArgument('log_level', default_value='fatal', description=''),
        OpaqueFunction(function=launch_setup),
    ])
