#!/usr/bin/env python3
# nav2_bringup's bringup_launch.py with the lifecycle bond watchdog disabled.
#
# Under CPU load a manager sometimes stops seeing a node's heartbeat although the
# node is alive (20 failures logged up to 2026-11-06, on 7 different nodes).  It
# then deactivates every node and hangs half way, needing a full Nav2 restart.
# Raising bond_timeout does not help: at 20 s it failed the same way, 45 s after
# bringup, so both ends of the bond are switched off instead.
#
# These cannot go in nav2_params_*.yaml: nav2_bringup gives the lifecycle managers
# no params_file.  A real server crash is no longer propagated to the other nodes;
# the motors still stop on Mcnamu_driver's own 0.3 s watchdog.
#
# Takes the same arguments as bringup_launch.py.  Started by start_nav2.sh.
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
    return LaunchDescription([
        SetParameter(name='bond_timeout', value=0.0),
        SetParameter(name='bond_heartbeat_period', value=0.0),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup)),
    ])
