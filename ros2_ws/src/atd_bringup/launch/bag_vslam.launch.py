"""Notebook で再生するステレオ bag から地図を作り、点群と軌跡を配信する。"""
from launch import LaunchDescription

from atd_bringup.gazebo_stereo import stereo_container


def generate_launch_description():
    return LaunchDescription([stereo_container(True, mapping=True, visualize_landmarks=True)])
