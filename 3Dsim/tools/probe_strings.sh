#!/usr/bin/env bash
# 对比三个传感器插件 .so 的内置字符串(话题名/参数名),推断行为差异
for so in gazebo_ros_imu_sensor gazebo_ros_camera gazebo_ros_ray_sensor; do
  echo "########## $so ##########"
  strings /opt/ros/humble/lib/lib${so}.so | grep -iE "imu_data|imu$|frame_name|topic_name|~/out|:=|orientation_as_ref" | sort -u | head -12
  echo "--- 默认话题候选 ---"
  strings /opt/ros/humble/lib/lib${so}.so | grep -E "^~?/?[a-z_]+$" | grep -iE "imu|out|scan|image" | sort -u | head -8
done
