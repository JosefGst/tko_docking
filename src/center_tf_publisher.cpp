/*
 * MIT License
 *
 * Copyright (c) 2024 Josef Gstoettner
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <string>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped transformStamped;

  // init params
  std::string center_tf_name;
  if (!node.param("center_tf", center_tf_name, std::string("center")))
  {
    ROS_WARN("center_tf_name not set, using default: center");
  }

  ros::Rate rate(10.0);
  while (node.ok())
  {
    geometry_msgs::TransformStamped transformStamped_0, transformStamped_1;
    try
    {
      transformStamped_0 = tfBuffer.lookupTransform("map", "corner0",
                                                    ros::Time(0));
      transformStamped_1 = tfBuffer.lookupTransform("map", "corner1",
                                                    ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    // ROS_INFO_STREAM("transformStamped_0: [" << transformStamped_0 << "]");
    // ROS_INFO_STREAM("transformStamped_1: [" << transformStamped_1 << "]");

    // tf2 tf_broadcaster the center
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = center_tf_name;
    transformStamped.transform.translation.x = (transformStamped_0.transform.translation.x +
                                                transformStamped_1.transform.translation.x) /
                                               2;
    transformStamped.transform.translation.y = (transformStamped_0.transform.translation.y +
                                                transformStamped_1.transform.translation.y) /
                                               2;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;

    // calculate the yaw between the two corners
    double yaw = atan2(transformStamped_1.transform.translation.y - transformStamped_0.transform.translation.y,
                       transformStamped_1.transform.translation.x - transformStamped_0.transform.translation.x);

    q.setRPY(1.5708, 0, yaw);  // rotate similar as the apriltag tf
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(transformStamped);

    rate.sleep();
  }
  return 0;
};
