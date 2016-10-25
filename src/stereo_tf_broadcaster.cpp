#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_tf_publisher");
  ros::NodeHandle n;
  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;
  while(n.ok())
  {
    broadcaster.sendTransform(
    tf::StampedTransform(
    tf::Transform(tf::Quaternion(1, 1, 1, 1), tf::Vector3(0.1, 0.0, 0.2)),
    ros::Time::now(),"base_link", "stereo_frame"));
    r.sleep();
  }
}
