// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

//std
#include <iostream>

using namespace std;

class ur_moveit
{

public:
    ur_moveit()
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();

        //Move group definition
        moveit::planning_interface::MoveGroup group("manipulator");

        //Print the name of base link and end-effector link
        ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
        ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

        //Move to home position. Check SRDF for this
        group.setNamedTarget("home");
        group.move();
        sleep(2);

        //Move to up position.
        group.setNamedTarget("up");
        group.move();
        sleep(2);
    }
    ~ur_moveit(){
        ros::shutdown();
    }


private:
    ros::NodeHandle nh_;

};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"ur_moveie_node");
    ur_moveit um;
    cout<<"finish"<<endl;



}
