//ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <string>

Eigen::Affine3d generateRandomHemispherePose(const Eigen::Vector3d &obj_origin, const Eigen::Vector3d &tool_origin)
{
  // Generate random point on upper hemisphere :
  Eigen::Vector3d point;
  point[2] = obj_origin[2];
  double radius = (obj_origin - tool_origin).norm();

  while (point[2] < obj_origin[2] + 0.9 * radius)
  {
    double phy = rand() % 161 + 10;
    double teta = rand() % 360;

    point[0] = radius * cos(phy) * cos(teta) + obj_origin[0];
    point[1] = radius * cos(phy) * sin(teta) + obj_origin[1];
    point[2] = radius * sin(phy) + obj_origin[2];
  }

  // Z axis = obj_origin -> point
  Eigen::Vector3d z_axis;
  z_axis = obj_origin - point;
  z_axis.normalize();

  // Y axis = Generate random point on plane represented by Z vector
  Eigen::Vector3d y_axis;
  y_axis << z_axis[1], z_axis[0], z_axis[2];
  y_axis = y_axis.cross(z_axis); // Temporary y axis

  y_axis = y_axis + point;
  y_axis.normalize();

  // X axis = Cross product
  Eigen::Vector3d x_axis(y_axis.cross(z_axis));
  x_axis.normalize();

  // Recompute Y axis
  y_axis = (z_axis.cross(x_axis));
  y_axis.normalize();

  // Assign rotations and translation
  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  pose.matrix().col(0) << x_axis, 0;
  pose.matrix().col(1) << y_axis, 0;
  pose.matrix().col(2) << z_axis, 0;
  pose.translation() = point;
  return pose;
}

class ex_cal
{
    ros::NodeHandle nh;
    ros::Publisher status_pub;
    ros::Publisher marker_pub;

public:
    ex_cal(int num_poses,std::string tcp_name):
        num_poses_(num_poses),
        tcp_name_(tcp_name)
    {
        //initialize publisher
        status_pub=nh.advertise<std_msgs::String>("ensenso_calibration_status",1);
        marker_pub=nh.advertise<visualization_msgs::Marker>("hemisphere",1);
    }

    bool performCalibration()
    {
        //get UR initial pose
        tf::TransformListener listener;
        listener.waitForTransform("/base",tcp_name_,ros::Time::now(),ros::Duration(3.0));
        tf::StampedTransform transform_stamped;
        Eigen::Affine3d initial_pose;

        std_msgs::String status;
        try{
            listener.lookupTransform("/base",tcp_name_,ros::Time(0),transform_stamped);
            tf::transformTFToEigen(transform_stamped,initial_pose);
        }
        catch (tf::TransformException &ex)
        {
            status.data=ex.what();
            status_pub.publish(status);
            return false;
        }

        Eigen::Vector3d tool_origin(initial_pose.translation()[0],
                                    initial_pose.translation()[1],
                                    initial_pose.translation()[2]),obj_origin(tool_origin[0],tool_origin[1],tool_origin[2]-calTabDistance/1000.0);

        sphere.header.stamp=ros::Time::now();
        sphere.pose.position.x=obj_origin[0];
        sphere.pose.position.y=obj_origin[1];
        sphere.pose.position.z=obj_origin[2];
        sphere.scale.x=2*(calTabDistance/1000);
        sphere.scale.y=2*(calTabDistance/1000);
        sphere.scale.z=2*(calTabDistance/1000);

        marker_pub.publish(sphere);

        //Store UR poses
        std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses;
        int fail_count=0;

        while(nh.ok() && robot_poses.size()<num_poses_)
        {
            if(fail_count>3){
                //Deal with failure...
                ROS_ERROR("Too many failures! Abort calibration!");

                //Go back to initial pose
                    //...

                return false;
            }

            geometry_msgs::Pose wayPoint;
            tf::poseEigenToMsg(generateRandomHemispherePose(obj_origin,tool_origin),wayPoint);
            //Move UR to wayPoint
                //...

            //sleep until UR reach the goal point
            sleep(1);

            //collect pattern

            //collect robot pose
            try{
                ros::Time now(ros::Time::now());
                listener.waitForTransform("/base","/tool0",now,ros::Duration(1.5));
                listener.lookupTransform("/base","/tool0",now,transform_stamped);
                Eigen::Affine3d robot_pose;
                tf::transformTFToEigen(transform_stamped,robot_pose);
                robot_poses.push_back(robot_pose);
            }
            catch(tf::TransformException& ex){
                status.data = ex.what();
                status_pub.publish(status);
                return false;
            }

            sleep(1);

            //Move UR to initial pose
                //...

        }

        //Perform calibration
            //...

        //save calibration
            //...

        return true;
    }

public:
    int num_poses_;
    std::string tcp_name_;
    float calTabDistance; //unit: meter
    visualization_msgs::Marker sphere;
};


int main(int argc, char** argv)
{

    return 0;
}
