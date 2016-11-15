//ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <ensenso/CapturePattern.h>
#include <ensenso/InitCalibration.h>
#include <ensenso/ComputeCalibration.h>
#include <ensenso/RegistImage.h>

//msg
#include <control_msgs/FollowJointTrajectoryAction.h>

//action
#include <actionlib/client/simple_action_client.h>

//trac_ik
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

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
    //Publisher
    ros::Publisher status_pub;
    ros::Publisher marker_pub;
    //Service client
    ros::ServiceClient capture_pattern_client;
    ros::ServiceClient init_cal_client;
    ros::ServiceClient compute_cal_client;
    //Service srv
    ensenso::InitCalibration init_cal_srv;
    ensenso::CapturePattern capture_pattern_srv;
    ensenso::ComputeCalibration compute_cal_srv;

public:
    ex_cal(int num_poses):
        num_poses_(num_poses)
    {
        //initialize parameters
        nh.param("chain_start", chain_start_, std::string("base"));
        nh.param("chain_end", chain_end_, std::string("tool0"));
        base_name_=chain_start_;
        tcp_name_=chain_end_;
        nh.param("timeout", timeout_, 0.005);
        nh.param("urdf_param", urdf_param_, std::string("/robot_description"));
        joint_names.push_back("shoulder_pan_joint");
        joint_names.push_back("shoulder_lift_joint");
        joint_names.push_back("elbow_joint");
        joint_names.push_back("wrist_1_joint");
        joint_names.push_back("wrist_2_joint");
        joint_names.push_back("wrist_3_joint");
        //initialize publisher
        status_pub=nh.advertise<std_msgs::String>("ensenso_calibration_status",1);
        marker_pub=nh.advertise<visualization_msgs::Marker>("hemisphere",1);
        //initialize service client
        capture_pattern_client = nh.serviceClient<ensenso::CapturePattern>("capture_pattern");
        init_cal_client = nh.serviceClient<ensenso::InitCalibration>("init_calibration");
        compute_cal_client = nh.serviceClient<ensenso::ComputeCalibration>("compute_calibration");
    }

    bool ur5_trac_ik(const std::string& chain_start, const std::string& chain_end,const geometry_msgs::Pose& pose,control_msgs::FollowJointTrajectoryActionGoal& actGoal)
    {
        double eps = 1e-5;

        // This constructor parses the URDF loaded in rosparm urdf_param into the
        // needed KDL structures.  We then pull these out to compare against the KDL
        // IK solver.
        TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param_, timeout_, eps);

        KDL::Chain chain;
        KDL::JntArray ll, ul; //lower joint limits, upper joint limits

        bool valid = tracik_solver.getKDLChain(chain);
        if (!valid) {
          ROS_ERROR("There was no valid KDL chain found");
          return false;
        }

        valid = tracik_solver.getKDLLimits(ll,ul);
        if (!valid) {
          ROS_ERROR("There were no valid KDL joint limits found");
          return false;
        }

        assert(chain.getNrOfJoints() == ll.data.size());
        assert(chain.getNrOfJoints() == ul.data.size());

        // Create Nominal chain configuration midway between all joint limits
        KDL::JntArray nominal(chain.getNrOfJoints());
        for (uint j=0; j<nominal.data.size(); j++) {
          nominal(j) = (ll(j)+ul(j))/2.0;
        }

        //Convert geometry_msg/pose to eigen pose
        Eigen::Affine3d eiPose;
        tf::poseMsgToEigen(pose,eiPose);
        //Asign value to KDL pose
        KDL::Frame end_effector_pose;
        end_effector_pose.M.data[0] = eiPose(0,0);
        end_effector_pose.M.data[1] = eiPose(0,1);
        end_effector_pose.M.data[2] = eiPose(0,2);
        end_effector_pose.p.data[0] = eiPose(0,3);
        end_effector_pose.M.data[3] = eiPose(1,0);
        end_effector_pose.M.data[4] = eiPose(1,1);
        end_effector_pose.M.data[5] = eiPose(1,2);
        end_effector_pose.p.data[1] = eiPose(1,3);
        end_effector_pose.M.data[6] = eiPose(2,0);
        end_effector_pose.M.data[7] = eiPose(2,1);
        end_effector_pose.M.data[8] = eiPose(2,2);
        end_effector_pose.p.data[2] = eiPose(2,3);

        //Compute IK
        KDL::JntArray result;
        int rc = tracik_solver.CartToJnt(nominal,end_effector_pose,result);
        if (rc>=0) {
              ROS_INFO("J0=%f,J1=%f,J2=%f,J3=%f,J4=%f,J5=%f;",
                                  result.data[0],result.data[1],result.data[2],result.data[3],result.data[4],result.data[5]);
              return true;
        }
          else {
              ROS_INFO("No valid solution found!!!");
              return false;
          }
        //Define action Goal
        actGoal.goal.trajectory.joint_names=joint_names;
        trajectory_msgs::JointTrajectoryPoint joint_angles;
        for(int i=0;i<6;++i)
            joint_angles.positions.push_back((double)result.data[i]);
        actGoal.goal.trajectory.points.push_back(joint_angles);

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

        //Generate hemisphere for robot poses generation
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

        //vector for storing UR poses
        std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_poses;
        int fail_count=0;

        //Initialize Calibration (Set grid space & Clear buffer)
        ros::service::waitForService("init_calibration");
        init_cal_srv.request.grid_spacing=20.0;
        init_cal_client.call(init_cal_srv);
        if(!init_cal_srv.response.success){
            ROS_ERROR("Initialize calibration fail!");
            return false;
        }

        while(nh.ok() && robot_poses.size()<num_poses_)
        {
            geometry_msgs::Pose wayPoint;

            if(fail_count>5){
                //Deal with failure...
                ROS_ERROR("Too many failures! Abort calibration!");

                //Go back to initial pose
                tf::poseEigenToMsg(initial_pose,wayPoint);
                    //...

                return false;
            }

            //get a random point on the hemisphere
            tf::poseEigenToMsg(generateRandomHemispherePose(obj_origin,tool_origin),wayPoint);

            //Move UR to the random point
                //...

            //sleep until UR reach the goal point
            sleep(1);

            //Collect patterns
                //block until service available
            ros::service::waitForService("capture_pattern");
            capture_pattern_client.call(capture_pattern_srv);

            if(!capture_pattern_srv.response.success)
            {
                fail_count++;
                //Move UR to defalut pose
                    //...
                continue;
            }

            //collect robot pose
            try{
                ros::Time now(ros::Time::now());
                listener.waitForTransform("/base",tcp_name_,now,ros::Duration(1.5));
                listener.lookupTransform("/base",tcp_name_,now,transform_stamped);
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
        status.data = "Computing calibration matrix...";
        status_pub.publish(status);

        if(robot_poses.size()!=capture_pattern_srv.response.pattern_count){
            ROS_ERROR("The number of robot poses is not consistent with the counts of pattern. Aborting calibraiton!");
            return false;
        }else{
            std::string result;
            //Initialize srv request
            compute_cal_srv.request.store_to_eeprom=true;
            tf::poseEigenToMsg(Eigen::Affine3d::Identity(),compute_cal_srv.request.seed);
                //Populate the srv poses
            compute_cal_srv.request.robotposes.poses.resize(robot_poses.size());
            for(int i=0;i<robot_poses.size();++i)
                tf::poseEigenToMsg(robot_poses[i],compute_cal_srv.request.robotposes.poses[i]);
            //call the srv
            compute_cal_client.call(compute_cal_srv);
        }
        if(compute_cal_srv.response.success){
            ROS_INFO("Calibraiton computation finishes");
            ROS_INFO("Result: ");
            ROS_INFO("Position: x = %f, y = %f, z = %f",compute_cal_srv.response.result.position.x,compute_cal_srv.response.result.position.y,compute_cal_srv.response.result.position.z);
            ROS_INFO("Orientation: w = %f, x = %f, y = %f, z = %f",compute_cal_srv.response.result.orientation.w,compute_cal_srv.response.result.orientation.x,compute_cal_srv.response.result.orientation.y,compute_cal_srv.response.result.orientation.z);

        }else{
            ROS_ERROR("Fail to compute extrinsic calibration!");
            return false;
        }


        //save calibration
            //...

        return true;
    }

public:
    //number of poses for calibration
    int num_poses_;
    //Define kinematics chain for IK solver
    std::string chain_start_;
    std::string chain_end_;
    std::string tcp_name_;
    std::string base_name_;
    //Define joint names
    std::vector<std::string> joint_names;
    //Time duration for IK solver
    double timeout_;
    //URDF varable
    std::string urdf_param_;
    //Approximate distance for cal board to camera
    float calTabDistance; //unit: meter
    //Visualization hemisphere for pose sampling
    visualization_msgs::Marker sphere;
};


int main(int argc, char** argv)
{

    ros::init(argc,argv,"extrin_cal");
    ros::NodeHandle nh;

    //Publisher
    ros::Publisher image_pub=nh.advertise<sensor_msgs::Image>("/registered_image",1);
    ros::Publisher cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/registered_pointcloud",1);

    //Service client
    ros::ServiceClient capture_pattern_client=nh.serviceClient<ensenso::CapturePattern>("capture_pattern");
    ros::ServiceClient ini_cal_client=nh.serviceClient<ensenso::InitCalibration>("init_calibration");
    ros::ServiceClient grab_registered_image_client=nh.serviceClient<ensenso::RegistImage>("grab_registered_image");
    //Service srv
    ensenso::RegistImage grab_registImg_srv;
    grab_registImg_srv.request.is_rgb=true;

    ros::Rate loop(1);
    while(ros::ok()){
        grab_registered_image_client.call(grab_registImg_srv);
        image_pub.publish(grab_registImg_srv.response.image);
        cloud_pub.publish(grab_registImg_srv.response.pointcloud);
        loop.sleep();
    }


    return 0;
}
