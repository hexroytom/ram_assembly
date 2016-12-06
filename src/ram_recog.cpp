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
#include <ensenso/ConfigureStreaming.h>
#include <ensenso/CaptureSinglePointCloud.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ram_assembly/moment_of_inertia_estimation.h"

//std
#include <string>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;

class ramRecg
{

private:
    ros::NodeHandle nh;
    //Service client
    ros::ServiceClient grab_pointcloud_client;
    //Publisher
    ros::Publisher traj_publisher;
    //Move group
    boost::shared_ptr<moveit::planning_interface::MoveGroup> group;

public:
    PointCloudXYZ::Ptr pts_;
    PointCloudXYZ::Ptr pts_no_plane;

    string PCD_file_;

public:

    //Constructor
    ramRecg(const string PCD_file) :
        PCD_file_(PCD_file),
        pts_(new PointCloudXYZ),
        pts_no_plane(new PointCloudXYZ)
    {
        if (read_PCD(PCD_file_, pts_) == -1)
             PCL_ERROR("Initiate object failed!");
    }

    ramRecg() :
        pts_(new PointCloudXYZ),
        pts_no_plane(new PointCloudXYZ)
    {
        //Service client
        grab_pointcloud_client=nh.serviceClient<ensenso::CaptureSinglePointCloud>("capture_single_point_cloud");
        //Publisher
        traj_publisher=nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_trajectory",1);
        //Move group

        group.reset();
        group = boost::make_shared<moveit::planning_interface::MoveGroup>("manipulator");

        group->setPoseReferenceFrame("/base");
        group->setMaxVelocityScalingFactor(0.3);
        group->setMaxAccelerationScalingFactor(0.3);
    }

    //Grab pointcloud via service
    bool grab_single_pointcloud(PointCloudXYZ::Ptr output_pc)
    {

        if(!ros::service::waitForService("capture_single_point_cloud",ros::Duration(0)))
            {
            ROS_ERROR("Wait for service: capture_single_point_cloud. Time Out!");
            return false;
        }

        ensenso::CaptureSinglePointCloud srv;
        srv.request.req=true;
        grab_pointcloud_client.call(srv);
        pcl::fromROSMsg(srv.response.pc,*output_pc);
        return true;

    }

    //Read pcd into pointcloud
    int read_PCD(const std::string path, PointCloudXYZ::Ptr pts)
    {
        int result = pcl::io::loadPCDFile(path, *pts);
        if (result == -1)
            PCL_ERROR("Could not load PCD file! Please check your path.");
        return result;
    }

    //Visualize pointcloud
    bool viz_pc(PointCloudXYZ::Ptr pts_)
    {
        pcl::visualization::PCLVisualizer view("Viz");
        if(pts_->empty())
        {
            PCL_ERROR("Invaild pointcloud!");
            return false;
        }else
        {
            view.addPointCloud<pcl::PointXYZ>(pts_);
            view.spin();
            return true;
        }
    }

    //Plane segmentation
    void plane_segment_proc(PointCloudXYZ::Ptr input_pts, PointCloudXYZ::Ptr output_pts,float distance_thresh, pcl::PointIndices::Ptr inliers,bool is_negative)
    {
        //Plane segmentation
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
        // Optional
        plane_seg.setOptimizeCoefficients(false);
        // Mandatory
        plane_seg.setModelType(pcl::SACMODEL_PLANE);
        plane_seg.setMethodType(pcl::SAC_RANSAC);
        plane_seg.setDistanceThreshold(distance_thresh);
        plane_seg.setInputCloud(input_pts);
        plane_seg.segment(*inliers, *coefficients);
        //Filter pts of plane
        extractPointsByIndices(inliers,input_pts,output_pts,is_negative,false);
    }

    //Extract PointXYZ by indices
    void extractPointsByIndices(pcl::PointIndices::Ptr indices, const PointCloudXYZ::Ptr ref_pts, PointCloudXYZ::Ptr extracted_pts, bool is_negative,bool is_organised)
    {
        pcl::ExtractIndices<pcl::PointXYZ> tmp_extractor;
        tmp_extractor.setKeepOrganized(is_organised);
        tmp_extractor.setInputCloud(ref_pts);
        tmp_extractor.setNegative(is_negative);
        tmp_extractor.setIndices(indices);
        tmp_extractor.filter(*extracted_pts);
    }

    void regionGrowingseg(PointCloudXYZ::ConstPtr input_pts,PointCloudXYZ::Ptr output_pts)
    {
        //Remove Nan pts from input points
        PointCloudXYZ::Ptr NanFree_pts(new PointCloudXYZ);
        vector<int> pts_index;
        pcl::removeNaNFromPointCloud<pcl::PointXYZ>(*input_pts,*NanFree_pts,pts_index);

        //Normal estimation
        pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> ne(4);
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(tree);
        ne.setInputCloud(NanFree_pts);
        ne.setRadiusSearch(0.01);
        ne.compute(*normal);

        //Region growing
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMaxClusterSize(50000);
        reg.setMinClusterSize(200);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(5);
        reg.setInputCloud(NanFree_pts);
        reg.setInputNormals(normal);
        reg.setSmoothnessThreshold(2.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(1);
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        //Extract cluster
        //output_pts=boost::make_shared<PointCloudXYZ>();
        std::vector<pcl::PointIndices>::iterator single_cluster = clusters.begin();
        pcl::PointIndices::Ptr indices_ptr = boost::make_shared<pcl::PointIndices>(*single_cluster);
        extractPointsByIndices(indices_ptr,NanFree_pts,output_pts,false,false);
        //viz_pc(cluster);

    }

    Eigen::Affine3d poseEstimationByMomentOfInertia(PointCloudXYZ::Ptr input_pts,bool is_viz)
    {
        pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud (input_pts);
        feature_extractor.compute ();

        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;

        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues (major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter (mass_center);

        //Angle
            //z axis
        Eigen::Vector4f v1(minor_vector[0],minor_vector[1],minor_vector[2],0.0);
        Eigen::Vector4f v2(0.0,0.0,1.0,0.0);
        double theta1= pcl::getAngle3D(v1,v2);
        theta1=theta1/M_PI*180.0;
        if(theta1<90.0){
            minor_vector[0]=-minor_vector[0];
            minor_vector[1]=-minor_vector[1];
            minor_vector[2]=-minor_vector[2];
        }
            //y axis
        Eigen::Vector4f v3(middle_vector[0],middle_vector[1],middle_vector[2],0.0);
        Eigen::Vector4f v4(0.0,1.0,0.0,0.0);
        double theta2= pcl::getAngle3D(v3,v4);
        theta2=theta2/M_PI*180.0;
        if(theta2<90.0){
            middle_vector[0]=-middle_vector[0];
            middle_vector[1]=-middle_vector[1];
            middle_vector[2]=-middle_vector[2];
        }
            // x axis
        Eigen::Vector4f v5(major_vector[0],major_vector[1],major_vector[2],0.0);
        Eigen::Vector4f v6(1.0,0.0,0.0,0.0);
        double theta3= pcl::getAngle3D(v5,v6);
        theta3=theta3/M_PI*180.0;
        if(theta3>90.0){
            major_vector[0]=-major_vector[0];
            major_vector[1]=-major_vector[1];
            major_vector[2]=-major_vector[2];
        }

        //graping pose
        double y_offset = (max_point_OBB.y-min_point_OBB.y)/2;
        y_offset += 0.015;
        Eigen::Vector3d mass_center_offset;
        mass_center_offset(0)=mass_center(0)+y_offset*middle_vector(0);
        mass_center_offset(1)=mass_center(1)+y_offset*middle_vector(1);
        mass_center_offset(2)=mass_center(2)+y_offset*middle_vector(2);

        double z_offset=0.002;
        mass_center_offset(0)=mass_center_offset(0)-z_offset*minor_vector(0);
        mass_center_offset(1)=mass_center_offset(1)-z_offset*minor_vector(1);
        mass_center_offset(2)=mass_center_offset(2)-z_offset*minor_vector(2);

        //Fill the pose matrix
        Eigen::Matrix4d m;
        m << major_vector[0],middle_vector[0],minor_vector[0],mass_center_offset[0],
                major_vector[1],middle_vector[1],minor_vector[1],mass_center_offset[1],
                major_vector[2],middle_vector[2],minor_vector[2],mass_center_offset[2],
                0,0,0,1;
        //std::cout<<Pose<<endl;
        Eigen::Affine3d pose(m);

        if(is_viz)
        {
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->setBackgroundColor (0, 0, 0);
            viewer->addCoordinateSystem (0.1);
            viewer->initCameraParameters ();

            Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
            Eigen::Quaternionf quat (rotational_matrix_OBB);
            viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z,"OBB");

            pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
            pcl::PointXYZ x_axis (major_vector (0)/10 + mass_center (0), major_vector (1)/10 + mass_center (1), major_vector (2)/10 + mass_center (2));
            pcl::PointXYZ y_axis (middle_vector (0)/10 + mass_center (0), middle_vector (1)/10 + mass_center (1), middle_vector (2)/10 + mass_center (2));
            pcl::PointXYZ z_axis (minor_vector (0)/10 + mass_center (0), minor_vector (1)/10 + mass_center (1), minor_vector (2)/10 + mass_center (2));
            viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
            viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
            viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

            pcl::PointXYZ center_offset (mass_center_offset (0), mass_center_offset (1), mass_center_offset (2));
            pcl::PointXYZ x_axis_off (major_vector (0)/10 + mass_center_offset (0), major_vector (1)/10 + mass_center_offset (1), major_vector (2)/10 + mass_center_offset (2));
            pcl::PointXYZ y_axis_off (middle_vector (0)/10 + mass_center_offset (0), middle_vector (1)/10 + mass_center_offset (1), middle_vector (2)/10 + mass_center_offset (2));
            pcl::PointXYZ z_axis_off (minor_vector (0)/10 + mass_center_offset (0), minor_vector (1)/10 + mass_center_offset (1), minor_vector (2)/10 + mass_center_offset (2));
            viewer->addLine (center_offset, x_axis_off, 1.0f, 0.0f, 0.0f, "pose x");
            viewer->addLine (center_offset, y_axis_off, 0.0f, 1.0f, 0.0f, "pose y");
            viewer->addLine (center_offset, z_axis_off, 0.0f, 0.0f, 1.0f, "pose z");


            viewer->addPointCloud(pts_);
            viewer->spin();

        }

        return pose;

    }

    //linear interpolation for approaching the taerger. Base frame: /base.
    bool linear_trajectory_planning(tf::Transform target_pose, double line_offset, double num_of_interpolation, moveit_msgs::RobotTrajectory& target_traj )
    {
//        //Y axis
//        Eigen::Vector3d direc = Eigen::Vector3d(target_pose(0,1),target_pose(1,1),target_pose(2,1));
//        double step = line_offset/num_of_interpolation;

//        //Target pose
//        geometry_msgs::Pose target_pose_msg;
//        tf::poseEigenToMsg(target_pose,target_pose_msg);

//        std::vector<geometry_msgs::Pose> waypoints(num_of_interpolation+1);
//        for(int i=0;i<waypoints.size()-1;++i)
//        {
//            geometry_msgs::Pose tmp_pose;
//            tmp_pose.position.x = target_pose_msg.position.x + step * direc(0) * (num_of_interpolation-i);
//            tmp_pose.position.y = target_pose_msg.position.y + step * direc(1) * (num_of_interpolation-i);
//            tmp_pose.position.z = target_pose_msg.position.z + step * direc(2) * (num_of_interpolation-i);
//            tmp_pose.orientation = target_pose_msg.orientation;
//        }
//        waypoints[num_of_interpolation]=target_pose_msg;

        //Y axis
        tf::Matrix3x3 rot_mat=target_pose.getBasis();
        tf::Vector3 direc = rot_mat.getColumn(1);;
        double step = line_offset/num_of_interpolation;

        //Interpolate n points
        std::vector<geometry_msgs::Pose> waypoints(num_of_interpolation+1);
        geometry_msgs::Pose target_pose_msg;
        tf::poseTFToMsg(target_pose,target_pose_msg);
        for(int i=0;i<waypoints.size()-1;++i)
        {
            geometry_msgs::Pose tmp_pose;
            tmp_pose.position.x = target_pose_msg.position.x + step * direc[0] * (num_of_interpolation-i);
            tmp_pose.position.y = target_pose_msg.position.y + step * direc[1] * (num_of_interpolation-i);
            tmp_pose.position.z = target_pose_msg.position.z + step * direc[2] * (num_of_interpolation-i);
            tmp_pose.orientation = target_pose_msg.orientation;
            waypoints[i]=tmp_pose;
        }
        waypoints[num_of_interpolation]=target_pose_msg;

        //ComputeCartesian...        
        double score=group->computeCartesianPath(waypoints,0.05,0.0,target_traj);


        if(score > 0.95)
        {
//            for(int i=0;i<target_traj.joint_trajectory.points.size();++i)
//            {
//                for(int j=0;j<6;++j)
//                {
//                    target_traj.joint_trajectory.points[i].velocities.push_back(0.1);
//                }
//            }

            moveit::core::RobotStatePtr kinematic_state(group->getCurrentState());
            robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(),"manipulator");
            rt.setRobotTrajectoryMsg(*kinematic_state,target_traj);
            trajectory_processing::IterativeParabolicTimeParameterization iptp;

            if(iptp.computeTimeStamps(rt,0.2,0.2))
                {

                rt.getRobotTrajectoryMsg(target_traj);
                return true;
            }

            return true;
        }
        else
        {
            ROS_ERROR("Cartessian Planning fail!");
            return false;
        }


    }

    void performGrasping(const moveit_msgs::RobotTrajectory& robot_traj)
    {
        //Moveit
        ros::AsyncSpinner spinner(1);
        spinner.start();

        group->setMaxVelocityScalingFactor(0.1);
        group->setMaxAccelerationScalingFactor(0.1);

        moveit::planning_interface::MoveGroup::Plan planner;
        planner.trajectory_=robot_traj;

//            moveit_msgs::DisplayTrajectory display_traj;
//            display_traj.trajectory_start=planner.start_state_;
//            display_traj.trajectory.push_back(planner.trajectory_);
//            traj_publisher.publish(display_traj);
        group->execute(planner);
    }

    bool moveToInitPose()
    {
        //Moveit
        ros::AsyncSpinner spinner(1);
        spinner.start();

        moveit::planning_interface::MoveGroup::Plan planner;

        group->setNamedTarget("up");
        group->move();

        std::vector<double> joint_angles(6);
        joint_angles[0]=-37.31/180.0*M_PI;
        joint_angles[1]=-18.85/180.0*M_PI;
        joint_angles[2]=-158.31/180.0*M_PI;
        joint_angles[3]=-42.02/180.0*M_PI;
        joint_angles[4]=84.37/180.0*M_PI;
        joint_angles[5]=-88.68/180.0*M_PI;
        group->setJointValueTarget(joint_angles);

        if(group->plan(planner))
        {
            group->move();
        }else{
            std::cout<<"Planning fail!"<<std::endl;\
            return false;
        }
        spinner.stop();
        return true;
    }

};

tf::StampedTransform get_Transform(std::string parent,std::string child)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform(parent,child,ros::Time(0),ros::Duration(1.5));
    listener.lookupTransform(parent,child,ros::Time(0),transform);
    std::cout<<parent<<" to "<<child<<" transfotm: "<<std::endl;
    std::cout<<"Position: "<<"x: "<<transform.getOrigin().getX()<<" y: "<<transform.getOrigin().getY()<<" z: "<<transform.getOrigin().getZ()<<std::endl;
    std::cout<<"Orientation: "<<"x: "<<transform.getRotation().getX()<<" y: "<<transform.getRotation().getY()<<" z: "<<transform.getRotation().getZ()<<" w: "<<transform.getRotation().getW()<<std::endl;

    return transform;
}

int main(int argc,char** argv)
{
    //Ros service call
    ros::init(argc,argv,"ram_recognition_node");
    ramRecg ram_rec;

    //UR move to init pose
    if(!ram_rec.moveToInitPose()) return -1;

    //Grab pointcloud
    ram_rec.grab_single_pointcloud(ram_rec.pts_);
    //ram_rec.viz_pc(ram_rec.pts_);

    //Remove plane
    pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
    ram_rec.plane_segment_proc(ram_rec.pts_,ram_rec.pts_no_plane,0.01,plane_indices,true);

    //Use region growing to segment pts
    PointCloudXYZ::Ptr ram(new PointCloudXYZ);
    ram_rec.regionGrowingseg(ram_rec.pts_no_plane,ram);

    //Use PCA analysis to get pose
    Eigen::Affine3d cam2obj_tf_ = ram_rec.poseEstimationByMomentOfInertia(ram,true);
    tf::Transform cam2obj_tf;
    tf::poseEigenToTF(cam2obj_tf_,cam2obj_tf);

    //Get base--->camera transform
    tf::Transform base2cam_tf=get_Transform("base","camera_link");
    tf::Transform base2obj_tf=base2cam_tf * cam2obj_tf;

    //Interpolation
    moveit_msgs::RobotTrajectory robot_traj;
    ram_rec.linear_trajectory_planning(base2obj_tf,0.1,10,robot_traj);

    //Perform grasping
    ram_rec.performGrasping(robot_traj);


}
