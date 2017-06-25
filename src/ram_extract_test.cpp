//ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <ensenso/CapturePattern.h>
#include <ensenso/InitCalibration.h>
#include <ensenso/ComputeCalibration.h>
#include <ensenso/RegistImage.h>
#include <ensenso/ConfigureStreaming.h>
#include <ensenso/CaptureSinglePointCloud.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

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
#include <pcl/surface/mls.h>

#include "ram_assembly/moment_of_inertia_estimation.h"

//std
#include <string>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;

bool sortRegionIndx(const pcl::PointIndices& indice1,const pcl::PointIndices& indice2)
{
    return (indice1.indices.size()>indice2.indices.size());
}

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

    //tf from tool0 to depth camera
    Eigen::Affine3d pose_tool0Tdep;

    //UR script publisher
    ros::Publisher ur_script_pub;

public:

    //Constructor
    ramRecg(const string PCD_file) :
        PCD_file_(PCD_file),
        pts_(new PointCloudXYZ),
        pts_no_plane(new PointCloudXYZ)
    {
        if (read_PCD(PCD_file_, pts_) == -1)
             PCL_ERROR("Initiate object failed!");
        //Service client
        grab_pointcloud_client=nh.serviceClient<ensenso::CaptureSinglePointCloud>("capture_single_point_cloud");
        //Publisher
        traj_publisher=nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_trajectory",1);
        ur_script_pub = nh.advertise<std_msgs::String>("/ur_driver/URScript",1);
        //Move group
        group.reset();
        group = boost::make_shared<moveit::planning_interface::MoveGroup>("manipulator");
        group->setPoseReferenceFrame("/base");
        group->setMaxVelocityScalingFactor(0.3);
        group->setMaxAccelerationScalingFactor(0.3);


    }

    ramRecg() :
        pts_(new PointCloudXYZ),
        pts_no_plane(new PointCloudXYZ)
    {
        //Service client
        grab_pointcloud_client=nh.serviceClient<ensenso::CaptureSinglePointCloud>("capture_single_point_cloud");
        //Publisher
        traj_publisher=nh.advertise<moveit_msgs::DisplayTrajectory>("display_planned_trajectory",1);
        ur_script_pub = nh.advertise<std_msgs::String>("/ur_driver/URScript",1);
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

    //Plane segmentation and get coefficients make by csl
    void plane_segment_proc(PointCloudXYZ::Ptr input_pts, PointCloudXYZ::Ptr output_pts,float distance_thresh, pcl::PointIndices::Ptr inliers,bool is_negative,pcl::ModelCoefficients::Ptr coefficients)
    {
        //Plane segmentation
        //pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
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

    //region growing to find max volum
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
        reg.setMinClusterSize(500);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(20);
        reg.setInputCloud(NanFree_pts);
        reg.setInputNormals(normal);
        reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(1);
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);
        std::sort(clusters.begin(),clusters.end(),sortRegionIndx);

        //Extract cluster
        //output_pts=boost::make_shared<PointCloudXYZ>();
        std::vector<pcl::PointIndices>::iterator single_cluster = clusters.begin();
        pcl::PointIndices::Ptr indices_ptr = boost::make_shared<pcl::PointIndices>(*single_cluster);
        extractPointsByIndices(indices_ptr,NanFree_pts,output_pts,false,false);
        //viz_pc(cluster);

    }

    void regionGrowingseg(PointCloudXYZ::ConstPtr input_pts,PointCloudXYZ::Ptr output_pts ,double smoothThread ,double curvatureThread)
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
        reg.setMinClusterSize(500);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(20);
        reg.setInputCloud(NanFree_pts);
        reg.setInputNormals(normal);
        reg.setSmoothnessThreshold(smoothThread / 180.0 * M_PI);
        reg.setCurvatureThreshold(curvatureThread);
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);
        std::sort(clusters.begin(),clusters.end(),sortRegionIndx);

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

//        Eigen::Affine3d pose_rgbTdep=Eigen::Affine3d::Identity ();
//        pose_rgbTdep.affine()=rotational_matrix_OBB;
//        Eigen::Affine3d rotational_matrix_OBB_invert_=pose_rgbTdep.inverse();
//        Eigen::Matrix3f rotational_matrix_OBB_invert=rotational_matrix_OBB_invert_.affine();
//        PointCloudXYZ::Ptr out_pts(new PointCloudXYZ);
//        pcl::transformPointCloud (*input_pts, *out_pts, rotational_matrix_OBB_invert);

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
        y_offset -= 0.02;
        Eigen::Vector3d mass_center_offset;
        mass_center_offset(0)=mass_center(0)+y_offset*middle_vector(0);
        mass_center_offset(1)=mass_center(1)+y_offset*middle_vector(1);
        mass_center_offset(2)=mass_center(2)+y_offset*middle_vector(2);

        double z_offset=0.0;
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

//            position[0]=0;
//            position[1]=0;
//            position[2]=0;
//            rotational_matrix_OBB = Eigen::Matrix3f::Identity();

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
        spinner.stop();
    }

    bool moveToInitPose()
    {
        //Moveit
        ros::AsyncSpinner spinner(1);
        spinner.start();

        moveit::planning_interface::MoveGroup::Plan planner;

        group->setNamedTarget("up");
        group->move();
        sleep(1);

        std::vector<double> joint_angles(6);
        joint_angles[0]=-50.54/180.0*M_PI;
        joint_angles[1]=-40.64/180.0*M_PI;
        joint_angles[2]=-142.28/180.0*M_PI;
        joint_angles[3]=-49.39/180.0*M_PI;
        joint_angles[4]=78.64/180.0*M_PI;
        joint_angles[5]=-73.15/180.0*M_PI;
        group->setJointValueTarget(joint_angles);

        if(group->plan(planner))
        {
            group->move();
            sleep(1);
        }else{
            std::cout<<"Planning fail!"<<std::endl;\
            return false;
        }
        spinner.stop();
        return true;
    }

    void mls_smooth(PointCloudXYZ::Ptr input_pts, float search_radius,PointCloudXYZ::Ptr output_pts)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ> mls;
        mls.setInputCloud(input_pts);
        mls.setComputeNormals(false);
        mls.setPolynomialFit(true);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(search_radius);

        mls.process(*output_pts);

    }

    // do it myself

    void moveToLookForTargetsPose(string namedTarget ,int sleepTime)
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();
        group->setMaxVelocityScalingFactor(0.3);
        group->setMaxAccelerationScalingFactor(0.3);
        group->setNamedTarget(namedTarget);       //group->setPoseTarget(goalPose);
        group->move();
        sleep(sleepTime);
        spinner.stop();
    }

    void moveToLookForTargetsPoseUsingJointAngles(int sleepTime)
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();
        moveit::planning_interface::MoveGroup::Plan planner;
        vector<double> joint_angles(6);
        joint_angles[0]=-46.35/180.0*M_PI;
        joint_angles[1]=-31.26/180.0*M_PI;
        joint_angles[2]=-153.4/180.0*M_PI;
        joint_angles[3]=-38.97/180.0*M_PI;
        joint_angles[4]=89.04/180.0*M_PI;
        joint_angles[5]=-90.65/180.0*M_PI;
        group->setJointValueTarget(joint_angles);
        bool is_success=group->plan(planner);
        if(is_success)
        {
            group->move();
            //sleep(0.5);
        }else{
            cout<<"Move to place pose: Planning fail!"<<endl;
        }
        sleep(sleepTime);
        spinner.stop();
    }

    void getPC(ensenso::CaptureSinglePointCloud& srv_)
    {
        grab_pointcloud_client.call(srv_);

    }

    void broadcastTF(Eigen::Affine3d& pose_eigen, const string& parent_frame, const string& child_frame)
    {
        tf::Transform pose_tf;
        tf::poseEigenToTF(pose_eigen,pose_tf);
        tf::TransformBroadcaster tf_broadcaster;
        tf_broadcaster.sendTransform (tf::StampedTransform(pose_tf,ros::Time::now(),parent_frame,child_frame));

    }

    Eigen::Affine3d getTool0toDepthTF(double x, double y,double z, double qw,double qx,double qy,double qz)
    {
        Eigen::Affine3d pose_Tool0Tdep_;

        //Translation
        pose_Tool0Tdep_.translation()<< x,y,z;

        //Rotation
        Eigen::Quaterniond quat(qw,qx,qy,qz);
        pose_Tool0Tdep_.linear() = quat.toRotationMatrix();

        return pose_Tool0Tdep_;
    }

    void setTool0tDepth_broadcastTF(double x, double y,double z, double qw,double qx,double qy,double qz)
    {
        //Conversion
        pose_tool0Tdep = getTool0toDepthTF(x,y,z,qw,qx,qy,qz);
        //Broadcaster
        broadcastTF(pose_tool0Tdep,"tool0","camera_link");
    }

    Eigen::Affine3d transformPoseToBase(Eigen::Affine3d& pose_depTgrasp_)
    {
        //Get tf from BASE to TOOL0
        tf::TransformListener listener;
        tf::StampedTransform transform_stamped;
        ros::Time now(ros::Time::now());
        listener.waitForTransform("base","tool0",now,ros::Duration(1.5));
        listener.lookupTransform("base","tool0",ros::Time(0),transform_stamped);
        Eigen::Affine3d pose_baseTtool0;
        tf::poseTFToEigen(transform_stamped,pose_baseTtool0);

        //Get tf from BASE to OBJECT
        Eigen::Affine3d pose_baseTgrasp;
        pose_baseTgrasp = pose_baseTtool0 * pose_tool0Tdep * pose_depTgrasp_;
        return pose_baseTgrasp;
    }

    void moveToPrePickPose(Eigen::Affine3d& grasp_pose,double z_offset)
    {
        geometry_msgs::Pose goalPose;

        //Orientation
        tf::TransformListener listener;
        tf::StampedTransform transform_stamped;
        tf::Transform transform_tmp;
        ros::Time now(ros::Time::now());
        listener.waitForTransform("base","claw_Memory_chip",now,ros::Duration(1.5));
        listener.lookupTransform("base","claw_Memory_chip",ros::Time(0),transform_stamped);
        transform_tmp= transform_stamped;
        tf::poseTFToMsg(transform_tmp,goalPose);

        //Position
        goalPose.position.x=grasp_pose.translation()[0];
        goalPose.position.y=grasp_pose.translation()[1];
        goalPose.position.z=grasp_pose.translation()[2]+z_offset;

        //grasp position is precision position instead of prePickPose
//        tf::Transform grasp_pose_tf;
//        tf::poseEigenToTF(grasp_pose,grasp_pose_tf);
//        tf::poseTFToMsg(grasp_pose_tf,goalPose);


        //Move
        ros::AsyncSpinner spinner(1);
        spinner.start();

        group->setMaxVelocityScalingFactor(0.3);
        group->setMaxAccelerationScalingFactor(0.3);
        group->setPoseTarget(goalPose);
        moveit::planning_interface::MoveGroup::Plan planner;
        bool is_success=group->plan(planner);
        if(is_success)
        {
            group->move();
            sleep(1);
        }else{
            cout<<"Move to pre-pick pose: Planning fail!"<<endl;
        }
        spinner.stop();
    }

    //linear interpolation for approaching the taerger. Base frame: /base.
    bool linear_trajectory_planning_come(Eigen::Affine3d grasp_pose, double line_offset, double num_of_interpolation,
                                    moveit_msgs::RobotTrajectory& target_traj ,double distance_offset)
    {
        tf::Transform grasp_pose_tf;
        tf::poseEigenToTF(grasp_pose,grasp_pose_tf);

        //Y axis
        tf::Matrix3x3 rot_mat=grasp_pose_tf.getBasis();
        tf::Vector3 direc = rot_mat.getColumn(1);
        double step = line_offset/num_of_interpolation;

        //Interpolate n points
        std::vector<geometry_msgs::Pose> waypoints(num_of_interpolation+1);
        geometry_msgs::Pose grasp_pose_msg;
        tf::poseTFToMsg(grasp_pose_tf,grasp_pose_msg);

        //---------do it myself  for grasp position Y offset
//        grasp_pose_msg.position.z = grasp_pose_msg.position.z + 0.04;
        grasp_pose_msg.position.x = grasp_pose_msg.position.x + distance_offset * direc[0] ;
        grasp_pose_msg.position.y = grasp_pose_msg.position.y + distance_offset * direc[1] ;
        grasp_pose_msg.position.z = grasp_pose_msg.position.z + distance_offset * direc[2] ;
        //---------

        for(int i=0;i<waypoints.size();++i)
        {
            geometry_msgs::Pose tmp_pose;
            tmp_pose.position.x = grasp_pose_msg.position.x + step * direc[0] * (num_of_interpolation-i);
            tmp_pose.position.y = grasp_pose_msg.position.y + step * direc[1] * (num_of_interpolation-i);
            tmp_pose.position.z = grasp_pose_msg.position.z + step * direc[2] * (num_of_interpolation-i);
            tmp_pose.orientation = grasp_pose_msg.orientation;
            waypoints[i]=tmp_pose;
        }


//        waypoints[num_of_interpolation]=grasp_pose_msg;

        //ComputeCartesian...
        double score=group->computeCartesianPath(waypoints,0.05,0.0,target_traj);


        if(score > 0.95)
        {
            moveit::core::RobotStatePtr kinematic_state(group->getCurrentState());
            robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(),"manipulator");
            rt.setRobotTrajectoryMsg(*kinematic_state,target_traj);
            trajectory_processing::IterativeParabolicTimeParameterization iptp;

            if(iptp.computeTimeStamps(rt,0.3,0.3))
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

    bool linear_trajectory_planning_back(Eigen::Affine3d grasp_pose, double line_offset, double num_of_interpolation,
                                    moveit_msgs::RobotTrajectory& target_traj ,double distance_offset)
    {
        tf::Transform grasp_pose_tf;
        tf::poseEigenToTF(grasp_pose,grasp_pose_tf);

        //Y axis
        tf::Matrix3x3 rot_mat=grasp_pose_tf.getBasis();
        tf::Vector3 direc = rot_mat.getColumn(1);
        double step = line_offset/num_of_interpolation;

        //Interpolate n points
        std::vector<geometry_msgs::Pose> waypoints(num_of_interpolation+1);
        geometry_msgs::Pose grasp_pose_msg;
        tf::poseTFToMsg(grasp_pose_tf,grasp_pose_msg);

        //---------do it myself  for grasp position Y offset
//        grasp_pose_msg.position.z = grasp_pose_msg.position.z + 0.04;
        grasp_pose_msg.position.x = grasp_pose_msg.position.x + distance_offset * direc[0] ;
        grasp_pose_msg.position.y = grasp_pose_msg.position.y + distance_offset * direc[1] ;
        grasp_pose_msg.position.z = grasp_pose_msg.position.z + distance_offset * direc[2] ;
        //---------

        for(int i=0;i<waypoints.size();++i)
        {
            geometry_msgs::Pose tmp_pose;
            tmp_pose.position.x = grasp_pose_msg.position.x + step * direc[0] * i;
            tmp_pose.position.y = grasp_pose_msg.position.y + step * direc[1] * i;
            tmp_pose.position.z = grasp_pose_msg.position.z + step * direc[2] * i;
            tmp_pose.orientation = grasp_pose_msg.orientation;
            waypoints[i]=tmp_pose;
        }


//        waypoints[num_of_interpolation]=grasp_pose_msg;

        //ComputeCartesian...
        double score=group->computeCartesianPath(waypoints,0.05,0.0,target_traj);


        if(score > 0.95)
        {
            moveit::core::RobotStatePtr kinematic_state(group->getCurrentState());
            robot_trajectory::RobotTrajectory rt(kinematic_state->getRobotModel(),"manipulator");
            rt.setRobotTrajectoryMsg(*kinematic_state,target_traj);
            trajectory_processing::IterativeParabolicTimeParameterization iptp;

            if(iptp.computeTimeStamps(rt,0.3,0.3))
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


    void moveToPickTargetPose(const moveit_msgs::RobotTrajectory& robot_traj)
    {
        //Moveit
        ros::AsyncSpinner spinner(1);
        spinner.start();

        group->setMaxVelocityScalingFactor(0.3);
        group->setMaxAccelerationScalingFactor(0.3);

        moveit::planning_interface::MoveGroup::Plan planner;
        planner.trajectory_=robot_traj;
        planner.trajectory_.joint_trajectory.points[0].time_from_start=ros::Duration(0);

        //Viz for test
        //        moveit_msgs::DisplayTrajectory display_traj;
        //        display_traj.trajectory_start=planner.start_state_;
        //        display_traj.trajectory.push_back(planner.trajectory_);
        //        traj_publisher.publish(display_traj);

        group->execute(planner);
//        sleep(1.5);
        spinner.stop();
    }

    void pickTarget()
    {
       std_msgs::String cmd;
       cmd.data="set_digital_out(0,True)";
       ur_script_pub.publish(cmd);
       sleep(4);    // origin is 1
    }

    void moveToPLaceTargetsPose()
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();

        moveit::planning_interface::MoveGroup::Plan planner;
        bool is_success;

        //group->setNamedTarget("place_target");
        vector<double> joint_angles(6);
        joint_angles[0]=5.46/180.0*M_PI;
        joint_angles[1]=-31.4/180.0*M_PI;
        joint_angles[2]=-152.15/180.0*M_PI;
        joint_angles[3]=-40.75/180.0*M_PI;
        joint_angles[4]=90/180.0*M_PI;
        joint_angles[5]=-90/180.0*M_PI;
        group->setJointValueTarget(joint_angles);
        is_success=group->plan(planner);
        if(is_success)
        {
            group->move();
            //sleep(0.5);
        }else{
            cout<<"Move to place pose: Planning fail!"<<endl;
        }
        sleep(0.2);
        spinner.stop();
    }

    void moveToPoseUsingJointAngle(double joint_angles0,double joint_angles1,double joint_angles2,double joint_angles3,double joint_angles4,double joint_angles5)
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();

        moveit::planning_interface::MoveGroup::Plan planner;
        bool is_success;

        //group->setNamedTarget("place_target");
        vector<double> joint_angles(6);
        joint_angles[0]=joint_angles0/180.0*M_PI;
        joint_angles[1]=joint_angles1/180.0*M_PI;
        joint_angles[2]=joint_angles2/180.0*M_PI;
        joint_angles[3]=joint_angles3/180.0*M_PI;
        joint_angles[4]=joint_angles4/180.0*M_PI;
        joint_angles[5]=joint_angles5/180.0*M_PI;
        group->setJointValueTarget(joint_angles);
        is_success=group->plan(planner);
        if(is_success)
        {
            group->move();
            //sleep(0.5);
        }else{
            cout<<"Move to place pose: Planning fail!"<<endl;
        }
        sleep(0.2);
        spinner.stop();
    }

    void placeTarget()
    {
       std_msgs::String cmd;
       cmd.data="set_digital_out(0,False)";
       ur_script_pub.publish(cmd);
       sleep(2);
    }




};

tf::StampedTransform get_Transform(std::string parent,std::string child)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    bool is_received = listener.waitForTransform(parent,child,ros::Time(0),ros::Duration(4.0));
    if(is_received){
        listener.lookupTransform(parent,child,ros::Time(0),transform);}
    else
        {
        ROS_ERROR("Can not receive transform from base to camera_link");
    }
    std::cout<<parent<<" to "<<child<<" transfotm: "<<std::endl;
    std::cout<<"Position: "<<"x: "<<transform.getOrigin().getX()<<" y: "<<transform.getOrigin().getY()<<" z: "<<transform.getOrigin().getZ()<<std::endl;
    std::cout<<"Orientation: "<<"x: "<<transform.getRotation().getX()<<" y: "<<transform.getRotation().getY()<<" z: "<<transform.getRotation().getZ()<<" w: "<<transform.getRotation().getW()<<std::endl;

    return transform;
}

int main(int argc,char** argv)
{
    //Ros service call
    ros::init(argc,argv,"ram_extraction_node");
    //if not ensenso ,you can use pcd to check it in gazebo.
//    ramRecg ram_rec("/home/csl/catkin_test1/src/ram_assembly/pcd/1494676385_pc.pcd");
    ramRecg ram_rec;

    //move to first position   -------zero step 0.0  it must be 0.1 first 0.1
    ram_rec.moveToLookForTargetsPoseUsingJointAngles(0.2);

    string cmd;
    while(ros::ok())
    {
        cout<<"Start a new detetcion? Input [y] to begin, or [n] to quit. "<<endl;
        cin>>cmd;
        if(cmd == "y")
        {
                //subscribe pc            ------zero step 0.1
                ensenso::CaptureSinglePointCloud srv;
                srv.request.req = true;
                ram_rec.getPC(srv);
                PointCloudXYZ::Ptr pc_tmp (new PointCloudXYZ);
                pcl::fromROSMsg(srv.response.pc,*pc_tmp);
                *ram_rec.pts_ = *pc_tmp;
                ram_rec.viz_pc(ram_rec.pts_);

                //Bradocast tf from tool0 to depth    ------zero step 0.2
                ram_rec.setTool0tDepth_broadcastTF(0.0667948, -0.0499201, 0.0446197 , 0.717103, 0.00203555, 0.00307061, 0.696957);

                //Remove plane ------first step 1
                pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
                ram_rec.plane_segment_proc(ram_rec.pts_,ram_rec.pts_no_plane,0.01,plane_indices,true);
                ram_rec.viz_pc(ram_rec.pts_no_plane);

                //Use region growing to segment pts   ------second step 2.0
                PointCloudXYZ::Ptr ram(new PointCloudXYZ);
                ram_rec.regionGrowingseg(ram_rec.pts_no_plane,ram);
                ram_rec.viz_pc(ram);
                //Use region growing to segment pts   ------second step 2.1
                PointCloudXYZ::Ptr ram_(new PointCloudXYZ);
                ram_rec.regionGrowingseg(ram ,ram_ ,7.0 ,0.02);
                ram_rec.viz_pc(ram_);


            //    //Remove plane and get coefficients
            //    pcl::PointIndices::Ptr plane_indices2(new pcl::PointIndices);
            //    PointCloudXYZ::Ptr ram_second(new PointCloudXYZ);
            //    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            //    ram_rec.plane_segment_proc(ram,ram_second,0.1,plane_indices2,false,coefficients);
            //    ram_rec.viz_pc(ram_second);

            //    //print model coefficients
            //    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
            //    <<coefficients->values[1] << " "
            //    <<coefficients->values[2] << " "
            //    <<coefficients->values[3] << " "<<std::endl;

                //Smooth the pointcloud using MLS   ------third step 3        // if you only see paln segment ,you can operate step 1,2,3,4.0 and ram_rec(path_pcd)
                PointCloudXYZ::Ptr ram_smooth(new PointCloudXYZ);
                ram_rec.mls_smooth(ram_,0.01,ram_smooth);

                //Use PCA analysis to get pose      -------four step 4.0
                Eigen::Affine3d pose_depTgrasp = ram_rec.poseEstimationByMomentOfInertia(ram_smooth,true);

                //Transform grasping pose to robot base frame  -------four step 4.1
                Eigen::Affine3d pose_baseTgrasp = ram_rec.transformPoseToBase(pose_depTgrasp);

                //Move to prepick pose       -------five step 5.0
                ram_rec.moveToPrePickPose(pose_baseTgrasp,0.1);

                //Trajectory from pre-pick to pick -------five step 5.0
                moveit_msgs::RobotTrajectory traj_come;
                bool is_succeed_come=ram_rec.linear_trajectory_planning_come(pose_baseTgrasp,0.07,10,traj_come,0.012); //origin is  0.05 ,diatance 0.012
                if(is_succeed_come)
                {
                   ram_rec.moveToPickTargetPose(traj_come);
                   ram_rec.pickTarget();
                   //Move back  first line stright
                   moveit_msgs::RobotTrajectory traj_back;
                   bool is_succeed_back=ram_rec.linear_trajectory_planning_back(pose_baseTgrasp,0.07,10,traj_back,0.012);
                   if(is_succeed_back)
                   {
                      ram_rec.moveToPickTargetPose(traj_back);
                   }
                   ram_rec.moveToLookForTargetsPoseUsingJointAngles(0.2);
//                   ram_rec.moveToPLaceTargetsPose();
                   ram_rec.moveToPoseUsingJointAngle(4.4, -26.8, -145.58, -53.32, 90, -90);
                   ram_rec.moveToPoseUsingJointAngle(-2.38, -87.41, -109.08, -72.59, 89.73, -27.65);
                   ram_rec.moveToPoseUsingJointAngle(-2.41, -90.20, -114.50, -64.38, 89.73, -27.69);
                   ram_rec.moveToPoseUsingJointAngle(-2.44, -93.62, -118.62, -56.85, 89.73, -27.72);
                   ram_rec.moveToPoseUsingJointAngle(-2.20, -94.35, -118.13, -57.48, 89.16, -27.47);
                   ram_rec.moveToPoseUsingJointAngle(-1.72, -94.94, -116.92, -59.76, 88.35, -26.95);
                   ram_rec.moveToPoseUsingJointAngle(-1.73, -95.22, -117.23, -59.18, 88.35, -26.96);
                   ram_rec.moveToPoseUsingJointAngle(-1.75, -95.68, -117.72, -58.23, 88.35, -26.98);
                   ram_rec.moveToPoseUsingJointAngle(-1.78, -96.44, -118.46, -56.72, 88.35, -27.01);
                   ram_rec.moveToPoseUsingJointAngle(-1.78, -96.44, -118.47, -56.55, 88.36, -27.01);
                   ram_rec.moveToPoseUsingJointAngle(-1.79, -96.47, -118.51, -56.47, 88.36, -27.01);
                   ram_rec.placeTarget();
                   ram_rec.moveToPoseUsingJointAngle(-1.75, -95.63, -117.67, -58.33, 88.35, -26.98);
                   ram_rec.moveToPoseUsingJointAngle(-1.73, -95.17, -117.18, -59.36, 88.35, -26.96);
                   ram_rec.moveToPoseUsingJointAngle(-1.30, -88.75, -104.55, -78.42, 88.35, -26.53);
                }

                else
                {
                    cout<<"Skip to next object"<<endl;
                }

                //Go back to initial state
                ram_rec.moveToLookForTargetsPoseUsingJointAngles(0.2);
        }
        else if(cmd == "n")
            break;
     }



    return 0;


}
