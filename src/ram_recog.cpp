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

#include "ram_assembly/moment_of_inertia_estimation.h"

//std
#include <string>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;

class ramRecg
{

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

    void poseEstimationByMomentOfInertia(PointCloudXYZ::Ptr input_pts,Eigen::Matrix4f& Pose,bool is_viz)
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
        Eigen::Vector4f v1(minor_vector[0],minor_vector[1],minor_vector[2],0.0);
        Eigen::Vector4f v2(0.0,0.0,1.0,0.0);
        double theta= pcl::getAngle3D(v1,v2);
        theta=theta/M_PI*180.0;
        if(theta<90.0){
            minor_vector[0]=-minor_vector[0];
            minor_vector[1]=-minor_vector[1];
            minor_vector[2]=-minor_vector[2];
        }

        //Fill the pose matrix
        Pose << major_vector[0],middle_vector[0],minor_vector[0],mass_center[0],
                major_vector[1],middle_vector[1],minor_vector[1],mass_center[1],
                major_vector[2],middle_vector[2],minor_vector[2],mass_center[2],
                0,0,0,1;
        std::cout<<Pose<<endl;

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
            viewer->addPointCloud(pts_);
            viewer->spin();

        }

    }

};

int main(int argc,char** argv)
{
    ramRecg ram_rec("/home/yake/catkin_ws/src/ram_assembly/pcd/1477109696_pc.pcd");
    pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
    ram_rec.plane_segment_proc(ram_rec.pts_,ram_rec.pts_no_plane,0.01,plane_indices,true);

    //Use region growing to segment pts
    PointCloudXYZ::Ptr ram(new PointCloudXYZ);
    ram_rec.regionGrowingseg(ram_rec.pts_no_plane,ram);

    //Use PCA analysis to get pose
    Eigen::Matrix4f pose;
    ram_rec.poseEstimationByMomentOfInertia(ram,pose,false);



}
