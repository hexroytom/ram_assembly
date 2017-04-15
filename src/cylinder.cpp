#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <opencv2/opencv.hpp>

using namespace pcl;
using namespace std;
using namespace cv;

int main()
{
    //Params
    pcl::PCDReader reader;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>), cloud_normals1(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>), cloud_normals3(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients), coefficients_cc(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices), inliers_cc(new pcl::PointIndices);
    PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud<pcl::PointXYZ>);

    //Load pointcloud
    double total_time;
    double t;
    t=cv::getTickCount ();
    reader.read("/home/yake/Temp/simple_scene_only_cylinder.pcd", *cloud);
    t=(cv::getTickCount ()-t)/cv::getTickFrequency ();
    cout<<"Load pointcloud: "<<t<<endl;

    //Pass through
    total_time=cv::getTickCount();
    t=cv::getTickCount ();
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 0.9);
    pass.filter(*cloud_filtered);
    t=(cv::getTickCount ()-t)/cv::getTickFrequency ();
    cout<<"Pass through: "<<t<<endl;

    //Normal estimation
    t=cv::getTickCount ();
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
    t=(cv::getTickCount ()-t)/cv::getTickFrequency ();
    cout<<"Normal estimation: "<<t<<endl;

    //Cylinder segmentation
    t=cv::getTickCount ();
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.1);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    t=(cv::getTickCount ()-t)/cv::getTickFrequency ();
    cout<<"Cylinder segmentation: "<<t<<endl;

    total_time=(cv::getTickCount ()-total_time)/cv::getTickFrequency ();
    cout<<"Total time: "<<total_time<<endl;

    //Extract point cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>()), cloud_cylinder_c(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder_removed(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*cloud_cylinder);

    //Visualization
    pcl::visualization::PCLVisualizer view("cylinder");
    view.addPointCloud<pcl::PointXYZ>(cloud_cylinder);
    view.spin();


}
