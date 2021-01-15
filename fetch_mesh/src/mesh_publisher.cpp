//
// Created by jacob on 10/10/20.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>
#include <geometry_msgs/Point.h>
#include <pcl/Vertices.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <chrono>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <boost/geometry.hpp>
#include <boost/geometry/arithmetic/cross_product.hpp>

namespace bg = boost::geometry;

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    // Convert from sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*cloud_msg, *cloud2);

    // Remove Nans from the cloud with a pcl::PassThrough filter
    // There might be some problem with this step
    pcl::PCLPointCloud2 filtered_cloud;
    pcl::PassThrough<pcl::PCLPointCloud2> filter (true);
    filter.setInputCloud(cloud2);
    //bool organize = false;
    //filter.setKeepOrganized(organize);
    filter.setFilterLimits(0, 20);
    filter.filter(filtered_cloud);

    // Convert cloud to pcl::PointCloud<pcl::PointXYZ> format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(filtered_cloud, *cloud);

    // Downsample the pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_ds;
    voxel_ds.setInputCloud(cloud);
    voxel_ds.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_ds.filter(*downsampled_cloud);

    // Estimate the normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3 (new pcl::PointCloud<pcl::Normal> ());

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (downsampled_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod (tree_n);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);

//    // Create plane segmentation object and set parameters
//    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//    seg.setNormalDistanceWeight (0.1);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (100);
//    seg.setDistanceThreshold (0.05);
//    seg.setInputCloud (downsampled_cloud);
//    seg.setInputNormals (cloud_normals);
//
//    // Obtain plane inliers and coefficients
//    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
//    seg.segment (*inliers_plane, *coefficients_plane);
//
//    // Extract plane inliers from the pointcloud
//    pcl::ExtractIndices<pcl::PointXYZ> extract;
//    extract.setInputCloud (downsampled_cloud);
//    extract.setIndices (inliers_plane);
//    extract.setNegative (true);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud (new pcl::PointCloud<pcl::PointXYZ>());
//    extract.filter(*extracted_cloud);
//    pcl::ExtractIndices<pcl::Normal> extract_normals;
//    extract_normals.setNegative (true);
//    extract_normals.setInputCloud (cloud_normals);
//    extract_normals.setIndices (inliers_plane);
//    extract_normals.filter (*cloud_normals2);
//
//    // Do statistical outlier removal to get rid of any remaining weird bits
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter; // Initializing with true will allow us to extract the removed indices
//    pcl::PointCloud<pcl::PointXYZ>::Ptr soRemoved (new pcl::PointCloud<pcl::PointXYZ>);
//    sorfilter.setInputCloud(extracted_cloud);
//    sorfilter.setMeanK(5);
//    sorfilter.setStddevMulThresh(1.0);
//    sorfilter.filter(*soRemoved);
//
//    // Recalculate normals
//    ne.setInputCloud(soRemoved);
//    ne.compute(*cloud_normals2);    // This is very efficient, change this in the final version of this script
//
//    // Do planar segmentation again
//    seg.setInputCloud (soRemoved);
//    seg.setInputNormals (cloud_normals2);
//    pcl::ModelCoefficients::Ptr coefficients_plane2 (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_plane2 (new pcl::PointIndices);
//    seg.segment (*inliers_plane2, *coefficients_plane2);
//
//    // Extract inliers again
//    extract.setInputCloud (soRemoved);
//    extract.setIndices (inliers_plane2);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
//    extract.filter(*extracted_cloud2);
//
//    // Set parameters for cylinder segmentation
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_CYLINDER);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setNormalDistanceWeight (0.1);
//    seg.setMaxIterations (10000);
//    seg.setDistanceThreshold (0.03);
//    seg.setRadiusLimits (0, 0.1);
//    seg.setInputCloud (extracted_cloud2);
//    ne.setInputCloud(extracted_cloud2);
//    ne.compute(*cloud_normals3);
//    seg.setInputNormals(cloud_normals3);
//
//    // Calculate and extract the cylinder inliers
//    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
//    seg.segment (*inliers_cylinder, *coefficients_cylinder);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinderless_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    extract.setInputCloud(extracted_cloud2);
//    extract.setIndices (inliers_cylinder);
//    extract.setNegative (true);
//    extract.filter(*cylinderless_cloud);

    // Find the concave hull
    pcl::PointCloud<pcl::PointXYZ> hullPoints;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    std::vector<pcl::Vertices> hullPolygons;
    chull.setAlpha(0.05);   // Play around with this guy too
    chull.setInputCloud(downsampled_cloud);
    chull.reconstruct(hullPoints, hullPolygons);
    ROS_INFO("Publishing mesh with %d points and %d triangles", (int)hullPoints.points.size(), (int)hullPolygons.size());

    //Convert PCL PolygonMesh to mesh_msgs/TriangleMesh
    shape_msgs::Mesh output;

    // Convert hullPoints.points into output.vertices
    // hullPoints.points: std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >
    // output.vertices: std::vector<geometry_msgs::Point_<std::allocator<void> >
    std::vector<geometry_msgs::Point > verts;
    for ( int i=0; i<hullPoints.points.size(); i++ ) {
        pcl::PointXYZ pcl_point = hullPoints.points[i];
        geometry_msgs::Point ros_point;
        ros_point.x = pcl_point.x;
        ros_point.y = pcl_point.y;
        ros_point.z = pcl_point.z;
        verts.push_back(ros_point);
    }
    output.vertices = verts;

    //Convert hullPolygons into output.triangles object
    //hullPolygons: std::vector<pcl:Vertices>
    //output.triangles std::vector<shape_msgs::MeshTriangle >
    std::vector<shape_msgs::MeshTriangle > polygons;
    for ( int j=0; j<hullPolygons.size(); j++ ) {
        pcl::Vertices triangle = hullPolygons[j]; //Indices are stored in triangle.vertices
        shape_msgs::MeshTriangle ros_triangle;  //Indices are stored in vertex_indices, a uint32[3] array
        ros_triangle.vertex_indices[0] = triangle.vertices[0];
        ros_triangle.vertex_indices[1] = triangle.vertices[1];
        ros_triangle.vertex_indices[2] = triangle.vertices[2];
        polygons.push_back(ros_triangle);
    }

    //Publish the mesh!
    output.triangles = polygons;
    ROS_INFO("Publishing the mesh!");
    pub.publish(output);

//    // Visualize the mesh (this is optional)
//    pcl::PolygonMesh hull_mesh;
//    hull_mesh.polygons = hullPolygons;
//    pcl::PCLPointCloud2 hull_points2;
//    pcl::toPCLPointCloud2(hullPoints, hull_points2);
//    hull_mesh.cloud = hull_points2;
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->addPolygonMesh(hull_mesh);
//    while (!viewer->wasStopped ()) {
//        viewer->spinOnce (100);
//    }
//    pub.publish(output);
}

int
main (int argc, char **argv) {
    // Initialize ROS
    ros::init (argc, argv, "mesh_publisher");
    ros::NodeHandle nh;
    ROS_INFO("starting up the mesh_publisher...");
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/head_camera/depth_registered/points", 5, cloud_cb);
    // Create a ROS publisher for the output mesh
    pub = nh.advertise<shape_msgs::Mesh> ("fetch_mesh", 5);

    // Spin
    ros::spin();
}