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
#include <pcl/filters/passthrough.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>

// Jacob Epstein
// 11/10/20
// This script breaks up a pointcloud into clusters, and visualizes the mesh of each cluster separately

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert from sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*cloud_msg, *cloud2);

    // Remove Nans from the cloud with a pcl::PassThrough filter
    // There might be some problem with this step
    pcl::PCLPointCloud2 filtered_cloud;
    pcl::PassThrough<pcl::PCLPointCloud2> filter (true);
    filter.setInputCloud(cloud2);
    bool organize = false;
    filter.setKeepOrganized(organize);
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

    //Estimating normals
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (downsampled_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod (tree_n);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);

    // Create k-d tree to prepare for extraction
    pcl::KdTree<pcl::PointXYZ>::Ptr tree_ec  (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
    tree_ec->setInputCloud (downsampled_cloud);

    // Do the euclidean cluster extraction, play around with these parameters
    std::vector<pcl::PointIndices> cluster_indices; // Clusters stored here
    const float tolerance = 0.50f; // 50cm tolerance in (x, y, z) coordinate system
    const double eps_angle = 5 * (M_PI / 180.0); // 5degree tolerance in normals
    const unsigned int min_cluster_size = 100;
    pcl::extractEuclideanClusters (*downsampled_cloud, *cloud_normals, tolerance, tree_ec, cluster_indices, eps_angle, min_cluster_size);

    // Initialize the viewer and hull object
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::PointCloud<pcl::PointXYZ> hullPoints;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    std::vector<pcl::Vertices> hullPolygons;
    chull.setAlpha(0.05);   // Play around with this guy too

    // Loop through cluster_indices, find the mesh of each cluster, and add the mesh to the viewer
    int counter = 1;
    ROS_INFO("number of clusters: %d", (int)cluster_indices.size());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &index : it->indices)
            cloud_cluster->push_back ((*downsampled_cloud)[index]);
        // Look up what all of this stuff does
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Generate a surface mesh for cloud_cluster
        hullPoints.clear();
        hullPolygons.clear();
        chull.setInputCloud(cloud_cluster);
        chull.reconstruct(hullPoints, hullPolygons);
        ROS_INFO("concave hull of cluster %d has %d points, %d triangles", counter, (int) hullPoints.points.size(), (int) hullPolygons.size());
        pcl::PolygonMesh hull_mesh;
        hull_mesh.polygons = hullPolygons;
        pcl::PCLPointCloud2 hull_points2;
        pcl::toPCLPointCloud2(hullPoints, hull_points2);
        hull_mesh.cloud = hull_points2;

        // Add the newly generated mesh to the visualizer
        // testing to see what happens with only one cluster
        if (counter == 1) {
            viewer->addPolygonMesh(hull_mesh);
        }

        // Update counter
        counter ++;
    }

    // Visualize the meshes!
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
    }
}
int main(int argc, char **argv) {
    // Initialize ROS
    ros::init (argc, argv, "mesh_publisher");
    ros::NodeHandle nh;
    ROS_INFO("reading in meshes from fetch gazebo world...");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/head_camera/depth_registered/points", 5, cloud_cb);

    // Spin
    ros::spin();
}