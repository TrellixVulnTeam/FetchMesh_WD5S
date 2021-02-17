//
// Created by Jacob Epstein on 2/15/21.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
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
#include <vector>
#include <cmath>
#include <time.h>
#include <boost/geometry.hpp>
#include <pcl/features/boundary.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>

// Variables
bool add_pointcloud = false;
bool appending_to_pcd = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
tf::TransformListener *tf_listener = NULL;
ros::Publisher pub;

// Functions
void start_cb(std_msgs::String start);
void reconstruct_cb(std_msgs::String reconstruct);
void export_mesh(pcl::PolygonMesh mesh);
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

// Implementations
void start_cb(std_msgs::String start) {
    add_pointcloud = true;
}

// preforms MLS and greedy triangulation on cloud, exports mesh to unity for visualization
void reconstruct_cb(std_msgs::String reconstruct) {
    ROS_INFO("Reconstructing the mesh...");

    // do Mean Least Squares (MLS) smoothing on cloud
    ROS_INFO("Started MLS smoothing on pointcloud of %d points ", (int)cloud->points.size());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);
    mls.setInputCloud (cloud);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);
    mls.process (*mls_points);

    // do greedy triangulation on mls_points to get a mesh output
    ROS_INFO("Started Greedy triangulation on pointcloud of %d points ", (int)cloud->points.size());
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (mls_points);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius (0.025);
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setInputCloud (mls_points);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // export the mesh to unity
    ROS_INFO("Exporting the mesh to unity");
    export_mesh(triangles);
}

void export_mesh(pcl::PolygonMesh mesh) {
    //Convert PCL PolygonMesh to shape_msgs/Mesh
    shape_msgs::Mesh output;

    // convert the mesh cloud to a pcl::PointXYZ type
    pcl::PointCloud<pcl::PointXYZ> pcd;
    pcl::fromPCLPointCloud2(mesh.cloud, pcd);

    // Convert the vertices
    std::vector<geometry_msgs::Point > verts;
    for (int i=0; i<pcd.points.size(); i++) {
        pcl::PointXYZ pcl_point = pcd.points[i];
        geometry_msgs::Point ros_point;
        ros_point.x = pcl_point.x;
        ros_point.y = pcl_point.y;
        ros_point.z = pcl_point.z;
        verts.push_back(ros_point);
    }

    // Convert the polygons
    std::vector<shape_msgs::MeshTriangle > polygons;
    for ( int j=0; j<mesh.polygons.size(); j++ ) {
        pcl::Vertices triangle = mesh.polygons[j]; //Indices are stored in triangle.vertices
        shape_msgs::MeshTriangle ros_triangle;  //Indices are stored in vertex_indices, a uint32[3] array
        ros_triangle.vertex_indices[0] = triangle.vertices[0];
        ros_triangle.vertex_indices[1] = triangle.vertices[1];
        ros_triangle.vertex_indices[2] = triangle.vertices[2];
        polygons.push_back(ros_triangle);
    }

    // Publish the shape_msgs::Mesh
    output.vertices = verts;
    output.triangles = polygons;
    ROS_INFO("Publishing the mesh...");
    pub.publish(output);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    appending_to_pcd = true;
    if (not add_pointcloud) {
        return;
    }
    add_pointcloud = false;

    // Convert from sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*cloud_msg, *cloud2);

    // Remove Nans from the cloud with a pcl::PassThrough filter
    pcl::PCLPointCloud2 filtered_cloud;
    pcl::PassThrough<pcl::PCLPointCloud2> filter (true);
    filter.setInputCloud(cloud2);
    filter.setFilterLimits(0, 20);
    filter.filter(filtered_cloud);

    // convert to pointXYZ pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2(filtered_cloud, *pcd);

    // move from the /head_camera_rgb_optical_frame to /base_link or /world
    // pcl_ros::transformPointCloud (const std::string &target_frame, const pcl::PointCloud< PointT > &cloud_in, pcl::PointCloud
    // < PointT > &cloud_out, const tf::TransformListener &tf_listener
    // might have to specify time and the fixed frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl_ros::transformPointCloud("/map", *pcd, *cloud_out, *tf_listener);

    // Downsample the pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_ds;
    voxel_ds.setInputCloud(cloud_out);
    voxel_ds.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_ds.filter(*downsampled_cloud);

    // add pointcloud to global variable cloud
    ROS_INFO("Adding cloud of %d points to the main cloud.", (int)(downsampled_cloud->points.size()));
    *cloud += *downsampled_cloud;
    appending_to_pcd = false;
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init (argc, argv, "mesh_generator");
    tf_listener = new (tf::TransformListener);
    ros::NodeHandle nh;
    ROS_INFO("starting up greedy triangulation algorithm...");

    // Create a subscriber to tell script when to start segmentation
    ros::Subscriber start_sub = nh.subscribe("/start_segmentation", 5, start_cb);

    // Create a subscriber to know when to do surface reconstruction
    ros::Subscriber finish_sub = nh.subscribe("/start_reconstruction", 5, reconstruct_cb);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/head_camera/depth_registered/points", 5, cloud_cb);

    // Create a ROS publisher for the output mesh
    pub = nh.advertise<shape_msgs::Mesh> ("big_mesh", 5);

    // Spin
    ros::spin();
}