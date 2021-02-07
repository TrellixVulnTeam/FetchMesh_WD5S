//
// Created by Jacob Epstein on 11/20/20.
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <chrono>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <boost/geometry.hpp>
#include <boost/geometry/arithmetic/cross_product.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/features/boundary.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

//TODO:


namespace bg = boost::geometry;
int jcount = 0;
bool publish_rectangle = true, start_segmentation = false;
std_msgs::Float32MultiArray toRosArray(std::vector<float> vertex);
float distance (std::vector<float> v1, std::vector<float> v2);
bool publish_fitted_rectangle(pcl::PointCloud<pcl::PointXYZ> &cloud_plane,
                              pcl::ModelCoefficients plane_coeffs);
ros::Publisher pub, pub2, pub3, pub4, pub5, pub6;
bool planar_seg(pcl::PointCloud<pcl::PointXYZ> &cloud);
void project_onto_plane(pcl::ModelCoefficients plane, pcl::ModelCoefficients line,
        pcl::ModelCoefficients &projected_line);
void create_rectangle(pcl::ModelCoefficients l1, pcl::ModelCoefficients l2, pcl::ModelCoefficients l3,
                      pcl::ModelCoefficients l4);
std::vector<float> unit_vector(std::vector<float> vec);
float magnitude(std::vector<float> vec);
std::vector<float> intersect(pcl::ModelCoefficients l1, pcl::ModelCoefficients l2);
geometry_msgs::Point toROSPoint(std::vector<float> input);
bool close_to_cloud(std::vector<float> point, pcl::PointCloud<pcl::PointXYZ> &cloud);
void export_mesh(std::vector<std::vector<float>> intersection_points);
void start_cb(std_msgs::String start);
void odom_cb(nav_msgs::Odometry odom);
geometry_msgs::Pose2D current_pose;
float transform_x, transform_y;
tf::TransformListener *tf_listener = NULL;

void odom_cb(nav_msgs::Odometry odom) {
    current_pose.x = odom.pose.pose.position.x;
    current_pose.y = odom.pose.pose.position.y;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if (not start_segmentation) {
        return;
    }

    start_segmentation = false;

    // Set the transform variables
    transform_x = current_pose.x;
    transform_y = current_pose.y;

    ROS_INFO("Starting segmentation...");
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2(filtered_cloud, *cloud);

    // move from the /head_camera_rgb_optical_frame to /base_link or /world
    // pcl_ros::transformPointCloud (const std::string &target_frame, const pcl::PointCloud< PointT > &cloud_in, pcl::PointCloud
    // < PointT > &cloud_out, const tf::TransformListener &tf_listener
    // might have to specify time and the fixed frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl_ros::transformPointCloud("/base_link", *cloud, *cloud_out, *tf_listener);

    // Downsample the pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_ds;
    voxel_ds.setInputCloud(cloud_out);
    voxel_ds.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_ds.filter(*downsampled_cloud);

//    // Do SOR on the pointcloud
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter; // Initializing with true will allow us to extract the removed indices
//    pcl::PointCloud<pcl::PointXYZ>::Ptr soRemoved (new pcl::PointCloud<pcl::PointXYZ>);
//    sorfilter.setInputCloud(cloud);
//    sorfilter.setMeanK(5);
//    sorfilter.setStddevMulThresh(1.0);
//    sorfilter.filter(*soRemoved);

    int counter = 0;
    if ((int)downsampled_cloud->size() < 1000) {
        ROS_INFO("pointcloud too small, aborting");
        return;
    }

    ROS_INFO("Read in cloud with %d points", (int)downsampled_cloud->size());

    while (planar_seg(*downsampled_cloud)) {
        ROS_INFO("Finished segmenting rectangle %d.", counter);
        counter ++;
    }

    //sleep(2); // Go two seconds between each pass of the algorithm
    return;
}

bool planar_seg (pcl::PointCloud<pcl::PointXYZ> &cloud) {
    // Determine whether to extract the rectangle or to return false
    if ((int)cloud.size() < 100) {
        ROS_INFO("pointcloud too small, aborting.");
        return false;
    }

    // Estimate the normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr (new pcl::PointCloud<pcl::PointXYZ>);
    *cloudptr = cloud;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloudptr);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod (tree_n);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);


    // Intiate and set basic planar segmentation object with basic parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud(cloudptr);
    seg.setInputNormals(cloud_normals);

    // Obtain plane inliers and coefficients
    pcl::ModelCoefficients coefficients_plane;
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    seg.segment (*inliers_plane, coefficients_plane);

    // Extract plane inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloudptr);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter (*cloud_plane);

    // Find and publish the rectangle of best fit as a 2-triangle mesh
    ROS_INFO("Constructing rectangle from subcloud with %d points", (int)cloud_plane->size());
    if (not publish_fitted_rectangle(*cloud_plane, coefficients_plane)) {
        return false;
    }

    // Update cloud
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr updated_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter(*updated_cloud);
    cloud = *updated_cloud;
    ROS_INFO("Main cloud now has %d points", (int) cloud.size());

    return true;
}

bool publish_fitted_rectangle(pcl::PointCloud<pcl::PointXYZ> &cloud_plane,
                              pcl::ModelCoefficients plane_coeffs) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr (new pcl::PointCloud<pcl::PointXYZ>);
    *cloudptr = cloud_plane;

    // NEW: extract boundary points using boundary estimation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloudptr);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod (tree_n);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud (cloudptr);
    est.setInputNormals (cloud_normals);
    est.setRadiusSearch (0.03);
    est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.compute(boundaries);

    // Send boundary points to panda3d visualizer and fill up the boundary_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    std_msgs::Float32MultiArray boundary_arr;
    for (int i=0; i<cloudptr->size(); i++) {
        if (boundaries.points[i].boundary_point == 1) {
            boundary_cloud->push_back(cloud_plane.points[i]);
            boundary_arr.data.push_back(i);
        }
    }

    // Send pointcloud and boundary data to panda3d visualizer
    if (true) {
        std_msgs::Float32MultiArray pcd_arr;
        for (int i=0; i<(int)cloud_plane.size(); i++) {
            pcd_arr.data.push_back((float)(cloud_plane.points[i].x));
            pcd_arr.data.push_back((float)(cloud_plane.points[i].y));
            pcd_arr.data.push_back((float)(cloud_plane.points[i].z));
        }
        pub3.publish(pcd_arr);
        //pub4.publish(boundary_indices);
        pub4.publish(boundary_arr);
        publish_rectangle = false;
    }

    ROS_INFO("Boundary cloud has %d points", (int)boundary_cloud->size());
    if (boundary_cloud->size() < 150) {
        ROS_INFO("Aborting linear segmentation, boundary too small");
        return false;
    }

    // Initializing variables for linear segmentation
    std::vector<pcl::ModelCoefficients> line_equations;
    pcl::ModelCoefficients line;
    pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (boundary_cloud);
    extract.setIndices (line_inliers);
    extract.setNegative (true);
    pcl::PointCloud<pcl::PointXYZ> updated_cloud;

    int counter = 1;
    while (boundary_cloud->size() > 25) {
        seg.setInputCloud (boundary_cloud);
        seg.segment(*line_inliers, line);
        line_equations.push_back(line);

        // Update the boundary_cloud by extracting indices
        extract.setInputCloud (boundary_cloud);
        extract.setIndices (line_inliers);
        extract.filter(updated_cloud);
        ROS_INFO("updated cloud has %d points", updated_cloud.size());
        *boundary_cloud = updated_cloud;
        ROS_INFO("Segmented line %d, boundary now has %d points", counter, boundary_cloud->size());
        // Reset updated_cloud, line, line_inliers, MIGHT NEED TO RESET LINE_INLIERS
        updated_cloud.clear();
        line.values.clear();
        line_inliers->indices.clear();
        counter ++;
        if (boundary_cloud->size() < 25) {
            ROS_INFO("Quitting the loop after segmenting %d lines, boundary cloud has length %d ", counter-1, (int)boundary_cloud->size());
        }
    }

    if (line_equations.size() <= 2) {
        ROS_INFO("pointcloud boundary is degenerate, too few edges, aborting...");
        return false;
    }

    // Project each line onto the plane
    std::vector<pcl::ModelCoefficients> projected_line_equations;
    pcl::ModelCoefficients projected_l;
    for (int i=0; i<line_equations.size(); i++) {
        project_onto_plane(plane_coeffs, line_equations[i], projected_l);
        pub5.publish(toRosArray(projected_l.values));
        projected_line_equations.push_back(projected_l);
        projected_l.values.clear();
    }

    pub6.publish(toRosArray(plane_coeffs.values));

    // Calculate intersection points between all lines, discard if they are far away from the pointcloud
    std::vector<float> intersection_point;
    std::vector<std::vector<float>> intersection_points;
    for (int i=0; i<projected_line_equations.size() - 1; i++) {
        for (int j=i+1; j<projected_line_equations.size(); j++) {
            intersection_point = intersect(projected_line_equations[i], projected_line_equations[j]);
            if (close_to_cloud(intersection_point, *cloudptr)) {
                intersection_points.push_back(intersection_point);
            }
        }
    }

    if (intersection_points.size() < 3) {
        ROS_INFO("pointcloud is degenerate, too few intersection points, aborting...");
        return false;
    }

    // Export the mesh
    ROS_INFO("Exporting the mesh...");
    export_mesh(intersection_points);

    return true;
}

bool close_to_cloud(std::vector<float> point, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr (new pcl::PointCloud<pcl::PointXYZ>);
    *cloudptr = cloud;

    // Initializing variables
    pcl::PointXYZ searchPoint;
    searchPoint.x = point[0];
    searchPoint.y = point[1];
    searchPoint.z = point[2];

    // Return false if there are nans or infs
    if (not std::isfinite(searchPoint.x) or not std::isfinite(searchPoint.y) or not std::isfinite(searchPoint.z)) {
        return false;
    }

    float radius = 0.02;
    float threshold = 0;
    std::vector<int> found_indices;
    std::vector<float> k_sqr_distances;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr plane_tree (new pcl::search::KdTree<pcl::PointXYZ>());
    plane_tree->setInputCloud(cloudptr);

    // Search in radius around the query point
    plane_tree->radiusSearch(searchPoint, radius, found_indices, k_sqr_distances);

    // Determine whether the point is close or not
    if (found_indices.size() > 1) {
        return true;
    }

    return false;
}

void export_mesh(std::vector<std::vector<float>> intersection_points) {
    shape_msgs::Mesh output;
    std::vector<geometry_msgs::Point > vertices;
    std::vector<shape_msgs::MeshTriangle > polygons;

    // Convert all the vertices to geometry_msgs::Point
    for (int i=0; i<intersection_points.size(); i++) {
        vertices.push_back(toROSPoint(intersection_points[i]));

        // Correct for transform
        vertices[i].x += transform_x;
        vertices[i].y += transform_y;
    }

    // Construct each of the (vertices choose 3) triangles, put into polygons
    shape_msgs::MeshTriangle triangle;
    for (int i=0; i<intersection_points.size()-2; i++) {
        for (int j=i+1; j<intersection_points.size()-1; j++) {
            for (int k=j+1; k<intersection_points.size(); k++) {
                triangle.vertex_indices[0] = i;
                triangle.vertex_indices[1] = j;
                triangle.vertex_indices[2] = k;
                polygons.push_back(triangle);
            }
        }
    }
    if (polygons.size() >= 30) {
        ROS_INFO("Too many triangles. Aborting");
        return;
    }
    output.triangles = polygons;
    output.vertices = vertices;
    pub.publish(output);
}

void project_onto_plane(pcl::ModelCoefficients plane, pcl::ModelCoefficients line,
                        pcl::ModelCoefficients &projected_line) {
    // Calculate unit normal to the plane
    std::vector<float> normal = {plane.values[0], plane.values[1], plane.values[2]};
    auto scalar = (float) sqrt(pow(normal[0],2)+pow(normal[1],2)+pow(normal[2],2));
    std::vector<float> unit_normal = {normal[0]/scalar, normal[1]/scalar, normal[2]/scalar};

    // Choose arbitrary origin on the plane, choose p1 and p2 on the line
    std::vector<float> origin {0, 0, -plane.values[4]/plane.values[3]};
    std::vector<float> p1 {line.values[0], line.values[1], line.values[2]};
    std::vector<float> p2 {line.values[0] + line.values[3],
                           line.values[1] + line.values[4],
                           line.values[2] + line.values[5]};

    // project p1 onto the plane
    std::vector<float> orig_to_p1 = {p1[0]-origin[0], p1[1]-origin[1], p1[2]-origin[2]};
    float dist = orig_to_p1[0] * unit_normal[0] + orig_to_p1[1] * unit_normal[1] +
                 orig_to_p1[2] * unit_normal[2];
    std::vector<float> projected_p1 = {p1[0] - dist*unit_normal[0], p1[1] - dist*unit_normal[1],
                                             p1[2] - dist*unit_normal[2]};

    // project p2 onto the plane
    std::vector<float> orig_to_p2 = {p2[0]-origin[0], p2[1] - origin[1], p2[2] - origin[2]};
    dist = orig_to_p2[0] * unit_normal[0] + orig_to_p2[1] * unit_normal[1] +
                 orig_to_p2[2] * unit_normal[2];
    std::vector<float> projected_p2 = {p2[0] - dist*unit_normal[0], p2[1] - dist*unit_normal[1],
                                       p2[2] - dist*unit_normal[2]};

    // find and store line between p1 and p2
    std::vector<float> line_vec = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    for (int i=0; i<3; i++)
        projected_line.values.push_back(p1[i]);
    for (int i=0; i<3; i++)
        projected_line.values.push_back(line_vec[i]);
}

float distance (std::vector<float> v1, std::vector<float> v2) {
    return (float)sqrt(pow((v1[0] - v2[0]), 2) + pow((v1[1] - v2[1]), 2) + pow((v1[2] - v2[2]), 2));
}

std_msgs::Float32MultiArray toRosArray(std::vector<float> vertex) {
    std_msgs::Float32MultiArray ros_arr;
    for (int i = 0; i<(int)vertex.size(); i++) {
        ros_arr.data.push_back(vertex[i]);
    }
    return ros_arr;
}

std::vector<float> unit_vector(std::vector<float> vec) {
    float magnitude = (float) sqrt(pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2));
    std::vector<float> unit_vec = {vec[0]/magnitude, vec[1]/magnitude, vec[2]/magnitude};
    return unit_vec;
}

float magnitude(std::vector<float> vec) {
    return (float) sqrt(pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2));
}

// Assume that these two lines are coplanar
// Rewrite this to find midpoint of smallest line segment between two lines
// Used method detailed at http://paulbourke.net/geometry/pointlineplane/
// Pa and Pb are endpoints of shortest linesegment between the two lines P1P2, P3P4
// Pa = P1 + mua (P2 - P1)
// Pb = P3 + mub (P4 - P3)
//dmnop = (xm - xn)(xo - xp) + (ym - yn)(yo - yp) + (zm - zn)(zo - zp)
//mua = ( d1343 d4321 - d1321 d4343 ) / ( d2121 d4343 - d4321 d4321 )
//  mub = ( d1343 + mua d4321 ) / d4343
// Note that dmnop = dopmn
std::vector<float> intersect(pcl::ModelCoefficients l1, pcl::ModelCoefficients l2) {
    std::vector<float> P1 = {l1.values[0], l1.values[1], l1.values[2]};
    std::vector<float> P3 = {l2.values[0], l2.values[1], l2.values[2]};
    std::vector<float> slope1 = {l1.values[3], l1.values[4], l1.values[5]};
    std::vector<float> slope2 = {l2.values[3], l2.values[4], l2.values[5]};
    std::vector<float> P2 = {l1.values[0] + slope1[0], l1.values[1] + slope1[1], l1.values[2] + slope1[2]};
    std::vector<float> P4 = {l2.values[0] + slope2[0], l2.values[1] + slope2[1], l2.values[2] + slope2[2]};

    float d1343 = (P1[0] - P3[0]) * (P4[0] - P3[0]) + (P1[1] - P3[1]) * (P4[1] - P3[1]) + (P1[2] - P3[2]) * (P4[2] - P3[2]);
    float d4321 = (P4[0] - P3[0]) * (P2[0] - P1[0]) + (P4[1] - P3[1]) * (P2[1] - P1[1]) + (P4[2] - P3[2]) * (P2[2] - P1[2]);
    float d1321 = (P1[0] - P3[0]) * (P2[0] - P1[0]) + (P1[1] - P3[1]) * (P2[1] - P1[1]) + (P1[2] - P3[2]) * (P2[2] - P1[2]);
    float d4343 = (P4[0] - P3[0]) * (P4[0] - P3[0]) + (P4[1] - P3[1]) * (P4[1] - P3[1]) + (P4[2] - P3[2]) * (P4[2] - P3[2]);
    float d2121 = (P2[0] - P1[0]) * (P2[0] - P1[0]) + (P2[1] - P1[1]) * (P2[1] - P1[1]) + (P2[2] - P1[2]) * (P2[2] - P1[2]);

    float mua = ( d1343 * d4321 - d1321 * d4343 ) / ( d2121 * d4343 - d4321 * d4321 );
    float mub = ( d1343 + mua * d4321 ) / d4343;

    std::vector<float> Pa = {P1[0] + mua * slope1[0], P1[1] + mua * slope1[1], P1[2] + mua * slope1[2]};
    std::vector<float> Pb = {P3[0] + mub * slope2[0], P3[1] + mub * slope2[1], P3[2] + mub * slope2[2]};

    std::vector<float> midpoint = {(Pa[0]+Pb[0])/2, (Pa[1]+Pb[1])/2, (Pa[2]+Pb[2])/2};
    return midpoint;
}

geometry_msgs::Point toROSPoint(std::vector<float> input) {
    geometry_msgs::Point pt;
    pt.x = input[0];
    pt.y = input[1];
    pt.z = input[2];
    return pt;
}

void start_cb(std_msgs::String start) {
    ROS_INFO("starting up segmentation algorithm");
    start_segmentation = true;
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init (argc, argv, "mesh_publisher");
    tf_listener = new (tf::TransformListener);
    ros::NodeHandle nh;
    ROS_INFO("starting up the mesh_publisher...");

    // Create a subscriber to tell script when to start segmentation, subscribe to /odom
    ros::Subscriber start_sub = nh.subscribe ("/start_segmentation", 5, start_cb);
    ros::Subscriber odom_sub = nh.subscribe ("/odom", 5, odom_cb);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/head_camera/depth_registered/points", 5, cloud_cb);

    // Create a ROS publisher for the output mesh
    pub = nh.advertise<shape_msgs::Mesh> ("fetch_mesh", 5);

    // Largely uneeded subscribers to link up with the python visualizer script
    pub2 = nh.advertise<std_msgs::Float32MultiArray> ("rectangle_vertices", 5);
    pub3 = nh.advertise<std_msgs::Float32MultiArray> ("fetch_pointcloud", 5);
    pub4 = nh.advertise<std_msgs::Float32MultiArray> ("fetch_pointcloud_rgb_data", 5);
    pub5 = nh.advertise<std_msgs::Float32MultiArray> ("line_equations", 5);
    pub6 = nh.advertise<std_msgs::Float32MultiArray> ("plane_equation", 5);


    // Spin
    ros::spin();
}



