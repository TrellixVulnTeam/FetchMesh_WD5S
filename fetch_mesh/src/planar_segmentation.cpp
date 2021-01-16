//
// Created by Jacob Epstein on 11/20/20.
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
#include <vector>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <boost/geometry.hpp>
#include <boost/geometry/arithmetic/cross_product.hpp>
#include <std_msgs/Float32MultiArray.h>

//TODO:
//Find a way to test which points are being selected somehow
// Find a way to test where the corner points of each rectangle are calculated by spawning in unity spheres to
// corresponding locations overlayed with the larger mesh.
// Figure out why the published meshes are incomplete and WAY too small.

namespace bg = boost::geometry;
int jcount = 0;
bool publish_rectangle = true;
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

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
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

    if ((int)cloud->size() < 100) {
        ROS_INFO("pointcloud too small, aborting");
        return;
    }

    // Downsample the pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_ds;
    voxel_ds.setInputCloud(cloud);
    voxel_ds.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel_ds.filter(*downsampled_cloud);

    int counter = 0;

    ROS_INFO("Read in cloud with %d points", (int)downsampled_cloud->size());

    while (planar_seg(*downsampled_cloud)) {
        ROS_INFO("Finished segmenting rectangle %d.", counter);
        counter ++;
    }

    sleep(2); // Go two seconds between each pass of the algorithm
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
    // Find the centroid of cloud_plane, project onto the fitted plane
    ROS_INFO("Publishing the fitted rectangle");

    // Store the cloud_plane in a KD tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr plane_tree (new pcl::search::KdTree<pcl::PointXYZ>());
    plane_tree->setInputCloud(cloudptr);

    // Generate threshold for point being on the boundary
    float radius = 0.08;
    float threshold = 0;
    std::vector<int> found_indices;
    std::vector<float> k_sqr_distances;
    int random_index;
    srand(time(NULL));
    for(int i=0; i<(int)cloud_plane.size()/10; i++) {
        random_index = rand() % (int)cloud_plane.size();
        plane_tree->radiusSearch(cloud_plane.points[random_index], radius, found_indices, k_sqr_distances);
        threshold += (float) found_indices.size();
    }
    threshold /= (float) (cloud_plane.size()/10);
    threshold *= 0.75;

    // Extract the boundary points using the threshold
    std_msgs::Float32MultiArray boundary_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; i<cloud_plane.size(); i++) {
        found_indices.clear();
        plane_tree->radiusSearch(cloud_plane.points[i], radius, found_indices, k_sqr_distances);
        if (found_indices.size() < threshold) {
            boundary_cloud->push_back(cloud_plane.points[i]);
            boundary_indices.data.push_back(i);
        }
    }
    // Send pointcloud and rgb data to panda3d visualizer
    if (true) {
        std_msgs::Float32MultiArray pcd_arr;
        std_msgs::Float32MultiArray boundary_arr;
        for (int i=0; i<(int)cloud_plane.size(); i++) {
            pcd_arr.data.push_back((float)(cloud_plane.points[i].x));
            pcd_arr.data.push_back((float)(cloud_plane.points[i].y));
            pcd_arr.data.push_back((float)(cloud_plane.points[i].z));
        }

        pub3.publish(pcd_arr);
        pub4.publish(boundary_indices);
        publish_rectangle = false;
    }

    ROS_INFO("Boundary cloud has %d points", (int)boundary_cloud->size());
    if (boundary_cloud->size() < 150) {
        ROS_INFO("Aborting linear segmentation, boundary too small");
        return false;
    }

    // Do linear segmentation 4 times on the boundary points to get the edges, store equations of the edges
    pcl::ModelCoefficients coefficients_l1;
    pcl::PointIndices::Ptr inliers_l1 (new pcl::PointIndices);
    pcl::ModelCoefficients coefficients_l2;
    pcl::PointIndices::Ptr inliers_l2 (new pcl::PointIndices);
    pcl::ModelCoefficients coefficients_l3;
    pcl::PointIndices::Ptr inliers_l3 (new pcl::PointIndices);
    pcl::ModelCoefficients coefficients_l4;
    pcl::PointIndices::Ptr inliers_l4 (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud (boundary_cloud);

    seg.segment(*inliers_l1, coefficients_l1); // finding inliers and coeffts of l1
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (boundary_cloud);
    extract.setIndices (inliers_l1);
    extract.setNegative (true);
    extract.filter(*cloud_f);

    seg.setInputCloud (cloud_f);
    seg.segment(*inliers_l2, coefficients_l2); // l2
    extract.setInputCloud(cloud_f);
    extract.setIndices(inliers_l2);
    extract.filter(*cloud2);

    ROS_INFO("Got through three lines...");
    seg.setInputCloud (cloud2);
    seg.segment(*inliers_l3, coefficients_l3); // l3
    extract.setInputCloud(cloud2);
    extract.setIndices(inliers_l3);
    extract.filter(*cloud3);

    seg.setInputCloud (cloud3);
    seg.segment(*inliers_l4, coefficients_l4); // l4
    ROS_INFO("got through all the lines");
    // Project each edge line onto the fitted plane
    pcl::ModelCoefficients projected_l1;
    pcl::ModelCoefficients projected_l2;
    pcl::ModelCoefficients projected_l3;
    pcl::ModelCoefficients projected_l4;
    project_onto_plane(plane_coeffs, coefficients_l1, projected_l1);
    project_onto_plane(plane_coeffs, coefficients_l2, projected_l2);
    project_onto_plane(plane_coeffs, coefficients_l3, projected_l3);
    project_onto_plane(plane_coeffs, coefficients_l4, projected_l4);

    // Visualization feature: send projected_l(1-4) to the pointcloud viewer
    pub5.publish(toRosArray(projected_l1.values));
    pub5.publish(toRosArray(projected_l2.values));
    pub5.publish(toRosArray(projected_l3.values));
    pub5.publish(toRosArray(projected_l4.values));
    pub6.publish(toRosArray(plane_coeffs.values));
    // Calculate intersections of non "almost parallel" edges
    // Note - non almost parallel algo should check one line with all of the other three, then pair the comparison line
    // with whichever line had the smallest slope vector net change
    // This'll be sloppy, but can get cleaned up later...

    // Store the "rectangle" in a mesh

    // Publish the mesh!
    create_rectangle(projected_l1, projected_l2, projected_l3, projected_l4);
    return true;
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

void create_rectangle(pcl::ModelCoefficients l1, pcl::ModelCoefficients l2, pcl::ModelCoefficients l3,
                      pcl::ModelCoefficients l4) {
    // Scale down each slope to a unit vector
    std::vector<float> v1 = {l1.values[3], l1.values[4], l1.values[5]};
    std::vector<float> unit_l1 = unit_vector(v1);
    std::vector<float> v2 = {l2.values[3], l2.values[4], l2.values[5]};
    std::vector<float> unit_l2 = unit_vector(v2);
    std::vector<float> v3 = {l3.values[3], l3.values[4], l3.values[5]};
    std::vector<float> unit_l3 = unit_vector(v3);
    std::vector<float> v4 = {l4.values[3], l4.values[4], l4.values[5]};
    std::vector<float> unit_l4 = unit_vector(v1);

    // Add each other unit vector to l1, pair the two together which are the smallest magnitude away from the origin
    std::vector<float> v12 = {v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]};
    std::vector<float> v13 = {v1[0] - v3[0], v1[1] - v3[1], v1[2] - v3[2]};
    std::vector<float> v14 = {v1[0] - v4[0], v1[1] - v4[1], v1[2] - v4[2]};

    std::vector<float> vertex1;
    std::vector<float> vertex2;
    std::vector<float> vertex3;
    std::vector<float> vertex4;
    std::vector<float> vertex5;
    if (magnitude(v12) < magnitude(v13) && magnitude(v12) < magnitude(v14)) {
        vertex1 = intersect(l1, l3);
        vertex2 = intersect(l1, l4);
        vertex3 = intersect(l2, l3);
        vertex4 = intersect(l2, l4);
    }

    else if (magnitude(v13) < magnitude(v12) && magnitude(v13) < magnitude(v14)) {
        vertex1 = intersect(l1, l2);
        vertex2 = intersect(l1, l4);
        vertex3 = intersect(l3, l2);
        vertex4 = intersect(l3, l4);
    }

    else {
        vertex1 = intersect(l1, l3);
        vertex2 = intersect(l1, l2);
        vertex3 = intersect(l4, l3);
        vertex4 = intersect(l2, l4);
    }
    // Probably temporary feature: publish the coords of vertices to a topic
    pub2.publish(toRosArray(vertex1));
    pub2.publish(toRosArray(vertex2));
    pub2.publish(toRosArray(vertex3));
    pub2.publish(toRosArray(vertex4));


    // Figure out which vertices are diagonal to eachother.
    int diagonal = 2;
    int other1 = 3;
    int other2 = 4;
    float greatest_distance = distance(vertex1, vertex2);
    float d3 = distance(vertex1, vertex3);
    float d4 = distance(vertex1, vertex4);
    if (d3 > greatest_distance) {
        diagonal = 3;
        greatest_distance = d3;
        other1 = 2;
        other2 = 4;
    }
    if (d4 > greatest_distance) {
        diagonal = 4;
        other1 = 2;
        other2 = 3;
    }

    // Make the vertices into two triangles, publish these triangles as a mesh
    shape_msgs::Mesh output;
    std::vector<geometry_msgs::Point > verts;
    std::vector<shape_msgs::MeshTriangle > polygons;

    verts.push_back(toROSPoint(vertex1));
    verts.push_back(toROSPoint(vertex2));
    verts.push_back(toROSPoint(vertex3));
    verts.push_back(toROSPoint(vertex4));

    shape_msgs::MeshTriangle t1;
    shape_msgs::MeshTriangle t2;

    t1.vertex_indices[0] = 0 + 4*jcount;
    t1.vertex_indices[1] = diagonal-1 + 4*jcount;
    t1.vertex_indices[2] = other1-1 + 4*jcount;

    t2.vertex_indices[0] = 0 + 4*jcount;
    t2.vertex_indices[1] = diagonal-1 + 4*jcount;
    t2.vertex_indices[2] = other2-1 + 4*jcount;

    polygons.push_back(t1);
    polygons.push_back(t2);

    output.triangles = polygons;
    output.vertices = verts;
    pub.publish(output);
    jcount ++;

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

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init (argc, argv, "mesh_publisher");
    ros::NodeHandle nh;
    ROS_INFO("starting up the mesh_publisher...");
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/head_camera/depth_registered/points", 5, cloud_cb);
    // Create a ROS publisher for the output mesh
    pub = nh.advertise<shape_msgs::Mesh> ("fetch_mesh", 5);
    pub2 = nh.advertise<std_msgs::Float32MultiArray> ("rectangle_vertices", 5);
    pub3 = nh.advertise<std_msgs::Float32MultiArray> ("fetch_pointcloud", 5);
    pub4 = nh.advertise<std_msgs::Float32MultiArray> ("fetch_pointcloud_rgb_data", 5);
    pub5 = nh.advertise<std_msgs::Float32MultiArray> ("line_equations", 5);
    pub6 = nh.advertise<std_msgs::Float32MultiArray> ("plane_equation", 5);

    // Spin
    ros::spin();
}