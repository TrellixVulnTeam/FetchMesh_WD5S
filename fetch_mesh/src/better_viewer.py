import rospy
import pptk
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from panda3d_viewer import Viewer, ViewerConfig


class CloudViewer:

    def __init__(self):
        # Various pointclouds
        self.cloud = np.zeros((1))
        self.boundary_indices = np.zeros((1))
        print("initialized clouds")
        # Subscribers
        self.pcd_sub = rospy.Subscriber("fetch_pointcloud", Float32MultiArray, self.cloud_cb)
        self.boundary_sub = rospy.Subscriber("fetch_pointcloud_rgb_data", Float32MultiArray, self.boundary_cb)
        print("started up the subscribers")

        # Booleans to tell if clouds have been gotten
        self.got_cloud = False
        self.got_boundary_cloud = False

        # visualize the cloud
        self.visualize_cloud()

    def cloud_cb(self, ros_cloud):
        if self.got_cloud:
            return 0
        self.cloud = np.zeros((len(ros_cloud.data) / 3, 3))
        for i in range(len(ros_cloud.data)):
            self.cloud[i / 3][i % 3] = ros_cloud.data[i]
        self.got_cloud = True
        rospy.loginfo("got_cloud")

    def boundary_cb(self, boundary_indices):
        if self.got_boundary_cloud:
            return 0
        self.boundary_indices = np.zeros((len(boundary_indices.data)))
        for i in range(len(boundary_indices.data)):
            self.boundary_indices[i] = boundary_indices.data[i]
        self.got_boundary_cloud = True
        rospy.loginfo("got the boundary")

    def visualize_cloud(self):
        while not self.got_cloud or not self.got_boundary_cloud:
            continue
        with Viewer(show_grid=False) as viewer:
            print "visualizing the cloud..."
            viewer.reset_camera((-5, 0, 0), look_at=(0, 0, 0))
            viewer.append_group('root')
            viewer.append_cloud('root', 'cloud', thickness=4)

            while True:
                # Coloring the boundary points of the rectangle
                colors = np.ones((len(self.cloud), 4), np.float32)
                vertices = self.cloud.astype(np.float32)
                for i in range(len(self.boundary_indices)):
                    if int(self.boundary_indices[i]) < len(self.cloud):
                        colors[int(self.boundary_indices[i])][0] = 200

                viewer.set_cloud_data('root', 'cloud', vertices, colors)


def main():
    rospy.init_node("pointcloud_visualizer")
    CloudViewer()
    print "started the cloud viewer"
    rospy.spin()


if __name__ == "__main__":
    main()
