import rospy
import pptk
import time
import numpy as np
from std_msgs.msg import Float32MultiArray
from panda3d_viewer import Viewer, ViewerConfig


# Communicates with the planar_segmentation script, visualizing pointclouds in the much more bearable panda_3d viewer


class PointCloudViewer:

    def __init__(self):
        print "started up the viewer"
        self.got_cloud = False
        self.got_boundary = False
        self.pcd_sub = rospy.Subscriber("fetch_pointcloud", Float32MultiArray, self.cloud_cb)
        self.rgb_sub = rospy.Subscriber("fetch_pointcloud_rgb_data", Float32MultiArray, self.boundary_cb)
        self.line_sub = rospy.Subscriber("line_equations", Float32MultiArray, self.line_cb)
        self.plane_sub = rospy.Subscriber("plane_equation", Float32MultiArray, self.plane_cb)
        self.vertex_sub = rospy.Subscriber("rectangle_vertices", Float32MultiArray, self.vertex_cb)
        self.vertices = list()
        self.got_plane = False
        self.plane = list()
        self.np_cloud = np.zeros((1))
        self.np_boundary = np.zeros((1))
        self.lines = list()
        self.counter = 0
        self.c2 = 0
        self.bc = 0
        self.pc = 0
        self.cloud_count = 0
        self.pandas_visualize()  # Always will be the last thing called here

    def vertex_cb(self, vertices):
        if self.c2 >= 4:
            self.c2 += 1
            return 0
        for i in range(len(vertices.data)):
            self.vertices.append(vertices.data[i])

        self.c2 += 1
        print("read in vertex %d (%f, %f, %f)" % (self.c2, vertices.data[0], vertices.data[1], vertices.data[2]))

    def plane_cb(self, plane):
        if not self.got_plane:
            print(len(plane.data))
            for i in range(len(plane.data)):
                self.plane.append(plane.data[i])
            self.got_plane = True

    def line_cb(self, line):
        if self.counter >= 4:
            self.counter += 1
            return 0
        for i in range(3):
            self.lines.append(line.data[i])
        vec = [0.1 * line.data[i] for i in range(3, 6)]
        for i in range(20):
            self.lines.append(line.data[0] + i * vec[0])
            self.lines.append(line.data[1] + i * vec[1])
            self.lines.append(line.data[2] + i * vec[2])
        for i in range(1, 20):
            self.lines.append(line.data[0] - i * vec[0])
            self.lines.append(line.data[1] - i * vec[1])
            self.lines.append(line.data[2] - i * vec[2])
        self.counter += 1
        print("Read in line %d" % (self.counter))

    def cloud_cb(self, cloud):
        if self.cloud_count == 1:
            self.cloud_count += 1
            return 0
        self.np_cloud = np.zeros((len(cloud.data) / 3, 3))
        for i in range(len(cloud.data)):
            self.np_cloud[i / 3][i % 3] = cloud.data[i]
        self.got_cloud = True
        rospy.loginfo("got_cloud")
        self.cloud_count += 1

    def boundary_cb(self, boundary_indices):
        if self.bc >= 1:
            self.bc += 1
            return 0
        self.np_boundary = np.zeros((len(boundary_indices.data)))
        for i in range(len(boundary_indices.data)):
            self.np_boundary[i] = boundary_indices.data[i]
        self.got_boundary = True
        rospy.loginfo("got rgb data")
        self.bc += 1

    def pandas_visualize(self):
        while self.c2 < 4 or not self.got_boundary or not self.got_cloud or self.counter < 4 or not self.got_plane:
            continue
        rospy.loginfo("inside pandas visualizer")
        with Viewer(show_grid=False) as viewer:
            viewer.reset_camera((-5, 0, 0), look_at=(0, 0, 0))
            viewer.append_group('root')
            viewer.append_cloud('root', 'cloud', thickness=4)
            viewer.append_cloud('root', 'lines', thickness=5)
            viewer.append_cloud('root', 'vertices', thickness = 20)
            while True:
                vertices = self.np_cloud.astype(np.float32)
                linesegs = np.asarray(self.lines).astype(np.float32)
                linesegs = np.resize(linesegs, (len(linesegs)/3, 3))
                # Loop through line segs, get rid of points not on the plane
                # c = 0
                # for i in range(len(self.plane)):
                #     print("plane[%d]" % i)
                #     print(str(self.plane[i]) + '\n')
                # for i in range(len(linesegs)):
                #     product = linesegs[i][0] * self.plane[0] + linesegs[i][1] * self.plane[1] + linesegs[i][2] * \
                #               self.plane[2]
                #     if product != self.plane[3]:
                #         c += 1
                #         print("Product: " + str(product))
                #         print("Point (%f, %f, %f)" % (linesegs[i][0], linesegs[i][1], linesegs[i][2]) + "\n")
                #
                #         # print("Lines have not been properly projected. C'mon bud.")
                #         if c > 20:
                #             return 0
                #         continue
                linecolors = np.ones((len(linesegs), 4), np.float32)
                for i in range(len(linesegs)):
                    linecolors[i][1] = 200
                viewer.set_cloud_data('root', 'lines', linesegs, linecolors)
                colors = np.ones((len(self.np_cloud), 4), np.float32)
                for i in range(len(self.np_boundary)):
                    if int(self.np_boundary[i]) < len(self.np_cloud):
                        colors[int(self.np_boundary[i])][0] = 200
                viewer.set_cloud_data('root', 'cloud', vertices, colors)

                # Set vertex pointcloud
                points = np.asarray(self.vertices).astype(np.float32)
                points = np.resize(points, (4, 3))
                vertex_color = np.ones((len(points), 4), np.float32)
                for i in range(len(vertex_color)):
                    vertex_color[i][0] = 200
                    vertex_color[i][2] = 0
                    vertex_color[i][3] = 0.2
                viewer.set_cloud_data('root', 'vertices', points, vertex_color)
                time.sleep(0.03)


def main():
    rospy.init_node("pointcloud_visualizer")
    PointCloudViewer()
    rospy.spin()


if __name__ == "__main__":
    main()
