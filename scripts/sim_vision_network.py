import cv2
import cv_bridge
import math
import numpy as np
import rospy
import tf

from gazebo_msgs.msg import LinkStates
from operator import add, sub
from rs_yolo.msg import DetectionArray, Detection
from sensor_msgs.msg import Image


class FisheyeLens:
    def __init__(self, c1, c2, c3, f, max_fov=np.pi):
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.focal_length = f
        self.max_fov = max_fov


links = {}
lens_parameters = FisheyeLens(c1=685.4, c2=525.2, c3=0, f=307.53251966606937)
boxes = []
max_distance = 7

detection_pub = rospy.Publisher('fake_net', DetectionArray, queue_size=10)


class Point:
    def __init__(self, pts):
        self.x = pts[0]
        self.y = pts[1]
        self.z = pts[2]


class LinkDescriptor:
    def __init__(self, label, width, height, depth, label_id):
        self.label = label
        self.label_id = label_id
        self.width = width
        self.height = height
        self.depth = depth


def point_to_pixel(point_array, lens_params=lens_parameters):
    point = Point(point_array)
    x = point.x / -point.z
    y = point.y / -point.z

    phi = np.arctan2(y, x)
    theta = np.arctan2(math.sqrt(point.x * point.x + point.y * point.y), -point.z)

    c1 = lens_params.c1
    c2 = lens_params.c2
    c3 = lens_params.c3
    f = lens_params.focal_length

    r = c1 * f * math.tan(theta / c2 + c3)

    theta_max = lens_params.max_fov / 2
    r_max = c1 * f * math.tan(theta_max / c2 + c3)

    return [math.cos(phi) * r / r_max, math.sin(phi) * r / r_max]


def rotate_point(point, quaternion):
    q2 = point
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(quaternion, q2),
            tf.transformations.quaternion_conjugate(quaternion))[:3]


def change_reference_frame(point, rot_quat, origin):
    x_camera = rotate_point([1, 0, 0], rot_quat)
    y_camera = rotate_point([0, 1, 0], rot_quat)
    z_camera = rotate_point([0, 0, 1], rot_quat)

    m_transform = np.matrix([[x_camera[0], x_camera[1], x_camera[2], 0],
                            [y_camera[0], y_camera[1], y_camera[2], 0],
                            [z_camera[0], z_camera[1], z_camera[2], 0],
                            [origin[0], origin[1], origin[2], 1]])

    m_inverse = np.linalg.inv(m_transform)
    point_4 = point
    point_4.append(1)
    p = (point_4 * m_inverse).tolist()[0][:3]
    return [-1 * p[1], -1 * p[0], -1 * p[2]]


class BoundingBox:
    def __init__(self, origin, descriptor, orientation):
        self.origin = origin
        self.label = descriptor.label
        self.label_id = descriptor.label_id

        self.pts = []
        for i in range(0, 2):
            x = descriptor.width / 2 - i * descriptor.width
            for j in range(0, 2):
                y = descriptor.depth / 2 - j * descriptor.depth
                for k in range(0, 2):
                    z = descriptor.height / 2 - k * descriptor.height
                    offset = rotate_point([x, y, z], orientation).tolist()
                    self.pts.append(map(add, origin, offset))


    def change_reference(self, rot, origin):
        updated_pts = []
        for pt in self.pts:
            updated_pts.append(change_reference_frame(pt, rot, origin))

        self.origin = change_reference_frame(self.origin, rot, origin)
        self.pts = updated_pts


    def calculate_pixels(self):
        pixels = []
        pixel_origin = point_to_pixel(self.origin)
        for pt in self.pts:
            pixels.append(point_to_pixel(pt))

        return pixel_origin, pixels


    def distance(self):
        np_array = np.array(self.origin)
        return np.sqrt(np_array.dot(np_array))


def callback(link_states):
    links_local = set(list(links.keys())).intersection(link_states.name)

    camera_index = link_states.name.index('robosub::left_camera')
    q = link_states.pose[camera_index].orientation
    p = link_states.pose[camera_index].position
    camera_orientation = [q.x, q.y, q.z, q.w]

    sensor_offset = rotate_point([0, 0, 0.14], camera_orientation).tolist()
    camera_position = map(add, [p.x, p.y, p.z], sensor_offset)

    if len(boxes) == 0:
        for link in links_local:
            index = link_states.name.index(link)
            p = link_states.pose[index].position
            q = link_states.pose[index].orientation
            link_orientation = [q.x, q.y, q.z, q.w]
            link_position = [p.x, p.y, p.z]

            box = BoundingBox(link_position, links[link], link_orientation)
            box.change_reference(camera_orientation, camera_position)

            if box.origin[2] < 0 and box.distance() < max_distance:
                boxes.append(box)


def camera_callback(image):
    global boxes

    cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(image, "bgr8")
#    undistorted = cv_image.copy()
    undistorted = cv2.remap(cv_image, map1, map2, cv2.INTER_LINEAR)

    detections_msg = DetectionArray()

    rows = cv_image.shape[0]
    cols = cv_image.shape[1]
    ratio_scale_y = float(rows) / cols
    for box in boxes:
        origin, pixels = box.calculate_pixels()
        points = []
        origin[1] /= ratio_scale_y
        points.append([int((origin[0] + 1) / 2 * cols), int((1 - (origin[1] + 1) / 2) * rows)])
        for location in pixels:
            location[1] /= ratio_scale_y
            points.append([int((location[0] + 1)/2 * cols), int((1 - (location[1] + 1) / 2) * rows)])

        np_array = np.ndarray(shape=(len(points),1,2))
        for i in range(0, len(points)):
            np_array[i] = points[i]

#        new_locations = np_array.reshape(-1, 2)
        new_locations = cv2.fisheye.undistortPoints(np_array, K1, D2, R=R1, P=P1).reshape(-1,2)

        original_x = points[0][0] - undistorted.shape[1] / 2
        new_x = new_locations[0][0] - undistorted.shape[1] / 2
        if (original_x > 0 and new_x < 0) or (original_x < 0 and new_x > 0):
            continue

        point = new_locations[0].flatten()
        cv2.circle(undistorted, (int(point[0]), int(point[1])), 5, (0, 0, 255), 2)

        locs = new_locations[1:]
        max_x = min(locs[0][0], undistorted.shape[1])
        min_x = max(locs[0][0], 0)
        max_y = min(locs[0][1], undistorted.shape[0])
        min_y = max(locs[0][1], 0)
        draw_box = True
        index = 1
        for p in locs:
            original_x = points[index][0] - undistorted.shape[1] / 2
            new_x = p[0] - undistorted.shape[1] / 2
            index = index + 1
            if (original_x > 0 and new_x < 0) or (original_x < 0 and new_x > 0):
                draw_box = False

            if abs(locs[0][0] - p[0]) >= undistorted.shape[1] or \
               abs(locs[0][1] - p[1]) >= undistorted.shape[0]:
                print 'Bad'
                draw_box = False

            p[0] = min(undistorted.shape[1], p[0])
            p[0] = max(0, p[0])
            p[1] = min(undistorted.shape[0], p[1])
            p[1] = max(0, p[1])
            if p[0] < min_x:
                min_x = p[0]
            if p[0] > max_x:
                max_x = p[0]
            if p[1] < min_y:
                min_y = p[1]
            if p[1] > max_y:
                max_y = p[1]

            cv2.circle(undistorted, (int(p[0]), int(p[1])), 5, (255, 0, 0), 2)

        if draw_box and min_x != max_x and min_y != max_y:
            detection = Detection()
            detection.label = box.label
            detection.label_id = box.label_id
            detection.probability = 1
            detection.x = min_x
            detection.y = min_y
            detection.width = max_x - min_x
            detection.height = max_y - min_y
            detections_msg.detections.append(detection)

            cv2.rectangle(undistorted, (int(min_x), int(min_y)), (int(max_x), int(max_y)), (0,255,0), 2)

    boxes = []

    detection_pub.publish(detections_msg)


    img_small = cv2.resize(undistorted, (0,0), fx=0.5, fy=0.5)
    cv2.imshow("Image", img_small)
    cv2.waitKey(1)


if __name__ == '__main__':
    data = cv2.FileStorage('/home/ryan/repositories/robosub/src/vision/calib/stereo_out_camera_data.xml', cv2.FILE_STORAGE_READ)
    K1 = data.getNode('K1').mat()
    D2 = data.getNode('D2').mat()
    R1 = data.getNode('R1').mat()
    P1 = data.getNode('P1').mat()
    image_size = (1384, 1032)

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K1, D2, R1, P1, image_size, cv2.CV_16SC2)

    all_links = rospy.get_param('/simulator/significant_links')
    for link in all_links:
        links[link['name']] = LinkDescriptor(link['label'], link['width'], link['height'], link['depth'], link['label_id'])

    rospy.init_node('fake_network')
    detection_pub = rospy.Publisher('fake_net', DetectionArray, queue_size=10)
    rospy.Subscriber('gazebo/link_states', LinkStates, callback)
    rospy.Subscriber('/camera/left/image_raw', Image, camera_callback)

    rospy.spin()
