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


links = {}
boxes = []
max_distance = 7

detection_pub = rospy.Publisher('fake_net', DetectionArray, queue_size=10)


class FisheyeLens:
    """Describes a Fisheye Lens.

    Description:
        Follows the standard fisheye model of R = c1 * f * tan(theta / c2 + c3),
        where theta is the angle from the focal axis to the real point X, R is
        the radius of the polar coordinate that describes the real point X when
        projected onto a 2D image, and phi is the polar angle about the focal
        axis of the projected coordinate. See
        https://wiki.panotools.org/Fisheye_Projection for more details.

    Attributes:
        c1: The c1 scalar of the mapping function.
        c2: The scaling coefficient of the distortion function.
        c3: The angle offset of the distortion function (in radians).
        f: The focal length of the lens.
        max_fov: The maximum field of view (in radians).
    """

    def __init__(self, c1, c2, c3, f, max_fov=np.pi):
        """Initializes a fisheye camera lens."""
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.focal_length = f
        self.max_fov = max_fov


lens_parameters = FisheyeLens(c1=685.4, c2=525.2, c3=0, f=307.53251966606937)


class Point:
    """Describes a 3D point.

    Attributes:
        x: A float describing the X position.
        y: A float describing the Y position.
        z: A float describing the Z position.
    """

    def __init__(self, pts):
        """Inits the Point from a list of values."""
        self.x = pts[0]
        self.y = pts[1]
        self.z = pts[2]


class LinkDescriptor:
    """Describes an SDF Link.

    Attributes:
        label: The detection label to assign to the link.
        label_id: The detection label id to assign to the link.
        width: The width of the link (X dimension).
        height: The height of the link (Z dimension).
        depth: The depth of the link (Y dimension).
    """

    def __init__(self, label, width, height, depth, label_id):
        """Initializes a link descriptor."""
        self.label = label
        self.label_id = label_id
        self.width = width
        self.height = height
        self.depth = depth


def point_to_pixel(point_array, lens_params=lens_parameters):
    """Converts a real-world point (or vector) to normalized image coordinates
        [-1, 1] within an image with fisheye distortion.

    Note:
        This function assumes that the camera coordinate system positions the
        negative Z axis in the direction of the lens, X to the right, and Y
        upwards.

    Args:
        point_array: A list containing the X, Y, and Z coordinates of the
            point/vector.
        lens_params: A FisheyeLens object to describe the fisheye distortion on
            the point.

    Returns:
        A list of two values, each normalized within [-1,1] and centered at the
        center of the image. E.g. [0, 0] indicates the pixel would be at the
        direct center of the image.
    """
    # Calculate the X,Y coordinates of the point projected onto the camera
    # scene. See https://www.scratchapixel.com/lessons/3d-basic-rendering/
    # computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-
    # coordinates-of-3d-points for more details.
    point = Point(point_array)
    x = point.x / -point.z
    y = point.y / -point.z

    # Calculate the polar angle of the projected point. The polar angle does not
    # change through application of fisheye distortions.
    phi = np.arctan2(y, x)

    # Calculate the angle (theta) between the focal axis and the real world
    # point. This is the angle between a line from the camera lens through the
    # center of the image and a line from the camera lens to the real-world
    # point.
    theta = np.arctan2(math.sqrt(point.x * point.x + point.y * point.y), -point.z)

    c1 = lens_params.c1
    c2 = lens_params.c2
    c3 = lens_params.c3
    f = lens_params.focal_length

    # Calculate the modified radius of the projected point's polar coordinate.
    r = c1 * f * math.tan(theta / c2 + c3)

    # Calculate the maximum radius that is supported by the lens to normalize
    # the value.
    theta_max = lens_params.max_fov / 2
    r_max = c1 * f * math.tan(theta_max / c2 + c3)

    # Convert the polar coordinates back into cartesian and normalize them to
    # provide the fisheye'd position of the point.
    return [math.cos(phi) * r / r_max, math.sin(phi) * r / r_max]


def rotate_point(point, quaternion):
    """Rotates a point (or vector) by a quaternion.

    Args:
        point: A list of 3 elements that makes up the point or vector.
        quaternion: A list of 4 elements in [x, y, z, w] that form the
            quaternion.

    Returns:
        The rotated point (or vector) after applying the quaternion.
    """
    q2 = point
    q2.append(0.0)

    # The function for quaternion multiplication can be found here:
    # https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-
    # quaternion
    return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(quaternion, q2),
            tf.transformations.quaternion_conjugate(quaternion))[:3]


def change_reference_frame(point, rot_quat, origin):
    """Converts a point to a new frame of reference.

    Args:
        point: A list of 3 elements that compose the point to transform.
        rot_quat: A quaternion (in [x, y, z, q] form) that describes the
            rotation of the new reference frame.
        origin: A list of 3 elements that makes up the origin of the new
            reference frame.
    Returns:
        The same point when viewed from the new frame of reference.
    """
    x_camera = rotate_point([1, 0, 0], rot_quat)
    y_camera = rotate_point([0, 1, 0], rot_quat)
    z_camera = rotate_point([0, 0, 1], rot_quat)

    # Calculate the M transformation matrix for conversion between the global
    # frame of reference and the camera frame. Details can be found at
    # https://www.scratchapixel.com/lessons/3d-basic-rendering/
    # computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-
    # coordinates-of-3d-points.
    m_transform = np.matrix([[x_camera[0], x_camera[1], x_camera[2], 0],
                            [y_camera[0], y_camera[1], y_camera[2], 0],
                            [z_camera[0], z_camera[1], z_camera[2], 0],
                            [origin[0], origin[1], origin[2], 1]])

    # Invert the matrix so that conversion from the global frame to the camera
    # frame is possible.
    m_inverse = np.linalg.inv(m_transform)
    point_4 = point
    point_4.append(1)
    p = (point_4 * m_inverse).tolist()[0][:3]

    # Apply an empirically determined transformation to relocate the -Z axis along
    # the lens of the camera, X to the right, and Y upwards.
    return [-1 * p[1], -1 * p[0], -1 * p[2]]


class BoundingBox:
    """Describes a 3D cube and bounding box of a Gazebo link.

    Attributes:
        origin: The origin of the link wrt the current frame of reference.
        label: The detection label to assign to the link.
        label_id: The detection label ID to assign to the link.
        pts: A list of 3D points that describe a bounding box around the link.
    """

    def __init__(self, origin, descriptor, orientation):
        """Initializes the bounding box."""
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
        """Modifies the current frame of reference.

        Arguments:
            self: The object to modify.
            rot: The rotation quaternion of the new frame of reference (in
                [x, y, z, w] form).
            origin: The origin of the new frame of reference.

        Returns:
            None.
        """
        updated_pts = []
        for pt in self.pts:
            updated_pts.append(change_reference_frame(pt, rot, origin))

        self.origin = change_reference_frame(self.origin, rot, origin)
        self.pts = updated_pts


    def calculate_pixels(self):
        """Calculate the (normalized) image pixels of the 2D bounding box.

        Note: This calculates a 2D box from the origin of the current frame of
            reference.

        Returns:
            The origin and a list of 8 verticies, each described as [x, y]
            lists.
        """
        pixels = []
        pixel_origin = point_to_pixel(self.origin)
        for pt in self.pts:
            pixels.append(point_to_pixel(pt))

        return pixel_origin, pixels


    def distance(self):
        """Calculates the distance of the link from the current origin."""
        np_array = np.array(self.origin)
        return np.sqrt(np_array.dot(np_array))


def callback(link_states):
    """Callback for gazebo link_state update messages.

    Args:
        link_states: A ROS LinkStates message.

    Returns:
        None.
    """
    links_local = set(list(links.keys())).intersection(link_states.name)

    # Calculate the camera's position and orientation for frame of reference
    # conversions.
    camera_index = link_states.name.index('robosub::left_camera')
    q = link_states.pose[camera_index].orientation
    p = link_states.pose[camera_index].position
    camera_orientation = [q.x, q.y, q.z, q.w]

    # The physical camera sensor is actually offset by 14cm along the Z axis
    # from the link, which causes non-trivial distortions in mappings. Correct
    # for this by setting the proper camera position.
    sensor_offset = rotate_point([0, 0, 0.14], camera_orientation).tolist()
    camera_position = map(add, [p.x, p.y, p.z], sensor_offset)

    # Only perform this update loop if the previous iteration has sent off a
    # DetectionArray message. It will clear the boxes array when it is done.
    if len(boxes) == 0:
        for link in links_local:
            index = link_states.name.index(link)
            p = link_states.pose[index].position
            q = link_states.pose[index].orientation

            # Determine the current position and orientation of the link.
            link_orientation = [q.x, q.y, q.z, q.w]
            link_position = [p.x, p.y, p.z]

            # Construct a bounding box (3D) for the link and convert it's frame
            # of reference to that of the camera's. The box must be constructed
            # before the frame of reference is changed so that the vertices of
            # the box are properly transformed.
            box = BoundingBox(link_position, links[link], link_orientation)
            box.change_reference(camera_orientation, camera_position)

            # Only add the box to the display list if its within the maximum
            # distance from the camera and is infront of the camera (negative
            # Z).
            if box.origin[2] < 0 and box.distance() < max_distance:
                boxes.append(box)


def camera_callback(image):
    """ROS callback functon for left camera images.

    Args:
        image: The sensor_msgs/Image message.
    """
    global boxes

    # Convert the Image message into an open CV message and apply a filter to
    # remove fisheye distortions.
    cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(image, "bgr8")
    undistorted = cv2.remap(cv_image, map1, map2, cv2.INTER_LINEAR)

    detections_msg = DetectionArray()

    rows = cv_image.shape[0]
    cols = cv_image.shape[1]

    # Note that the calculated normalized pixel coordinates rely on a square
    # image. The image output by Gazebo is scaled to fir the HFOV, which crops
    # the Y dimension. Scale the Y dimensions appropriate so they aren't offset
    # when plotting the detection windows. This can be done by dividing all of
    # the Y measurements by the calculated scaling factor.
    ratio_scale_y = float(rows) / cols

    for box in boxes:
        origin, pixels = box.calculate_pixels()
        points = []

        # Convert the normalized pixel coordinates centered around the middle
        # of the image into real coordinates starting at the top left with
        # positive Y down and positive X right.
        origin[1] /= ratio_scale_y
        points.append([int((origin[0] + 1) / 2 * cols), int((1 - (origin[1] + 1) / 2) * rows)])
        for location in pixels:
            location[1] /= ratio_scale_y
            points.append([int((location[0] + 1)/2 * cols), int((1 - (location[1] + 1) / 2) * rows)])

        np_array = np.ndarray(shape=(len(points),1,2))
        for i in range(0, len(points)):
            np_array[i] = points[i]

        # Apply fisheye undistortion to the list of points to locate them within
        # the undistorted image.
        new_locations = cv2.fisheye.undistortPoints(np_array, K1, D2, R=R1, P=P1).reshape(-1,2)

        # There appears to be some form of bug that causes errant
        # transformations from the fisheye'd pixel to undistorted coordinates.
        # Fisheye undistortion should never cause the new location to cross
        # into a new cartesian quadrant. If this happens, ignore this box as it
        # must not be visible in the frame.
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
            # Verify that the point has not changed quadrants. If it has, there
            # was an errant transformation and the bounding box should not be
            # drawn.
            original_x = points[index][0] - undistorted.shape[1] / 2
            new_x = p[0] - undistorted.shape[1] / 2
            index = index + 1
            if (original_x > 0 and new_x < 0) or (original_x < 0 and new_x > 0):
                draw_box = False

            if abs(locs[0][0] - p[0]) >= undistorted.shape[1] or \
               abs(locs[0][1] - p[1]) >= undistorted.shape[0]:
                draw_box = False

            # Calculate the min and max X,Y coordinates to draw a box that
            # encapsulates the link.
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

            # For debug purposes, draw the points onto the undistorted CV image.
            cv2.circle(undistorted, (int(p[0]), int(p[1])), 5, (255, 0, 0), 2)

        # If the box was not marked as errant, mark a successful detection and
        # draw the box onto a CV image for debugging.
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

    # Clear the boxes list to indicate the message has been sent.
    boxes = []

    detection_pub.publish(detections_msg)


    # Display the un-fisheye'd image with bounding points, link origins, and
    # bounding boxes for debugging purposes.
    img_small = cv2.resize(undistorted, (0,0), fx=0.5, fy=0.5)
    cv2.imshow("Image", img_small)
    cv2.waitKey(1)


if __name__ == '__main__':
    filename = rospy.get_param('~fisheye_camera_file')
    data = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    K1 = data.getNode('K1').mat()
    D2 = data.getNode('D2').mat()
    R1 = data.getNode('R1').mat()
    P1 = data.getNode('P1').mat()

    # Python cv2.FileStorage appears to be unable to parse image_size from the
    # XML (which also appears to be a known bug). It is hardcoded here instead.
    image_size = (1384, 1032)

    # Create a rectification map to correct fisheye distortions.
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K1, D2, R1, P1, image_size, cv2.CV_16SC2)

    # Load a list of important links that need to be detected.
    all_links = rospy.get_param('/simulator/significant_links')
    for link in all_links:
        links[link['name']] = LinkDescriptor(link['label'], link['width'], link['height'], link['depth'], link['label_id'])

    # Start subscribes and publishers for the ROS framework.
    rospy.init_node('fake_network')
    detection_pub = rospy.Publisher('vision', DetectionArray, queue_size=10)
    rospy.Subscriber('gazebo/link_states', LinkStates, callback)
    rospy.Subscriber('camera/left/image_raw', Image, camera_callback)

    rospy.spin()
