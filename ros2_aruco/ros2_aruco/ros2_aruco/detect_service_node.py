import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros2_aruco_interfaces.srv import GetMapData

# for detection
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers

class MapDataService(Node):

    def __init__(self):
        super().__init__('map_data_service')

        # Detection info
        self.declare_parameter("marker_size", .0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        
        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data)

        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()

        # Useful variables
        self.id = None
        self.position_x = None
        self.position_y = None
        self.marker_detected = False
        self.detect_active = False

        # Publish conrol velocity of the robot
        self.pub_vel_control = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe robot's odom
        self.position = self.create_subscription(Odometry, '/odom', self.orientation_and_position_callback, 10)

        # Creiamo il servizio
        self.srv = self.create_service(GetMapData, 'get_map_data', self.get_map_data_callback)  


    # Callback used to estimate the orientation from the odom
    def orientation_and_position_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y

        #self.get_logger().info(f'x={self.position_x}, y={self.position_y}')

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        # Routine for controlling robot velocity

        if not self.detect_active:
            return

        control_msg = Twist()
        
        #self.get_logger().info('Detection')
    
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
    
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')
    
        markers = ArucoMarkers()
        pose_array = PoseArray()
    
        if self.camera_frame is None:
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame
    
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp
    
        # Rilevazione dei marker
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dictionary, parameters=self.aruco_parameters)

        print(marker_ids)
        if marker_ids is None:
            # Nessun marker rilevato, aggiorniamo lo stato
            #self.marker_detected = False
            #self.id = None  # Reset dell'id
            control_msg.angular.z = 0.7
            self.pub_vel_control.publish(control_msg)
        else:
            # Marker rilevati
            for id in marker_ids.flatten().tolist():
                if id != 0 and id is not None:  # Condizione per elemento non nullo
                    self.id = id
                    break
                else:
                    self.id = None  # Nel caso non ci siano elementi validi

            self.marker_detected = True
            control_msg.angular.z = 0.0  # Ferma la rotazione
            self.pub_vel_control.publish(control_msg)


    def get_map_data_callback(self, request, response):
        # retrieve info pose
        self.detect_active = True

        if self.marker_detected:
            response.x = self.position_x
            response.y = self.position_y
            response.marker_id = self.id
            self.marker_detected = False
            self.detect_active = False

            self.get_logger().info(f"Sending map data: x={response.x}, y={response.y}, marker_id={response.marker_id}")

        return response


def main(args=None):
    rclpy.init(args=args)

    map_data_service = MapDataService()

    rclpy.spin(map_data_service)

    map_data_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()