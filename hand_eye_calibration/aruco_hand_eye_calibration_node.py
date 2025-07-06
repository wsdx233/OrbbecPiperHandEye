import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import TransformException

class ArucoHandEyeCalibrationNode(Node):
    """
    ROS2 Node for performing Eye-on-Base hand-eye calibration.

    This node subscribes to camera images and robot end-effector poses.
    It detects an ArUco marker (target) attached to the robot's gripper.
    It provides services to capture pose pairs and calculate the calibration matrix.

    The calibration finds the transformation from the robot base to the camera frame.
    """

    def __init__(self):
        super().__init__('aruco_hand_eye_calibration_node')

        # --- Parameters ---
        self.declare_parameter('marker_size', 0.05)  # Marker size in meters
        self.declare_parameter('aruco_dictionary_id', 'DICT_6X6_250')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'link6') # From piper URDF
        self.declare_parameter('camera_frame', 'camera_color_optical_frame') # From Orbbec launch files

        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.aruco_dictionary_id = self.get_parameter('aruco_dictionary_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.end_effector_frame = self.get_parameter('end_effector_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        
        self.get_logger().info(f"Using parameters: marker_size={self.marker_size}, aruco_dict={self.aruco_dictionary_id}")
        self.get_logger().info(f"TF frames: base='{self.base_frame}', end_effector='{self.end_effector_frame}'")
        
        # --- Data Storage for Calibration ---
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []

        # --- Camera and ArUco Setup ---
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        try:
            aruco_dict_id = cv2.aruco.__getattribute__(self.aruco_dictionary_id)
            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
            self.aruco_parameters = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        except AttributeError:
            self.get_logger().error(f"Invalid ArUco dictionary ID: {self.aruco_dictionary_id}")
            raise

        # --- ROS2 Communications ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cv_bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 1)

        # Publishers
        self.debug_image_pub = self.create_publisher(Image, '/aruco_detect/image_raw', 10)

        # Services
        self.capture_pose_srv = self.create_service(
            Trigger, 'capture_pose', self.capture_pose_callback)
        self.calculate_calibration_srv = self.create_service(
            Trigger, 'calculate_calibration', self.calculate_calibration_callback)
            
        self.last_seen_rvec = None
        self.last_seen_tvec = None

        self.get_logger().info('Aruco Hand-Eye Calibration Node is ready.')
        self.get_logger().info('Move the robot arm with the ArUco marker in front of the camera.')
        self.get_logger().info('Call `/capture_pose` service to record a pose pair.')
        self.get_logger().info('Call `/calculate_calibration` service to compute the result after collecting enough poses (>= 5).')

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('Camera info received.')
            self.destroy_subscription(self.camera_info_sub) # We only need it once

    def image_callback(self, msg: Image):
        if not self.camera_info_received:
            self.get_logger().warn('Waiting for camera info...', throttle_duration_sec=2)
            return

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        corners, ids, rejected = self.aruco_detector.detectMarkers(cv_image)

        debug_image = cv_image.copy()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

            # Store the first detected marker's pose for capturing
            self.last_seen_rvec = rvecs[0]
            self.last_seen_tvec = tvecs[0]
            
            # Draw axis for visualization
            for i in range(len(ids)):
                cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
        else:
            self.last_seen_rvec = None
            self.last_seen_tvec = None

        try:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert debug image: {e}')
    
    def capture_pose_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info('Capture pose service called.')

        # 1. Get marker pose relative to camera (from image_callback)
        if self.last_seen_rvec is None or self.last_seen_tvec is None:
            response.success = False
            response.message = "No ArUco marker detected in the current view."
            self.get_logger().error(response.message)
            return response

        R_target2cam, _ = cv2.Rodrigues(self.last_seen_rvec)
        t_target2cam = self.last_seen_tvec.T  # Transpose to get a column vector

        # 2. Get end-effector pose relative to robot base from TF
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.end_effector_frame, rclpy.time.Time())
            
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            t_gripper2base = np.array([[trans.x], [trans.y], [trans.z]])
            R_gripper2base = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()

        except TransformException as ex:
            response.success = False
            response.message = f"Could not get transform from '{self.base_frame}' to '{self.end_effector_frame}': {ex}"
            self.get_logger().error(response.message)
            return response
            
        # 3. Store the data
        self.R_gripper2base.append(R_gripper2base)
        self.t_gripper2base.append(t_gripper2base)
        self.R_target2cam.append(R_target2cam)
        self.t_target2cam.append(t_target2cam)

        response.success = True
        response.message = f"Pose pair captured. Total poses: {len(self.R_gripper2base)}"
        self.get_logger().info(response.message)
        
        return response

    def calculate_calibration_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info('Calculate calibration service called.')

        if len(self.R_gripper2base) < 5:
            response.success = False
            response.message = f"Not enough poses captured. Need at least 5, have {len(self.R_gripper2base)}."
            self.get_logger().error(response.message)
            return response
        
        self.get_logger().info("Performing hand-eye calibration...")
        
        # OpenCV expects lists of numpy arrays
        R_gripper2base_np = [np.array(r) for r in self.R_gripper2base]
        t_gripper2base_np = [np.array(t) for t in self.t_gripper2base]
        R_target2cam_np = [np.array(r) for r in self.R_target2cam]
        t_target2cam_np = [np.array(t) for t in self.t_target2cam]

        # This solves the AX=XB problem for an eye-on-base setup.
        # It calculates the transformation from the robot base to the camera frame.
        try:
            R_base2cam, t_base2cam = cv2.calibrateHandEye(
                R_gripper2base=R_gripper2base_np,
                t_gripper2base=t_gripper2base_np,
                R_target2cam=R_target2cam_np,
                t_target2cam=t_target2cam_np,
                method=cv2.CALIB_HAND_EYE_PARK # Other options: TSAI, HORAUD, ANDREFF
            )

            # --- Log and display the results ---
            self.get_logger().info("="*30)
            self.get_logger().info("Hand-Eye Calibration Result (Base to Camera)")
            self.get_logger().info("="*30)

            # Construct 4x4 transformation matrix
            transform_matrix = np.identity(4)
            transform_matrix[:3, :3] = R_base2cam
            transform_matrix[:3, 3] = t_base2cam.flatten()
            
            self.get_logger().info("Transformation Matrix (T_base_cam):\n" + str(np.round(transform_matrix, 4)))

            # Convert to ROS-friendly format (translation in meters, quaternion)
            translation = t_base2cam.flatten()
            quat = R.from_matrix(R_base2cam).as_quat()
            
            self.get_logger().info("\n--- For ROS static_transform_publisher ---")
            self.get_logger().info(f"Translation (x, y, z) [m]: {np.round(translation, 4)}")
            self.get_logger().info(f"Rotation (x, y, z, w) [quaternion]: {np.round(quat, 4)}")
            
            # Also provide RPY for easier understanding
            rpy = R.from_matrix(R_base2cam).as_euler('xyz', degrees=True)
            self.get_logger().info(f"Rotation (roll, pitch, yaw) [degrees]: {np.round(rpy, 2)}")
            
            response.success = True
            response.message = "Calibration successful. Results printed to console."

        except cv2.error as e:
            response.success = False
            response.message = f"OpenCV Error during calibration: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArucoHandEyeCalibrationNode()
        rclpy.spin(node)
    except Exception as e:
        rclpy.logging.get_logger("main").error(f"An error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
