#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from tf_transformations import quaternion_from_euler

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Parameters
        self.declare_parameter('marker_length', 0.1)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('max_detection_distance', 5.0)
        
        self.marker_length = self.get_parameter('marker_length').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.max_distance = self.get_parameter('max_detection_distance').value
        
        # ArUco dictionary
        aruco_dict_name = self.get_parameter('dictionary').value
        self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, aruco_dict_name))
        self.aruco_params = aruco.DetectorParameters()
        
        # Camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # CV bridge
        self.bridge = CvBridge()
        
        # Publishers and Subscribers
        self.detection_pub = self.create_publisher(PoseArray, '/aruco_detections', 10)
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.get_logger().info('ArUco detector node started')

    def camera_info_callback(self, msg):
        """Extract camera calibration parameters"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration parameters received')

    def image_callback(self, msg):
        """Process image to detect ArUco markers"""
        if self.camera_matrix is None:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect markers
            corners, ids, rejected = aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.aruco_params
            )
            
            if ids is not None:
                # Estimate pose
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                )
                
                # Create detection message
                pose_array = PoseArray()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = self.camera_frame
                
                for i in range(len(ids)):
                    # Filter by distance
                    distance = np.linalg.norm(tvecs[i][0])
                    if distance > self.max_distance:
                        continue
                    
                    # Convert rotation vector to quaternion
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                    quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                    
                    # Create pose
                    pose = Pose()
                    pose.position = Point(
                        x=float(tvecs[i][0][0]),
                        y=float(tvecs[i][0][1]), 
                        z=float(tvecs[i][0][2])
                    )
                    pose.orientation = Quaternion(
                        x=quaternion[0],
                        y=quaternion[1],
                        z=quaternion[2], 
                        w=quaternion[3]
                    )
                    
                    pose_array.poses.append(pose)
                
                # Publish detections
                self.detection_pub.publish(pose_array)
                
                # Optional: Draw detections for visualization
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                for i in range(len(ids)):
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, 
                                    rvecs[i], tvecs[i], self.marker_length * 0.5)
                
                # Display image (for debugging)
                cv2.imshow('ArUco Detection', cv_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert rotation matrix to quaternion"""
        # Simple conversion - for better accuracy use tf_transformations
        trace = np.trace(rotation_matrix)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (rotation_matrix[2,1] - rotation_matrix[1,2]) / S
            y = (rotation_matrix[0,2] - rotation_matrix[2,0]) / S
            z = (rotation_matrix[1,0] - rotation_matrix[0,1]) / S
        elif rotation_matrix[0,0] > rotation_matrix[1,1] and rotation_matrix[0,0] > rotation_matrix[2,2]:
            S = np.sqrt(1.0 + rotation_matrix[0,0] - rotation_matrix[1,1] - rotation_matrix[2,2]) * 2
            w = (rotation_matrix[2,1] - rotation_matrix[1,2]) / S
            x = 0.25 * S
            y = (rotation_matrix[0,1] + rotation_matrix[1,0]) / S
            z = (rotation_matrix[0,2] + rotation_matrix[2,0]) / S
        elif rotation_matrix[1,1] > rotation_matrix[2,2]:
            S = np.sqrt(1.0 + rotation_matrix[1,1] - rotation_matrix[0,0] - rotation_matrix[2,2]) * 2
            w = (rotation_matrix[0,2] - rotation_matrix[2,0]) / S
            x = (rotation_matrix[0,1] + rotation_matrix[1,0]) / S
            y = 0.25 * S
            z = (rotation_matrix[1,2] + rotation_matrix[2,1]) / S
        else:
            S = np.sqrt(1.0 + rotation_matrix[2,2] - rotation_matrix[0,0] - rotation_matrix[1,1]) * 2
            w = (rotation_matrix[1,0] - rotation_matrix[0,1]) / S
            x = (rotation_matrix[0,2] + rotation_matrix[2,0]) / S
            y = (rotation_matrix[1,2] + rotation_matrix[2,1]) / S
            z = 0.25 * S
            
        return [x, y, z, w]

def main():
    rclpy.init()
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
