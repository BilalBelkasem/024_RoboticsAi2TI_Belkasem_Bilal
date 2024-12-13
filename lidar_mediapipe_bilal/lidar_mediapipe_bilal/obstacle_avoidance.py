import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random
import cv2
import mediapipe as mp
import numpy as np

class Lidar(Node):
    def __init__(self):
        super().__init__('lidar_navigation')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Laser scan subscription
        self.laser_subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT)
        )
        
        # Gesture command subscription
        self.gesture_subscriber = self.create_subscription(
            String, 'gesture_commands', self.gesture_callback, 10
        )
        
        self.timer_period = 0.1
        self.laser_forward = 0.0
        self.laser_frontLeft = 0.0
        self.laser_frontRight = 0.0
        self.current_gesture = "STOP"
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def gesture_callback(self, msg):
        self.current_gesture = msg.data

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[0]
        self.laser_frontLeft = min(msg.ranges[0:45])
        self.laser_frontRight = min(msg.ranges[-45:])

    def motion(self):
        self.get_logger().info(f"Forward: {self.laser_forward:.2f}, Left: {self.laser_frontLeft:.2f}, "
                              f"Right: {self.laser_frontRight:.2f}, Gesture: {self.current_gesture}")
        
        # Default to stopped unless commanded forward
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        # Only move if gesture commands forward AND no obstacles
        if self.current_gesture == "FORWARD":
            if self.laser_forward > 0.5 and self.laser_frontLeft > 0.5 and self.laser_frontRight > 0.5:
                self.cmd.linear.x = 0.2
            else:
                # Obstacle avoidance logic
                if self.laser_forward < 0.5:
                    self.cmd.angular.z = random.choice([-1.0, 1.0])
                elif self.laser_frontLeft < 0.5:
                    self.cmd.angular.z = -1.0
                elif self.laser_frontRight < 0.5:
                    self.cmd.angular.z = 1.0

        self.publisher_.publish(self.cmd)

class GestureDetectionNode(Node):
    def __init__(self):
        super().__init__('gesture_detection')
        self.publisher_ = self.create_publisher(String, 'gesture_commands', 10)
        self.timer = self.create_timer(0.1, self.detect_gesture)
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        
        # Add variable to track last command
        self.last_command = "STOP"

    def detect_gesture(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Convert BGR to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = self.hands.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        current_command = "STOP"  # Default command

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_draw.draw_landmarks(
                    image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Get index finger tip position
                index_tip = hand_landmarks.landmark[8]
                
                # Simple gesture detection based on index finger position
                if index_tip.y < 0.5:  # If finger points up
                    current_command = "FORWARD"
                
                # Draw coordinates on image
                h, w, c = image.shape
                cx, cy = int(index_tip.x * w), int(index_tip.y * h)
                cv2.circle(image, (cx, cy), 5, (255, 0, 0), cv2.FILLED)

        # Only publish if the command has changed
        if current_command != self.last_command:
            command = String()
            command.data = current_command
            self.publisher_.publish(command)
            self.last_command = current_command
            self.get_logger().info(f'New command published: {current_command}')
        
        # Display current command on image
        cv2.putText(image, f"Current Command: {self.last_command}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Display image
        cv2.imshow('Gesture Detection', image)
        cv2.waitKey(1)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar()
    gesture_detector = GestureDetectionNode()
    rclpy.spin(lidar)
    rclpy.spin(gesture_detector)
    lidar.destroy_node()
    gesture_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
