import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import mediapipe as mp
import numpy as np

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

        command = String()
        command.data = "STOP"  # Default command

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw hand landmarks
                self.mp_draw.draw_landmarks(
                    image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                # Get index finger tip position
                index_tip = hand_landmarks.landmark[8]
                
                # Simple gesture detection based on index finger position
                if index_tip.y < 0.5:  # If finger points up
                    command.data = "FORWARD"
                
                # Draw coordinates on image
                h, w, c = image.shape
                cx, cy = int(index_tip.x * w), int(index_tip.y * h)
                cv2.circle(image, (cx, cy), 5, (255, 0, 0), cv2.FILLED)

        # Publish command
        self.publisher_.publish(command)
        
        # Display image
        cv2.imshow('Gesture Detection', image)
        cv2.waitKey(1)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    gesture_detector = GestureDetectionNode()
    rclpy.spin(gesture_detector)
    gesture_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()