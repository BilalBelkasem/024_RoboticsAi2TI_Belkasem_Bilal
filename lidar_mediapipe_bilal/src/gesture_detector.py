#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
from std_msgs.msg import String
import numpy as np

class GestureDetector(Node):
    def __init__(self):
        super().__init__('gesture_detector')
        self.publisher_ = self.create_publisher(String, 'gesture_commands', 10)
        self.cap = cv2.VideoCapture(0)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()
        self.timer = self.create_timer(0.1, self.detect_gesture)
        self.mp_draw = mp.solutions.drawing_utils

    def detect_gesture(self):
        success, image = self.cap.read()
        if not success:
            return

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                thumb_tip = hand_landmarks.landmark[4]
                index_tip = hand_landmarks.landmark[8]
                
                distance = np.sqrt((thumb_tip.x - index_tip.x)**2 + 
                                 (thumb_tip.y - index_tip.y)**2)
                
                h, w, c = image.shape
                thumb_pos = (int(thumb_tip.x * w), int(thumb_tip.y * h))
                index_pos = (int(index_tip.x * w), int(index_tip.y * h))
                cv2.line(image, thumb_pos, index_pos, (255, 0, 0), 2)
                
                msg = String()
                if distance < 0.1:
                    msg.data = 'start'
                    cv2.putText(image, 'START', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    msg.data = 'stop'
                    cv2.putText(image, 'STOP', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                self.get_logger().info(f'Publishing: {msg.data}')
                self.publisher_.publish(msg)

        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow('Gesture Detection', image)
        cv2.waitKey(1)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    detector = GestureDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 