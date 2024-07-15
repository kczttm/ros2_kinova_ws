import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from gen3_action_interfaces.action import TakePictures
import cv2
import numpy as np

class TakePicturesActionClient(Node):
    def __init__(self):
        super().__init__('take_pictures_action_client')
        self.get_logger().info('TakePicturesActionClient started, waiting for action server...')
        self._action_client = ActionClient(self, TakePictures, 'gen3_action/take_pictures')

    def send_goal(self, spacing):
        goal_msg = TakePictures.Goal()
        goal_msg.spacing = spacing
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback received: {feedback_msg.feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # Call the display_images function
        # self.display_images(result) 

        #shutdown after the result is received
        rclpy.shutdown() 


    def display_images(self, result):
        for imgmsg in result.images:
            # Convert ROS Image message to OpenCV image
            img = np.frombuffer(imgmsg.data, dtype=np.uint8).reshape((imgmsg.height, imgmsg.width, 3))
            
            # Display the image for 2 seconds
            cv2.imshow("Image", img)
            cv2.waitKey(2000)
            cv2.destroyAllWindows()

    def to_cv2_img(self, result):
        images = []
        for imgmsg in result.images:
            img = np.frombuffer(imgmsg.data, dtype=np.uint8).reshape((imgmsg.height, imgmsg.width, 3))
            images.append(img)
        return images


    
def main(args=None,spacing = 0.005):
    rclpy.init(args=args)
    action_client = TakePicturesActionClient()
    # spacing = 0.01 # meters
    action_client.send_goal(spacing)
    rclpy.spin(action_client)
    future = action_client._get_result_future
    result = future.result().result
    images = action_client.to_cv2_img(result)
    action_client.destroy_node()
    return images

if __name__ == '__main__':
    main()