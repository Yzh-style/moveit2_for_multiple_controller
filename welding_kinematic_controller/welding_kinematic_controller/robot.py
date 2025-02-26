from rclpy.node import Node
from sensor_msgs.msg import JointState
from .inverse_kinematics import next_point, path_plan, slope_next_point
from math import radians, pi
import math
import numpy as np
import rclpy
from std_msgs.msg import String


class RobotDefinition(Node):
    
    def __init__(self):

        super().__init__('main_node')

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.joint = JointState()

        self.joint.header.frame_id = 'robot'
        self.joint.name            = ['body_rotation', 'body_to_telescopic_joint', 'robotic_arm_base_to_shoulder', 'shoulder_to_upper_arm', 'upper_arm_to_forearm', 'forearm_to_wrist1', 'wrist1_to_wrist2', 'wrist2_to_last_link']

        #initial angles for each joint
        self.body_rotation = 0.0
        self.body_to_telescopic_joint = 0
        self.robotic_arm_base_to_shoulder = 0
        self.shoulder_to_upper_arm = 0
        self.upper_arm_to_forearm = 0
        self.forearm_to_wrist1 = 0
        self.wrist1_to_wrist2 = 0
        self.wrist2_to_last_link = 0
        
        #Creating a subscription to get the input for the values
        self.subscription = self.create_subscription(String,
                                                     'entry',
                                                     self.get_values_callback,
                                                     10)
        #initial input if some initial numbers is missing
        self.xE=0
        self.yE=0
        self.slope=0
        self.emergency_stop= False 
        
        #Inverse Kinematics calculation, the IK have different references so have different angles
        self.alpha=[0, 0, 0, 0, 0, 0, 0, 0] # angles for each joint body_rotation=alpha[1], body_to_telescopic_joint=alpha[2], robotic_arm_base_to_shoulder=alpha[3], shoulder_to_upper_arm=alpha[4], upper_arm_to_forearm=alpha[5], forearm_to_wrist1=alpha[6], wrist1_to_wrist2=alpha[7], wrist2_to_last_link=alpha[8]
        self.q=[] #angle for calculation
        self.success=False #it's possible to reach the next point, starts as false because it's waiting the entry
        self.alpha2=[] #to create the motion alpha2-alpha
        self.a=[0, 0, 0, 0, 0] #dimensions in x for the IK
        self.b = [0, 0, 0, 0, 0] #dimensions in y for the IK
        self.q, self.success = next_point(self.xE, self.yE, self.a, self.b, self.alpha)
        self.calculation=True
        self.i=0
        self.first_time=True
        self.angle=[]
        
        
        self.joint.position = [self.body_rotation, self.robotic_arm_base_to_shoulder, self.shoulder_to_upper_arm, self.upper_arm_to_forearm, self.forearm_to_wrist1, self.wrist1_to_wrist2, self.wrist2_to_last_link]

        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.direction = 0

        self.get_logger().info("Main node is now on")

        self.mensagem = ""
        
    def timer_callback(self):
        
        if self.emergency_stop:
            return
        
        self.joint.header.stamp = self.get_clock().now().to_msg()
        
        self.get_joints()

        self.joint.position = [np.radians(self.body_rotation), np.radians(self.robotic_arm_base_to_shoulder), np.radians(self.shoulder_to_upper_arm), np.radians(self.upper_arm_to_forearm)]

        #messages to print in the prompt

        self.get_logger().info(f"Activity : {self.mensagem}")
        
        self.get_logger().info(f"Sending 'body_rotation': {self.body_rotation}")

        self.get_logger().info(f"Sending 'body_to_telescopic_joint': {self.body_to_telescopic_joint}")

        self.get_logger().info(f"Sending 'robotic_arm_base_to_shoulder': {self.robotic_arm_base_to_shoulder}")

        self.get_logger().info(f"Sending 'shoulder_to_upper_arm': {self.shoulder_to_upper_arm}")

        self.get_logger().info(f"Sending 'upper_arm_to_forearm': {self.upper_arm_to_forearm}")

        self.get_logger().info(f"Sending 'upper_arm_to_forearm': {self.forearm_to_wrist1}")

        self.get_logger().info(f"Sending 'wrist1_to_wrist2': {self.wrist1_to_wrist2}")

        self.get_logger().info(f"Sending 'wrist2_to_last_link': {self.wrist2_to_last_link}")
        
        self.joint_state_publisher.publish(self.joint)
        

    
def main(args = None):

    rclpy.init(args = args)
    robot = RobotDefinition()
    
    rclpy.spin(robot)

if __name__ == '__main__':
    
    main()