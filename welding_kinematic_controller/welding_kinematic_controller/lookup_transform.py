import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import PoseStamped

class TransListener(Node):

    def __init__(self):

        super().__init__('frame_listener_node')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.torch_to_tooth()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_publisher = self.create_publisher(PoseStamped, '/torch_tip_pose', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.torch_pose_stamped = PoseStamped()

    def torch_to_tooth(self):
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'torch'
        ts.child_frame_id = 'torch_tip'

        ts.transform.translation.x =0.588         
        ts.transform.translation.y =0.0         
        ts.transform.translation.z =0.456

        ts.transform.rotation.x = 0.0
        ts.transform.rotation.y = 0.0
        ts.transform.rotation.z = 0.0
        ts.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(ts)

    def timer_callback(self):

        source_frame = "torch_tip"
        target_frame = "world"
        transforming = None   # Transform
        try:
            # Get the transform from the 'target' to 'source'
            transforming = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {target_frame} to {source_frame}: {ex}')
            return
        
        # Update the current_pose_stamped using the transformation information
        self.torch_pose_stamped.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.torch_pose_stamped.header.frame_id = source_frame
        self.torch_pose_stamped.pose.position.x = transforming.transform.translation.x
        self.torch_pose_stamped.pose.position.y = transforming.transform.translation.y
        self.torch_pose_stamped.pose.position.z = transforming.transform.translation.z
        self.torch_pose_stamped.pose.orientation.x = transforming.transform.rotation.x
        self.torch_pose_stamped.pose.orientation.y = transforming.transform.rotation.y
        self.torch_pose_stamped.pose.orientation.z = transforming.transform.rotation.z
        self.torch_pose_stamped.pose.orientation.w = transforming.transform.rotation.w

        # Publish the PoseStamped message to the /torch_pose topic
        self.pose_publisher.publish(self.torch_pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    frame_listener = TransListener()
    rclpy.spin(frame_listener)
    frame_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()