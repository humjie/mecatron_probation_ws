import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Float32


class GateNavigationController(Node):

    def __init__(self):
        super().__init__('gate_navigation_controller')
        self.vision_subscription = self.create_subscription(
            BoundingBoxArray,
            '/main_camera/detection/bounding_boxes',
            self.listener_callback,
            10)
        self.vision_subscription  # prevent unused variable warning

        self.x_publisher = self.create_publisher(Float32, '/mavros/setpoint_velocity/cmd_vel_unstamped/x', 10)
        self.y_publisher = self.create_publisher(Float32, '/mavros/setpoint_velocity/cmd_vel_unstamped/y', 10)
        self.z_publisher = self.create_publisher(Float32, '/mavros/setpoint_velocity/cmd_vel_unstamped/z', 10)
        self.r_publisher = self.create_publisher(Float32, '/mavros/setpoint_velocity/cmd_vel_unstamped/r', 10)

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.has_gate = False
        self.distance_tolerance = 0.1
        self.min_width_height_ratio = 1
        self.last_move = None
        self.last_width_height_ratio = None
        self.width_height_ratio_diff = None
        self.max_depth = 1
        self.moved_depth = 0

    def publish_velocity(self, x, y, z, r):
        self.x_publisher.publish(Float32(data=x))
        self.y_publisher.publish(Float32(data=y))
        self.z_publisher.publish(Float32(data=z))
        self.r_publisher.publish(Float32(data=r))

    def listener_callback(self, bounding_boxes_msg):
        if not self.has_gate:
            for box in bounding_boxes_msg.bounding_boxes:
                if box.label_name == "gate":
                    msg = box
                    self.get_logger().info("Gate detected at center x: {}, center y: {}".format(msg.x, msg.y))
                    self.get_logger().info("Gate detected at width: {}, height: {}".format(msg.w, msg.h))
                    self.has_gate = True
                    break
                    
        if self.has_gate:
            self.publish_velocity(0.0, 0.0, 0.0, 0.0) # stop previous action

            if msg.y < 0.5 - self.distance_tolerance:
                self.get_logger().info("Gate is above, moving down.")
                self.publish_velocity(0.0, 0.0, -0.1, 0.0) # Move down
            elif msg.y > 0.5 + self.distance_tolerance:
                self.get_logger().info("Gate is below, moving up.")
                self.publish_velocity(0.0, 0.0, 0.1, 0.0) # Move up

            elif msg.x < 0.5 - self.distance_tolerance:
                self.get_logger().info("Gate is to the left, rotating left.")
                self.publish_velocity(0.0, 0.0, 0.0, -0.1) # Rotate left
            elif msg.x > 0.5 + self.distance_tolerance:
                self.get_logger().info("Gate is to the right, rotating right.")
                self.publish_velocity(0.0, 0.0, 0.0, 0.1) # Rotate right

            elif abs(msg.w - msg.h) < self.min_width_height_ratio:
                self.get_logger().info("Gate is misaligned, adjusting orientation.")
                current_width_height_ratio = msg.w / msg.h

                if self.last_width_height_ratio is None:
                    self.last_width_height_ratio = current_width_height_ratio
                    self.publish_velocity(0.0, 0.1, 0.0, 0.0)
                    self.last_move = "right"

                elif self.last_width_height_ratio is not None:
                    self.width_height_ratio_diff = abs(current_width_height_ratio - self.last_width_height_ratio)
                    self.last_width_height_ratio = current_width_height_ratio

                    if self.last_move == "right" and self.width_height_ratio_diff > 0.0:
                        self.publish_velocity(0.0, 0.1, 0.0, 0.0) # Rotate right
                        self.last_move = "right"
                    elif self.last_move == "right" and self.width_height_ratio_diff <= 0.0:
                        self.publish_velocity(0.0, -0.1, 0.0, 0.0) # Rotate left
                        self.last_move = "left"
                    elif self.last_move == "left" and self.width_height_ratio_diff > 0.0:
                        self.publish_velocity(0.0, -0.1, 0.0, 0.0) # Rotate left
                        self.last_move = "left"
                    elif self.last_move == "left" and self.width_height_ratio_diff <= 0.0:
                        self.publish_velocity(0.0, 0.1, 0.0, 0.0) # Rotate right
                        self.last_move = "right"

            else:
                self.get_logger().info("Gate is well aligned, moving forward.")
                self.publish_velocity(0.1, 0.0, 0.0, 0.0) # Move forward

        else:
            self.publish_velocity(0.0, 0.0, 0.0, 0.0) # stop previous action
            self.get_logger().info("No gate detected.")
            self.get_logger().info("Searching for gate.")

            if self.moved_depth < self.max_depth:
                self.publish_velocity(0.0, 0.0, 0.1, 0.1) # Move down and rotate to search for gate
                self.moved_depth += 0.1
            else:
                self.publish_velocity(0.0, 0.0, -0.1, 0.1) # Move back to original depth
                self.moved_depth -= 0.1

    def control_loop(self):
        self.get_logger().info("Timer tick - control loop running")

def main(args=None):
    rclpy.init(args=args)

    gate_navigation_controller = GateNavigationController()

    try:
        rclpy.spin(gate_navigation_controller)
    except KeyboardInterrupt:
        pass
    finally:
        gate_navigation_controller.publish_velocity(0.0, 0.0, 0.0, 0.0)
        gate_navigation_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()