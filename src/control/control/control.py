import rclpy
from rclpy.node import Node
from vision_msgs.msg import BoundingBox, BoundingBoxArray
from std_msgs.msg import Float32
import time


class GateNavigationController(Node):

    def __init__(self):
        super().__init__('gate_navigation_controller')
        self.vision_subscription = self.create_subscription(
            BoundingBoxArray,
            '/main_camera/detection/bounding_boxes',
            self.listener_callback,
            10)
        self.vision_subscription  # prevent unused variable warning

        # publishers for changing velocity
        self.x_publisher = self.create_publisher(Float32, '/mavros/setpoint_velocity/cmd_vel_unstamped/x', 10)
        self.y_publisher = self.create_publisher(Float32, '/mavros/setpoint_velocity/cmd_vel_unstamped/y', 10)
        self.z_publisher = self.create_publisher(Float32, '/mavros/setpoint_velocity/cmd_vel_unstamped/z', 10)
        self.r_publisher = self.create_publisher(Float32, '/mavros/setpoint_velocity/cmd_vel_unstamped/r', 10)

        # control loop timer
        self.control_timer = self.create_timer(0.2, self.control_loop)

        # State variables
        self.has_gate = False
        self.distance_tolerance = 0.03
        self.min_width_height_ratio = 0.5
        self.last_move = None
        self.last_width_height_ratio = None
        self.max_depth = -1.0
        self.moved_depth = 0
        
        # Frame persistence for ratio improvement tracking
        self.consecutive_improving_frames = 0
        self.consecutive_worsening_frames = 0
        self.frames_threshold = 3  # Require 3 consecutive frames before switching
        self.last_ratio_trend = None  # "improving", "worsening", or None
        
        # Last move tracking for centering validation
        self.last_gate_position = {'x': 0.5, 'y': 0.5}
        self.last_distance_from_center = 0.0
        self.last_move_time = time.time()
        
        # Add state tracking for well-aligned gate
        self.is_well_aligned = False
        self.alignment_start_time = None
        
        # Add state for close proximity (height > 0.7)
        self.is_close_to_gate = False
        self.close_gate_start_time = None

        # Vision detection tracking for noise filtering
        self.consecutive_detections = 0
        self.consecutive_no_detections = 0
        self.detection_threshold = 3  # Need 3 consecutive detections to confirm
        self.no_detection_threshold = 5  # Need 5 consecutive no-detections to confirm loss
        self.last_confirmed_gate_msg = None

    # change velocity
    def publish_velocity(self, x, y, z, r):
        self.x_publisher.publish(Float32(data=x))
        self.y_publisher.publish(Float32(data=y))
        self.z_publisher.publish(Float32(data=z))
        self.r_publisher.publish(Float32(data=r))

    # Update last known gate position and distance from center
    def update_last_position(self, gate_x, gate_y):
        
        self.last_gate_position = {'x': gate_x, 'y': gate_y}
        distance_x = abs(gate_x - 0.5)
        distance_y = abs(gate_y - 0.5)
        self.last_distance_from_center = distance_x + distance_y
        self.last_move_time = time.time()
    
    # Check if current position is closer to center than last position
    def is_centering_improved(self, current_x, current_y):
        current_distance_x = abs(current_x - 0.5)
        current_distance_y = abs(current_y - 0.5)
        current_total_distance = current_distance_x + current_distance_y
        
        # If this is better than last position, movement was correct
        if current_total_distance < self.last_distance_from_center:
            return True
        elif current_total_distance > self.last_distance_from_center:
            self.get_logger().warn(f"Movement made centering worse! Distance: {current_total_distance:.3f} vs {self.last_distance_from_center:.3f}")
            return False
        return True  # Same distance, assume OK

    # Check if gate is well aligned (centered and proper ratio)
    def check_if_well_aligned(self, gate_msg):
        
        # Check if centered
        x_centered = abs(gate_msg.x - 0.5) <= self.distance_tolerance
        y_centered = abs(gate_msg.y - 0.5) <= self.distance_tolerance
        
        # Check if ratio is good
        ratio_good = (gate_msg.w / gate_msg.h) >= self.min_width_height_ratio
        
        return x_centered and y_centered and ratio_good

    # Track consecutive frames of ratio improvement/worsening
    def update_ratio_trend(self, ratio_improvement):
        if ratio_improvement > 0:
            # Improving this frame
            self.consecutive_improving_frames += 1
            self.consecutive_worsening_frames = 0
            
            if self.consecutive_improving_frames >= self.frames_threshold:
                if self.last_ratio_trend != "improving":
                    self.get_logger().info(f"Confirmed improving trend after {self.consecutive_improving_frames} frames")
                self.last_ratio_trend = "improving"
        else:
            # Worsening this frame
            self.consecutive_worsening_frames += 1
            self.consecutive_improving_frames = 0
            
            if self.consecutive_worsening_frames >= self.frames_threshold:
                if self.last_ratio_trend != "worsening":
                    self.get_logger().info(f"Confirmed worsening trend after {self.consecutive_worsening_frames} frames")
                self.last_ratio_trend = "worsening"
        
        return self.last_ratio_trend

    # Reset all ratio tracking when starting new alignment
    def reset_ratio_tracking(self): 
        self.consecutive_improving_frames = 0
        self.consecutive_worsening_frames = 0
        self.last_ratio_trend = None

    # Check for red flare blocking the path
    def check_red_flare_blocking(self, bounding_boxes_msg):
        for box in bounding_boxes_msg.bounding_boxes:
            if box.label_name == "red_flare" and box.w > 0.015:
                flare_center_x = box.x
                flare_width = box.w
                self.get_logger().info(f"Red flare detected (ID: {box.label_id}, name: {box.label_name}) at x: {flare_center_x:.3f}, width: {flare_width:.4f}")
                
                # Check if flare is large enough and blocking the path
                    self.get_logger().warn(f"Red flare blocking path at x: {flare_center_x:.3f}, width: {flare_width:.4f}")
                    return True, flare_center_x
                elif flare_width > 0.022 and 0.05 <= flare_center_x <= 0.95:
                    self.get_logger().warn(f"Red flare potentially blocking path at x: {flare_center_x:.3f}, width: {flare_width:.4f}")
                    return True, flare_center_x
                else:
                    if flare_width <= 0.018:
                        self.get_logger().info(f"Red flare too small to avoid (width: {flare_width:.4f})")
                    else:
                        self.get_logger().info(f"Red flare detected but not blocking (x: {flare_center_x:.3f})")
                    return False, flare_center_x
        
        return False, None

    def avoid_red_flare(self, flare_x):
        # Determine which side to avoid based on flare position
        if flare_x < 0.5:
            # Flare on left side, move right to avoid
            self.get_logger().info("Red flare on left, moving right to avoid.")
            self.publish_velocity(1.0, -1.0, 0.0, 0.0)  # Move right while forward
        else:
            # Flare on right side, move left to avoid
            self.get_logger().info("Red flare on right, moving left to avoid.")
            self.publish_velocity(0.0, 1.0, 0.0, 0.0)  # Move left
        
    # Track consecutive detections and no-detections to filter noise
    def update_detection_tracking(self, gate_detected, gate_msg=None):
        if gate_detected:
            self.consecutive_detections += 1
            self.consecutive_no_detections = 0
            
            # Store the gate message for confirmed detection
            if gate_msg is not None:
                self.last_confirmed_gate_msg = gate_msg
            
            # Confirm gate presence after threshold consecutive detections
            if self.consecutive_detections >= self.detection_threshold:
                if not self.has_gate:
                    self.get_logger().info(f"Gate confirmed after {self.consecutive_detections} consecutive detections")
                self.has_gate = True
                return True
        else:
            self.consecutive_no_detections += 1
            self.consecutive_detections = 0
            
            # Confirm gate loss only after threshold consecutive no-detections
            if self.consecutive_no_detections >= self.no_detection_threshold:
                if self.has_gate:
                    self.get_logger().info(f"Gate loss confirmed after {self.consecutive_no_detections} consecutive no-detections")
                self.has_gate = False
                self.last_confirmed_gate_msg = None
                return False
        
        # Return current state if thresholds not met
        return self.has_gate

    # main control loop
    def listener_callback(self, bounding_boxes_msg):
        gate_msg = None
        gate_detected_this_frame = False
        
        # Check for gate detection in current frame
        for box in bounding_boxes_msg.bounding_boxes:
            if box.label_name == "gate":
                gate_msg = box
                gate_detected_this_frame = True
                self.get_logger().info("Gate detected at center x: {}, center y: {}".format(gate_msg.x, gate_msg.y))
                self.get_logger().info("Gate detected at width: {}, height: {}".format(gate_msg.w, gate_msg.h))
                break
        
        # Update detection tracking with noise filtering
        confirmed_has_gate = self.update_detection_tracking(gate_detected_this_frame, gate_msg)
        
        # Check for red flare blocking the path
        is_flare_blocking, flare_x = self.check_red_flare_blocking(bounding_boxes_msg)
        
        if is_flare_blocking:
            # Red flare is blocking, avoid it
            self.avoid_red_flare(flare_x)
            return  # Skip all other logic while avoiding flare
        
        # If we're close to the gate (height > 0.7), only move forward
        if self.is_close_to_gate:
            self.get_logger().info("Close to gate - moving forward only, ignoring alignment.")
            self.publish_velocity(1.0, 0.0, 0.0, 0.0)  # Move forward only
            return  # Skip all other logic
        
        # Only process gate logic with NEW detections (not cached ones)
        if confirmed_has_gate and gate_msg is not None:
            # Update position tracking
            self.update_last_position(gate_msg.x, gate_msg.y)
            
            # Check if gate is well aligned
            if self.check_if_well_aligned(gate_msg):
                if not self.is_well_aligned:
                    # First time detecting well alignment
                    self.is_well_aligned = True
                    self.alignment_start_time = time.time()
                    self.get_logger().info("Gate is well aligned! Starting forward movement.")
                
                # Check if gate height is greater than 0.7 ONLY after well aligned
                if gate_msg.h > 0.7:
                    if not self.is_close_to_gate:
                        self.is_close_to_gate = True
                        self.close_gate_start_time = time.time()
                        self.get_logger().info("Gate height > 0.7 after alignment - Very close! Entering forward-only mode.")
                
                # Continue moving forward - no more adjustments needed
                self.get_logger().info("Gate remains well aligned, continuing forward.")
                self.publish_velocity(1.0, 0.0, 0.0, 0.0) # Move forward
                return  # Skip all other alignment checks
            else:
                # Gate is no longer well aligned, reset state
                if self.is_well_aligned:
                    self.get_logger().info("Gate alignment lost, resuming adjustment mode.")
                self.is_well_aligned = False
                self.alignment_start_time = None
            
            self.publish_velocity(0.0, 0.0, 0.0, 0.0) # stop previous action
            
            # Check if previous movement improved centering
            if time.time() - self.last_move_time > 0.5:
                if not self.is_centering_improved(gate_msg.x, gate_msg.y):
                    self.get_logger().warn("Previous movement didn't improve centering!")

            # Alignment logic (only runs when not well aligned)
            if gate_msg.y < 0.5 - self.distance_tolerance:
                self.get_logger().info("Gate is above, moving up.")
                self.publish_velocity(0.0, 0.0, 1.0, 0.0) # Move up
            elif gate_msg.y > 0.5 + self.distance_tolerance:
                self.get_logger().info("Gate is below, moving down.")
                self.publish_velocity(0.0, 0.0, -1.0, 0.0) # Move down

            elif gate_msg.x < 0.5 - self.distance_tolerance:
                self.get_logger().info("Gate is at the left, rotating left.")
                self.publish_velocity(0.0, 0.0, 0.0, 0.5) # Rotate left
            elif gate_msg.x > 0.5 + self.distance_tolerance:
                self.get_logger().info("Gate is at the right, rotating right.")
                self.publish_velocity(0.0, 0.0, 0.0, -0.5) # Rotate right

            elif (gate_msg.w / gate_msg.h) < self.min_width_height_ratio:
                current_width_height_ratio = gate_msg.w / gate_msg.h
                self.get_logger().info(f"Current width/height ratio: {current_width_height_ratio:.3f}")
                
                # Do ratio alignment without distance adjustment
                if self.last_width_height_ratio is None:
                    # First time - try moving right and reset tracking
                    self.reset_ratio_tracking()
                    self.last_width_height_ratio = current_width_height_ratio
                    self.publish_velocity(0.0, -0.5, 0.0, 0.0)  # Move right
                    self.get_logger().info("First alignment attempt - moving right.")
                    self.last_move = "right"
                else:
                    # Calculate ratio improvement
                    ratio_improvement = current_width_height_ratio - self.last_width_height_ratio
                    
                    self.get_logger().info(f"Ratio change: {ratio_improvement:.3f} (positive = improving)")
                    
                    # Update trend tracking with frame persistence
                    confirmed_trend = self.update_ratio_trend(ratio_improvement)
                    
                    self.get_logger().info(f"Frame counters - Improving: {self.consecutive_improving_frames}, Worsening: {self.consecutive_worsening_frames}")
                    
                    # Make decisions based on confirmed trends
                    if confirmed_trend == "improving":
                        # Confirmed improving trend, continue same direction
                        if self.last_move == "right":
                            self.publish_velocity(0.5, -1.0, 0.0, 0.0)  # Continue right
                            self.get_logger().info("Confirmed improving trend - continuing right.")
                        else:  # last_move == "left"
                            self.publish_velocity(0.5, 1.0, 0.0, 0.0)  # Continue left
                            self.get_logger().info("Confirmed improving trend - continuing left.")
                            
                    elif confirmed_trend == "worsening":
                        # Confirmed worsening trend, reverse direction
                        if self.last_move == "right":
                            self.publish_velocity(0.0, 1.0, 0.0, 0.0)  # Switch to left
                            self.get_logger().info("Confirmed worsening trend - switching to left.")
                            self.last_move = "left"
                            self.reset_ratio_tracking()  # Reset counters after switch
                        else:  # last_move == "left"
                            self.publish_velocity(0.0, -1.0, 0.0, 0.0)  # Switch to right
                            self.get_logger().info("Confirmed worsening trend - switching to right.")
                            self.last_move = "right"
                            self.reset_ratio_tracking()  # Reset counters after switch
                            
                    else:
                        # No confirmed trend yet, continue current direction with gentle movement
                        if self.last_move == "right":
                            self.publish_velocity(0.0, -0.3, 0.0, 0.0)  # Gentle right
                            self.get_logger().info("No confirmed trend - gentle right movement.")
                        else:  # last_move == "left"
                            self.publish_velocity(0.0, 0.3, 0.0, 0.0)  # Gentle left
                            self.get_logger().info("No confirmed trend - gentle left movement.")
        
                # Update last ratio for next comparison
                self.last_width_height_ratio = current_width_height_ratio
        
        elif confirmed_has_gate and gate_msg is None:
            # Using cached detection due to noise - do nothing, keep stationary
            self.get_logger().info("Detection noise - staying stationary.")
            self.publish_velocity(0.0, 0.0, 0.0, 0.0)

        else:
            # No gate detected (confirmed after threshold)
            if self.is_close_to_gate:
                # If we were close to gate but now don't see it, continue forward (passed through)
                self.get_logger().info("Was close to gate, now no detection - continuing forward (passed through).")
                self.publish_velocity(1.0, 0.0, 0.0, 0.0)  # Continue forward
                return
            
            # Reset alignment state when no gate detected and not close
            self.is_well_aligned = False
            self.alignment_start_time = None
            self.reset_ratio_tracking()  # Reset ratio tracking when no gate
            
            self.publish_velocity(0.0, 0.0, 0.0, 0.0) # stop previous action
            self.get_logger().info("No gate detected (confirmed).")
            self.get_logger().info("Searching for gate.")

            if self.moved_depth > self.max_depth:
                self.publish_velocity(0.0, 0.0, -1.0, 0.5) # Move down and rotate to search for gate
                self.moved_depth += -0.01
            else:
                self.publish_velocity(0.0, 0.0, 1.0, 0.5)
                self.moved_depth += 0.01

    # Placeholder for control loop (not used since logic is in listener_callback)
    def control_loop(self):
        pass

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