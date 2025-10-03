# Probation Task: Going Through Gate with Unity Simulation
# Guide and explanation for my solution

## 1. How to Run
- Navigate to UnitySim_Linux, then run the simulation
```bash
./UnitySim.x86_64
```
- Clone the repo
```bash
git clone https://github.com/humjie/mecatron_probation_ws.git
cd mecatron_probation_ws
```
- Source
```bash
source /opt/ros/humble/setup.bash
source /install/local_setup.bash
```
- Colcon build
```bash
colcon build
```
- run the launch file
```bash
ros2 launch launch_all launch_all.py
```

## 2. Overview
The **GateNavigationController** in **control** package is a ROS2 node designed to autonomously navigate a drone through gates while avoiding red flare obstacles. The solution implements a state-based control system with robust detection filtering and intelligent movement strategies.

## 3. Chain of Thoughts & Design Philosophy
### 1. Problem Analysis
**Primary Goal:** Navigate through gates while maintaining proper alignment

**Secondary Goal:** Avoid red flare obstacles that block the path


**Challenges:**
- Noisy vision detection requiring filtering
- Need for smooth, stable movement without much oscillations
- State management for different phases of navigation

### 2. Workspace Structure
```
probation_ws/
├── src/
│   ├── ROS-TCP-Endpoint/          # Unity-ROS bridge
│   └── vision/
│       └── vision_msgs/           # Custom message definitions
│   └── control/
│       └── control/
│           └── control.py         # GateNavigationController node
│   └── launch_all/
│       └── launch/
│           └── launch_all.py      # Main launch file
```

### 3. Solution Architecture
**Core Control Flow**
```
Vision Input → Detection Filtering → Obstacle Avoidance → Gate Alignment → Navigation
```


**State Management**

The controller uses multiple state variables to track:

- **Detection State:** has_gate, consecutive_detections, consecutive_no_detections
- **Alignment State:** is_well_aligned, alignment_start_time
- **Proximity State:** is_close_to_gate, close_gate_start_time
- **Movement State:** last_move, last_ratio_trend


**Control Priorities (High to Low)**

- Red Flare Avoidance (Highest Priority)
- Close Proximity Mode (Forward-only when height > 0.7)
- Well-Aligned Mode (Forward movement when centered)
- Alignment Mode (Y → X → Width and Height Ratio correction)
- Search Mode (When no gate detected)

### 4. Technical Implementation Details
**Vision Message Processing**

The controller subscribes to **BoundingBoxArray** messages and processes:

- **Gate Detection:** Identifies gates and extracts position/dimensions

- **Red Flare Detection:** Identifies blocking obstacles with size thresholds


**Noise Filtering Strategy**
```
detection_threshold = 3      # Confirm gate presence
no_detection_threshold = 5   # Confirm gate absence
frames_threshold = 3         # Confirm movement trends
```


**Velocity Publishing**

The controller publishes to separate velocity topics:

- **x:** Forward/backward movement
- **y:** Left/right movement
- **z:** Up/down movement
- **r:** Rotation (yaw)


**Obstacle Avoidance Strategy**
- **Left Flare:** Move right while continuing forward
- **Right Flare:** Move left while maintaining progress


**Multi-Stage Alignment Process**
- **Y-Axis Alignment:** Vertical centering (up/down movement)
- **X-Axis Alignment:** Horizontal centering (rotation)
- **Ratio Optimization:** Lateral movement for optimal gate perspective


**Performance Optimizations**
**1. Adaptive Movement Speeds**
- **Gentle Movements:** 0.3 velocity for uncertain conditions
- **Standard Movements:** 0.5-1.0 velocity for confirmed actions
- **Combined Movements:** Forward + lateral for efficient flare avoidance
- 
**2. Frame Persistence**
- Prevents oscillatory behavior from single-frame noise
- Ensures stable trend confirmation before direction changes
- 
**3. Position Tracking**
- Validates that movements improve centering
- Provides feedback for movement effectiveness
- 
**4. Vision System Noise**
- Consecutive frame requirements for state changes
- Cached last known gate position during noise periods
- 
**5. Ratio Alignment Challenges**
- Trend-based movement with direction reversal capability
- Gentle movements when trend is uncertain
- 
**6. Gate Loss During Navigation**
- Continues forward if previously close to gate (passed through)
- Initiates search pattern if gate lost during approach


**Launch Files**
```bash
ros2 launch launch_all launch_all.py
```
**launch_all.py** is able to launch all the launch files in different package at once, including the **endpoint launch file** and **control launch file**

### 5. Future Improvements
**Potential Enhancements**

- **Adaptive Thresholds:** Dynamic tolerance based on gate distance
- **Predictive Movement:** Anticipate gate movement from velocity
- **Machine Learning:** Learn optimal parameters from successful runs
- **Multi-Gate Planning:** Handle sequences of gates with path planning


**Code Quality Improvements**

- **Modularization:** Extract alignment logic into separate classes
- **Configuration File:** Move parameters to external config
- **Unit Testing:** Add comprehensive test coverage
- **Logging Enhancement:** Structured logging with levels

### 6. Conclusion
This solution provides a robust, state-based approach to autonomous gate navigation with obstacle avoidance. The key innovations are:

- Frame persistence for noise filtering
- Trend-based movement for stable alignment
- Hierarchical state management for complex behaviors
- Comprehensive edge case handling
- Main launch file that include different packages' launch file

The system prioritizes safety and stability over speed, ensuring reliable navigation through challenging environments with noisy sensor data.





# Below is the part forked from original repo
# Probation Task: Going Through Gate with Unity Simulation

This repository contains the probation task, focusing on autonomous gate navigation using Unity simulation integrated with ROS 2.

**You should NOT clone this repository from Mecatron github organization directly.** Instead, you should **fork** this repository to your own github and clone from it *(Please ask ChatGPT how to fork a github repository if you are unsure)*. After you fork and clone the repository, you should be on branch `probation/task`. If you are not on this branch, please switch to it using:
```bash
git checkout probation/task
```

You are supposed to implement your solution in this branch `probation/task` in either Python or C++. After you finish the task, please send us the link to your forked repository.

## 1. Problem Statement

**Task Goal**: Navigate an autonomous underwater vehicle (AUV) through a gate in a simulated environment.

**Learning Objectives**:
- Working with a simulation using ROS2
- Process object detection data for meaningful insights
- Develop autonomous decision-making and control algorithms
- Manage a relatively large projects with many processes and nodes.

**Success Criteria**:
- The vehicle navigates autonomously through the gate 3 times, at 3 different random initial positions.

## 2. Setup and Dependencies

### Workspace Structure
```
probation_ws/
├── src/
│   ├── ROS-TCP-Endpoint/          # Unity-ROS bridge
│   └── vision/
│       └── vision_msgs/           # Custom message definitions
```

### Key Components

- **[ROS-TCP-Endpoint](src/ROS-TCP-Endpoint)**: Bridge between Unity simulation and ROS 2
- **[vision_msgs](src/vision/vision_msgs)**: Custom message types for bounding box data
  - [`BoundingBox.msg`](src/vision/vision_msgs/msg/BoundingBox.msg): Single detection with bounding box information (x, y, w, h), confidence, label name and label id.

    ![Image of BoundingBox](docs/images/bounding_box_description.png)

  - [`BoundingBoxArray.msg`](src/vision/vision_msgs/msg/BoundingBoxArray.msg): Array of detections with header

### Prerequisites

1. **ROS 2 Humble** - Full desktop installation
2. **MAVROS** - For vehicle communication and control
   ```bash
   sudo apt install ros-$ROS_DISTRO-mavros
   ```
3. **Unity Simulation** - Provided simulation environment
4. **Foxglove Bridge** - For monitoring various states of the vehicle
   ```bash
   sudo apt install ros-$ROS_DISTRO-foxglove-bridge
   ```

### Installation

0. Fork the repository to your own github account.

1. Clone and build the workspace:
   ```bash
   cd ~
   git clone your_forked_repo_url
   cd probation_ws
   colcon build --symlink-install
   source install/local_setup.bash
   ```

2. Additional setup:

   To avoid repeatedly sourcing the workspace, you may run the following command to add source to `~/.bashrc` file:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/probation_ws/install/local_setup.bash" >> ~/.bashrc
   ```

If you are unsure about how to run your ROS2 implementation with Unity simulation, please refer to the [Appendix 5.2](#52-proper-setup-flow) for a proper setup flow.

## 3. Things to Note About the Simulation

### Simulation Characteristics

- **Random Initial Position**: The vehicle spawns at a random location and orientation
- **Imperfect Detection**: Objects are only detected approximately 70% of the time
- **Flight Mode Requirement**: Vehicle must be in GUIDED mode for autonomous control via topics. To switch back to control by keyboard, set mode to `ALT_HOLD`.

### Vision System

- Bounding boxes are published from Unity simulation with message type [`vision_msgs/BoundingBoxArray`](src/vision/vision_msgs/msg/BoundingBoxArray.msg)
- Coordinates are normalized (0.0-1.0) relative to image frame

## 4. Suggested Logic Build-up

To support your implementation, here is a suggested logic flow:

1. Implement client to change vehicle to GUIDED mode
2. Move down until reaching target depth for gate visibility
3. Implement search pattern to locate gate
4. Center the gate and moving forward to approach
5. Go straight through the gate

The logic flow above is one of many possible solutions. Feel free to explore and implement your own strategies.

> **NOTE:**
> The vehicle's initial position may sometimes face obstacles. If this occurs, please refer the [Appendix 5.1](#51-obstacle-avoidance-note) for more details.

## 5. Appendix

### 5.1. Obstacle Avoidance Note

In the gate area of the simulation, there is a orange flare in front of the gate which acts as an obstacle. In this case, you may choose to implement obstacle avoidance logic if you wish, and of course it would be a bonus point. However, it is not a requirement for the probation task. 

If you choose not to implement obstacle avoidance, you may reset the simulation if the vehicle's initial position faces an obstacle.

> **NOTE:**  
> We advise you to focus on the main task, which is to go through the gate without obstacle avoidance.  If there is time left, you may then implement obstacle avoidance logic.

### 5.2. Proper Setup Flow

To set up and run the simulation with ROS2 properly, follow these steps before starting your implementation:

1. **Build Workspace**:
   ```bash
   cd probation_ws
   colcon build
   source install/setup.bash
   ```

2. **Start ROS TCP Endpoint**:
   ```bash
   ros2 run ros_tcp_endpoint default_server_endpoint
   ```
   The endpoint will start on `0.0.0.0:10000` by default.

3. **Launch Unity Simulation**:
   - Open the Unity simulation project. If you have not installed the simulation, please refer to our workshop notion page for the download link:
        [Notion Page](https://mecatron.notion.site/ros2)
   - Start the simulation

4. **Verify Communication**:
   ```bash
   # Check available topics
   ros2 topic list
   
   # Check MAVROS connection
   ros2 topic echo /mavros/state
   ```

5. You may now start your ROS2 implementation to control the vehicle.

### 5.3. Notes to Avoid Confusion
#### Publishers

In the workshop on Saturday, when working on the `minimal_publisher.py` file, we created a timer to call the timer_callback function every `0.5` seconds. The function publishes a `Float32` message with a value of `0.5` to the topic `/mavros/setpoint_velocity/cmd_vel_unstamped/x` at line 18, by:
```python
self.publisher_.publish(msg)
```

**Important Notes:**

* In order to publish, you don't need to have a timer
* You can call it anywhere: in a subscriber callback, service callback, or even in the class constructor (__init__).
* The timer is just a convenient way to call a function periodically.

### 5.4. Useful Reference

#### Useful commands
```bash
# Monitor system status
ros2 topic echo /mavros/state

# Check available topics, services
ros2 topic list
ros2 service list

# Set vehicle mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"

```

#### Useful links

- [ROS2 Official Tutorials](https://docs.ros.org/en/humble/Tutorials.html)

---

**Good luck with your probation task!** Focus on understanding the integration between simulation, vision processing, and vehicle control. The key is building a robust system that handles the imperfect nature of real-world sensing and control.


