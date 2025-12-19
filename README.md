

# Warehouse Bot Core - Autonomous Mobile Robot (AMR)

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue) ![Python](https://img.shields.io/badge/Python-3.10-yellow) ![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-orange) ![Nav2](https://img.shields.io/badge/Navigation-Nav2-green) ![OpenCV](https://img.shields.io/badge/Vision-OpenCV-red) ![YOLOv8](https://img.shields.io/badge/AI-YOLOv8-purple)

**Warehouse Bot Core** is a scalable and modular autonomous robot software architecture designed for industrial warehouse logistics.

The current phase of the project (**Phase 1.5**) integrates **Deep Learning-based Perception (YOLOv8)** with continuous patrol capabilities. The system is built on **ROS 2 Humble**, utilizing the **Nav2** stack for mobility and a deterministic **Finite State Machine (FSM)** for mission management.

<table style="width:100%">
  <tr>
    <th width="25%">Phase 1: Navigation  </th>
  </tr>
  <tr>
    <td><img src="/docs/phase1.gif" width="100%"></td>
  </tr>
  <tr>
    <th width="%25">Phase 1.5: Vision & Scan Beavior </th>
  </tr>
  <tr>
    <td><img src="/docs/phase1_5.gif" width="100%"></td>
  </tr>
  
</table>

## System Architecture

The project features a multi-layered architecture that bridges **Enterprise Level Control** with **Low-Level Hardware Abstraction**.

![System Architecture](docs/ware_house_workflow.drawio.png)

The architecture consists of 4 main layers:
1.  **Enterprise Layer:** (Future) WebUI and SQL-based fleet management layer.
2.  **Robot Brain:** Decision support mechanism. Currently executed via a *Logic Node*, to be integrated with *Behavior Trees* in future iterations.
3.  **ROS2 Skills:** Modular services for Navigation, Manipulation, and Perception.
4.  **Simulation/Hardware:** Gazebo simulation or physical robot sensor interfaces.

## Technical Implementation

### 1. Robust Localization Handshake
To prevent "Race Condition" errors during AMCL (Adaptive Monte Carlo Localization) startup, a custom **Subscriber-Aware Initialization** algorithm has been developed. The robot does not broadcast its initial pose until it detects a subscriber listening to the `/initialpose` topic.

$$
P_{init} = \{x=0.0, y=0.0, \theta=0.0\}_{map}
$$

This process prevents Nav2 costmap layers (`static_layer`, `obstacle_layer`) from crashing before the TF tree (`map` -> `odom` -> `base_link`) is fully established.

### 2. Configuration Management
The robot's patrol route is completely decoupled from the source code. Coordinates are loaded dynamically in `YAML` format.

* **Config Path:** `src/warehouse_autonomy/config/patrol_config.yaml`
* **Structure:**
    ```yaml
    /security_patrol_node:
      ros__parameters:
        waypoint__x: [1.45, -3.50, 5.63]
        waypoint__y: [6.64, -2.88, 2.07]
    ```

### 3. Perception Pipeline (YOLOv8 Integration)
The robot is equipped with a "Semantic Understanding" layer to detect objects in the warehouse environment.

* **Pipeline Architecture:** `Gazebo (Sim)` -> `sensor_msgs/Image` -> `cv_bridge` -> `OpenCV Matrix` -> `YOLOv8 Inference` -> `Annotated Frame`
* **Zero-Latency Bridge:** The integration uses an asynchronous subscriber with `SensorDataQoS` (Best Effort reliability) to ensure high-bandwidth video streaming without blocking the navigation stack.
* **Object Detection:** Currently utilizes the `yolov8n` (Nano) model for real-time inference on CPU.

### 4. Finite State Machine (FSM)
Unlike simple loops, the robot's behavior is governed by a robust FSM loop running at 2Hz. This prevents "Command Spamming" to the navigation stack and allows for interruptible behaviors.

**States:**
1.  **IDLE:** Waiting for initialization.
2.  **NAVIGATING:** Moving to the next waypoint (Async Action Client).
3.  **SCANNING:** Rotating 360° to inspect the area (Behavior Tree Spin).

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> NAVIGATING : Start Mission
    NAVIGATING --> SCANNING : Waypoint Reached
    NAVIGATING --> NAVIGATING : Retry (If Failed)
    SCANNING --> NAVIGATING : Scan Complete (Next WP)
    NAVIGATING --> [*] : All Waypoints Done
```

## 📥 Installation

### Prerequisites
The following system requirements must be met to run the project:
* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Middleware:** ROS 2 Humble Hawksbill
* **Simulation:** Gazebo Classic
* **Language:** Python 3.10+

### Clone Repository
Navigate to the `src` directory of your workspace and clone the project:

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone [https://github.com/mertaren/warehouse_bot_core.git](https://github.com/mertaren/warehouse_bot_core.git)
```

### Install Dependencies

```bash
cd ~/robot_ws
# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y
# Install AI/Cv dependencies
pip install ultralytics opencv-python
```

### Part 2: Build Instructions

```markdown
Use the `colcon` tool to build the project. Symlink installation allows changes in Python files to take effect without recompiling.
```

1.  **Build the packages:**
```bash
cd ~/robot_ws
colcon build --packages-select warehouse_autonomy --symlink-install
```

2.  **Source the environment:**
You must run this command in every new terminal you open:
```bash
source install/setup.bash
 ```
## Usage

Launch the entire system (Gazebo, Nav2, Security Node) with a single command:

```bash
ros2 launch warehouse_autonomy start_all.launch.py
```

### Part 4: Roadmap & Footer

## Roadmap

## Roadmap

- [x] **Phase 1: Navigation & Architecture**
  - Master Launch file & Robust AMCL initialization.
  - YAML based dynamic configuration.
  - Finite State Machine implementation.
- [x] **Phase 1.5: Perception Layer**
  - ROS2 <-> OpenCV Bridge implementation.
  - YOLOv8 Real-time Object Detection integration.
  - "Scan & Secure" behavior logic.
- [ ] **Phase 2: Manipulation (MoveIt 2)**
  - Integration of OpenMANIPULATOR-X URDF.
  - Pick & Place pipeline implementation.
- [ ] **Phase 3: Enterprise Integration**
  - SQL Database connection for inventory tracking.
  - Web Interface for fleet control.

---
*Maintained by [Mert Aren](https://github.com/mertaren)*
