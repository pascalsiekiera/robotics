## HOW TO RUN
cd code
python3 IR/mp1/scripts/read_rosbag.py turtlebot3_datasets/data/slam_easy


## REPO STRUCTURE
code
|
-- IR
|   |
|   -- mp1
|   |
|   -- mp2
|
-- turtlebot3_datasets


## NOTES
Q groß, R klein: EKF folgt der Odometrie mehr, ignoriert die Messungen
Q klein, R groß: EKF vertraut voll den Messungen, ignoriert die Odometrie
Q und R beide klein: Gut ausbalanciert

Messungen sichtbarer machen mit Hinzufuegen von Rauschen

# Task 1a: Extended Kalman Filter (EKF) Implementation

## Objective

Implement an Extended Kalman Filter to estimate the robot's position and orientation using odometry, IMU data, and ground-truth measurements from a recorded dataset.

## What is the Problem?

A robot moves through an environment and needs to know where it is. It has several sensors:
- **Odometry**: Wheel encoders measuring movement (displacement, rotation)
- **IMU**: Accelerometer and gyroscope measuring acceleration and angular velocity
- **Laser Scanner**: Measures distances to obstacles (used in later tasks)

The challenge: **all sensor measurements are noisy and imperfect**. Wheel encoders slip, the IMU drifts over time, and laser reflections can be misleading. If you trust only one sensor, the robot becomes hopelessly lost after a few meters.

## Solution: Extended Kalman Filter (EKF)

The EKF combines multiple sensors intelligently:
- It trusts the odometry **partially**
- It trusts the measurements **partially**
- It weights both based on uncertainty parameters (Q and R)

This produces a better position estimate than relying on any single sensor.

## Why Extended (not just Kalman)?

The standard Kalman Filter works only for **linear systems**. Robot motion is **non-linear** (rotation + translation creates curved paths). The Extended Kalman Filter solves this by linearizing the motion model using Jacobian matrices, allowing the Kalman Filter framework to still apply.

## What We Did

### 1. Extract Sensor Data from Rosbag

We read the recorded sensor data from a TurtleBot3 dataset:
- Odometry measurements (position, orientation)
- IMU measurements (acceleration, angular velocity)
- Ground-truth poses from a motion capture system (used as measurements)

### 2. Implement the EKF

The filter runs two steps alternately:

**Predict Step (Motion Model)**
- Input: linear velocity `v` and angular velocity `omega` from odometry
- Output: predicted next state (position x, y, orientation theta)
- Uncertainty increases because motion is imperfect

**Update Step (Measurement Correction)**
- Input: ground-truth position measurement [x, y]
- Correction: adjust the state based on how well it matches the measurement
- Uncertainty decreases due to sensor feedback

### 3. Tune Parameters

Two noise parameters control the filter's behavior:

**Q (Process Noise Covariance)**
- How much do you trust the motion model?
- Large Q: wheels might slip, don't trust the prediction
- Small Q: motion is predictable, trust the prediction

**R (Measurement Noise Covariance)**
- How much do you trust the measurements?
- Large R: measurements are noisy, don't trust them
- Small R: measurements are accurate, trust them

We tested three configurations:
1. **Q small, R small**: Trust both equally → balanced estimate
2. **Q large, R small**: Trust measurements more → corrects motion errors aggressively
3. **Q small, R large**: Trust motion model more → smooth but may drift

### 4. Results

The EKF successfully estimates the robot's trajectory:
- **Blue line**: Ground-truth odometry (actual path)
- **Red line**: EKF estimate (our filter's output)
- **Error plot**: Position error over time (typically < 2.5 cm)

The red line follows the blue line closely, showing that the EKF combines the sensors effectively.

## How to Run

```bash
# Extract sensor data from rosbag and save as pickle
python3 scripts/read_rosbag.py ~/turtlebot3_datasets/data/slam_easy

# Run the EKF with different parameter configurations
python3 scripts/test_ekf.py

# Output: Plots saved to IR/mp1/figures/
```

## Key Insights

- The EKF is a **fusion algorithm** that combines multiple imperfect sensors
- Parameter tuning (Q and R) is critical - wrong values lead to poor estimates
- The filter works by balancing **prediction** (motion model) and **correction** (measurements)
- This task focuses on **position estimation** using ground-truth; later tasks (gmapping, AMCL) will use laser scanner measurements instead

## Files

- `src/ekf/ekf/ekf_filter.py`: Core EKF implementation
- `scripts/read_rosbag.py`: Extracts sensor data from rosbag
- `scripts/test_ekf.py`: Tests the EKF with different parameter configurations
- `data/rosbag_data.pkl`: Cached sensor data
- `figures/`: Generated plots comparing EKF estimate to ground truth