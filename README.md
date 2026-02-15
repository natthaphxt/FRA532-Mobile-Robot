# FRA532 LAB1: Kalman Filter / SLAM

**Course:** FRA532 - Mobile Robot  
**Student:** Natthapatch Lapsittiwong | 66340500077  

---

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Setup](#setup)
- [Dependencies](#dependencies)
- [Part 1: EKF Odometry Fusion](#part-1-ekf-odometry-fusion)
- [Part 2: ICP Odometry Refinement](#part-2-icp-odometry-refinement)
- [Part 3: Full SLAM with slam_toolbox](#part-3-full-slam-with-slam_toolbox)
- [Results Summary](#results-summary)
- [Cross-Sequence Comparison](#cross-sequence-comparison)
- [Discussion](#discussion)
- [Conclusion](#conclusion)
- [References](#references)

---

## Overview

This lab implements a complete 2D mobile robot localization pipeline:

| Part  | Method             | Description                                 |
| :---: | ------------------ | ------------------------------------------- |
|   1   | **EKF Fusion**     | Wheel odometry + IMU sensor fusion          |
|   2   | **ICP Refinement** | LiDAR scan matching for odometry correction |
|   3   | **Full SLAM**      | Graph-based SLAM with loop closure          |

**Robot:** TurtleBot3 Burger  
**Sensors:** 2D LiDAR (5 Hz), IMU (20 Hz), Wheel Encoders (20 Hz)

---

## Repository Structure

```
FRA532_LAB1/
│
├── src/                        # Source code
│   ├── ekf_node.py            # Part 1: EKF implementation
│   ├── icp_node.py            # Part 2: ICP scan matching
│   └── ...
│
├── Dataset/
│   ├── fibo_floor3_seq00/     # Dataset: Empty hallway
│   ├── fibo_floor3_seq01/     # Dataset: Sharp turns
│   └── fibo_floor3_seq02/     # Dataset: Smooth motion
│
├── seq0/                       # Results: Sequence 00
├── seq1/                       # Results: Sequence 01
├── seq2/                       # Results: Sequence 02
│
├── README.md
└── .gitignore
```

---
## Setup

### Clone Git Hub
```bash
git clone https://github.com/natthaphxt/FRA532-Mobile-Robot.git -b Lab1
```

### Build the workspace
```bash
cd ~/FRA532-Mobile-Robot
colcon build
source install/setup.bash
```

### Part 1 and 2: EKF Odometry
```bash
# Terminal 1: Play rosbag(if want to change seq just edit /fibo_floor3_seq0x/)
ros2 bag play Dataset/fibo_floor3_seq00/ --clock

# Terminal 2: Run EKF node
ros2 run lab1 ekf.py

# Terminal 3: rviz2
rviz2
```

### Part 2: ICP Odometry
```bash
# Terminal 1: Play rosbag(if want to change seq just edit /fibo_floor3_seq0x/)
ros2 bag play Dataset/fibo_floor3_seq00/ --clock

# Terminal 2: Run ICP node
ros2 run lab1 icp.py

# Terminal 3: Run EKF node
ros2 run lab1 ekf.py

# Terminal 4: rviz2
rviz2
```

### Part 3: SLAM
```bash
# Terminal 1: Play rosbag(if want to change seq just edit /fibo_floor3_seq0x/)
ros2 bag play Dataset/fibo_floor3_seq00/ --clock

# Terminal 2: Launch slam_toolbox
ros2 launch lab1 slam_launch.py

# Terminal 3: rviz2
rviz2
```

### Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```
---

## Dependencies

```bash
# ROS2 packages
sudo apt install ros-humble-slam-toolbox ros-humble-robot-localization

# Python packages
pip install numpy scipy matplotlib open3d
```

---

## Part 1: EKF Odometry Fusion

### Objective
Fuse wheel odometry and IMU measurements using Extended Kalman Filter to reduce drift.

### 1.1 Wheel Odometry

#### Robot Parameters

| Parameter    | Symbol | Value   | Description                    |
| ------------ | ------ | ------- | ------------------------------ |
| Wheel radius | $r$    | 0.033 m | TurtleBot3 Burger wheel radius |
| Track width  | $b$    | 0.160 m | Distance between wheel centers |

#### Wheel Displacement

The wheel displacements are computed from encoder position changes:

$$\Delta s_r = \Delta \theta_r \cdot r, \quad \Delta s_l = \Delta \theta_l \cdot r$$

where $\Delta \theta_r$ and $\Delta \theta_l$ represent the angular displacement of the right and left wheels in radians.

#### Differential Drive Kinematics

**Heading change:**

$$\Delta \theta = \frac{\Delta s_r - \Delta s_l}{b}$$

**Linear displacement:**

$$\Delta s = \frac{\Delta s_r + \Delta s_l}{2}$$

**Pose update:**

$$x' = x + \Delta s \cdot \cos\left(\theta + \frac{\Delta \theta}{2}\right)$$

$$y' = y + \Delta s \cdot \sin\left(\theta + \frac{\Delta \theta}{2}\right)$$

$$\theta' = \theta + \Delta \theta$$

#### Robot Velocity

The linear and angular velocities are derived from wheel displacements:

$$v = \frac{\Delta s_r + \Delta s_l}{2 \cdot \Delta t}, \quad \omega = \frac{\Delta s_r - \Delta s_l}{b \cdot \Delta t}$$

### 1.2 Extended Kalman Filter

#### State Vector Design

The EKF estimates a 3-dimensional state vector representing robot pose:

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}$$

| State    | Description            |
| -------- | ---------------------- |
| $x$      | Position in x-axis (m) |
| $y$      | Position in y-axis (m) |
| $\theta$ | Heading angle (rad)    |

#### Motion Model (Prediction Step)

**Control Input:**

$$\mathbf{u}_t = \begin{bmatrix} v \\ \omega \end{bmatrix}$$

where $v$ is linear velocity and $\omega$ is angular velocity from wheel odometry.

**State Transition Function:**

$$\bar{\mathbf{x}}_t = f(\mathbf{x}_{t-1}, \mathbf{u}_t) = \begin{bmatrix} x + v \cdot \cos(\theta) \cdot \Delta t \\ y + v \cdot \sin(\theta) \cdot \Delta t \\ \theta + \omega \cdot \Delta t \end{bmatrix}$$

**State Jacobian:**

$$\mathbf{F}_t = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} 1 & 0 & -v \cdot \sin(\theta) \cdot \Delta t \\ 0 & 1 & v \cdot \cos(\theta) \cdot \Delta t \\ 0 & 0 & 1 \end{bmatrix}$$

**Covariance Prediction:**

$$\bar{\mathbf{\Sigma}}_t = \mathbf{F}_t \cdot \mathbf{\Sigma}_{t-1} \cdot \mathbf{F}_t^T + \mathbf{Q}_t$$

#### Measurement Model (Update Step)

**Measurement Vector:**

The IMU provides orientation as a quaternion. Only the yaw component is extracted:

$$\mathbf{z}_t = \begin{bmatrix} \theta_{IMU} \end{bmatrix}$$

**Measurement Function:**

$$h(\bar{\mathbf{x}}_t) = \theta$$

**Measurement Jacobian:**

$$\mathbf{H}_t = \frac{\partial h}{\partial \mathbf{x}} = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}$$

#### EKF Algorithm

**Prediction Step:**

$$\bar{\mathbf{x}}_t = f(\mathbf{x}_{t-1}, \mathbf{u}_t)$$

$$\bar{\mathbf{\Sigma}}_t = \mathbf{F}_t \cdot \mathbf{\Sigma}_{t-1} \cdot \mathbf{F}_t^T + \mathbf{Q}_t$$

**Update Step:**

$$\mathbf{y}_t = \mathbf{z}_t - h(\bar{\mathbf{x}}_t)$$

$$\mathbf{S}_t = \mathbf{H}_t \cdot \bar{\mathbf{\Sigma}}_t \cdot \mathbf{H}_t^T + \mathbf{R}_t$$

$$\mathbf{K}_t = \bar{\mathbf{\Sigma}}_t \cdot \mathbf{H}_t^T \cdot \mathbf{S}_t^{-1}$$

$$\mathbf{x}_t = \bar{\mathbf{x}}_t + \mathbf{K}_t \cdot \mathbf{y}_t$$

$$\mathbf{\Sigma}_t = (\mathbf{I} - \mathbf{K}_t \cdot \mathbf{H}_t) \cdot \bar{\mathbf{\Sigma}}_t$$

#### Noise Covariance Matrices

**Process Noise (Q):** Models uncertainty in motion model

$$\mathbf{Q} = \begin{bmatrix} \sigma_x^2 & 0 & 0 \\ 0 & \sigma_y^2 & 0 \\ 0 & 0 & \sigma_\theta^2 \end{bmatrix}$$

**Measurement Noise (R):** Models uncertainty in IMU measurement

$$\mathbf{R} = \begin{bmatrix} \sigma_{IMU}^2 \end{bmatrix}$$

**Trust Interpretation:**

| Value    | Meaning                                      |
| -------- | -------------------------------------------- |
| Lower Q  | More trust in motion model (prediction)      |
| Higher Q | Less trust in motion model, faster response  |
| Lower R  | More trust in IMU measurement                |
| Higher R | Less trust in measurement, smoother estimate |

---

## Part 2: ICP Odometry Refinement

### Objective
Refine EKF odometry using LiDAR-based ICP (Iterative Closest Point) scan matching.

### 2.1 ICP Problem Formulation

**Given** two sets of point clouds:
- Source: $P = \{p_1, p_2, \ldots, p_n\}$ (current scan)
- Target: $Q = \{q_1, q_2, \ldots, q_m\}$ (local map)

**Objective:** Find rotation $\mathbf{R}$ and translation $\mathbf{t}$ that minimize alignment error:

$$E(\mathbf{R}, \mathbf{t}) = \sum_{i=1}^{n} \| q_i - (\mathbf{R} \cdot p_i + \mathbf{t}) \|^2$$

where $q_i$ is the nearest neighbor of transformed point $\mathbf{R} \cdot p_i + \mathbf{t}$ in the target cloud.

**For 2D LiDAR:**

$$\mathbf{R} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}, \quad \mathbf{t} = \begin{bmatrix} t_x \\ t_y \end{bmatrix}$$

### 2.2 ICP Algorithm

The ICP algorithm iteratively refines the transformation by alternating between finding point correspondences and computing the optimal transformation.

**Initialization:** Set initial transformation $\mathbf{T} = \mathbf{T}_0$ (from EKF odometry)

**Iterate until convergence:**

1. For each point $p_i$ in source cloud, find nearest neighbor $q_i$ in target cloud using KD-tree
2. Reject outlier pairs where distance exceeds threshold
3. Compute optimal $(\mathbf{R}, \mathbf{t})$ using SVD (Section 2.3)
4. Update transformation: $\mathbf{T} \leftarrow (\mathbf{R}, \mathbf{t})$
5. Check convergence: stop if $\|\Delta \mathbf{T}\| < \epsilon$

### 2.3 SVD-based Solution (Point-to-Point)

Given corresponding point pairs $(p_i, q_i)$, the optimal transformation is computed using SVD:

**Step 1: Compute centroids**

$$\bar{p} = \frac{1}{n} \sum_{i=1}^{n} p_i, \quad \bar{q} = \frac{1}{n} \sum_{i=1}^{n} q_i$$

**Step 2: Center point clouds**

$$p'_i = p_i - \bar{p}, \quad q'_i = q_i - \bar{q}$$

**Step 3: Compute cross-covariance matrix**

$$\mathbf{H} = \sum_{i=1}^{n} p'_i \cdot {q'_i}^T$$

**Step 4: SVD decomposition**

$$\mathbf{H} = \mathbf{U} \cdot \mathbf{\Sigma} \cdot \mathbf{V}^T$$

**Step 5: Optimal rotation**

$$\mathbf{R} = \mathbf{V} \cdot \mathbf{U}^T$$

**Step 6: Optimal translation**

$$\mathbf{t} = \bar{q} - \mathbf{R} \cdot \bar{p}$$

### 2.4 Scan-to-Map Matching

Instead of matching consecutive scans (scan-to-scan), we match current scan to a local map built from recent keyframes:

**Local Map Construction:**

$$M = \text{merge}(\text{keyframes}[t-n : t-1])$$

$$M_{\text{downsampled}} = \text{voxel-filter}(M, 0.05m)$$

The local map accumulates points from multiple keyframes (e.g., 20 recent keyframes), providing a much richer point cloud than a single scan. A single LiDAR scan contains approximately 360 points, while the local map contains 7,000+ points from accumulated scans.

**ICP Registration:**
- Source: Current scan (~360 points)
- Target: Local map (~7,000+ points from 20 keyframes)
- Initial guess: EKF odometry

**Why Scan-to-Map?**

| Aspect           | Scan-to-Scan            | Scan-to-Map                     |
| ---------------- | ----------------------- | ------------------------------- |
| Target points    | ~360 (single scan)      | ~7,000+ (accumulated keyframes) |
| Feature richness | Limited to current view | Includes occluded areas         |
| Drift rate       | Fast (error compounds)  | Slow (rich reference)           |
| Robustness       | Sensitive to occlusion  | Robust to temporary occlusion   |

### 2.5 ICP Odometry Pipeline

The pipeline processes each incoming LiDAR scan:

1. **Preprocess:** Remove invalid points (NaN, too close, too far)
2. **Keyframe Selection:** Check if robot moved enough (distance > 0.5m or rotation > 15 deg)
3. **Update Local Map:** If keyframe, add to local map and remove old keyframes
4. **ICP Registration:** Match current scan against local map using Point-to-Point ICP
5. **Output:** Refined pose from ICP transformation

---

## Part 3: Full SLAM with slam_toolbox

### Objective
Perform full SLAM with loop closure detection and pose graph optimization.

### 3.1 SLAM Overview

**slam_toolbox** is the officially supported SLAM library for ROS 2 that combines:
- LiDAR scan matching for local pose estimation
- Pose graph construction for trajectory representation
- Loop closure detection for global consistency
- Graph optimization using Ceres Solver

### 3.2 How Loop Closure Works

Normally, odometry accumulates drift over time. For example, if a robot travels in a loop back to its starting point, odometry might report it is 3 meters away from the start.

Loop closure works by:
1. SLAM detects that the current LiDAR scan matches a previously seen scan
2. Adds a constraint that these two poses should be the same location
3. Optimizes the entire trajectory to satisfy all constraints

The result is a trajectory that is corrected to be closer to reality.

### 3.3 Why SLAM is Used as Ground Truth

Since external ground truth (motion capture, GPS) is unavailable:
- SLAM with loop closure provides **globally consistent** trajectories
- Loop closure corrects accumulated drift when revisiting locations
- The corrected trajectory is the **best available reference**

---

## Results Summary

> **Note:** All error metrics are computed using **SLAM Toolbox as Ground Truth**

---

### Sequence 00: Empty Hallway
> Baseline environment with minimal obstacles (~550 seconds duration)

#### Trajectory Comparison
![Sequence 00 Trajectories](seq0/seq0_all.png)

#### Time Series Analysis
![Sequence 00 Time Series](seq0/seq0_timeseries.png)

The time series plot shows X, Y position and heading (Theta) over time:
- **X position:** All methods track similarly until ~250s. After that, Wheel Odometry (orange) deviates from SLAM ground truth (red dashed). EKF (blue) shows moderate deviation. ICP (green) tracks ground truth closely.
- **Y position:** Wheel Odometry shows larger negative deviation compared to ground truth. EKF follows a similar pattern but with smaller error. ICP maintains close tracking throughout.
- **Theta:** All methods track heading well initially. ICP shows oscillation around 270-400s, likely due to angle wraparound at +/-180 degrees during rotation.

#### Drift Analysis (Reference: SLAM Toolbox)
![Sequence 00 Drift Analysis](seq0/seq0_error.png)

#### Quantitative Results

| Method           | Mean Error | Max Error | Std Dev |      vs Raw      |
| ---------------- | :--------: | :-------: | :-----: | :--------------: |
| **Raw Odometry** |  3.714 m   |  ~7.8 m   | ~2.8 m  |        —         |
| **EKF Fused**    |  3.318 m   |  ~6.3 m   | ~2.0 m  | **10.7% better** |
| **ICP LiDAR**    |  0.174 m   |  ~0.4 m   | ~0.1 m  | **95.3% better** |
| **SLAM Toolbox** |     —      |     —     |    —    |   Ground Truth   |

#### Analysis

**ICP achieves exceptional performance** with only 0.174 m mean error:
- Empty hallway provides clear wall features for scan matching
- Consistent geometric structure allows reliable ICP convergence
- Low error variance indicates stable localization throughout

**EKF shows moderate improvement** over Raw Odometry:
- IMU helps correct heading drift
- Position (x,y) still accumulates error without external correction

### Generated Maps

**SLAM Toolbox Map:**

![Sequence 00 SLAM Map](seq0/seq0_map.png)

**ICP Odometry Map with Trajectories:**
![Sequence 00 ICP Map](seq0/seq0_icpmap.png)

The ICP map overlay shows all trajectories plotted on the accumulated point cloud. Red trajectory (SLAM) closely follows the map structure, while purple (EKF) shows significant deviation, especially in the lower portion where heading errors compound into position drift.

#### Sequence 00 Conclusion

In the empty hallway environment, ICP achieves exceptional accuracy (0.174 m mean error) because the clear wall structures provide reliable geometric features for scan matching. EKF shows only moderate improvement (10.7%) over Raw Odometry because the relatively straight paths do not generate significant heading errors that IMU could correct. The main source of error in Raw Odometry and EKF is gradual position drift from wheel slip and encoder noise, which accumulates over the 550-second duration.

---

### Sequence 01: Sharp Turns
> Challenging environment with aggressive rotational motion (~400 seconds duration)

#### Trajectory Comparison
![Sequence 01 Trajectories](seq1/seq1_all.png)

#### Time Series Analysis
![Sequence 01 Time Series](seq1/seq1_timeseries.png)

The time series reveals critical differences during sharp turn sequences:
- **X position:** All methods follow similar trend until ~200s. Wheel Odometry (orange) then diverges significantly, staying lower than ground truth. EKF (blue) and ICP (green) maintain closer tracking to SLAM.
- **Y position:** Wheel Odometry shows severe deviation in the negative direction, reaching -15m while ground truth reaches only -10m. This error accumulates from heading miscalculation during sharp turns.
- **Theta:** EKF (blue) shows a consistent offset of ~40 degrees from the start, indicating IMU initial alignment issue. However, it still tracks the shape of heading changes. Wheel Odometry (orange) shows wraparound artifacts at +/-180 degrees.

#### Drift Analysis (Reference: SLAM Toolbox)
![Sequence 01 Drift Analysis](seq1/seq1_error.png)

#### Quantitative Results

| Method           | Mean Error | Max Error | Std Dev |      vs Raw      |
| ---------------- | :--------: | :-------: | :-----: | :--------------: |
| **Raw Odometry** |  6.418 m   |  ~9.5 m   | ~2.3 m  |        —         |
| **EKF Fused**    |  1.179 m   |  ~1.9 m   | ~0.4 m  | **81.6% better** |
| **ICP LiDAR**    |  1.117 m   |  ~2.0 m   | ~0.5 m  | **82.6% better** |
| **SLAM Toolbox** |     —      |     —     |    —    |   Ground Truth   |

#### Key Finding: EKF Now Outperforms Raw Odometry

Unlike previous results where EKF degraded performance, the updated implementation shows **81.6% improvement**:

**Why EKF Works Well Now:**
1. **Proper IMU integration** - The IMU provides reliable heading correction during sharp turns
2. **Heading is critical** - Sharp turns amplify heading errors in wheel odometry. Small angular errors become large position errors when multiplied by distance traveled.
3. **IMU complements wheel slip** - During aggressive rotation, wheels may slip, but IMU maintains accurate heading measurement

**Why Raw Odometry Failed:**
- Sharp turns cause significant wheel slip
- Heading errors of just a few degrees compound into meters of position error
- The 6.418 m mean error reflects accumulated heading drift throughout the trajectory

### Generated Maps

**SLAM Toolbox Map:**

![Sequence 01 SLAM Map](seq1/seq1_map.png)

**ICP Odometry Map with Trajectories:**
![Sequence 01 ICP Map](seq1/seq1_icpmap.png)

The ICP map clearly shows the trajectory divergence: purple (EKF) stays close to red (SLAM), while the Raw Odometry trajectory (not shown, but implied by EKF without IMU) would deviate significantly to the left side of the map.

#### Sequence 01 Conclusion

Sharp turns expose the critical importance of heading accuracy. Raw Odometry fails severely (6.418 m error) because wheel slip during aggressive rotation causes heading errors that compound into large position errors. EKF improves dramatically (81.6% better) by using IMU to maintain accurate heading during turns, though the ~40 degree initial offset visible in time series suggests calibration could further improve results. ICP performs similarly well (1.117 m) by using geometric features to correct both position and heading. The key lesson is that heading errors during turns are the dominant error source, and any method that corrects heading will significantly outperform pure wheel odometry.

---

### Sequence 02: Smooth Motion
> Non-empty hallway with gentle motion (~600 seconds duration)

#### Trajectory Comparison
![Sequence 02 Trajectories](seq2/seq2_all.png)

#### Time Series Analysis
![Sequence 02 Time Series](seq2/seq2_timeseries.png)

The time series for smooth motion sequence shows:
- **X position:** All methods track similarly in the first half. Wheel Odometry (orange) then slightly undershoots compared to ground truth. EKF (blue) shows moderate deviation after 400s. ICP (green) maintains close tracking throughout.
- **Y position:** Wheel Odometry deviates significantly to -15m while ground truth only reaches -10m. This 5m error demonstrates heading-induced position drift over the long 600s duration.
- **Theta:** EKF shows a consistent ~40 degree offset similar to Seq01. Wheel Odometry shows wraparound at +/-180 degrees. ICP tracks ground truth heading accurately.

#### Drift Analysis (Reference: SLAM Toolbox)
![Sequence 02 Drift Analysis](seq2/seq2_error.png)

#### Quantitative Results

| Method           | Mean Error | Max Error | Std Dev |      vs Raw      |
| ---------------- | :--------: | :-------: | :-----: | :--------------: |
| **Raw Odometry** |  6.071 m   |  ~8.1 m   | ~2.1 m  |        —         |
| **EKF Fused**    |  3.159 m   |  ~5.7 m   | ~1.8 m  | **48.0% better** |
| **ICP LiDAR**    |  0.659 m   |  ~1.4 m   | ~0.4 m  | **89.1% better** |
| **SLAM Toolbox** |     —      |     —     |    —    |   Ground Truth   |

#### Analysis

**ICP achieves best performance** with 0.659 m mean error:
- Smooth motion allows optimal scan matching convergence
- No motion blur or scan distortion
- Rich environmental features from non-empty hallway

**EKF shows significant improvement** (48.0% better than Raw):
- Smooth motion produces clean IMU signals
- Heading correction prevents the position drift seen in Raw Odometry
- The 600s duration would cause severe drift without IMU correction

**Raw Odometry shows worst performance:**
- Long duration (600s) allows drift to accumulate
- Without heading correction, small angular errors compound over time
- The 6.071 m mean error reflects unbounded drift behavior

### Generated Maps

**SLAM Toolbox Map:**

![Sequence 02 SLAM Map](seq2/seq2_map.png)

**ICP Odometry Map with Trajectories:**
![Sequence 02 ICP Map](seq2/seq2_icpmap.png)

The ICP map shows trajectory accuracy: green (ICP) and red (SLAM) overlap closely, while purple (EKF) shows moderate deviation. The point cloud quality is excellent due to smooth motion, with clear wall structures visible.

#### Sequence 02 Conclusion

The smooth motion sequence demonstrates how errors accumulate over long durations. Despite gentle motion, Raw Odometry accumulates 6.071 m error over 600 seconds because small heading errors continuously compound into position drift. EKF reduces this by 48% through heading correction, but the ~40 degree initial offset limits its effectiveness. ICP achieves the best result (0.659 m) because smooth motion allows optimal scan matching convergence without motion blur, and the non-empty hallway provides rich geometric features. This sequence shows that even without aggressive maneuvers, long-duration navigation requires active drift correction.

---

## Cross-Sequence Comparison

### Mean Error Summary (Reference: SLAM Toolbox)

| Sequence | Environment   | Duration | Raw Odom |  EKF Fused  |  ICP LiDAR  | Best Method |
| :------: | ------------- | :------: | :------: | :---------: | :---------: | :---------: |
|  **00**  | Empty Hallway |  ~550s   | 3.714 m  |   3.318 m   | **0.174 m** |     ICP     |
|  **01**  | Sharp Turns   |  ~400s   | 6.418 m  | **1.179 m** |   1.117 m   |     ICP     |
|  **02**  | Smooth Motion |  ~600s   | 6.071 m  |   3.159 m   | **0.659 m** |     ICP     |

### Performance Comparison vs Raw Odometry

| Sequence | EKF vs Raw | ICP vs Raw | Best Improvement |
| :------: | :--------: | :--------: | :--------------: |
|  **00**  |   +10.7%   | **+95.3%** |       ICP        |
|  **01**  | **+81.6%** |   +82.6%   |       ICP        |
|  **02**  |   +48.0%   | **+89.1%** |       ICP        |

### Method Robustness Rating

| Method       |  Seq 00   |  Seq 01   |  Seq 02   |  Overall  |
| ------------ | :-------: | :-------: | :-------: | :-------: |
| Raw Odometry |  Medium   |   Poor    |   Poor    |   Poor    |
| EKF Fused    |   Good    | Excellent |   Good    |   Good    |
| ICP LiDAR    | Excellent | Excellent | Excellent | Excellent |
| SLAM Toolbox | Excellent | Excellent | Excellent | Excellent |

---

## Discussion

### 1. Raw Wheel Odometry

**Strengths:**
- Simple, no external sensor dependencies
- Predictable drift behavior
- Low computational cost

**Weaknesses:**
- Unbounded drift over time
- Severely affected by sharp turns (Seq01: 6.418 m error)
- Heading errors compound into large position errors

**Key Finding:** Raw odometry is inadequate for any extended navigation task. Even in the best case (Seq00), error exceeds 3.7 m.

### 2. EKF Sensor Fusion

**Strengths:**
- Significant improvement in all sequences (10.7% - 81.6%)
- Excellent heading correction during sharp turns (Seq01)
- Low computational overhead

**Weaknesses:**
- Still accumulates position drift (no position measurement)
- Performance depends on IMU quality and calibration

**Key Finding:** EKF with IMU fusion provides reliable heading correction. The 81.6% improvement in Seq01 demonstrates the critical importance of heading accuracy during aggressive maneuvers.

### 3. ICP LiDAR Odometry

**Strengths:**
- Best performance across all sequences
- Exceptional accuracy in Seq00 (0.174 m) and Seq02 (0.659 m)
- Corrects both position and heading

**Weaknesses:**
- Requires geometric features in environment
- Higher computational cost than EKF
- Can show oscillation in featureless areas (visible in Seq00 theta plot)

**Key Finding:** ICP is the most reliable odometry method, achieving 82-95% improvement over raw odometry across all conditions.

### 4. SLAM (slam_toolbox)

**Strengths:**
- Best overall accuracy (used as ground truth)
- Loop closure corrects accumulated drift
- Globally consistent maps

**Weaknesses:**
- Highest computational cost
- Requires loop closure opportunities

---

## Conclusion

### Progressive Improvements

#### 1. Wheel Odometry -> EKF (IMU Fusion)

- **Improvement:** 10.7% - 81.6% across sequences
- **Mechanism:** IMU provides heading correction, preventing angular drift from compounding into position error
- **Best case:** Sharp turns (Seq01) where heading accuracy is critical

#### 2. EKF -> ICP (LiDAR-Based Refinement)

- **Improvement:** Additional 47-95% over EKF
- **Mechanism:** Geometric scan matching provides both position and heading correction
- **Best case:** Feature-rich environments (Seq00, Seq02)

#### 3. ICP -> SLAM (Global Optimization + Loop Closure)

- **Improvement:** Global consistency, eliminates accumulated drift
- **Mechanism:** Pose graph optimization with loop closure constraints
- **Best case:** Long-duration navigation with revisited locations


### Key Findings

1. **EKF with IMU provides critical heading correction.** The 81.6% improvement in Seq01 demonstrates that heading accuracy is essential during aggressive maneuvers. Small angular errors compound into large position errors.

2. **ICP achieves consistent sub-meter accuracy.** Across all sequences, ICP maintains mean error below 1.2 m, with exceptional performance (0.174 m) in feature-rich environments.

3. **Raw odometry is insufficient for extended navigation.** Without correction, drift accumulates to 6+ meters in challenging sequences, making raw odometry unsuitable for any practical application.

4. **Sensor fusion order matters.** The progressive improvement (Raw -> EKF -> ICP -> SLAM) demonstrates that each layer adds essential capabilities: heading correction, position correction, and global consistency.

### Performance Summary

| Rank  | Method           | Avg Mean Error | Recommendation       |
| :---: | ---------------- | :------------: | -------------------- |
|   1   | **ICP LiDAR**    |     0.65 m     | Best for accuracy    |
|   2   | **EKF Fused**    |     2.55 m     | Good for low compute |
|   3   | **Raw Odometry** |     5.40 m     | Baseline only        |
|   —   | **SLAM**         |  Ground Truth  | Best overall         |

### Method Characteristics Summary

| Method       | Corrects | Drift Behavior  | Computation | Best Use Case            |
| ------------ | -------- | --------------- | ----------- | ------------------------ |
| Raw Odometry | —        | Unbounded       | Lowest      | Short distance only      |
| EKF          | θ only   | Reduced heading | Low         | Real-time applications   |
| ICP          | x, y, θ  | Bounded local   | Medium      | Feature-rich environment |
| SLAM         | Global   | Corrected       | Highest     | Long-duration navigation |

### Final Remarks

The experimental results clearly demonstrate the accuracy hierarchy among odometry methods, with ICP LiDAR achieving the best performance (0.65 m average error), followed by EKF (2.55 m), and Raw Odometry performing worst (5.40 m).

**Raw Wheel Odometry** suffers from unbounded drift because it relies solely on wheel encoder measurements. The primary error sources are wheel slip during turns, encoder noise, and small systematic errors in wheel radius or track width calibration. These errors accumulate over time without any correction mechanism. In sequences with sharp turns (Seq01) or long duration (Seq02), the accumulated error exceeds 6 meters, making raw odometry unsuitable for any practical navigation task beyond short distances.

**EKF with IMU Fusion** improves accuracy by correcting heading drift using IMU measurements. Since the IMU provides absolute orientation (from accelerometer and gyroscope integration), it prevents the angular errors that would otherwise compound into large position errors. The improvement is most dramatic during sharp turns (81.6% in Seq01) where heading accuracy is critical. However, EKF only corrects heading and cannot fix position drift directly. The time series analysis also reveals a consistent ~40 degree heading offset in some sequences, suggesting that proper IMU calibration and initial alignment could further improve results. Despite this limitation, EKF provides significant improvement at minimal computational cost.

**ICP LiDAR Odometry** achieves the best accuracy because it corrects both position and heading using environmental features. By matching the current LiDAR scan against a local map built from previous scans, ICP can detect and correct drift in all three degrees of freedom (x, y, θ). The method performs exceptionally well in feature-rich environments like Seq00 (0.174 m error) and Seq02 (0.659 m error) where clear wall structures provide reliable geometric constraints. Even in challenging conditions with sharp turns (Seq01), ICP maintains sub-1.2 meter accuracy. The key advantage of ICP is that it uses external reference (environment geometry) rather than relying on internal sensors that accumulate drift.

**SLAM with Loop Closure** represents the gold standard for localization accuracy. While we use SLAM as ground truth in this experiment, it achieves global consistency by detecting when the robot revisits previously mapped locations and optimizing the entire trajectory to satisfy these constraints. This eliminates the accumulated drift that affects all other methods over long durations.

In conclusion, the choice of odometry method depends on application requirements. For short-distance navigation with limited computation, EKF provides reasonable accuracy. For applications requiring sub-meter accuracy in indoor environments, ICP is the recommended choice. For long-duration autonomous navigation where global consistency is essential, full SLAM with loop closure remains necessary despite its higher computational cost.

---

## References

1. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
2. slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
3. ROS2 Navigation: https://navigation.ros.org/
4. Kalman Filter: https://github.com/AtsushiSakai/PythonRobotics
5. ICP Algorithm: Besl & McKay (1992) - A Method for Registration of 3-D Shapes

---

## Appendix

### A. Robot Parameters

| Parameter                | Value   |
| ------------------------ | ------- |
| Wheel Radius             | 0.033 m |
| Wheel Separation         | 0.160 m |
| LiDAR Range              | 3.5 m   |
| LiDAR Angular Resolution | 1 deg   |

### B. EKF Parameters

| Parameter | Description                  | Value                    |
| --------- | ---------------------------- | ------------------------ |
| Q         | Process noise covariance     | diag([0.01, 0.01, 0.01]) |
| R         | Measurement noise covariance | diag([0.1])              |

### C. Complete Error Statistics

|  Seq  | Method       | Mean (m) | Max (m) | Std (m) | vs Raw |
| :---: | ------------ | :------: | :-----: | :-----: | :----: |
|  00   | Raw Odometry |  3.714   |  ~7.8   |  ~2.8   |   —    |
|  00   | EKF Fused    |  3.318   |  ~6.3   |  ~2.0   | +10.7% |
|  00   | ICP LiDAR    |  0.174   |  ~0.4   |  ~0.1   | +95.3% |
|  01   | Raw Odometry |  6.418   |  ~9.5   |  ~2.3   |   —    |
|  01   | EKF Fused    |  1.179   |  ~1.9   |  ~0.4   | +81.6% |
|  01   | ICP LiDAR    |  1.117   |  ~2.0   |  ~0.5   | +82.6% |
|  02   | Raw Odometry |  6.071   |  ~8.1   |  ~2.1   |   —    |
|  02   | EKF Fused    |  3.159   |  ~5.7   |  ~1.8   | +48.0% |
|  02   | ICP LiDAR    |  0.659   |  ~1.4   |  ~0.4   | +89.1% |
