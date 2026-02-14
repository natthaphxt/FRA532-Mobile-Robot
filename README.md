# FRA532 LAB1: Kalman Filter / SLAM

**Course:** FRA532 - Mobile Robot  
**Student:** Natthapatch Lapsittiwong | 66340500077  

---

## 📋 Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Dependencies](#dependencies)
- [Part 1: EKF Odometry Fusion](#part-1-ekf-odometry-fusion)
- [Part 2: ICP Odometry Refinement](#part-2-icp-odometry-refinement)
- [Part 3: Full SLAM with slam_toolbox](#part-3-full-slam-with-slam_toolbox)
- [Results Summary](#results-summary)
  - [Sequence 00: Empty Hallway](#-sequence-00-empty-hallway)
  - [Sequence 01: Sharp Turns](#-sequence-01-sharp-turns)
  - [Sequence 02: Smooth Motion](#-sequence-02-smooth-motion)
- [Cross-Sequence Comparison](#cross-sequence-comparison)
- [Discussion](#discussion)
- [Conclusion](#conclusion)
- [How to Run](#how-to-run)
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
- Target: $Q = \{q_1, q_2, \ldots, q_m\}$ (reference scan/map)

**Objective:** Find rotation $\mathbf{R}$ and translation $\mathbf{t}$ that minimize alignment error:

$$E(\mathbf{R}, \mathbf{t}) = \sum_{i=1}^{n} \| p_i - (\mathbf{R} \cdot q_i + \mathbf{t}) \|^2$$

**For 2D LiDAR:**

$$\mathbf{R} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}, \quad \mathbf{t} = \begin{bmatrix} t_x \\ t_y \end{bmatrix}$$

### 2.2 ICP Algorithm

```
Input:  Source cloud P, Target cloud Q, Initial guess T₀
Output: Optimal transformation T = (R, t)

1. Initialize: T = T₀

2. While not converged:
   
   a. Find Correspondences:
      For each pᵢ in P:
          qᵢ = nearest_neighbor(T·pᵢ, Q)  // Using KD-tree
   
   b. Reject Outliers:
      Remove pairs where ‖T·pᵢ - qᵢ‖ > threshold
   
   c. Compute Optimal Transformation:
      (R, t) = minimize Σᵢ ‖R·pᵢ + t - qᵢ‖²
      
   d. Update: T = (R, t)
   
   e. Check Convergence:
      If ΔT < tolerance → break

3. Return T
```

### 2.3 SVD-based Solution (Point-to-Point)

For Point-to-Point ICP, the optimal transformation is computed using SVD:

**Step 1: Compute centroids**

$$\bar{p} = \frac{1}{n} \sum_{i=1}^{n} p_i, \quad \bar{q} = \frac{1}{n} \sum_{i=1}^{n} q_i$$

**Step 2: Center point clouds**

$$P' = P - \bar{p}, \quad Q' = Q - \bar{q}$$

**Step 3: Compute cross-covariance matrix**

$$\mathbf{H} = P'^T \cdot Q'$$

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

$$M_{\text{downsampled}} = \text{voxel\_filter}(M, 0.05m)$$

**ICP Registration:**
- Source: Current scan (full resolution)
- Target: Local map (downsampled)
- Initial guess: EKF odometry

**Why Scan-to-Map?**

| Method       | Pros                         | Cons                      |
| ------------ | ---------------------------- | ------------------------- |
| Scan-to-Scan | Simple, fast                 | Drift accumulates quickly |
| Scan-to-Map  | More stable, richer features | Higher computation        |

### 2.5 ICP Odometry Pipeline

```
LiDAR Scan → Preprocess → Keyframe Selection → Update Local Map → ICP Registration → Refined Pose
```

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

### 📍 Sequence 00: Empty Hallway
> Baseline environment with minimal obstacles (~550 seconds duration)

#### Trajectory Comparison
![Sequence 00 Trajectories](seq0/seq0_all.png)

#### Drift Analysis (Reference: SLAM Toolbox)
![Sequence 00 Drift Analysis](seq0/seq0_error.png)

#### Quantitative Results

| Method           | Mean Error | Max Error | Std Dev |       vs Raw       |
| ---------------- | :--------: | :-------: | :-----: | :----------------: |
| **Raw Odometry** |  3.714 m   |  ~7.8 m   | ~2.8 m  |         —          |
| **EKF Fused**    |  3.316 m   |  ~6.3 m   | ~2.0 m  | **10.7% better** ✅ |
| **ICP LiDAR**    |  1.268 m   |  ~6.0 m   | ~1.4 m  | **65.9% better** ✅ |
| **SLAM Toolbox** |     —      |     —     |    —    |    Ground Truth    |

#### Analysis

**Raw Odometry:**
- Gradual drift accumulation over ~550 seconds
- Error increases steadily, reaching ~7.8 m maximum
- Heading drift causes trajectory to deviate from ground truth

**EKF Fused:**
- **10.7% improvement** over Raw Odometry
- IMU helps correct heading drift in this baseline environment
- Moderate improvement due to relatively simple motion profile

**ICP LiDAR:**
- **Best performance** with 65.9% improvement
- Environmental features (walls) provide strong geometric constraints
- Consistent low error throughout trajectory

#### Generated Map
![Sequence 00 Map](seq0/seq0_map.png)

---

### 📍 Sequence 01: Sharp Turns
> Challenging environment with aggressive rotational motion (~400 seconds duration)

#### Trajectory Comparison
![Sequence 01 Trajectories](seq1/seq1_all.png)

#### Drift Analysis (Reference: SLAM Toolbox)
![Sequence 01 Drift Analysis](seq1/seq1_error.png)

#### Quantitative Results

| Method           | Mean Error | Max Error | Std Dev |      vs Raw       |
| ---------------- | :--------: | :-------: | :-----: | :---------------: |
| **Raw Odometry** |  3.181 m   |  ~6.3 m   | ~1.4 m  |         —         |
| **EKF Fused**    |  4.419 m   |  ~7.7 m   | ~2.1 m  | **38.9% worse** ❌ |
| **ICP LiDAR**    |  3.873 m   |  ~6.5 m   | ~1.7 m  | **21.8% worse** ❌ |
| **SLAM Toolbox** |     —      |     —     |    —    |   Ground Truth    |

#### ⚠️ Unexpected Result: EKF Performs Worse Than Raw Odometry

**Observation:**
- EKF error (4.419 m) is **38.9% higher** than Raw Odometry (3.181 m)
- This contradicts the expected behavior where sensor fusion should improve accuracy

**Root Cause Analysis:**

1. **IMU Gyroscope Bias Accumulation** - During sharp turns, IMU experiences high angular velocity, causing gyroscope bias to have a larger effect. EKF trusts IMU heading, so it accumulates bias error over time.

2. **EKF Covariance Mismatch** - Process noise (Q) may be set too low, causing over-trust in the motion model. Measurement noise (R) may be set too high, causing under-trust in wheel odometry.

3. **Drift Pattern from Graph** - During 0-100s, EKF error rises rapidly (when sharp turns begin), reaching peak ~7.5m at 100-200s, then gradually decreasing as the robot returns.

**Why Raw Odometry Performs Better:**
- Wheel encoders have bounded and predictable drift
- No external sensor bias to accumulate
- Sharp turns cause wheel slip, but error doesn't compound as severely as IMU bias

#### Generated Map
![Sequence 01 Map](seq1/seq1_map.png)

---

### 📍 Sequence 02: Smooth Motion
> Non-empty hallway with gentle motion (~650 seconds duration)

#### Trajectory Comparison
![Sequence 02 Trajectories](seq2/seq2_all.png)

#### Drift Analysis (Reference: SLAM Toolbox)
![Sequence 02 Drift Analysis](seq2/seq2_error.png)

#### Quantitative Results

| Method           | Mean Error | Max Error | Std Dev |       vs Raw       |
| ---------------- | :--------: | :-------: | :-----: | :----------------: |
| **Raw Odometry** |  3.298 m   |  ~7.2 m   | ~2.0 m  |         —          |
| **EKF Fused**    |  5.733 m   |  ~8.5 m   | ~2.2 m  | **73.8% worse** ❌  |
| **ICP LiDAR**    |  1.068 m   |  ~1.9 m   | ~0.4 m  | **67.6% better** ✅ |
| **SLAM Toolbox** |     —      |     —     |    —    |    Ground Truth    |

#### ⚠️ EKF Shows Worst Performance Across All Sequences

**Observation:**
- EKF error (5.733 m) is **73.8% higher** than Raw Odometry (3.298 m)
- This is the most severe EKF degradation among all sequences

**Root Cause Analysis:**

1. **IMU Bias Drift During Slow Motion** - Due to smooth motion, angular velocity is very low, resulting in small gyroscope signals where bias becomes dominant. Additionally, this sequence is 650s long, allowing significant bias accumulation.

2. **Signal-to-Noise Ratio Problem** - During smooth motion, the actual rotation signals are small, but IMU noise and bias are relatively larger in comparison. EKF integrates this noise, causing heading error to grow over time.

3. **Drift Pattern from Graph** - During 0-200s, error increases steadily. From 200-450s, it reaches peak ~8.5m and remains high. Finally, during 450-650s, it gradually decreases but stays above Raw.

**Why ICP Performs Excellently:**
- Smooth motion allows scan matching to converge well
- No motion blur or scan distortion
- Environmental features are captured clearly, resulting in mean error of only 1.068 m

#### Generated Map
![Sequence 02 Map](seq2/seq2_map.png)

---

## Cross-Sequence Comparison

### Mean Error Summary (Reference: SLAM Toolbox)

| Sequence | Environment   | Duration |  Raw Odom   | EKF Fused |  ICP LiDAR  | Best Method |
| :------: | ------------- | :------: | :---------: | :-------: | :---------: | :---------: |
|  **00**  | Empty Hallway |  ~550s   |   3.714 m   |  3.316 m  | **1.268 m** |    ICP ✅    |
|  **01**  | Sharp Turns   |  ~400s   | **3.181 m** |  4.419 m  |   3.873 m   |    Raw ⚠️    |
|  **02**  | Smooth Motion |  ~650s   |   3.298 m   |  5.733 m  | **1.068 m** |    ICP ✅    |

### Performance Comparison vs Raw Odometry

| Sequence | EKF vs Raw | ICP vs Raw | EKF Status |
| :------: | :--------: | :--------: | :--------: |
|  **00**  |   +10.7%   |   +65.9%   | ✅ Improved |
|  **01**  | **-38.9%** |   -21.8%   | ❌ Degraded |
|  **02**  | **-73.8%** |   +67.6%   | ❌ Degraded |

### Method Robustness Rating

| Method       | Seq 00 | Seq 01 | Seq 02 | Overall |
| ------------ | :----: | :----: | :----: | :-----: |
| Raw Odometry |   ⭐⭐   |  ⭐⭐⭐   |  ⭐⭐⭐   |   ⭐⭐⭐   |
| EKF Fused    |  ⭐⭐⭐   |   ⭐    |   ⭐    |   ⭐⭐    |
| ICP LiDAR    | ⭐⭐⭐⭐⭐  |  ⭐⭐⭐   | ⭐⭐⭐⭐⭐  |  ⭐⭐⭐⭐   |
| SLAM Toolbox | ⭐⭐⭐⭐⭐  | ⭐⭐⭐⭐⭐  | ⭐⭐⭐⭐⭐  |  ⭐⭐⭐⭐⭐  |

---

## Discussion

### 1. Raw Wheel Odometry

**Strengths:**
- Simple, predictable drift behavior
- No external sensor dependencies
- Surprisingly robust in Seq01 (sharp turns)

**Weaknesses:**
- Unbounded drift over time
- Affected by wheel slip
- No correction mechanism

**Key Finding:** Raw odometry provides a stable baseline. Its predictable drift pattern makes it more reliable than poorly-tuned sensor fusion in certain scenarios.

### 2. EKF Sensor Fusion — Why It Failed

The EKF performed **worse than Raw Odometry** in 2 out of 3 sequences. This unexpected result reveals critical issues:

#### Problem 1: IMU Gyroscope Bias

The TurtleBot3 uses a low-cost IMU which has significant gyroscope bias.

Assuming the gyroscope has a constant bias, when EKF integrates the angular velocity:

$$\theta_{EKF} = \int \omega_{measured} \, dt = \int (\omega_{true} + bias) \, dt = \theta_{true} + bias \cdot t$$

The error grows linearly with time, so longer sequences accumulate more error.

#### Problem 2: Incorrect Covariance Tuning

| Parameter             | Current Issue | Effect                           |
| --------------------- | ------------- | -------------------------------- |
| Q (Process Noise)     | Too small     | EKF trusts motion model too much |
| R (Measurement Noise) | Too large     | EKF trusts biased IMU            |

**Recommendation:** Add gyroscope bias to EKF state vector:

$$\mathbf{x} = [x, y, \theta, b_{gyro}]^T \quad \text{(4-state EKF)}$$

#### Problem 3: Motion-Dependent Performance

| Motion Type   | IMU Reliability                | EKF Performance |
| ------------- | ------------------------------ | --------------- |
| Sharp Turns   | Low (saturation, nonlinearity) | Poor            |
| Smooth Motion | Low (bias dominates signal)    | Worst           |
| Normal Motion | Moderate                       | Acceptable      |

### 3. ICP LiDAR Odometry

**Strengths:**
- **Most consistent** across all sequences
- Uses environmental features (independent of motion)
- Best in Seq00 and Seq02

**Weaknesses:**
- Degraded in Seq01 (sharp turns cause scan distortion)
- Requires geometric features
- Higher computational cost

**Key Finding:** ICP is the most reliable odometry method when environmental features are available.

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

#### 1. Wheel Odometry → EKF (IMU Fusion)

- **Heading correction:** IMU provides absolute orientation to correct encoder drift
- **Expected improvement:** Heading deviation should decrease significantly
- **Actual result:** EKF **failed** in 2/3 sequences due to IMU gyroscope bias
- **What remains:** Position (x, y) still drifts without independent position measurements
- **Cost:** Minimal computation overhead

#### 2. EKF → ICP (LiDAR-Based Refinement)

- **Paradigm shift:** From dead reckoning to environment-based localization
- **Mechanism:** Geometric scan matching provides position and heading corrections
- **Improvement:** Best performer in Seq00 (1.268 m) and Seq02 (1.068 m)
- **What remains:** Long-term drift accumulates without loop closure
- **Cost:** Real-time scan matching

#### 3. ICP → SLAM (Global Optimization + Loop Closure)

- **Paradigm shift:** From local scan matching to global pose-graph optimization
- **Mechanism:** Backend solver minimizes cumulative pose-graph errors, loop closure adds global constraints
- **Improvement:** Globally consistent maps, used as ground truth
- **Trade-off:** Higher computational cost
- **Cost:** Backend optimization overhead, but maintains real-time on modern hardware

### Key Results

#### Part 1 - EKF Odometry Fusion

| Metric          |  Seq 00  |    Seq 01    |    Seq 02    |
| --------------- | :------: | :----------: | :----------: |
| Mean Error      | 3.316 m  |   4.419 m    |   5.733 m    |
| vs Raw Odometry | +10.7% ✅ |   -38.9% ❌   |   -73.8% ❌   |
| Status          | Improved | **Degraded** | **Degraded** |

**Limitation:** IMU gyroscope bias causes error accumulation. Current 3-state EKF lacks bias estimation.

#### Part 2 - ICP Odometry Refinement

| Metric          |  Seq 00  |  Seq 01  |  Seq 02  |
| --------------- | :------: | :------: | :------: |
| Mean Error      | 1.268 m  | 3.873 m  | 1.068 m  |
| vs Raw Odometry | +65.9% ✅ | -21.8% ❌ | +67.6% ✅ |
| Status          | **Best** | Degraded | **Best** |

**Limitation:** Sharp turns (Seq01) cause scan distortion, degrading performance.

#### Part 3 - SLAM with slam_toolbox

- Used as **ground truth** for error calculation
- Loop closure provides globally consistent trajectories
- Essential for long-duration navigation

### Key Findings

1. **Sensor fusion effectiveness:** EKF with low-cost IMU can **degrade** performance if gyroscope bias is not estimated. Failed in 2/3 sequences with up to 73.8% worse accuracy than raw odometry.

2. **Environment-based localization:** ICP provides the most consistent improvement, achieving 65-68% error reduction in feature-rich environments (Seq00, Seq02).

3. **Motion-dependent performance:** Sharp turns (Seq01) challenge all methods. Raw odometry (3.181 m) outperformed both EKF (4.419 m) and ICP (3.873 m).

4. **Coverage trade-off:** ICP builds dense local maps for high coverage, while SLAM selectively processes keyframes for global consistency.

### Comparison Summary

#### Accuracy

| Method             | Description                                                      |
| ------------------ | ---------------------------------------------------------------- |
| **Wheel Odometry** | Baseline with unbounded drift (3.18-3.71 m mean error)           |
| **EKF**            | Should improve heading, but failed due to IMU bias (3.32-5.73 m) |
| **ICP**            | Best accuracy in 2/3 sequences (1.07-3.87 m)                     |
| **SLAM**           | Ground truth with globally consistent maps                       |

#### Drift

| Method             | Drift Behavior                                      |
| ------------------ | --------------------------------------------------- |
| **Wheel Odometry** | Unbounded drift in position and heading             |
| **EKF**            | Reduces heading drift only (when IMU is calibrated) |
| **ICP**            | Corrects both position and heading using LiDAR      |
| **SLAM**           | Minimizes cumulative drift through loop closure     |

#### Robustness

| Method             | Robustness                                                    |
| ------------------ | ------------------------------------------------------------- |
| **Wheel Odometry** | Predictable but unbounded drift                               |
| **EKF**            | Sensitive to IMU bias and motion profile                      |
| **ICP**            | Robust in feature-rich environments, sensitive to sharp turns |
| **SLAM**           | Most robust due to global optimization and loop closure       |

### Why EKF Failed — Summary

| Cause                  | Seq 01 Impact                  | Seq 02 Impact             |
| ---------------------- | ------------------------------ | ------------------------- |
| **Gyroscope Bias**     | High (sharp turns amplify)     | Very High (long duration) |
| **Covariance Tuning**  | Mismatch for aggressive motion | Mismatch for slow motion  |
| **No Bias Estimation** | Bias accumulates               | Bias dominates            |

### Recommendations

| Issue             | Solution                                                         |
| ----------------- | ---------------------------------------------------------------- |
| IMU bias          | Add bias state to EKF: $\mathbf{x} = [x, y, \theta, b_{gyro}]^T$ |
| Covariance tuning | Implement adaptive Q/R based on motion                           |
| Validation        | Always compare EKF against Raw baseline                          |

### Performance Summary

| Rank  | Method           | Avg Mean Error | Recommendation      |
| :---: | ---------------- | :------------: | ------------------- |
|   1   | **ICP LiDAR**    |     2.07 m     | ✅ Best for accuracy |
|   2   | **Raw Odometry** |     3.40 m     | ✅ Reliable baseline |
|   3   | **EKF Fused**    |     4.49 m     | ⚠️ Needs improvement |
|   —   | **SLAM**         |  Ground Truth  | ✅ Best overall      |

### Method Characteristics Summary

| Method       | Corrects | Drift Behavior  | Computation | Best Use Case            |
| ------------ | -------- | --------------- | ----------- | ------------------------ |
| Raw Odometry | —        | Unbounded       | Lowest      | Baseline, short distance |
| EKF          | θ only   | Reduced θ drift | Low         | When IMU is calibrated   |
| ICP          | x, y, θ  | Bounded local   | Medium      | Feature-rich environment |
| SLAM         | Global   | Corrected       | Highest     | Long-duration navigation |

### Final Remarks

The experiments demonstrate that robust indoor localization requires progressive capability layers: IMU fusion corrects heading (when properly calibrated), LiDAR scan matching enables geometric correction, and pose-graph optimization provides global consistency. 

**Critical finding:** Sensor fusion can **degrade** performance if not properly tuned. In this experiment, EKF with uncalibrated low-cost IMU performed worse than raw wheel odometry in 2/3 sequences. This highlights the importance of:
1. IMU bias estimation and compensation
2. Proper covariance tuning (Q, R matrices)
3. Always validating sensor fusion against baseline methods

Each method introduces specific trade-offs in computational cost, accuracy, and robustness that must match application requirements.

---

## How to Run

### Build the workspace
```bash
cd ~/FRA532_LAB1
colcon build
source install/setup.bash
```

### Part 1 & 2: EKF + ICP Odometry
```bash
# Terminal 1: Play rosbag
ros2 bag play Dataset/fibo_floor3_seq00/

# Terminal 2: Run EKF + ICP node
ros2 run <package_name> ekf_icp_node
```

### Part 3: SLAM
```bash
# Terminal 1: Play rosbag
ros2 bag play Dataset/fibo_floor3_seq01/

# Terminal 2: Launch slam_toolbox
ros2 launch slam_toolbox online_async_launch.py
```

### Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

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
| LiDAR Angular Resolution | 1°      |

### B. EKF Parameters

| Parameter | Description                  | Value                    |
| --------- | ---------------------------- | ------------------------ |
| Q         | Process noise covariance     | diag([σ_x², σ_y², σ_θ²]) |
| R         | Measurement noise covariance | diag([σ_ω²])             |

### C. Complete Error Statistics

|  Seq  | Method       | Mean (m) | Max (m) | Std (m) | vs Raw |
| :---: | ------------ | :------: | :-----: | :-----: | :----: |
|  00   | Raw Odometry |  3.714   |  ~7.8   |  ~2.8   |   —    |
|  00   | EKF Fused    |  3.316   |  ~6.3   |  ~2.0   | +10.7% |
|  00   | ICP LiDAR    |  1.268   |  ~6.0   |  ~1.4   | +65.9% |
|  01   | Raw Odometry |  3.181   |  ~6.3   |  ~1.4   |   —    |
|  01   | EKF Fused    |  4.419   |  ~7.7   |  ~2.1   | -38.9% |
|  01   | ICP LiDAR    |  3.873   |  ~6.5   |  ~1.7   | -21.8% |
|  02   | Raw Odometry |  3.298   |  ~7.2   |  ~2.0   |   —    |
|  02   | EKF Fused    |  5.733   |  ~8.5   |  ~2.2   | -73.8% |
|  02   | ICP LiDAR    |  1.068   |  ~1.9   |  ~0.4   | +67.6% |