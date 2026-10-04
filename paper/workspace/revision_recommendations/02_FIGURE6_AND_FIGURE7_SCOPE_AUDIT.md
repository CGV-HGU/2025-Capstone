# 🔍 Figure 6 & Figure 7 Deep-Dive Scope Audit and Vulnerability Analysis

> **Target Manuscript**: `paper/workspace/IEEE_Access/main.tex`  
> **Images Examined**:  
> - Figure 6: `paper/workspace/IEEE_Access/figures/experiment2_setup.jpeg`  
> - Figure 7: `paper/workspace/IEEE_Access/figures/nav2_obstacle_avoidance_sequence.png`  
> **Primary Scope**: Monocular RGB $\rightarrow$ 2D Virtual LaserScan $\rightarrow$ Nav2 Local Costmap Reactive Avoidance without physical LiDAR.

---

## 1. Deep Dive: Figure 6 (`fig:exp2_setup`)

### 1.1 Content and Metadata Analysis
- **Image File**: `experiment2_setup.jpeg` ($3{,}212{,}186$\,bytes)
- **Current Caption** (`main.tex`, Line 465):
  > *"Real-world test environment for dynamic obstacle avoidance. The OMO-R1 mobile robot navigates along a 10.0-meter trajectory facing a static cardboard box placed at the center ($Y=0.0$\,m, $X \approx 4.0\text{--}4.5$\,m)."*
- **Visual Breakdown**:
  - **Foreground**: A single standard cardboard box ($0.40 \times 0.40 \times 0.50$\,m) positioned on a glossy tiled floor. Distinct specular highlights from overhead fluorescent fixtures are clearly visible on the tile surface.
  - **Background (Right)**: The OMO-R1 mobile robot parked at the starting line with an extruded aluminum sensor tower carrying the monocular camera at height $h = 1.05$\,m.
  - **Environment**: A wide open architectural lobby ($\sim 6$\,m wide) featuring a structural cylindrical column, an open atrium with a glass and metal balustrade on the left, an elevator vestibule in the background, and posters on the right wall.

---

### 1.2 Identified Scope Vulnerabilities & Reviewer Risks

#### ⚠️ Vulnerability 6.1: Misleading "Dynamic Obstacle" Nomenclature
- **The Issue**: The caption and subsection heading refer to *"dynamic obstacle avoidance"*, but the obstacle is a stationary cardboard box.
- **Reviewer Attack Vector**: In mobile robotics and control literature, *"Dynamic Obstacle Avoidance"* specifically denotes avoiding moving obstacles (such as walking pedestrians, moving carts, or oncoming robots). When reviewers read *"dynamic obstacle avoidance"* only to discover a stationary cardboard box, they will challenge the validity of the evaluation:
  > *"Reviewer Critique: The authors claim dynamic obstacle avoidance, yet Section IV-B and Figure 6 evaluate exclusively stationary obstacles. No dynamic obstacle tracking or moving obstacle velocity compensation is evaluated."*
- **Root Cause in Paper**: The authors used "dynamic" to signify closed-loop dynamic robot navigation (in contrast to the stationary sensor bench test in Experiment 1).
- **Recommendation**:
  - Replace "Dynamic Obstacle Avoidance" with **"Closed-Loop Autonomous Obstacle Avoidance"** or **"Dynamic Robot Navigation and Obstacle Avoidance"**.
  - Explicitly qualify that the navigation is closed-loop dynamic robot transit against stationary hazards.

#### ⚠️ Vulnerability 6.2: "Corridor" vs. "Open Testing Hall" Terminology Conflict
- **The Issue**:
  - Abstract (Line 49) states: *"across a 10.0-m corridor trajectory without pre-built maps"*.
  - Section IV (Line 394) states: *"in real-world indoor corridor environments"*.
  - Section IV-A (Line 439) states: *"stationed on a structured indoor corridor floor"*.
  - **However**, Section VI-B (Line 623) explicitly argues:
    > *"A critical question arising from our evaluation is why experiments were conducted in an open testing hall ($\sim 6$\,m wide) rather than narrow corridors (width $1.8\text{--}2.2$\,m). Our empirical findings and theoretical derivations reveal that deploying this monocular pipeline inside narrow corridors without auxiliary sensors is physically infeasible..."*
- **Reviewer Attack Vector**: Figure 6 clearly depicts an open, spacious lobby ($\sim 6$\,m wide), NOT a standard corridor. A reviewer will point out the direct contradiction between claiming "corridor navigation" in the Abstract while admitting in Section VI-B that corridors are infeasible.
- **Recommendation**:
  - Harmonize terminology throughout: describe the arena as a **"spacious indoor testing hall (width $\sim 6$\,m)"** or **"wide open corridor / lobby environment"**.
  - In the Abstract and Section I, replace *"10.0-m corridor trajectory"* with *"10.0-m linear navigation trajectory in an open testing hall"*.

#### ⚠️ Vulnerability 6.3: Lack of Spatial Annotations
- **The Issue**: Figure 6 is an unannotated wide-angle raw photo. While it authentically documents the real-world setup, it does not visually delineate the experimental trajectory.
- **Recommendation**:
  - Update the caption to clearly orient the reader:
    > *Proposed Caption*: *"Real-world test arena for closed-loop autonomous obstacle avoidance. The OMO-R1 mobile robot (background right) executes a 10.0-meter navigation trajectory along the hall centerline, reacting to a stationary cardboard box obstacle ($0.40 \times 0.40 \times 0.50$\,m) positioned at $X \approx 4.0\text{--}4.5$\,m on glossy, reflective floor tiles."*

---

## 2. Deep Dive: Figure 7 (`fig:exp2_sequence`) — CRITICAL SCOPE AUDIT

### 2.1 Content and Metadata Analysis
- **Image File**: `nav2_obstacle_avoidance_sequence.png` ($10{,}695{,}878$\,bytes)
- **Current Caption** (`main.tex`, Line 472):
  > *"Sequential real-world execution of autonomous obstacle avoidance: (a) robot approaches obstacle while marking costmap, (b) global and local planners generate lateral swerve maneuver to bypass obstacle, and (c) robot re-aligns toward the 10.0-meter terminal goal."*
- **Visual Breakdown of Each Sequence Panel ((a), (b), (c))**:
  1. **Top-Left Subwindow**: Live RGB feed with floor segmentation overlay (`In ROI: False` banner, blue bounding box on the cardboard box, ROS terminal output).
  2. **Middle-Left Subwindow**: **`PangolinViewer: Frame Viewer`** displaying monocular feature keypoints (cyan dots on doors, pillars, and ceiling lights).
  3. **Bottom-Left Subwindow**: **`PangolinViewer: Map Viewer`** displaying a **3D Visual SLAM map** containing 3D sparse point clouds, camera frustum pyramids (green), and co-visibility graph lines!
  4. **Right Subwindow**: RViz showing the robot footprint, 2D LaserScan points, local costmap (cyan lethal cost, magenta inflation zone), and planned trajectory (red path).

---

### 2.2 Major Reviewer Vulnerability: The "Hidden SLAM" Contradiction

#### 🚨 Critical Finding
The paper's entire thesis rests upon:
- *"mapless autonomous navigation without physical rangefinders"* (Abstract, Line 49)
- *"without a pre-built static occupancy map... Localization relied solely on wheel odometry fused with camera optical geometry"* (Line 513)
- Purely 2D reactive obstacle avoidance using synthesized `LaserScan` messages ingested into Nav2 local costmaps.

**YET, FIGURE 7 PROMINENTLY SHOWS PANGOLINVIEWER (ORB-SLAM3) RUNNING AND DISPLAYING A 3D SLAM MAP!**

#### Reviewer Attack Vectors
1. *"The authors claim their framework achieves mapless navigation without LiDAR, but Figure 7 clearly shows ORB-SLAM3 (PangolinViewer) running in the background and generating a 3D sparse feature map. Is the robot actually using Visual SLAM for localization or state estimation?"*
2. *"If ORB-SLAM3 is active, the navigation is NOT 'mapless' nor is it relying 'solely on wheel odometry'. The authors have omitted critical information about the SLAM backend."*
3. *"Why is a 3D SLAM viewer taking up half of the sequential navigation figure when the paper claims to be a purely 2D floor-segmentation-to-costmap conversion?"*

#### Root Cause Analysis
During physical experiments in the CGV Lab, the experimental workstation ran the lab's standard robot launch suite, which included an ORB-SLAM3 tracking node in an adjacent window. When the sequential multi-window desktop was captured, PangolinViewer was included alongside RViz and the floor detector.

#### Proposed Actionable Remedies (Ranked)
- **Option 1 (Strongly Recommended — Drop-in Image Cropping)**:
  - Crop `nav2_obstacle_avoidance_sequence.png` to remove the middle-left and bottom-left PangolinViewer panels.
  - The cropped figure will showcase:
    - **Left**: The monocular camera feed with real-time floor segmentation mask and obstacle detection bounding box.
    - **Right**: The corresponding Nav2 RViz local costmap, synthesized LaserScan beams, and planned evasive path.
  - *Advantage*: Completely eliminates the SLAM contradiction and keeps the visual presentation 100% focused on V-LiDAR + Nav2.
- **Option 2 (Textual Defense / Clarification)**:
  - If the figure cannot be cropped at this stage, add an explicit disclaimer in the caption and Section IV-B:
    > *"Note: The PangolinViewer panels shown in Fig. 7 were recorded from an auxiliary visual tracking node running in parallel for offline telemetry inspection; all online trajectory planning and obstacle avoidance strictly consumed only the 2D V-LiDAR LaserScan in the Nav2 local costmap without 3D map feedback or SLAM loop closures."*
