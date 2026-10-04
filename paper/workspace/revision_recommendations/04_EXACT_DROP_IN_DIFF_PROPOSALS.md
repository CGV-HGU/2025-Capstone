# 📝 Exact Drop-In LaTeX Diff Proposals for `main.tex`

> **Target File**: `paper/workspace/IEEE_Access/main.tex`  
> **Status**: Ready-to-Apply Unified Diff Specification  
> **Notice**: As per the user constraint, existing files have NOT been modified. This document provides the exact line numbers and drop-in replacements for the next editing stage.

---

## Diff Block 1: Author List Reordering (Task 1)
- **Target Lines**: 31–36

```diff
<<<< EXISTING (Lines 31-36)
\author{\uppercase{Min-Seok Lee}\authorrefmark{1},
\uppercase{Hyun-Mo Kang}\authorrefmark{1},
\uppercase{Hyunseo Lee}\authorrefmark{1},
\uppercase{Gunmin Yoo}\authorrefmark{1},
\uppercase{Hyunwoo Gu}\authorrefmark{1},
and \uppercase{Sung Soo Hwang}\authorrefmark{1}, \IEEEmembership{Senior Member, IEEE}}
==== PROPOSED
\author{\uppercase{Hyunwoo Gu}\authorrefmark{1},
\uppercase{Gunmin Yoo}\authorrefmark{1},
\uppercase{Hyunseo Lee}\authorrefmark{1},
\uppercase{Hyun-Mo Kang}\authorrefmark{1},
\uppercase{Min-Seok Lee}\authorrefmark{1},
and \uppercase{Sung Soo Hwang}\authorrefmark{1}, \IEEEmembership{Senior Member, IEEE}}
>>>>
```

---

## Diff Block 2: Author Email Order Synchronization (Task 1)
- **Target Line**: 38

```diff
<<<< EXISTING (Line 38)
\address[1]{School of Artificial Intelligence, Computer and Electrical Engineering, Handong Global University, Pohang 37554, Republic of Korea\allowbreak\ (e-mail: glen@handong.ac.kr;\allowbreak\ hmkang012@gmail.com;\allowbreak\ hslee@handong.ac.kr;\allowbreak\ gunminy@handong.ac.kr;\allowbreak\ 21800030@handong.ac.kr;\allowbreak\ sshwang@handong.edu)}
==== PROPOSED
\address[1]{School of Artificial Intelligence, Computer and Electrical Engineering, Handong Global University, Pohang 37554, Republic of Korea\allowbreak\ (e-mail: 21800030@handong.ac.kr;\allowbreak\ gunminy@handong.ac.kr;\allowbreak\ hslee@handong.ac.kr;\allowbreak\ hmkang012@gmail.com;\allowbreak\ glen@handong.ac.kr;\allowbreak\ sshwang@handong.edu)}
>>>>
```

---

## Diff Block 3: Equal-Contribution Footnote for All 5 Co-First Authors (Task 1)
- **Target Line**: 40

```diff
<<<< EXISTING (Line 40)
\tfootnote{This research was supported by the ANCHOR program Glocal University 30 through the Gyeongbuk ANCHOR CENTER, funded by the Ministry of Education (MOE) and the Gyeongsangbuk-do, Republic of Korea (2026-ANCHOR-15-119). This work was also supported in part by the National Research Foundation of Korea (NRF) grant funded by the Korean government (MSIT) (No. RS-2025-24683458) and the Handong Global University Academic Research Grant (No. 202500590001). \textit{Min-Seok Lee and Hyun-Mo Kang contributed equally to this work.}}
==== PROPOSED
\tfootnote{This research was supported by the ANCHOR program Glocal University 30 through the Gyeongbuk ANCHOR CENTER, funded by the Ministry of Education (MOE) and the Gyeongsangbuk-do, Republic of Korea (2026-ANCHOR-15-119). This work was also supported in part by the National Research Foundation of Korea (NRF) grant funded by the Korean government (MSIT) (No. RS-2025-24683458) and the Handong Global University Academic Research Grant (No. 202500590001). \textit{Hyunwoo Gu, Gunmin Yoo, Hyunseo Lee, Hyun-Mo Kang, and Min-Seok Lee contributed equally to this work.}}
>>>>
```

---

## Diff Block 4: Running Header Update (Task 1)
- **Target Lines**: 42–44

```diff
<<<< EXISTING (Lines 42-44)
\markboth
{Lee \headeretal: V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis}
{Lee \headeretal: V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis}
==== PROPOSED
\markboth
{Gu \headeretal: V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis}
{Gu \headeretal: V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis}
>>>>
```

---

## Diff Block 5: Abstract Scope & Terminology Harmonization (Task 2)
- **Target Line**: 49

```diff
<<<< EXISTING (Line 49)
In 20 consecutive real-world dynamic obstacle avoidance trials across a 10.0-m corridor trajectory without pre-built maps, the robot achieved a 90.0\% goal completion rate (18/20 runs) with an average lateral clearance of 1.22\,m, with diagnostic analysis identifying costmap inflation entrapment in the remaining runs.
==== PROPOSED
In 20 consecutive real-world closed-loop obstacle avoidance trials across a 10.0-m trajectory in an open testing hall without pre-built maps, the robot achieved a 90.0\% goal completion rate (18/20 runs) with an average lateral clearance of 1.22\,m, with diagnostic analysis identifying costmap inflation entrapment in the remaining runs.
>>>>
```

---

## Diff Block 6: Section IV-B Heading and Figure 6 Caption (Task 2)
- **Target Lines**: 459 & 465

```diff
<<<< EXISTING (Line 459)
\subsection{Experiment 2: Dynamic Obstacle Avoidance under Monocular Nav2 Navigation}
==== PROPOSED
\subsection{Experiment 2: Closed-Loop Autonomous Obstacle Avoidance under Monocular Nav2 Navigation}
>>>>

<<<< EXISTING (Line 465)
  \caption{Real-world test environment for dynamic obstacle avoidance. The OMO-R1 mobile robot navigates along a 10.0-meter trajectory facing a static cardboard box placed at the center ($Y=0.0$\,m, $X \approx 4.0\text{--}4.5$\,m).}
==== PROPOSED
  \caption{Real-world test arena for closed-loop autonomous obstacle avoidance. The OMO-R1 mobile robot (background right) navigates along a 10.0-meter trajectory facing a stationary cardboard box obstacle ($0.40 \times 0.40 \times 0.50$\,m) positioned at $X \approx 4.0\text{--}4.5$\,m on glossy, reflective floor tiles.}
>>>>
```

---

## Diff Block 7: Odometry Ground Truth & Localization Qualification (Task 2)
- **Target Line**: 513

```diff
<<<< EXISTING (Line 513)
As shown in Fig.~\ref{fig:exp2_setup}, a straight 10.0-meter relative goal command ($(X, Y) = (10.0, 0.0)$\,m) was issued without a pre-built static occupancy map. An unknown cardboard box ($0.40 \times 0.40 \times 0.50$\,m) was placed at approximately $X \approx 4.0\text{--}4.5$\,m along the robot's centerline. Localization relied solely on wheel odometry fused with camera optical geometry. The robot was tasked with autonomously detecting the obstacle, replanning around it, and navigating to the terminal waypoint. A total of 20 consecutive trials were executed.
==== PROPOSED
As shown in Fig.~\ref{fig:exp2_setup}, a straight 10.0-meter relative goal command ($(X, Y) = (10.0, 0.0)$\,m) was issued without a pre-built static occupancy map. A stationary cardboard box ($0.40 \times 0.40 \times 0.50$\,m) was placed at approximately $X \approx 4.0\text{--}4.5$\,m along the robot's centerline. Robot state estimation relied on onboard wheel odometry, while the synthesized LaserScan was ingested into the local costmap for reactive avoidance without global mapping. Trajectory coordinates were recorded via calibrated wheel odometry. The robot was tasked with autonomously detecting the obstacle, replanning around it, and navigating to the terminal waypoint across 20 consecutive trials.
>>>>
```

---

## Diff Block 8: Table 5 Throughput Consistency (Task 2)
- **Target Line**: 547

```diff
<<<< EXISTING (Line 547)
    \textbf{V-LiDAR (Proposed)} & \textbf{OpenVINO FP16 (Intel CPU)} & \textbf{2.84\,M} & \textbf{5.4\,MB} & \textbf{12.9 $\pm$ 1.9} & \textbf{77.4\,FPS} & \textbf{0.192\,m} & \textbf{Yes (7.7$\times$ margin)} \\
==== PROPOSED
    \textbf{V-LiDAR (Proposed)} & \textbf{OpenVINO FP16 (Intel CPU)} & \textbf{2.84\,M} & \textbf{5.4\,MB} & \textbf{12.9 $\pm$ 1.9} & \textbf{78.4\,FPS} & \textbf{0.192\,m} & \textbf{Yes (7.7$\times$ margin)} \\
>>>>
```

---

## Diff Block 9: Rebuttal Leak Removal in Ablation Introduction (Task 2)
- **Target Line**: 587

```diff
<<<< EXISTING (Line 587)
The ablation trajectory in Table~\ref{tab:ablation_study} clearly demonstrates how each engineering design directly resolves the reviewer's critique:
==== PROPOSED
The ablation trajectory in Table~\ref{tab:ablation_study} clearly demonstrates the cumulative performance contribution of each post-processing and acceleration module:
>>>>
```

---

## Diff Block 10: Section VI-B Reframing from "Infeasibility" to "Operational Boundaries" (Task 2)
- **Target Lines**: 621–623

```diff
<<<< EXISTING (Lines 621-623)
\subsection{Physical Rationale for Narrow Corridor Infeasibility}
\label{subsec:corridor_limitations}
A critical question arising from our evaluation is why experiments were conducted in an open testing hall ($\sim 6$\,m wide) rather than narrow corridors (width $1.8\text{--}2.2$\,m). Our empirical findings and theoretical derivations reveal that deploying this monocular pipeline inside narrow corridors without auxiliary sensors is physically infeasible:
==== PROPOSED
\subsection{Operational Boundaries in Confined Spaces and Geometric Scaling}
\label{subsec:corridor_limitations}
An important operational consideration is why navigation trials were conducted in an open testing hall ($\sim 6$\,m wide) rather than confined corridors (width $1.8\text{--}2.2$\,m). Our empirical findings and theoretical derivations demonstrate how camera geometry and costmap parameters define these operational boundaries:
>>>>
```

---

## Diff Block 11: Section VI-C Multi-Sensor Roadmap Qualification (Task 2)
- **Target Line**: 634

```diff
<<<< EXISTING (Line 634)
Because acoustic sensors are immune to optical glare and glass barriers, ingesting sonar into a Nav2 \texttt{RangeSensorLayer} while V-LiDAR populates the primary long-range ($2.2\text{--}6.0$\,m) \texttt{ObstacleLayer} guarantees zero-blind-spot coverage under \$40 total BOM.
==== PROPOSED
Because acoustic sensors are immune to optical glare and glass barriers, ingesting sonar into a Nav2 \texttt{RangeSensorLayer} while V-LiDAR populates the primary long-range ($2.2\text{--}6.0$\,m) \texttt{ObstacleLayer} represents a promising architectural roadmap to achieve continuous proximity coverage under \$40 total BOM.
>>>>
```

---

## Diff Block 12: Section VI-D Dynamic Pitch Compensation Analytical Clarification (Task 2)
- **Target Lines**: 640–644

```diff
<<<< EXISTING (Lines 640-644)
To decouple robot dynamics from ranging precision, an immediate extension is to feed real-time pitch estimates $\hat{\gamma}_t = \gamma_0 + \Delta \gamma_{\mathrm{IMU}}$ from the robot's onboard Inertial Measurement Unit (IMU) into a parameterized distance formula:
\begin{equation}
  D_{\mathrm{dynamic}}(v, u) = \frac{h \cdot \sec(\theta(u))}{\tan(\phi(v) + \gamma_0 + \Delta \gamma_{\mathrm{IMU}})}.
\end{equation}
Because evaluating trigonometric functions on modern CPUs takes less than $0.05\,\mu$s per obstacle channel ($N_{\mathrm{ang}}=141$), dynamic pitch compensation can be computed directly online for identified obstacle contact points without sacrificing real-time throughput.
==== PROPOSED
To decouple robot dynamics from ranging precision on aggressive platforms, we formalize an analytical formulation feeding real-time pitch estimates $\hat{\gamma}_t = \gamma_0 + \Delta \gamma_{\mathrm{IMU}}$ from the robot's onboard Inertial Measurement Unit (IMU) into a parameterized distance formula:
\begin{equation}
  D_{\mathrm{dynamic}}(v, u) = \frac{h \cdot \sec(\theta(u))}{\tan(\phi(v) + \gamma_0 + \Delta \gamma_{\mathrm{IMU}})}.
\end{equation}
In our 20 navigation trials, the static LUT was utilized under moderate cruising acceleration ($a \le 0.2$\,m/s$^2$, where chassis pitch deflection remained below $0.3^\circ$). Because evaluating trigonometric functions takes less than $0.05\,\mu$s per channel ($N_{\mathrm{ang}}=141$), this formulation provides a drop-in compensation framework for high-acceleration platforms without sacrificing real-time throughput.
>>>>
```

---

## Diff Block 13: Author Biographies Reordering on Page 14 (Task 1)
- **Target Lines**: 659–678

```diff
<<<< EXISTING (Lines 659-678: Order: Min-Seok, Hyun-Mo, Hyunseo, Gunmin, Hyunwoo, Sung Soo)
\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{minseok.png}}]{Min-Seok Lee}
is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include mobile robotics, visual SLAM, robust navigation, and sensor fusion.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{hyunmo.png}}]{Hyun-Mo Kang}
is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include deep learning deployment, edge AI acceleration, autonomous robotics, and visual perception.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{hyunseo.jpg}}]{Hyunseo Lee}
was born in Busan, South Korea, in 2002. He is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include mobile robotics, vision-based perception, SLAM, and autonomous navigation.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{gunmin.jpg}}]{Gunmin Yoo}
was born in Seoul, South Korea, in 2000. He received the B.S. degree in computer science from Handong Global University, Pohang, South Korea, in 2025. He is currently pursuing the M.S. degree in the School of Computer Science and Electrical Engineering at Handong Global University. Since 2025, he has been with CGV Lab and a part-time research intern at the Korea Institute of Robot and Convergence (KIRO). His research interests include embodied artificial intelligence, vision-language navigation, and mobile robot perception.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{hyunwoo.jpg}}]{Hyunwoo Gu}
was born in Seoul, South Korea, in 1999. He received the B.S. degree in computer science from Handong Global University, Pohang, South Korea, in 2025. He is currently pursuing the M.S. degree in the School of Computer Science and Electrical Engineering at Handong Global University. Since 2025, he has been with CGV Lab and a part-time research intern at the Korea Institute of Robot and Convergence (KIRO). His research interests include robotics, visual SLAM, artificial intelligence, and computer vision.
\end{IEEEbiography}
==== PROPOSED (New Order: Hyunwoo, Gunmin, Hyunseo, Hyun-Mo, Min-Seok, Sung Soo)
\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{hyunwoo.jpg}}]{Hyunwoo Gu}
was born in Seoul, South Korea, in 1999. He received the B.S. degree in computer science from Handong Global University, Pohang, South Korea, in 2025. He is currently pursuing the M.S. degree in the School of Computer Science and Electrical Engineering at Handong Global University. Since 2025, he has been with CGV Lab and a part-time research intern at the Korea Institute of Robot and Convergence (KIRO). His research interests include robotics, visual SLAM, artificial intelligence, and computer vision.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{gunmin.jpg}}]{Gunmin Yoo}
was born in Seoul, South Korea, in 2000. He received the B.S. degree in computer science from Handong Global University, Pohang, South Korea, in 2025. He is currently pursuing the M.S. degree in the School of Computer Science and Electrical Engineering at Handong Global University. Since 2025, he has been with CGV Lab and a part-time research intern at the Korea Institute of Robot and Convergence (KIRO). His research interests include embodied artificial intelligence, vision-language navigation, and mobile robot perception.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{hyunseo.jpg}}]{Hyunseo Lee}
was born in Busan, South Korea, in 2002. He is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include mobile robotics, vision-based perception, SLAM, and autonomous navigation.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{hyunmo.png}}]{Hyun-Mo Kang}
is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include deep learning deployment, edge AI acceleration, autonomous robotics, and visual perception.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{minseok.png}}]{Min-Seok Lee}
is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include mobile robotics, visual SLAM, robust navigation, and sensor fusion.
\end{IEEEbiography}

\begin{IEEEbiography}[{\includegraphics[width=1in,height=1.25in,clip,keepaspectratio]{sungsoo.png}}]{Sung Soo Hwang}
was born in Busan, South Korea, in 1983. He received the B.S. degree in computer science and electrical engineering from Handong Global University, Pohang, South Korea, in 2008 and the M.S. and Ph.D. degrees in electrical engineering from the Korea Advanced Institute of Science and Technology (KAIST), Daejeon, South Korea, in 2010 and 2015, respectively. He is currently an Associate Professor with the School of Computer Science and Electrical Engineering, Handong Global University, Pohang, South Korea. His research interests include robotics, artificial intelligence, visual SLAM, and neural-rendering-based 3D reconstruction.
\end{IEEEbiography}
>>>>
```
