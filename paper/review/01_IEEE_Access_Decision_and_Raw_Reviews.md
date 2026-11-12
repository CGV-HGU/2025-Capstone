# IEEE Access Decision Letter and Verbatim Reviewers' Comments

- **Journal**: IEEE Access
- **Manuscript ID**: `Access-2026-51740`
- **Title**: *V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis for Autonomous Mobile Robot Obstacle Avoidance*
- **Decision Date**: `12-Nov-2026`
- **Decision**: **Reject (Resubmissions will not be accepted)**
- **Associate Editor**: Dr. Kumaradevan Punithakumar (`punithak@ualberta.ca`)
- **Article Administrator**: Miss Aarthi Parthasarathy (`a.parthasarathy@ieee.org`)

---

## Editorial Decision Email (Verbatim)

```text
12-Nov-2026

Dear Mr. Kang:

I am writing to you regarding manuscript # Access-2026-51740 entitled "V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis for Autonomous Mobile Robot Obstacle Avoidance" which you submitted to IEEE Access.

In view of the criticisms of the reviewers found at the bottom of this letter, your manuscript has not been recommended for publication in IEEE Access.  Unfortunately, we will not accept resubmissions of this article.

Thank you for considering IEEE Access for the publication of your research.

Sincerely,

Dr. Kumaradevan Punithakumar
Associate Editor, IEEE Access
punithak@ualberta.ca
```

---

## Reviewer 1 (Verbatim)

### Comments:
1. The proposed method reconstructs virtual laser scans from floor segmentation boundaries and suffers from inherent near-field blind spots. Although a blind-spot formula is derived, systematic tests under different camera mounting heights and tilt angles are missing. Blind spots directly cause obstacle avoidance failures in narrow corridors. Supplementary experiments with varied mounting parameters are required, and the strategy to mitigate blind-spot risks via parameter selection should be discussed.
2. The benchmark only compares inference speed with Depth Anything V2 Small. Comparisons in terms of accuracy and robustness against lightweight monocular depth estimation and virtual LiDAR methods (e.g., FastDepth, Lite-Mono) are absent. Additional quantitative metrics such as range error and false detection rate under reflective conditions should be added to fully demonstrate the overall advantages of the proposed pipeline.
3. Multimodal algorithms and multi-level features algorithms have great inspiration value for this task, such as Short-term load forecasting using an improved PatchTST model with time–frequency feature fusion, YOLO-TCS: an enhanced multi-scale network for traffic sign detection integrating multi-level feature fusion and attention, Multi-task learning for hand heat trace time estimation and identity recognition.
4. Closed-loop obstacle avoidance experiments are only conducted within a single test hall without pre-built maps, with limited scene diversity. Polished-floor specular reflection and transparent glass are key pain points of physical LiDAR highlighted in the paper. Dedicated test scenarios with reflective floors, glass partitions, low illumination and dynamic pedestrians should be constructed to quantitatively evaluate the performance of V-LiDAR under these challenging cases.

### Additional Questions:
- **Please confirm that you have reviewed all relevant files, including supplementary files and any author response files, which can be found in the "View Author's Response" link above (author responses will only appear for resubmissions):** `Yes, all files have been reviewed`
- **1) Does the paper contribute to the body of knowledge?:** `yes`
- **2) Is the paper technically sound?:** `yes`
- **3) Is the subject matter presented in a comprehensive manner?:** `yes`
- **4) Are the references provided applicable and sufficient?:** `yes`
- **5) Are there references that are not appropriate for the topic being discussed?:** `No`
- **5a) If yes, then please indicate which references should be removed.:** *(empty)*

---

## Reviewer 2 (Verbatim)

### Comments:
(1) The use of monocular vision is justified only under strict cost, weight, and power constraints. For general mobile robot obstacle avoidance, a commodity 2D LiDAR remains a substantially more reliable and straightforward solution than the proposed virtual scan synthesis framework. The authors eventually recommend additional ultrasonic or ToF sensors to compensate for blind-spot limitations, weakening the claim of camera-only obstacle avoidance (Section VI-C, page 12).

(2) The manuscript emphasizes reductions in hardware cost and sensor weight but omits discussion of the increased perception software complexity introduced by deep-learning-based floor segmentation, dataset collection, model training, camera calibration, etc. A holistic cost-benefit analysis comparing total system complexity against commodity 2D LiDAR solutions is missing.

(3) The reliability of the proposed method remains fundamentally dependent on learned floor segmentation and camera operating conditions. Unlike physical LiDAR sensors that directly measure geometric range, the proposed method relies on several intermediate perception stages and strong environmental assumptions. Consequently, the manuscript does not provide sufficient evidence that the proposed approach can achieve comparable robustness, accuracy, and operational reliability to commodity 2D LiDAR systems across diverse real-world deployment scenarios.

(4) The experimental section lacks comparison with state-of-the-art or practically deployed camera-based obstacle avoidance systems. Benchmarking only against general-purpose monocular depth estimation models does not adequately establish the advantages of the proposed framework relative to the most relevant alternatives.

(5) The manuscript presents a comprehensive computational latency analysis and demonstrates that the perception pipeline satisfies real-time constraints on the target hardware. However, the evaluation is restricted to static obstacle scenarios. The impact of perception latency, temporal filtering, and persistence mechanisms on obstacle detection delay in dynamic environments is not investigated.

(6) Although the system is validated on a physical robot, the computational evaluation is limited to a relatively high-performance Intel Core Ultra 7 platform (Section VII, page 13). The manuscript does not demonstrate whether the proposed perception pipeline can maintain comparable performance on resource-constrained embedded robotic computers, which are the most likely deployment targets for a low-cost camera-based alternative to LiDAR.

(7) The manuscript reports point estimates of obstacle distance but does not quantify the uncertainty associated with the generated virtual LaserScan observations. Since the proposed approach relies on learned floor segmentation, prediction confidence and uncertainty are critical for safety-critical navigation. The paper lacks confidence calibration, uncertainty propagation, confidence-aware planning, or out-of-distribution detection mechanisms. Consequently, it remains unclear how the system would behave under low-confidence perception conditions.

(8) Experiments are conducted mainly in a single indoor hall using one robot, one camera setup, and a simple obstacle scenario, limiting evidence of robustness and generalization.

(9) The derived 2.18 m near-field blind spot prevents reliable sensing of nearby low-profile obstacles, restricting deployment in realistic indoor environments (Section VI-B, page 12).

(10) The paper does not study robustness against unseen floor materials, lighting variations, shadows, clutter, or environmental domain shifts.

### Additional Questions:
- **Please confirm that you have reviewed all relevant files, including supplementary files and any author response files, which can be found in the "View Author's Response" link above (author responses will only appear for resubmissions):** `Yes, all files have been reviewed`
- **1) Does the paper contribute to the body of knowledge?:** `This paper presents V-LiDAR, a monocular vision-based alternative to physical LiDAR for mobile robot obstacle avoidance. A YOLOv11n-seg floor segmentation model converts RGB images into virtual ROS2 LaserScan data through geometric projection, lookup tables, and temporal filtering. While cost-effective, the approach suffers from a 2.18 m near-field blind spot and has not demonstrated LiDAR-level robustness across diverse environments.`
- **2) Is the paper technically sound?:** `The authors need to address or clarify some major issues. I put the questions and suggestions in the "Comments to authors" section.`
- **3) Is the subject matter presented in a comprehensive manner?:** `The manuscript is generally well organized, readable, and professionally prepared.`
- **4) Are the references provided applicable and sufficient?:** `The literature review overlooks key competing work on virtual LiDAR, image-to-LaserScan conversion, and camera-based obstacle avoidance. As a result, the paper is insufficiently positioned against state-of-the-art methods.`
- **5) Are there references that are not appropriate for the topic being discussed?:** `No`
- **5a) If yes, then please indicate which references should be removed.:** *(empty)*

---

## Reviewer 3 (Verbatim)

### Comments:
The paper proposes V-LiDAR, a monocular floor-segmentation pipeline that synthesizes ROS 2 LaserScan messages for Nav2-based reactive obstacle avoidance. The system combines a fine-tuned YOLOv11n-seg model, camera-geometry lookup tables, morphological closing, temporal EMA/persistence filtering, and a physical mobile-robot demonstration. The problem is relevant, the implementation is nontrivial, and the failure analysis is more candid than in many system papers.

However, the paper still has some issues that require revision. The revision comments are as follows:

1. Add a same-platform physical LiDAR reference and a fair comparison. The abstract and Introduction repeatedly position V-LiDAR as a low-cost alternative to planar LiDAR, and the paper discusses LiDAR failure modes, but no physical LiDAR is evaluated. Please compare against a representative 2-D LiDAR such as an RPLIDAR A2/S2 or an equivalent sensor on the same robot, trajectory, lighting, and obstacle set. Without such a comparison, the LiDAR-substitution claim is not established.

2. Provide the complete segmentation-model development and evaluation protocol (Section III-B).
The paper states that YOLOv11n-seg was fine-tuned on custom indoor datasets, but does not report the dataset source, size, annotation protocol, scene composition, train/validation/test split, augmentation, training hyperparameters, or segmentation accuracy.

3. Expand and stratify the static and closed-loop validation. The current validation is limited to one 2.50 m static distance with three targets and one 10 m straight closed-loop route with one cardboard box in a wide hall. Please add distance-stratified tests across the usable range, including points near the 2.18 m blind spot, multiple bearings, floor materials, lighting conditions, and obstacle properties, as well as partial occlusion and at least one dynamic obstacle or pedestrian.

4. The two physical failures are attributed to costmap inflation, aggressive DWB weights, and an incorrect use_sim_time setting, but the manuscript does not report results after these fixes. Please rerun the full physical trial set with the corrected configuration and report before/after outcomes.

5. Clarify the technical claims, operating envelope, and timing terminology. The near-field blind spot is acknowledged, but the abstract, title framing, and conclusion still imply a broad LiDAR alternative. Please state the operating envelope explicitly: open, well-lit indoor spaces, static obstacles at approximately 2.2–6.0 m, and no validated near-field coverage.

### Additional Questions:
- **Please confirm that you have reviewed all relevant files, including supplementary files and any author response files, which can be found in the "View Author's Response" link above (author responses will only appear for resubmissions):** `Yes, all files have been reviewed`
- **1) Does the paper contribute to the body of knowledge?:** `The paper addresses a practical integration gap between monocular floor segmentation and ROS 2 Nav2 LaserScan-based local costmaps. The combination of a lightweight segmentation model, camera-geometry lookup tables, temporal filtering, and a physical robot demonstration is useful to applied robotics. The contribution is primarily system integration rather than a new perception principle, so it should be framed at that level. Publication would require stronger evidence that the integrated system is reliable beyond the limited test conditions.`
- **2) Is the paper technically sound?:** `Partly. The pipeline and runtime budget are described clearly, and the authors candidly report the 2.178 m near-field blind spot and two failed physical trials. However, several central claims are not yet adequately supported:
(1) no physical LiDAR reference is evaluated on the same platform and route;
(2) the static test covers only one reference distance using 25 temporally correlated frames;
(3) the two failures are attributed to configuration errors, but the corrected configuration is not retested.`
- **3) Is the subject matter presented in a comprehensive manner?:** `Partly. The paper is logically organized and covers system architecture, geometry, implementation, experiments, failure analysis, and limitations. It is not yet comprehensive in model development, independent ground truth, environmental and obstacle diversity, statistical uncertainty, safety metrics, and reproducibility. These omissions are material because the paper makes real-time robustness and obstacle-avoidance claims.`
- **4) Are the references provided applicable and sufficient?:**
  - *System notice*: `Please note that suggesting one's own references as a reviewer if not relevant to the article, or at an excessive level, is unethical and is not permitted. If you are suggesting additional references they must be entered in the text box provided. All suggestions must include full bibliographic information plus a DOI.`
  - *Reviewer 3 Comments*: `Yes. The references are applicable to lightweight segmentation, monocular depth, mobile robot navigation, ROS 2, and active range sensing. No reference is inappropriate.`
- **5) Are there references that are not appropriate for the topic being discussed?:** `No`
- **5a) If yes, then please indicate which references should be removed.:** `NO`
