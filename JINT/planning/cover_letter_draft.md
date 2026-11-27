# JINT cover letter draft

Working copy for the corresponding author. Complete the bracketed fields using the verified trial results and editorial records.

[Submission date]

Dear Editor,

Please consider *V-LiDAR: Monocular Virtual Scans for Reactive Obstacle Avoidance in Structured Indoor Environments* as an original research article for the *Journal of Intelligent & Robotic Systems*. We investigate whether calibrated virtual scans from one RGB camera and wheel odometry support repeatable avoidance of stationary obstacles on a wheeled robot running ROS 2 Navigation2.

The study connects onboard processing cost with the scan updates and planning events observed on the robot. On archived RGB, the OpenVINO perception wrapper averaged 13.49 ms per frame. Replays of two stationary and two moving clips quantify the trade-off between scan stability and the time required to confirm new returns. Synchronized camera, scan, costmap, and odometry records document conditions that led to unreliable feedback or planning recovery. [Insert the verified formal outcome, using successful and evaluated attempts and the scope of the tested settings.] The work will interest readers who integrate vision-derived geometry with robot planning and control under onboard computing constraints.

Our earlier extended abstract, *Vision-Based 2D Scan Generation for Obstacle Avoidance Using Floor Segmentation*, appears in the ICCAS 2026 technical program as paper FrBT3.1. It covers the initial floor-to-scan pipeline and the 50-trial single-corridor evaluation with monocular SLAM also described in the related IJCAS manuscript. A related IEEE Access submission used the same virtual scan approach, compared it with monocular depth methods, and reported a separate 20-trial hall session. It received a rejection on 12 November 2026. The JINT manuscript discloses these sources, reanalyzes the 20 archived hall recordings using a stated odometric endpoint criterion, and adds analyses of scan timing, filter behavior, and failure recovery across other sites. [Insert the verified disposition of the related IJCAS manuscript and any updated ICCAS publication details.]

[Retain after verification: All authors have approved this version and agree to its exclusive submission to the Journal of Intelligent & Robotic Systems.]

Sincerely,

Sung Soo Hwang\
Handong Global University\
sshwang@handong.edu

---

## 공저자 확인용

| 선행 자료 | 확인한 상태 | JINT에서 공개할 관계 |
| --- | --- | --- |
| ICCAS 2026 FrBT3.1 | 2026-10-30 삿포로 구두 세션 프로그램에 저자 4명과 초록이 실려 있다. 공식 proceedings 페이지에는 2026-11-10 날짜의 배포본이 표시된다. 해당 논문의 개별 수록 파일·쪽수·DOI는 미확인이다. | 기존 단안 바닥 분할·LUT·Nav2 구성과 단일 복도 평가를 서론에서 프로그램 초록으로 인용한다. 개별 수록을 확인하면 서지를 갱신한다. [프로그램](https://ras.papercept.net/conferences/conferences/ICCAS2026/program/ICCAS2026_ContentListWeb_4.html#frbt3_01) · [Proceedings 페이지](https://sigongji.2026.iccas.org/Proceedings/HomeIF.asp) · [ICCAS–IJCAS 연계 안내](https://2026.iccas.org/iccas-news/?keyword=2026&mod=document&uid=1) |
| IJCAS 관련 원고 | 보관된 원고와 AE 의견 기록을 확인했다. 최종 처분과 현재 심사 상태는 저널 통지 또는 포털에서 확인해야 한다. | 원고에서 공유하는 바닥 투영과 초기 단일 복도 실험을 밝히고, JINT의 바퀴 odom 기반 다조건 본 주행 및 새 분석과 구분한다. [IJCAS 보관 원고](../../IJCAS/ICCAS/IJCAS_Manuscript.pdf) · [AE 의견 기록](../../paper/review/09_IJCAS_Review_Priority_and_3D_LiDAR_Option_20261118.md) |
| IEEE Access `Access-2026-51740` | 2026-11-12 거절 통지에 재투고 불가가 명시돼 있다. | 단안 가상 스캔, 반사·플리커 처리, MiDaS·Depth Anything 비교는 관련 원고에도 있었다. JINT에서는 같은 영상에서 CPU 시간을 다시 측정했고, 스캔 시각·필터 확인 지연·계획 회복과 기존 20회의 최종 odom 거리를 재분석했다. [IEEE Access 원고](../../paper/workspace/IEEE_Access/main.tex) · [거절 통지](../../paper/review/01_IEEE_Access_Decision_and_Raw_Reviews.md) |

JINT 지침은 같은 원고의 동시 심사를 제한하고 선행 연구를 확장할 때 재사용 범위를 밝히도록 안내한다. 교신저자가 IJCAS 상태와 공저자 동의를 확인한 뒤 위 편지의 선언을 확정한다. [JINT 투고 지침](https://link.springer.com/journal/10846/submission-guidelines#Ethical%20Responsibilities%20of%20Authors) · [Springer 커버레터 안내](https://support.springernature.com/en/support/solutions/articles/6000245674-cover-letter)
