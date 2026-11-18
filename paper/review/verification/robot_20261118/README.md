# 2026-11-18 로봇 원본 기록 점검

`omo-r1` SSH 접속으로 현재 소스 해시와 bag 내용을 읽기 전용으로 점검했다. ROS 노드 초기화·메시지 재생·모델 추론·로봇 구동·원격 코드/설정 변경은 수행하지 않았다. 비밀번호·키·토큰은 이 폴더에 저장하지 않았다.

| 자료 | 내용 |
| --- | --- |
| [inspection_manifest.json](inspection_manifest.json) | 점검 범위, 현재 로컬/로봇 버전, 소스·모델 해시, 제한 |
| [inventory.json](inventory.json) | 일반 28/휴지통 12개 폴더, 원본 경로, 메시지 종류·개수·기간 |
| [twenty_runs.json](twenty_runs.json) | 기존 20개 주행의 원본 CDR 메시지 기반 요약 |
| [sampled_frames.json](sampled_frames.json) | 영상 있는 비압축 6개 bag에서 10/50/90% 메시지 표본, 총 18장. 원본 메시지 ID·시각·인접 scan/odom. 이미지 본문 제외 |
| [twenty_run_diagnostics.png](twenty_run_diagnostics.png) / [PDF](twenty_run_diagnostics.pdf) | 종료 x·끝부분 정지 시간·scan/채널 발행률. 새 성능 실험이나 성공률 그림이 아님 |
| [render_diagnostics.py](render_diagnostics.py) | 저장한 `twenty_runs.json`에서 진단 그림 재생성. 로봇 접속/ROS 실행 없음 |

대표 영상 모음은 Git 밖의 `/storage/emulated/0/Documents/VLidar_Review_20261118/rosbag_대표프레임_18장.jpg`에 있다. JPEG 생성 시 다시 인코딩한 대표 프레임이며 원본 영상의 화소 해시가 아니다. `preview_jpeg_sha256`은 전달받은 표본 JPEG 바이트의 해시다.

## 읽기 방법과 수치 해석

비압축 bag은 SQLite URI `mode=ro`로 열어 topics/messages를 조회하고 ROS Humble의 메시지 역직렬화 함수로 CDR를 읽었다. `rclpy.init()`이나 publisher/subscriber를 생성하지 않았다. 영상은 `sensor_msgs/Image`를 변환해 대표 표본만 전송했다. 대형 원본은 내려받지 않았다.

26개 폴더는 SQLite를 조회했고, 압축본만 있는 14개는 metadata만 읽었다. 이번에 압축 해제·무결성 재검증은 하지 않았다. 기존 점검에서 손상으로 분류된 압축본도 있으므로 40개 모두 사용 가능하다고 해석하지 않는다. SQLite 조회 성공 역시 모든 영상 프레임·전체 DB의 무결성 검증을 뜻하지 않는다.

- 주기: `(메시지 수-1)/(마지막-첫 기록 시각)`. 이는 수신/기록 시각 기준 발행률이며 영상 획득부터의 지연이 아니다.
- `stationary_tail_s`: `abs(linear.x)>0.01m/s` 또는 `abs(angular.z)>0.02rad/s`인 마지막 odom 이후 끝까지의 시간. 접촉·성공·실패 판정이 아니다.
- `scan_share_without_new_channel_in_recorder_order`: scan 사이에 새 채널 메시지가 기록되지 않은 비율. 서로 다른 토픽의 기록 순서 정렬이므로 인과적 대응·실제 관측 age로 해석하지 않는다.
- `end_x_m>=9.5`인 18개는 위치의 대리 지표다. bag에 목표·action 결과·외부 접촉/개입 기록이 없어 검증된 성공률이 아니다.
- `last_15s_cmd_nonzero_fraction`의 null은 해당 창에 명령 메시지가 없다는 뜻이다. 0 명령을 확인했다는 뜻이 아니다.
- `floor_true`는 하단 ROI의 Bool이다. 전체 장면·표적 검출의 정확도가 아니다. 수신 간 비동기성 때문에 channel과 Bool 메시지 수가 조금 다를 수 있다.
- 표본의 `nearest_scan`/`nearest_odom`은 bag 기록 시각상 가장 가까운 메시지다. 영상과 해당 scan의 입력 대응을 증명하지 않는다.

20개 bag 취득일은 2026-09-21이며 현재 소스와의 버전 일치는 확인할 수 없다. 주행 방식·원인·개선 효과를 과거 데이터만으로 단정하지 않는다. [현행 계획](../../planning/04_Robot_Checked_Code_and_Experiment_Plan_20261118.md)에 실제 사용 범위를 정리했다.
