#!/usr/bin/env python3
import os
import sys
import time
import numpy as np
import cv2

# ==============================================================================
# V-LiDAR Robot On-board Environment & Benchmark Verifier
# Target: cgv-omor1v2-01 (Intel NUC 14 Pro, Intel Core Ultra 7 155H)
# ==============================================================================

def print_header(title):
    print("\n" + "=" * 65)
    print(f"🔍 {title}")
    print("=" * 65)

def check_hardware_devices():
    print_header("1. 하드웨어 장치 및 포트 연결 점검")
    # 1) 카메라 점검
    cam_dev = "/dev/video0"
    if os.path.exists(cam_dev):
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                h, w = frame.shape[:2]
                print(f"✅ USB 웹캠 ({cam_dev}): 정상 연결 확인 (해상도: {w}x{h})")
            else:
                print(f"⚠️ USB 웹캠 ({cam_dev}): 장치는 있으나 영상 프레임 수신 실패")
            cap.release()
        else:
            print(f"⚠️ USB 웹캠 ({cam_dev}): 열 수 없음 (권한 또는 사용 중)")
    else:
        print(f"❌ USB 웹캠 ({cam_dev}): 장치 미감지")

    # 2) 모터/MCU 시리얼 포트 점검
    for p in ["/dev/ttyMCU", "/dev/ttyMotor", "/dev/ttyUSB0"]:
        if os.path.exists(p):
            print(f"✅ 시리얼 포트 ({p}): 정상 인식됨")
        else:
            print(f"ℹ️ 시리얼 포트 ({p}): 없음 (다른 심볼릭 링크 확인 필요)")

    # 3) Intel iGPU 렌더 노드 점검
    dri_dev = "/dev/dri/renderD128"
    if os.path.exists(dri_dev):
        print(f"✅ Intel Arc iGPU 렌더 디바이스 ({dri_dev}): 정상 인식됨")
    else:
        print(f"⚠️ Intel iGPU 렌더 디바이스 ({dri_dev}): 미감지 (/dev/dri 점검 필요)")

def check_openvino_devices():
    print_header("2. Intel OpenVINO 가속 런타임 점검")
    try:
        import openvino as ov
        core = ov.Core()
        devices = core.available_devices
        print(f"✅ OpenVINO Core 초기화 성공!")
        print(f"📌 사용 가능 디바이스 목록: {devices}")
        for dev in devices:
            full_name = core.get_property(dev, "FULL_DEVICE_NAME")
            print(f"   • [{dev}]: {full_name}")
        return True, devices
    except ImportError:
        print("❌ OpenVINO 라이브러리가 설치되지 않았습니다. ('pip install openvino' 필요)")
        return False, []
    except Exception as e:
        print(f"⚠️ OpenVINO 디바이스 조회 실패: {e}")
        return False, []

def check_luts():
    print_header("3. V-LiDAR LUT 기하학 보정 파일 검증")
    base_dir = os.path.dirname(os.path.abspath(__file__))
    scripts_dir = os.path.join(base_dir, "src", "freespace_detection", "scripts")

    files = {
        "col_to_ch_lut.npy": (320,),
        "distance_lut_2d.npy": (256, 320),
        "distance_lut.npy": (256,),
    }

    all_ok = True
    for fname, expected_shape in files.items():
        fpath = os.path.join(scripts_dir, fname)
        if os.path.exists(fpath):
            arr = np.load(fpath)
            if arr.shape == expected_shape:
                status = "✅ 2D 유클리드 보정 완료" if fname == "distance_lut_2d.npy" else "✅ 정상"
                print(f"  • {fname}: {status} (Shape: {arr.shape})")
            else:
                print(f"  • {fname}: ⚠️ Shape 불일치 (실제: {arr.shape}, 예상: {expected_shape})")
                all_ok = False
        else:
            print(f"  • {fname}: ❌ 파일 없음")
            all_ok = False
    return all_ok

def run_ai_benchmark(devices):
    print_header("4. YOLO11 세그멘테이션 실시간 추론 속도 벤치마크")
    from ultralytics import YOLO

    base_dir = os.path.dirname(os.path.abspath(__file__))
    scripts_dir = os.path.join(base_dir, "src", "freespace_detection", "scripts")
    openvino_dir = os.path.join(scripts_dir, "best_openvino_model")
    pt_path = os.path.join(scripts_dir, "best.pt")

    # 더미 입력 이미지 (320x256)
    dummy_img = np.zeros((256, 320, 3), dtype=np.uint8)

    # 1) OpenVINO 모델 벤치마크
    if os.path.exists(openvino_dir):
        print(f"📂 OpenVINO 모델 감지됨: {openvino_dir}")
        model = YOLO(openvino_dir, task='segment')

        for target_dev in ["GPU", "CPU"]:
            if target_dev in devices or target_dev == "CPU":
                try:
                    # Warm-up
                    for _ in range(5):
                        _ = model(dummy_img, device=target_dev, conf=0.4, verbose=False)
                    
                    times = []
                    for _ in range(30):
                        t0 = time.perf_counter()
                        _ = model(dummy_img, device=target_dev, conf=0.4, verbose=False)
                        times.append(time.perf_counter() - t0)
                    
                    avg_ms = np.mean(times) * 1000.0
                    fps = 1000.0 / avg_ms
                    print(f"⚡ [OpenVINO - {target_dev:3s}]: 평균 {avg_ms:5.1f} ms  |  {fps:5.1f} FPS  |  P95: {np.percentile(times, 95)*1000:.1f} ms")
                except Exception as e:
                    print(f"⚠️ [OpenVINO - {target_dev:3s}] 벤치마크 에러: {e}")
    else:
        print(f"ℹ️ OpenVINO 모델 없음. (변환 스크립트 실행 권장: python3 src/freespace_detection/scripts/export_openvino.py)")

    # 2) PyTorch CPU 모델 벤치마크 (기존 비교군)
    if os.path.exists(pt_path):
        try:
            pt_model = YOLO(pt_path)
            for _ in range(3):
                _ = pt_model(dummy_img, device="cpu", conf=0.4, verbose=False)
            
            times = []
            for _ in range(20):
                t0 = time.perf_counter()
                _ = pt_model(dummy_img, device="cpu", conf=0.4, verbose=False)
                times.append(time.perf_counter() - t0)
            avg_ms = np.mean(times) * 1000.0
            fps = 1000.0 / avg_ms
            print(f"🐢 [PyTorch  - CPU]: 평균 {avg_ms:5.1f} ms  |  {fps:5.1f} FPS  |  (기존 런타임)")
        except Exception as e:
            print(f"⚠️ PyTorch CPU 벤치마크 실패: {e}")

def main():
    print("\n" + "🚀" * 30)
    print("  OMORobot R1 Mini / V-LiDAR 온보드 PC 시스템 자가 진단 및 벤치마크")
    print("🚀" * 30)

    check_hardware_devices()
    ov_ok, devices = check_openvino_devices()
    check_luts()
    if ov_ok:
        run_ai_benchmark(devices)

    print_header("진단 완료")
    print("💡 이상이 없으면 ./run_ros2_launch.sh 를 실행하여 전체 자율주행을 시작하세요.\n")

if __name__ == "__main__":
    main()
