#!/usr/bin/env python3
import os
from ultralytics import YOLO

# ==============================================================================
# Intel NUC 14 Pro (Intel Core Ultra 7 155H, Intel Arc iGPU / NPU)
# YOLO11 OpenVINO FP16 가속 엔진 변환 스크립트
# ==============================================================================

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pt_path = os.path.join(script_dir, "best.pt")

    if not os.path.exists(pt_path):
        print(f"❌ Error: {pt_path} not found.")
        return

    print(f"🚀 Loading YOLO model: {pt_path}")
    model = YOLO(pt_path)

    print("⚡ Exporting to Intel OpenVINO format (FP16)...")
    # Intel Arc GPU 및 NPU 가속을 위한 FP16 OpenVINO IR 모델 생성
    exported_path = model.export(format="openvino", half=True, imgsz=(256, 320))

    print("=================================================================")
    print(f"✅ OpenVINO 모델 변환 완료: {exported_path}")
    print("💡 floor_detector_node 실행 시 자동으로 이 OpenVINO 모델을 로드합니다.")
    print("=================================================================")

if __name__ == "__main__":
    main()
