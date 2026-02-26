#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
from datetime import datetime
from typing import Optional, Dict, Any, List

import pyrebase
from firebase_admin import credentials, initialize_app

# ============================
# Firebase 공통 초기화
# ============================

KEY_PATH = os.environ.get(
    "FIREBASE_KEY_PATH",
    "/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/db/tycoon-9ef3a-firebase-adminsdk-fbsvc-bd2a330770.json",
)

# ⚠️ 여기 네 Firebase Web 설정 값 넣기
FIREBASE_CONFIG = {
  "apiKey": "AIzaSyB5vyk8RdvoxyBoGfQ0IVqgSvSKd8CM0Ls",
  "authDomain": "tycoon-9ef3a.firebaseapp.com",
  "databaseURL": "https://tycoon-9ef3a-default-rtdb.asia-southeast1.firebasedatabase.app",
  "projectId": "tycoon-9ef3a",
  "storageBucket": "tycoon-9ef3a.firebasestorage.app",
  "messagingSenderId": "801741125493",
  "appId": "1:801741125493:web:cd1aa782a2bb5e1fba053c"
}

# firebase_admin 초기화
try:
    cred = credentials.Certificate(KEY_PATH)
    initialize_app(cred)
except Exception:
    # 이미 초기화 됐으면 그냥 통과
    pass

# pyrebase RTDB 클라이언트
firebase = pyrebase.initialize_app(FIREBASE_CONFIG)
rtdb = firebase.database()

# RTDB path 정의 (숫자 prefix로 순서 고정)
PATH_DEBUG           = "01_debug"
PATH_VIEW_LIVE       = "02_View_live"
PATH_UR10_LOG_ROOT   = "03_UR_10 로그"
PATH_IWHUB_LOG_ROOT  = "04_iw_hub 로그"
PATH_TOTAL_LOGS      = "05_total_process_logs"
PATH_TOTAL_LOGS_CNT  = "06_total_process_logs_count"
PATH_DT_LOGS         = "07_DT_logs"
PATH_METRICS_ROOT    = "08_metrics"

# ============================
# 공용 시간 유틸
# ============================

def now_ms() -> int:
    return int(time.time() * 1000)


def now_str(ts_ms: Optional[int] = None) -> str:
    if ts_ms is None:
        ts_ms = now_ms()
    return datetime.fromtimestamp(ts_ms / 1000.0).strftime("%Y-%m-%d %H:%M:%S")


# ============================
# total_process_logs 공통 헬퍼
# ============================

def _append_total_process_log(source: str, data: Dict[str, Any]) -> None:
    """
    total_process_logs/1,2,3,... 순서대로 기록.
    모든 이벤트를 시간순으로 모아볼 때 사용.
    KPI 분석 시 box_id, process_step, source 기반으로 join/필터 가능.
    """
    d = dict(data)  # copy
    if "timestamp" not in d:
        ts = now_ms()
        d["timestamp"] = ts
        d["time_str"] = now_str(ts)

    d["source"] = source  # "UR_Loader", "UR_Sorter", "Cam_Detect" 등

    # 현재 카운터 읽기
    try:
        snap = rtdb.child(PATH_TOTAL_LOGS_CNT).get()
        cur = snap.val() if snap.val() is not None else 0
        idx = int(cur) + 1
    except Exception:
        idx = 1

    try:
        rtdb.child(PATH_TOTAL_LOGS).child(str(idx)).set(d)
        rtdb.child(PATH_TOTAL_LOGS_CNT).set(idx)
        print(f"[Firebase] total_process_logs/{idx}:", d)
    except Exception as e:
        print("[Firebase] ERROR _append_total_process_log:", repr(e))


# ============================
# View_live 업데이트 (라인 전체 상태)
# ============================

def update_view_live(
    box_count_normal: int,
    box_count_fragile: int,
    state: str,
    conveyor_queue_len: Optional[int] = None,
    iw_state: Optional[str] = None,
) -> None:
    """
    View_live/
        box_count_normal
        box_count_fragile
        state
        conveyor_queue_len   (optional)
        iw_state             (optional)
        updated_at
        updated_at_str

    - KPI2(MSE/RMSE)에서 sys_count로 사용 가능
    - KPI5(병목)에서 queue_len, iw_state 분석에 사용 가능
    """
    ts = now_ms()
    payload: Dict[str, Any] = {
        "box_count_normal": int(box_count_normal),
        "box_count_fragile": int(box_count_fragile),
        "state": state,
        "updated_at": ts,
        "updated_at_str": now_str(ts),
    }
    if conveyor_queue_len is not None:
        payload["conveyor_queue_len"] = int(conveyor_queue_len)
    if iw_state is not None:
        payload["iw_state"] = iw_state

    try:
        rtdb.child(PATH_VIEW_LIVE).set(payload)
        print("[Firebase] update_view_live:", payload)
    except Exception as e:
        print("[Firebase] ERROR update_view_live:", repr(e))


# ============================
# UR_10 로그: Loader
#  - 컨베이어 적재 시점 포함 (KPI6, E2E 시작지점)
# ============================

def log_ur_loader(
    robot_id: str,
    state: str,
    process_step: str,
    step_label: str,
    box_id: Optional[str] = None,
) -> None:
    """
    UR_Loader 동작 로그.
    - box_id를 넣으면, 나중에 E2E 처리시간 계산에 사용 가능.
    """
    ts = now_ms()
    payload: Dict[str, Any] = {
        "robot_id": robot_id,
        "state": state,
        "process_step": process_step,
        "step_label": step_label,
        "timestamp": ts,
        "time_str": now_str(ts),
    }
    if box_id is not None:
        payload["box_id"] = box_id

    try:
        rtdb.child(PATH_UR10_LOG_ROOT).child("UR_Loader 로그").push(payload)
        print("[Firebase] log_ur_loader:", payload)
    except Exception as e:
        print("[Firebase] ERROR log_ur_loader:", repr(e))

    _append_total_process_log("UR_Loader", payload)


# ============================
# 컨베이어에 박스를 올리는 시점 (KPI6: Sorting Latency 시작)
# ============================

def log_conveyor_load(
    box_id: str,
    robot_id: Optional[str] = None,
    pallet_id: Optional[str] = None,
) -> None:
    """
    컨베이어에 박스를 올린 시점.
    - T_sort(i) = t_cam_detect(i) - t_on_conveyor(i) 에서 t_on_conveyor가 됨.
    - E2E 기준을 컨베이어 시작으로 잡고 싶으면 여기 timestamp를 사용.
    """
    ts = now_ms()
    payload: Dict[str, Any] = {
        "event": "ON_CONVEYOR",
        "box_id": box_id,
        "timestamp": ts,
        "time_str": now_str(ts),
    }
    if robot_id is not None:
        payload["robot_id"] = robot_id
    if pallet_id is not None:
        payload["pallet_id"] = pallet_id

    try:
        rtdb.child(PATH_UR10_LOG_ROOT).child("Conveyor_Load 로그").push(payload)
        print("[Firebase] log_conveyor_load:", payload)
    except Exception as e:
        print("[Firebase] ERROR log_conveyor_load:", repr(e))

    _append_total_process_log("Conveyor_Load", payload)


# ============================
# 카메라 탐지 시점 (KPI3: Stock Sync 시작, KPI6: Sorting Latency 끝)
# ============================

def log_cam_detect(
    box_id: str,
    pred_label: str,
    camera_id: str = "cam_1",
    confidence: Optional[float] = None,
) -> None:
    """
    카메라가 박스 i를 인식/분류 완료한 시점.
    - T_sync(i) = t_dt_update - t_cam_detect에서 t_cam_detect로 사용.
    - T_sort(i) = t_cam_detect - t_on_conveyor에서 끝 시점으로 사용.
    - 분류 정확도/Confusion Matrix에서 pred_label로 사용.
    """
    ts = now_ms()
    payload: Dict[str, Any] = {
        "box_id": box_id,
        "camera_id": camera_id,
        "pred_label": pred_label,  # "FRAGILE" / "NORMAL"
        "timestamp": ts,
        "time_str": now_str(ts),
    }
    if confidence is not None:
        payload["confidence"] = float(confidence)

    try:
        rtdb.child(PATH_UR10_LOG_ROOT).child("Cam_Detect 로그").push(payload)
        print("[Firebase] log_cam_detect:", payload)
    except Exception as e:
        print("[Firebase] ERROR log_cam_detect:", repr(e))

    _append_total_process_log("Cam_Detect", payload)


# ============================
# UR_10 로그: Sorter (팔레트/버퍼 적재)
# ============================

def log_ur_sorter(
    robot_id: str,
    state: str,
    pallet_id: str,
    box_type: str,        # GT label or final type ("FRAGILE"/"NORMAL")
    process_step: str,
    step_label: str,
    box_id: Optional[str] = None,
) -> None:
    """
    UR_Sorter 동작 로그.
    - pallet_id: "fragile_1", "normal_1" 등
    - box_type: GT label (시뮬 설정 라벨) 또는 최종 라벨
    - box_id: 개별 박스를 추적하기 위한 ID (권장)
    """
    ts = now_ms()
    payload: Dict[str, Any] = {
        "robot_id": robot_id,
        "state": state,
        "pallet_id": pallet_id,
        "box_type": box_type,
        "process_step": process_step,
        "step_label": step_label,
        "timestamp": ts,
        "time_str": now_str(ts),
    }
    if box_id is not None:
        payload["box_id"] = box_id

    try:
        rtdb.child(PATH_UR10_LOG_ROOT).child("UR_Sorter 로그").push(payload)
        print("[Firebase] log_ur_sorter:", payload)
    except Exception as e:
        print("[Firebase] ERROR log_ur_sorter:", repr(e))

    _append_total_process_log("UR_Sorter", payload)


# ============================
# iw_hub 로그
# ============================

def log_iw_hub(
    robot_id: str,
    state: str,
    pallet_id: str,
    pose: Optional[Dict[str, float]],
    process_step: str,
    step_label: str,
    box_ids: Optional[List[str]] = None,
) -> None:
    """
    pose 예시: {"x": 1.0, "y": 2.0, "theta": 1.57}
    - box_ids : 이 팔레트에 실려 있는 박스 id 리스트 (E2E 계산용)
    """
    ts = now_ms()
    payload: Dict[str, Any] = {
        "robot_id": robot_id,
        "state": state,
        "pallet_id": pallet_id,
        "pose": pose or {},
        "process_step": process_step,
        "step_label": step_label,
        "timestamp": ts,
        "time_str": now_str(ts),
    }
    if box_ids:
        payload["box_ids"] = list(box_ids)

    try:
        rtdb.child(PATH_IWHUB_LOG_ROOT).push(payload)
        print("[Firebase] log_iw_hub:", payload)
    except Exception as e:
        print("[Firebase] ERROR log_iw_hub:", repr(e))

    _append_total_process_log("iw_hub", payload)


# ============================
# Digital Twin 업데이트 로그 (KPI3: Stock Sync 끝 시점)
# ============================

def log_dt_update(
    box_id: str,
    dt_state: Dict[str, Any],
) -> None:
    """
    DT(재고 DB) 상태가 업데이트 완료된 시점.
    - T_sync(i) = t_dt_update(i) - t_cam_detect(i) 의 t_dt_update용.
    - dt_state 에 zone, count 등 재고 상태 스냅샷 넣으면 분석 용이.
    """
    ts = now_ms()
    payload: Dict[str, Any] = {
        "box_id": box_id,
        "event": "DT_UPDATE",
        "dt_state": dt_state,
        "timestamp": ts,
        "time_str": now_str(ts),
    }

    try:
        rtdb.child(PATH_DT_LOGS).push(payload)
        print("[Firebase] log_dt_update:", payload)
    except Exception as e:
        print("[Firebase] ERROR log_dt_update:", repr(e))

    _append_total_process_log("DT_Update", payload)


# ============================
# (선택) 세션/실험 단위 KPI 요약 저장
# ============================

def save_session_metrics(
    session_id: str,
    metrics: Dict[str, Any],
) -> None:
    """
    세션 단위 KPI 요약 값 저장용.
    예:
        metrics = {
            "gt_fragile": 2,
            "gt_normal": 2,
            "sys_fragile": 2,
            "sys_normal": 2,
            "mse_fragile": 0.0,
            "mse_normal": 0.0,
            "session_cycle_time_ms": 12345,
        }
    """
    ts = now_ms()
    payload = dict(metrics)
    payload["timestamp"] = ts
    payload["time_str"] = now_str(ts)

    try:
        rtdb.child(PATH_METRICS_ROOT).child(session_id).set(payload)
        print(f"[Firebase] save_session_metrics/{session_id}:", payload)
    except Exception as e:
        print("[Firebase] ERROR save_session_metrics:", repr(e))


# ============================
# 실행 시 전체 로그 초기화
# ============================

def clear_all_logs() -> None:
    """
    테스트용: 기존 로그들을 싹 비우는 함수.
    실제 운영에서는 __main__에서 이 함수 호출 부분을 제거하면 됨.
    """
    paths = [
        PATH_DEBUG,
        PATH_VIEW_LIVE,
        PATH_UR10_LOG_ROOT,
        PATH_IWHUB_LOG_ROOT,
        PATH_TOTAL_LOGS,
        PATH_TOTAL_LOGS_CNT,
        PATH_DT_LOGS,
        PATH_METRICS_ROOT,
    ]
    for p in paths:
        try:
            rtdb.child(p).remove()
            print(f"[Firebase] cleared: /{p}")
        except Exception as e:
            print(f"[Firebase] clear ERROR /{p}:", e)


# ============================
# 직접 실행했을 때 간단 테스트
# ============================

if __name__ == "__main__":
    print("=== db_helper_kpi 테스트 시작 ===")

    clear_all_logs()

    # debug
    ts = now_ms()
    try:
        rtdb.child(PATH_DEBUG).child("test").set({
            "msg": "hello from db_helper_kpi.py",
            "ts": ts,
            "ts_str": now_str(ts),
        })
        print("[Firebase] debug/test write OK")
    except Exception as e:
        print("[Firebase] debug/test write ERROR:", e)

    # View_live
    update_view_live(
        box_count_normal=0,
        box_count_fragile=0,
        state="INIT",
        conveyor_queue_len=0,
        iw_state="IDLE",
    )

    # UR_Loader + Conveyor load
    box_id = "box_001"
    log_ur_loader(
        robot_id="ur10_loader",
        state="RUNNING",
        process_step="PICK_START",
        step_label="박스 픽 시작",
        box_id=box_id,
    )
    log_conveyor_load(
        box_id=box_id,
        robot_id="ur10_loader",
        pallet_id="normal_1",
    )

    # Cam detect
    time.sleep(0.1)
    log_cam_detect(
        box_id=box_id,
        pred_label="NORMAL",
        camera_id="cam_top",
        confidence=0.98,
    )

    # UR_Sorter
    time.sleep(0.1)
    log_ur_sorter(
        robot_id="ur10_sorter",
        state="DONE",
        pallet_id="normal_1",
        box_type="NORMAL",  # GT or final
        process_step="PLACE_DONE",
        step_label="Normal 팔레트에 적재 완료",
        box_id=box_id,
    )

    # iw_hub
    time.sleep(0.1)
    log_iw_hub(
        robot_id="iw_hub_1",
        state="MOVING",
        pallet_id="normal_1",
        pose={"x": 1.0, "y": 2.0, "theta": 1.57},
        process_step="IW_PICKUP_START",
        step_label="Normal 팔레트 픽업 시작",
        box_ids=[box_id],
    )

    # DT Update (재고 동기화 끝)
    time.sleep(0.1)
    log_dt_update(
        box_id=box_id,
        dt_state={"zone": "NORMAL_ZONE", "count_normal": 1, "count_fragile": 0},
    )

    # 세션 메트릭 예시
    save_session_metrics(
        session_id="session_1",
        metrics={
            "gt_normal": 2,
            "gt_fragile": 2,
            "sys_normal": 2,
            "sys_fragile": 2,
            "mse_normal": 0.0,
            "mse_fragile": 0.0,
            "session_cycle_time_ms": 12345,
        },
    )

    print("=== 테스트 끝. Firebase 콘솔에서")
    print("    01_debug / 02_View_live / 03_UR_10 로그 / 04_iw_hub 로그 /")
    print("    05_total_process_logs / 07_DT_logs / 08_metrics 확인해봐 ===")#!/usr/bin/env python3