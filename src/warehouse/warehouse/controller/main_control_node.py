#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from typing import Dict, Any, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from warehouse.db.db_node import (
    now_ms,
    save_session_metrics,
    log_conveyor_load,
    log_ur_loader,
    log_ur_sorter,
    log_dt_update,
    log_iw_hub,
    update_view_live,
)


class SequencerNode(Node):
    def __init__(self):
        super().__init__("robot_sequencer_node")

        # ---- 이번 세션에 사용할 박스 계획 (ur10_node.py의 box_ids와 동일 순서로 맞춤) ----
        # ur10_node.py 에서:
        #   self.box_ids = ["box_N_1", "box_N_2", "box_F_1", "box_F_2"]
        self.box_plan: List[Dict[str, Any]] = [
            {"box_id": "box_N_1", "gt_label": "NORMAL"},
            {"box_id": "box_N_2", "gt_label": "NORMAL"},
            {"box_id": "box_F_1", "gt_label": "FRAGILE"},
            {"box_id": "box_F_2", "gt_label": "FRAGILE"},
        ]
        self.total_boxes = len(self.box_plan)
        self.current_box_index: int = 0

        # GT / Pred 라벨 관리 (KPI1,4 용)
        self.gt_label_map: Dict[str, str] = {
            b["box_id"]: b["gt_label"] for b in self.box_plan
        }
        self.pred_label_map: Dict[str, Optional[str]] = {}

        # 박스별 상태 (예: "LOADING", "ON_CONVEYOR", "SORTED", "IW_DONE")
        self.box_status: Dict[str, str] = {}

        # 팔레트 카운트 + threshold (KPI2,3,5 용)
        self.fragile_count: int = 0
        self.normal_count: int = 0
        self.fragile_threshold: int = 2
        self.normal_threshold: int = 2

        # iW 미션 발행 여부
        self.iw_fragile_dispatched: bool = False
        self.iw_normal_dispatched: bool = False

        # Digital Twin 재고 상태 (아주 단순 버전)
        self.dt_counts = {
            "fragile": 0,
            "normal": 0,
        }

        # 세션 시간
        self.session_start_ms: Optional[int] = None
        self.session_end_ms: Optional[int] = None
        self.session_id: str = "session_1"

        # ---- Publishers: 각 노드에 명령 보내기 ----
        self.pub_loader_cmd = self.create_publisher(String, "loader_command", 10)
        self.pub_sorter_cmd = self.create_publisher(String, "sorter_command", 10)
        self.pub_iw_cmd     = self.create_publisher(String, "iw_mission_command", 10)

        # ---- Subscribers: Isaac ur10_node / iW 노드 상태 받기 ----
        self.sub_loader_state = self.create_subscription(
            String, "loader_state", self.cb_loader_state, 10
        )
        self.sub_sorter_state = self.create_subscription(
            String, "sorter_state", self.cb_sorter_state, 10
        )
        self.sub_iw_state = self.create_subscription(
            String, "iw_state", self.cb_iw_state, 10
        )

        # View_live용 카운트 (시스템이 인식한 재고 수)
        self.sys_box_count_normal: int = 0
        self.sys_box_count_fragile: int = 0

        # 시작하자마자 첫 박스 로딩 시작
        self.get_logger().info("SequencerNode started. Start first box.")
        self.start_next_box()

    # --------------------------
    # 1) 다음 박스 로딩 시작
    # --------------------------
    def start_next_box(self):
        if self.current_box_index >= self.total_boxes:
            self.get_logger().info("모든 박스 로딩 요청 완료.")
            return

        box_info = self.box_plan[self.current_box_index]
        box_id = box_info["box_id"]
        gt_label = box_info["gt_label"]

        # 세션 시작 시간 최초 설정
        if self.session_start_ms is None:
            self.session_start_ms = now_ms()

        self.box_status[box_id] = "LOADING"

        cmd = {
            "cmd": "LOAD_BOX",
            "box_id": box_id,
            "gt_label": gt_label,
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.pub_loader_cmd.publish(msg)

        self.get_logger().info(f"[Sequencer] LOAD_BOX cmd sent: {cmd}")

    # --------------------------
    # 2) Loader 상태 콜백
    #    - ur10_node.py에서 /loader_state로 보내는 이벤트 처리
    # --------------------------
    def cb_loader_state(self, msg: String):
        """
        ur10_node.py 에서 보내는 loader_state 예시 JSON:

        1) PICK_DONE
        {
            "event": "PICK_DONE",
            "box_id": "box_N_1"
        }

        2) PLACE_ON_CONVEYOR_DONE
        {
            "event": "PLACE_ON_CONVEYOR_DONE",
            "box_id": "box_N_1",
            "placing_pos": [...]
        }

        3) CONVEYOR_END
        {
            "event": "CONVEYOR_END",
            "box_id": "box_N_1",
            "is_fragile": false,
            "queue_len": 1
        }
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"loader_state JSON parse error: {e}")
            return

        event = data.get("event")
        box_id = data.get("box_id")

        if not box_id:
            return

        # 2-1) UR_Loader PICK_DONE → log_ur_loader (KPI3,5에서 pick 타임라인 용)
        if event == "PICK_DONE":
            gt_label = self.gt_label_map.get(box_id, "UNKNOWN")
            log_ur_loader(
                robot_id="ur10_loader",
                state="RUNNING",
                process_step="PICK_DONE",
                step_label=f"{box_id} 픽 완료 (GT={gt_label})",
                box_id=box_id,
            )
            self.get_logger().info(f"[Sequencer] PICK_DONE logged for {box_id}")
            return

        # 2-2) 컨베이어에 올려진 시점 → log_conveyor_load (KPI6: Sorting Latency 시작)
        if event == "PLACE_ON_CONVEYOR_DONE":
            self.box_status[box_id] = "ON_CONVEYOR"

            placing_pos = data.get("placing_pos")
            log_conveyor_load(
                box_id=box_id,
                robot_id="ur10_loader",
                pallet_id=None,
            )

            self.get_logger().info(
                f"[Sequencer] box {box_id} ON_CONVEYOR (placing_pos={placing_pos})"
            )

            # 다음 박스 로딩 시작
            self.current_box_index += 1
            self.start_next_box()
            return

        # 2-3) 컨베이어 끝 도달 → 병목/큐 길이 분석용 (KPI5)
        if event == "CONVEYOR_END":
            self.box_status[box_id] = "CONVEYOR_END"
            is_fragile = bool(data.get("is_fragile", False))
            queue_len = int(data.get("queue_len", 0))

            self.get_logger().info(
                f"[Sequencer] box {box_id} reached conveyor end "
                f"(fragile={is_fragile}, queue_len={queue_len})"
            )

            # View_live에 현재 큐 길이 업데이트 (병목 시각화용)
            update_view_live(
                box_count_normal=self.sys_box_count_normal,
                box_count_fragile=self.sys_box_count_fragile,
                state="CONVEYOR_RUNNING",
                conveyor_queue_len=queue_len,
                iw_state="IDLE",  # iW 상태는 나중에 실제 값으로 대체 가능
            )
            return

    # --------------------------
    # 3) Sorter 상태 콜백
    #    - 팔레트에 최종 적재 완료 시점
    # --------------------------
    def cb_sorter_state(self, msg: String):
        """
        ur10_node.py 에서 보내는 sorter_state 예시 JSON:

        1) SORT_PICK_START
        {
            "event": "SORT_PICK_START",
            "box_id": "box_N_1",
            "is_fragile": false
        }

        2) PLACE_ON_PALLET_DONE
        {
            "event": "PLACE_ON_PALLET_DONE",
            "box_id": "box_N_1",
            "pallet_id": "normal_1",
            "label": "NORMAL",
            "slot_pos": [...]
        }

        3) SORT_RELEASE_DONE
        {
            "event": "SORT_RELEASE_DONE",
            "box_id": "box_N_1",
            "fragile_count": 0,
            "normal_count": 1
        }
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"sorter_state JSON parse error: {e}")
            return

        event      = data.get("event")
        box_id     = data.get("box_id")
        pallet_id  = data.get("pallet_id")
        label      = data.get("label")  # "FRAGILE" / "NORMAL"
        slot_pos   = data.get("slot_pos")

        if not box_id:
            return

        # 3-1) SORT_PICK_START → 필요하면 나중에 log_ur_sorter로 시작 시점도 찍을 수 있음
        if event == "SORT_PICK_START":
            self.get_logger().info(
                f"[Sequencer] SORT_PICK_START for {box_id}"
            )
            return

        # 3-2) 팔레트에 실제로 내려놓는 시점 (KPI1,2,3,5용)
        if event == "PLACE_ON_PALLET_DONE":
            self.box_status[box_id] = "SORTED"
            self.pred_label_map[box_id] = label  # 분류 결과 저장 (KPI1,4)

            self.get_logger().info(
                f"[Sequencer] box {box_id} SORTED to {pallet_id} (label={label}, slot={slot_pos})"
            )

            # UR_Sorter 동작 로그 (KPI3용)
            log_ur_sorter(
                robot_id="ur10_sorter",
                state="DONE",
                pallet_id=pallet_id or "",
                box_type=label or "UNKNOWN",
                process_step="PLACE_ON_PALLET_DONE",
                step_label=f"{box_id} → {pallet_id} 적재 완료",
                box_id=box_id,
            )

            # Digital Twin 재고 상태 업데이트 (KPI2: count 기반 MSE/RMSE, KPI3: 재고동기화 시간)
            if label == "FRAGILE":
                self.dt_counts["fragile"] += 1
                self.sys_box_count_fragile += 1
            else:
                self.dt_counts["normal"] += 1
                self.sys_box_count_normal += 1

            dt_state = {
                "count_fragile": self.dt_counts["fragile"],
                "count_normal": self.dt_counts["normal"],
                "last_box_id": box_id,
                "last_pallet_id": pallet_id,
            }
            log_dt_update(box_id=box_id, dt_state=dt_state)

            # View_live에도 현재 재고 상태 반영 (KPI2,5 시각화용)
            update_view_live(
                box_count_normal=self.sys_box_count_normal,
                box_count_fragile=self.sys_box_count_fragile,
                state="SORTING",
                conveyor_queue_len=None,  # 이 시점에서는 생략 가능
                iw_state="IDLE",
            )

            # GT 라벨 기준으로 팔레트 카운트 증가 (iW 트리거용)
            gt_label = self.gt_label_map.get(box_id, label)
            if gt_label == "FRAGILE":
                self.fragile_count += 1
                if (
                    self.fragile_count >= self.fragile_threshold
                    and not self.iw_fragile_dispatched
                ):
                    self.dispatch_iw_mission(
                        pallet_id="fragile_1",
                        box_ids=self._get_boxes_by_label("FRAGILE"),
                    )
                    self.iw_fragile_dispatched = True
            else:
                self.normal_count += 1
                if (
                    self.normal_count >= self.normal_threshold
                    and not self.iw_normal_dispatched
                ):
                    self.dispatch_iw_mission(
                        pallet_id="normal_1",
                        box_ids=self._get_boxes_by_label("NORMAL"),
                    )
                    self.iw_normal_dispatched = True

            return

        # 3-3) SORT_RELEASE_DONE → 카운트 정보만 참고해서 로그 남길 수 있음
        if event == "SORT_RELEASE_DONE":
            fragile_cnt = int(data.get("fragile_count", 0))
            normal_cnt  = int(data.get("normal_count", 0))
            self.get_logger().info(
                f"[Sequencer] SORT_RELEASE_DONE for {box_id} "
                f"(fragile_cnt={fragile_cnt}, normal_cnt={normal_cnt})"
            )
            return

    # --------------------------
    # 4) iW_hub 상태 콜백
    #    - 팔레트 픽업/도착 상태 감시
    # --------------------------
    def cb_iw_state(self, msg: String):
        """
        iw_state 예시 JSON (iW 노드에서 정의 필요):

        {
            "event": "MISSION_DONE",
            "pallet_id": "fragile_1",
            "box_ids": ["box_F_1", "box_F_2"],
            "pose": {"x": 1.0, "y": 2.0, "theta": 1.57}
        }
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"iw_state JSON parse error: {e}")
            return

        event     = data.get("event")
        pallet_id = data.get("pallet_id")
        box_ids   = data.get("box_ids", [])
        pose      = data.get("pose", None)

        if event == "MISSION_DONE":
            self.get_logger().info(
                f"[Sequencer] iW mission done for {pallet_id}, boxes={box_ids}"
            )

            # iW 로그 (KPI3,5용)
            log_iw_hub(
                robot_id="iw_hub_1",
                state="MISSION_DONE",
                pallet_id=pallet_id or "",
                pose=pose,
                process_step="IW_MISSION_DONE",
                step_label=f"{pallet_id} 최종 위치 도착",
                box_ids=box_ids,
            )

            for b in box_ids:
                self.box_status[b] = "IW_DONE"

            # 모두 IW_DONE이면 세션 종료
            if self._all_boxes_done():
                self.on_session_done()

    # --------------------------
    # 5) iW 미션 발행 함수
    # --------------------------
    def dispatch_iw_mission(self, pallet_id: str, box_ids: List[str]):
        cmd = {
            "cmd": "MOVE_PALLET",
            "pallet_id": pallet_id,
            "box_ids": box_ids,
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.pub_iw_cmd.publish(msg)

        self.get_logger().info(f"[Sequencer] IW mission cmd: {cmd}")

    # --------------------------
    # 6) 세션 종료 처리 + KPI 요약 저장
    # --------------------------
    def on_session_done(self):
        if self.session_end_ms is None:
            self.session_end_ms = now_ms()
        session_time = self.session_end_ms - (
            self.session_start_ms or self.session_end_ms
        )

        self.get_logger().info(
            f"[Sequencer] SESSION DONE. time={session_time} ms"
        )

        # ===== KPI 1: 인벤토리 정확도 (GT vs Pred) =====
        correct = 0
        total   = 0
        for b in self.box_plan:
            box_id   = b["box_id"]
            gt_label = b["gt_label"]
            pred     = self.pred_label_map.get(box_id, None)
            if pred is None:
                continue
            total += 1
            if pred == gt_label:
                correct += 1

        inventory_accuracy = (correct / total) * 100.0 if total > 0 else 0.0

        # ===== KPI 2: MSE / RMSE (zone/label별 count 차이) =====
        gt_fragile = sum(1 for b in self.box_plan if b["gt_label"] == "FRAGILE")
        gt_normal  = sum(1 for b in self.box_plan if b["gt_label"] == "NORMAL")

        sys_fragile = self.dt_counts["fragile"]
        sys_normal  = self.dt_counts["normal"]

        mse_fragile = float((gt_fragile - sys_fragile) ** 2)
        mse_normal  = float((gt_normal  - sys_normal) ** 2)

        rmse_fragile = math.sqrt(mse_fragile)
        rmse_normal  = math.sqrt(mse_normal)

        # ===== KPI 3: 세션 전체 Cycle Time (ms) =====
        session_cycle_time_ms = int(session_time)

        # KPI 4~6 (Sorting Latency, Stock Sync time 등)은
        # total_process_logs 에 저장된 타임스탬프 기반으로
        # 별도 분석 스크립트에서 계산하면 됨.
        # (여기서는 summary만 Firebase에 저장)

        metrics = {
            "gt_fragile": gt_fragile,
            "gt_normal": gt_normal,
            "sys_fragile": sys_fragile,
            "sys_normal": sys_normal,
            "inventory_accuracy_percent": inventory_accuracy,
            "mse_fragile": mse_fragile,
            "mse_normal": mse_normal,
            "rmse_fragile": rmse_fragile,
            "rmse_normal": rmse_normal,
            "session_cycle_time_ms": session_cycle_time_ms,
        }
        save_session_metrics(self.session_id, metrics)

        self.get_logger().info(f"[Sequencer] metrics saved: {metrics}")

    # --------------------------
    # 유틸: 특정 라벨 박스들 리스트, 전체 완료 체크
    # --------------------------
    def _get_boxes_by_label(self, label: str) -> List[str]:
        return [
            b["box_id"] for b in self.box_plan if b["gt_label"] == label
        ]

    def _all_boxes_done(self) -> bool:
        for b in self.box_plan:
            if self.box_status.get(b["box_id"]) != "IW_DONE":
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SequencerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()