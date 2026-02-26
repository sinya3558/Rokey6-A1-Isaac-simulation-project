from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.core.api.objects import DynamicCuboid  
from isaacsim.core.api.objects import VisualSphere  
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.types import ArticulationAction
import isaacsim.robot_motion.motion_generation as mg

import numpy as np
from pxr import Sdf, UsdLux
import omni.usd
import omni.graph.core as og
from collections import deque


class UR10RMPController(mg.MotionPolicyController):
    def __init__(self, name, robot_articulation, physics_dt=1.0/60.0):
        self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflowSuction")
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)
        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)
        super().__init__(name=name, articulation_motion_policy=self.articulation_rmp)
        pos, ori = robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(robot_position=pos, robot_orientation=ori)


class Load_Conveyer(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._fragile_flag = False
        return

    def setup_scene(self):
        world = self.get_world()
        world.scene.add_default_ground_plane()

        background_usd = "/home/rokey/conveyer_test.usd"
        add_reference_to_stage(usd_path=background_usd, prim_path="/World/Background")

        stage = omni.usd.get_context().get_stage()
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(300)

        set_camera_view(
            eye=[3.0, -2.2, 1.7], target=[0.82, -0.91, 0.95], camera_prim_path="/OmniverseKit_Persp"
        )

        self.box1 = world.scene.add(
            DynamicCuboid(
                prim_path="/World/Background/box1",
                name="box1",
                position=np.array([0.65, -0.83, 0.27]),
                scale=np.array([0.3, 0.3, 0.3])
            )
        )
        self.box2 = world.scene.add(
            DynamicCuboid(
                prim_path="/World/Background/box2",
                name="box2",
                position=np.array([0.65, -1.12, 0.27]),
                scale=np.array([0.3, 0.3, 0.3])
            )
        )

        self._box_s_list = [self.box1, self.box2]

        EE_LINK_PATH = "/World/Background/ur10_conveyor/ur10_short_suction/ee_link"
        SURFACE_GRIPPER_PATH = f"{EE_LINK_PATH}/SurfaceGripper"
        ROBOT_ROOT_PATH = "/World/Background/ur10_conveyor/ur10_short_suction"

        PAL_EE_LINK_PATH = "/World/Background/ur10_palletizing/ur10_long_suction/ee_link"
        PAL_SURFACE_GRIPPER_PATH = f"{PAL_EE_LINK_PATH}/SurfaceGripper"
        ROBOT_PAL_ROOT_PATH = "/World/Background/ur10_palletizing/ur10_long_suction"

        gripper = SurfaceGripper(end_effector_prim_path=EE_LINK_PATH, surface_gripper_path=SURFACE_GRIPPER_PATH)
        gripper_pal = SurfaceGripper(end_effector_prim_path=PAL_EE_LINK_PATH, surface_gripper_path=PAL_SURFACE_GRIPPER_PATH)

        self.ur10 = world.scene.add(SingleManipulator(
            prim_path=ROBOT_ROOT_PATH,
            name="ur10_robot",
            end_effector_prim_path=EE_LINK_PATH,
            gripper=gripper
        ))
        self.ur10_pal = world.scene.add(SingleManipulator(
            prim_path=ROBOT_PAL_ROOT_PATH,
            name="ur10_robot_pal",
            end_effector_prim_path=PAL_EE_LINK_PATH,
            gripper=gripper_pal
        ))

        default_joints = np.array([0, -np.pi/2, 0, -np.pi/2, 0, -np.pi/2])
        self.ur10.set_joints_default_state(positions=default_joints)

        default_j = np.array([-np.pi/2, -np.pi/2, np.pi/4, -np.pi/2, -np.pi/2, -np.pi/2])
        self.ur10_pal.set_joints_default_state(positions=default_j)
        return

    def _setup_ros2_subscriber(self):
        try:
            keys = og.Controller.Keys
            og.Controller.edit(
                {
                    "graph_path": "/World/ros2_fragile_sub_graph",
                    "evaluator_name": "execution",
                },
                {
                    # 정확한 노드 타입 -> 네가 확인한 이름 사용
                    keys.CREATE_NODES: [
                        ("on_tick", "omni.graph.action.OnTick"),
                        ("sub", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ],
                    keys.SET_VALUES: [
                        # topic 설정
                        ("sub.inputs:topicName", "/fragile_detected"),
                        # ROS2 메시지 구성
                        ("sub.inputs:messageName", "Bool"),
                        ("sub.inputs:messagePackage", "std_msgs"),
                        ("sub.inputs:messageSubfolder", "msg"),
                    ],
                    keys.CONNECT: [
                        ("on_tick.outputs:tick", "sub.inputs:execIn"),
                    ],
                },
            )
            print("[ROS2 SUB] Subscriber graph created")
        except Exception as e:
            print("[ROS2 SUB] Graph creation failed:", e)




    def _update_fragile_flag(self):
        try:
            msg = og.Controller.get(
                og.Controller.attribute("/World/ros2_fragile_sub_graph/sub.outputs:data")
            )

            #print("ROS2 Subscriber Raw MSG:", msg)

            if msg is not None:
                self._fragile_flag = bool(msg)

        except Exception as e:
            print("ROS2 _update_fragile_flag error:", e)




    async def setup_post_load(self):
        self._world = self.get_world()

        self._ur10     = self._world.scene.get_object("ur10_robot")
        self._ur10_pal = self._world.scene.get_object("ur10_robot_pal")
        self._box1     = self._world.scene.get_object("box1")
        self._box2     = self._world.scene.get_object("box2")

        from omni.isaac.core.prims import RigidPrim

        self._fragile_box_env1 = self._world.scene.add(
            RigidPrim(prim_path="/World/Background/fragile_box",      name="fragile_box_env1")
        )
        self._fragile_box_env2 = self._world.scene.add(
            RigidPrim(prim_path="/World/Background/fragile_box_env2", name="fragile_box_env2")
        )
        self._fragile_box1 = self._world.scene.get_object("fragile_box_env1")
        self._fragile_box2 = self._world.scene.get_object("fragile_box_env2")

        self._box_s_list.extend([self._fragile_box_env1, self._fragile_box_env2])

        self._fragile_box1.set_world_pose(position=np.array([1.07, -0.83, 0.27]))
        self._fragile_box2.set_world_pose(position=np.array([1.07, -1.11, 0.27]))

        self._box_list = [self._box1, self._box2, self._fragile_box1, self._fragile_box2]

        self._target_visual = self._world.scene.add(
            VisualSphere(prim_path="/World/visual_target", name="v_target", radius=0.03, color=np.array([1, 0, 0]))
        )
        self._ee_visual = self._world.scene.add(
            VisualSphere(prim_path="/World/visual_ee", name="v_ee", radius=0.03, color=np.array([0, 0, 1]))
        )

        self._rmp_controller          = UR10RMPController("ur10_rmp",     self._ur10)
        self._articulation_controller = self._ur10.get_articulation_controller()
        self._rmp_controller_pal          = UR10RMPController("ur10_pal_rmp", self._ur10_pal)
        self._articulation_controller_pal = self._ur10_pal.get_articulation_controller()

        self.task_phase    = 0
        self.wait_steps    = 0
        self.placing_pos   = np.array([-0.155, -0.16, 1.0])
        self.BOX_HALF_SIZE = 0.15
        self.CONVEYOR_HIDE_X = -1.68

        # 팔레타이징 위치 리스트 ======================================================================
        self.pal_list         = [np.array([-2.4,  0.94, 0.51]), np.array([-2.86, 0.94, 0.51])]
        self.pal_fragile_list = [np.array([-2.44, -0.98, 0.51]), np.array([-2.81, -0.98, 0.51])]
        
        # 상자 갯수 변수
        self.norm_count    = 0
        self.fragile_count = 0

        self.conv_box_order = 0
        self._box = self._box_list[self.conv_box_order]
        self.box  = self._box_s_list[self.conv_box_order]

        self.waiting_boxes: deque = deque()

        self._pal_box           = None
        self._pal_box_prim      = None
        self.current_is_fragile = False
        self.current_pal_pos    = self.pal_list[0]

        # ★ Phase 8의 소환 로직이 한 프레임만 실행되도록 제어하는 플래그
        self._phase8_spawned = False

        self._setup_ros2_subscriber()
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        self._ur10.gripper.open()
        await self._world.play_async()
        return

    '''def initialize(self, physics_sim_view):
        super().initialize(physics_sim_view)
        self._setup_ros2_subscriber()'''


    async def setup_post_reset(self):
        self.task_phase      = 0
        self.conv_box_order  = 0
        self.wait_steps      = 0
        self.norm_count      = 0
        self.fragile_count   = 0
        self._phase8_spawned = False
        self.waiting_boxes.clear()
        self._pal_box      = None
        self._pal_box_prim = None
        self._box = self._box_list[0]
        self.box  = self._box_s_list[0]
        self._ur10.gripper.open()
        self._ur10_pal.gripper.open()
        # self._setup_ros2_subscriber()
        await self._world.play_async()
        return

    def move_to(self, target_pos, target_ori_euler=np.array([-np.pi/2, np.pi/2, 0])):
        target_quat = euler_angles_to_quat(target_ori_euler)
        actions = self._rmp_controller.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=target_quat
        )
        self._articulation_controller.apply_action(actions)
        curr_joints = self._ur10.get_joint_positions()
        return np.all(np.abs(curr_joints - actions.joint_positions) < 0.005)

    def move_to_pal(self, target_pos, target_ori_euler=np.array([-np.pi/2, np.pi/2, 0])):
        target_quat = euler_angles_to_quat(target_ori_euler)
        actions = self._rmp_controller_pal.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=target_quat
        )
        self._articulation_controller_pal.apply_action(actions)
        curr_joints = self._ur10_pal.get_joint_positions()
        return np.all(np.abs(curr_joints - actions.joint_positions) < 0.005)

    def move_to_joint_positions(self, target_joints):
        action = ArticulationAction(joint_positions=target_joints)
        self._articulation_controller.apply_action(action)
        curr_joints = self._ur10.get_joint_positions()
        return np.all(np.abs(curr_joints - target_joints) < 0.15)

    def move_pal_to_joint_positions(self, target_joints):
        action = ArticulationAction(joint_positions=target_joints)
        self._articulation_controller_pal.apply_action(action)
        curr_joints = self._ur10_pal.get_joint_positions()
        return np.all(np.abs(curr_joints - target_joints) < 0.15)

    # ── 물리 토글 헬퍼 ───────────────────────────────────────────────────────────
    # DynamicCuboid / RigidPrim 둘 다 안전하게 처리
    def _enable_physics(self, obj):
        try:
            obj.enable_rigid_body_physics()
        except AttributeError:
            # RigidPrim은 USD attribute 직접 제어
            from pxr import UsdPhysics
            prim = obj.prim
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
            rigid_body_api.GetRigidBodyEnabledAttr().Set(True)

    def _disable_physics(self, obj):
        try:
            obj.disable_rigid_body_physics()
        except AttributeError:
            from pxr import UsdPhysics
            prim = obj.prim
            rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
            rigid_body_api.GetRigidBodyEnabledAttr().Set(False)

    def physics_step(self, step_size):
        self._update_fragile_flag()

        ee_pos, _ = self._ur10.gripper.get_world_pose()
        self._ee_visual.set_world_pose(position=ee_pos)

        # ════════════════════════════════════════════════════════
        #  Phase 0~6 : 컨베이어 로봇 — 원본 그대로 유지
        # ════════════════════════════════════════════════════════

        if self.task_phase == 0:
            box_pos, _ = self._box.get_world_pose()
            target_pos = box_pos + np.array([0, 0, self.BOX_HALF_SIZE + 0.2])
            self._target_visual.set_world_pose(position=target_pos)
            if self.move_to(target_pos):
                print("Phase 0 완료: 박스 상공 도달")
                self.task_phase = 1

        elif self.task_phase == 1:
            box_pos, _ = self._box.get_world_pose()
            target_pos = box_pos + np.array([0, 0, self.BOX_HALF_SIZE - 0.001])
            self._target_visual.set_world_pose(position=target_pos)
            if self.move_to(target_pos):
                print("Phase 1 완료: 박스 접촉")
                self._ur10.gripper.close()
                self.wait_steps = 0
                self.task_phase = 2

        elif self.task_phase == 2:
            self.wait_steps += 1
            if self.wait_steps > 30:
                print("Phase 2 완료: 흡착 안정화")
                self._ur10.gripper.close()
                self.task_phase = 3
                self.wait_steps = 0
                self.box.set_visibility(False)
                self._disable_physics(self.box)

        elif self.task_phase == 3:
            box_pos, _ = self._box.get_world_pose()
            target_pos = box_pos + np.array([0, 0, self.BOX_HALF_SIZE + 0.3])
            self._target_visual.set_world_pose(position=target_pos)
            if self.move_to(target_pos):
                print("Phase 3 완료: 박스 들어올리기 성공")
                self.task_phase = 4

        elif self.task_phase == 4:
            target_pos = self.placing_pos + np.array([0, 0, 0.2])
            self._target_visual.set_world_pose(position=target_pos)
            if self.move_to(target_pos):
                print("Phase 4 완료: 목표지점 도착")
                self.task_phase = 5

        elif self.task_phase == 5:
            target_pos = self.placing_pos + np.array([0, 0, 0.1])
            target_pos[1] = 0.0
            self._target_visual.set_world_pose(position=target_pos)
            if self.move_to(target_pos):
                print("Phase 5 완료: 하강 성공")
                self.box.set_world_pose(position=self.placing_pos)
                self.box.set_visibility(True)
                self._enable_physics(self.box)
                self.task_phase = 6

        elif self.task_phase == 6:
            self._ur10.gripper.open()
            box_pos, _ = self._box.get_world_pose()

            if box_pos[0] <= self.CONVEYOR_HIDE_X:
                print(f"Phase 6: 박스 x={box_pos[0]:.3f} → 컨베이어 끝 도달, 숨김 처리")

                self.box.set_visibility(False)
                self._disable_physics(self.box)

                self.waiting_boxes.append(
                    (self._box, self.box)
                )
                print(f"  → 대기 큐에 추가, 큐 길이={len(self.waiting_boxes)}")

                self.conv_box_order += 1

                if self.conv_box_order < len(self._box_list):
                    self._box = self._box_list[self.conv_box_order]
                    self.box  = self._box_s_list[self.conv_box_order]
                    print(f"  → 다음 박스({self.conv_box_order + 1}번째) 픽업 시작")
                    self.task_phase = 0
                else:
                    print("  → 모든 컨베이어 작업 완료, Phase 7(홈 복귀)로")
                    self.task_phase = 7

        # ════════════════════════════════════════════════════════
        #  Phase 7 : 컨베이어 로봇 홈 복귀
        # ════════════════════════════════════════════════════════

        elif self.task_phase == 7:
            home_joints = np.array([0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2])
            if self.move_to_joint_positions(home_joints):
                print("Phase 7 완료: 홈 복귀 → Phase 8 시작")
                self._phase8_spawned = False  # Phase 8 진입 전 초기화
                self.task_phase = 8

        # ════════════════════════════════════════════════════════
        #  Phase 8 : 박스 소환 (1회) + 팔레타이징 로봇 상공 이동
        #
        #  ★ 핵심: _phase8_spawned 플래그로 popleft/소환을
        #    딱 한 번만 실행하고, 이후 프레임엔 이동만 수행
        # ════════════════════════════════════════════════════════
        elif self.task_phase == 8:

            if not self._phase8_spawned:
                # 큐가 비었으면 종료
                if len(self.waiting_boxes) == 0:
                    print("Phase 8: 대기 큐 비어있음 → 종료")
                    self.task_phase = 16
                    return

                # ★ 딱 한 번만 실행: 큐에서 꺼내 placing_pos에 소환
                self._pal_box, self._pal_box_prim = self.waiting_boxes.popleft()
                

                # 🔥🔥🔥 추가된 안전 처리 시작 🔥🔥🔥

                # 1️⃣ 먼저 physics 활성화
                self._enable_physics(self._pal_box_prim)

                # 2️⃣ 속도 초기화 (중요)
                try:
                    self._pal_box_prim.set_linear_velocity(np.zeros(3))
                    self._pal_box_prim.set_angular_velocity(np.zeros(3))
                except:
                    pass

                # 3️⃣ 위치 세팅
                self._pal_box_prim.set_world_pose(position=self.placing_pos)

                # 4️⃣ wake up (있으면 실행)
                try:
                    self._pal_box_prim.wake_up()
                except:
                    pass

                # 5️⃣ 마지막에 보이게
                self._pal_box_prim.set_visibility(True)

                # 🔥🔥🔥 추가된 안전 처리 끝 🔥🔥🔥

                self._phase8_spawned = True  # ★ 이후 프레임엔 소환 건너뜀
            
            self.wait_steps += 1

            if self.wait_steps > 100:
                

                # 소환 여부와 관계없이 매 프레임: 박스 상공으로 이동
                box_pos, _ = self._pal_box.get_world_pose()
                target_pos = box_pos + np.array([0, 0, self.BOX_HALF_SIZE + 0.2])
                if self.move_to_pal(target_pos):
                    print("Phase 8 완료: 박스 상공 도달")
                    self._phase8_spawned = False  # 다음 사이클을 위해 리셋
                    self.task_phase = 9
                    self.wait_steps = 0

                print(f"Phase 8: 박스 소환 완료")

                        
        # ════════════════════════════════════════════════════════
        #  Phase 9~15 : 팔레타이징 사이클
        # ════════════════════════════════════════════════════════

        elif self.task_phase == 9:
            box_pos, _ = self._pal_box.get_world_pose()
            target_pos = box_pos + np.array([0, 0, self.BOX_HALF_SIZE + 0.165])
            print("Phase 9 완료: 박스 접근")
            if self.move_to_pal(target_pos):
                self.wait_steps += 1
                self.current_is_fragile = self._fragile_flag
                if self.wait_steps > 100:
                    self.wait_steps = 0
                    print(f"@ placing_pos, fragile={self.current_is_fragile}")

                    if self.current_is_fragile:
                        idx = min(self.fragile_count, len(self.pal_fragile_list) - 1)
                        self.current_pal_pos = self.pal_fragile_list[idx]
                        print(f"  → [FRAGILE] 슬롯 {idx}: {self.current_pal_pos}")
                    else:
                        idx = min(self.norm_count, len(self.pal_list) - 1)
                        self.current_pal_pos = self.pal_list[idx]
                        print(f"  → [NORMAL]  슬롯 {idx}: {self.current_pal_pos}")
                    self.task_phase = 10

        elif self.task_phase == 10:
            print("Phase 10: 박스 잡기")
            self._ur10_pal.gripper.close()
            self.wait_steps = 0
            self.task_phase = 11

        elif self.task_phase == 11:
            self.wait_steps += 1
            if self.wait_steps > 30:
                print("Phase 11 완료: 흡착 안정화")
                self._ur10_pal.gripper.close()
                self.task_phase = 12
                self.wait_steps = 0
                self._pal_box_prim.set_visibility(False)
                self._disable_physics(self._pal_box_prim)

        elif self.task_phase == 12:
            box_pos, _ = self._pal_box.get_world_pose()
            target_pos = box_pos + np.array([0, 0, self.BOX_HALF_SIZE + 0.3])
            self._target_visual.set_world_pose(position=target_pos)
            if self.move_to_pal(target_pos):
                print("Phase 12 완료: 리프팅 성공")
                self.task_phase = 13

        elif self.task_phase == 13:
            target_pos = self.current_pal_pos + np.array([0, 0, 0.374])
            self._target_visual.set_world_pose(position=target_pos)
            if self.move_to_pal(target_pos):
                print("Phase 13 완료: 팔레트 상공 도달")
                self.task_phase = 14

        elif self.task_phase == 14:
            target_pos = np.array([
                self.current_pal_pos[0],
                self.current_pal_pos[1],
                self.current_pal_pos[2] + 0.2
            ])
            box_fin = np.array([
                self.current_pal_pos[0],
                self.current_pal_pos[1],
                self.current_pal_pos[2] - 0.05
            ])
            self._target_visual.set_world_pose(position=target_pos)
            if self.move_to_pal(target_pos):
                print("Phase 14 완료: 하강 성공")
                self._pal_box_prim.set_world_pose(position=box_fin)
                self._pal_box_prim.set_visibility(True)
                self._enable_physics(self._pal_box_prim)
                self.task_phase = 15

        elif self.task_phase == 15:
            self._ur10_pal.gripper.open()
            self.wait_steps += 1
            if self.wait_steps > 30:
                print("Phase 15 완료: 물체 해제")

                if self.current_is_fragile:
                    self.fragile_count += 1
                else:
                    self.norm_count += 1

                self.wait_steps = 0

                if len(self.waiting_boxes) > 0:
                    print("홈 복귀 후 다시 시작")
                    home = np.array([0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2])
                    if self.move_pal_to_joint_positions(home):
                        print(f"  → 큐에 {len(self.waiting_boxes)}개 남음, Phase 8로 (다음 박스 소환)")
                        self._phase8_spawned = False  # ★ 반드시 리셋해야 다음 박스 소환됨
                        self.task_phase = 8
                else:
                    print("  → 모든 팔레타이징 완료, Phase 16(홈 복귀)으로")
                    self.task_phase = 16

        # ════════════════════════════════════════════════════════
        #  Phase 16 : 팔레타이징 로봇 홈 복귀
        # ════════════════════════════════════════════════════════

        elif self.task_phase == 16:
            home = np.array([0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2])
            if self.move_pal_to_joint_positions(home):
                print("Phase 16 완료: 모든 시퀀스 종료")
                self.task_phase = 17

        return