from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api.objects import DynamicCuboid
import numpy as np
import carb
from omni.isaac.core import World
# from isaacsim.core.api import World

from omni.isaac.core.objects import VisualSphere
# from omni.isaac.core.prims import RigidPrim
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.rotations import euler_angles_to_quat
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.utils.viewports import set_camera_view

# =================================================================================
# 1. RMPFlow Controller 설정 (장애물 회피 및 Suction 최적화)
# =================================================================================
class UR10RMPController(mg.MotionPolicyController):
    def __init__(self, name, robot_articulation, physics_dt=1.0/60.0):
        # UR10 Suction 버전에 맞는 설정 로드 (장애물 회피 기능 포함)
        self.rmp_flow_config = mg.interface_config_loader.load_supported_motion_policy_config("UR10", "RMPflowSuction")
        self.rmp_flow = mg.lula.motion_policies.RmpFlow(**self.rmp_flow_config)
        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)

        super().__init__(name=name, articulation_motion_policy=self.articulation_rmp)

        # 로봇 베이스 포즈 설정 (장애물과의 충돌 계산 기준점)
        pos, ori = robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(robot_position=pos, robot_orientation=ori)

# =================================================================================
# 2. 메인 시뮬레이션 설정
# =================================================================================
world = World(stage_units_in_meters=1.0)

# 배경 및 환경 로드
background_usd = "/home/rokey/conveyer_test.usd" 
add_reference_to_stage(usd_path=background_usd, prim_path="/World/Background")
world.scene.add_default_ground_plane()
set_camera_view(
    eye=[3.0, -2.2, 1.7], target=[0.82, -0.91, 0.95], camera_prim_path="/OmniverseKit_Persp"
)

# 로봇 및 그리퍼 경로 설정
EE_LINK_PATH = "/World/Background/ur10_conveyor/ur10_short_suction/ee_link"
SURFACE_GRIPPER_PATH = f"{EE_LINK_PATH}/SurfaceGripper"
ROBOT_ROOT_PATH = "/World/Background/ur10_conveyor/ur10_short_suction"

box = world.scene.add(DynamicCuboid(
            prim_path="/World/Background/box", 
            name="box",
            position=np.array([0.823, -0.906, 0.292]), 
            scale=np.array([0.3, 0.3, 0.3]),
            mass=0.0001
        ))

gripper = SurfaceGripper(end_effector_prim_path=EE_LINK_PATH, surface_gripper_path=SURFACE_GRIPPER_PATH)
ur10 = world.scene.add(SingleManipulator(
    prim_path=ROBOT_ROOT_PATH,
    name="ur10_robot",
    end_effector_prim_path=EE_LINK_PATH,
    gripper=gripper
))

# 초기 자세 설정
default_joints = np.array([0, -np.pi/2, 0, -np.pi/2, 0, -np.pi/2])
ur10.set_joints_default_state(positions=default_joints)

# 대상 물체 (박스) 및 시각화 마커
# box = world.scene.add(RigidPrim(prim_path="/World/Background/box", name="box_target"))

target_visual = world.scene.add(VisualSphere(prim_path="/World/visual_target", name="v_target", radius=0.03, color=np.array([1, 0, 0])))
ee_visual = world.scene.add(VisualSphere(prim_path="/World/visual_ee", name="v_ee", radius=0.03, color=np.array([0, 0, 1])))

world.reset()

# 컨트롤러 초기화
rmp_controller = UR10RMPController("ur10_rmp", ur10)
articulation_controller = ur10.get_articulation_controller()

# =================================================================================
# 3. 유틸리티 함수 및 상태 변수
# =================================================================================
def move_to(target_pos, target_ori_euler=np.array([-np.pi/2, np.pi/2, 0])):
    target_quat = euler_angles_to_quat(target_ori_euler)
    # RMPFlow를 사용하여 장애물을 우회하는 관절 궤적 계산
    actions = rmp_controller.forward(target_end_effector_position=target_pos, target_end_effector_orientation=target_quat)
    articulation_controller.apply_action(actions)

    # 도달 확인 로직 (관절 오차 기준)
    curr_joints = ur10.get_joint_positions()
    return np.all(np.abs(curr_joints - actions.joint_positions) < 0.005)

# 설정값
placing_pos = np.array([-0.35, -0.16, 1.0]) 
BOX_HALF_SIZE = 0.15  # 박스 스케일 0.3의 절반 (중심에서 표면까지 거리)
# BOX_HALF_SIZE = 0.05  # 박스 스케일 0.1의 절반 (중심에서 표면까지 거리)
task_phase = 0
wait_steps = 0

# =================================================================================
# 4. 시뮬레이션 루프
# =================================================================================
while simulation_app.is_running():
    world.step(render=True)
    if not world.is_playing(): continue

    # 현재 정보 갱신
    box_pos, _ = box.get_world_pose()
    box_pos, _ = box.get_world_pose()
    ee_pos, _ = ur10.gripper.get_world_pose()
    ee_visual.set_world_pose(position=ee_pos)

    # 상태 머신 (State Machine)
    if task_phase == 0: # 1. 박스 상공 접근 (장애물 회피 이동)
        target_pos = box_pos + np.array([0, 0, BOX_HALF_SIZE + 0.2])
        # target_pos = box_pos + np.array([0, 0, BOX_HALF_SIZE + 0.05])
        target_visual.set_world_pose(position=target_pos)
        if move_to(target_pos):
            print("Phase 0 완료: 박스 상공 도달")
            task_phase = 1

    elif task_phase == 1: # 2. 박스 표면 밀착 (흡착 최적화)
        target_pos = box_pos + np.array([0, 0, BOX_HALF_SIZE- 0.001]) 
        target_visual.set_world_pose(position=target_pos)
        if move_to(target_pos):
            print("Phase 1 완료: 박스 접촉")
            ur10.gripper.close() # 석션 작동
            wait_steps = 0
            task_phase = 2

    elif task_phase == 2: # 3. 흡착 대기 (그리퍼 성능 보장)
        wait_steps += 1
        if wait_steps > 300: # 약 0.5초 대기하여 물리적 결합 보장
            print("Phase 2 완료: 흡착 안정화")
            ur10.gripper.close() # 석션 작동
            task_phase = 3
            wait_steps = 0

    elif task_phase == 3: # 4. 박스 리프팅 (장애물 회피 리프팅)
        target_pos = box_pos + np.array([0, 0, BOX_HALF_SIZE + 0.3])
        target_visual.set_world_pose(position=target_pos)
        if move_to(target_pos):
            print("Phase 3 완료: 박스 들어올리기 성공")
            task_phase = 4

    elif task_phase == 4: # 5. 목표 지점 위로 이동 (Move to Place)
        target_pos = placing_pos + np.array([0, 0, 0.2])
        target_visual.set_world_pose(position=target_pos)
        if move_to(target_pos):
            print("Phase 4 완료: 목표지점 도착")
            task_phase = 5

    elif task_phase == 5: # 6. 내려놓기 (Place Down)
        target_pos = placing_pos
        target_visual.set_world_pose(position=target_pos)
        if move_to(target_pos):
            print("Phase 5 완료: 하강 성공")
            task_phase = 6

    elif task_phase == 6: # 7. 그리퍼 해제 (Release)
        ur10.gripper.open()
        wait_steps += 1
        if wait_steps > 30:
            print("Phase 6 완료: 물체 해제")
            task_phase = 7
            wait_steps = 0

    elif task_phase == 7: # 8. 홈 위치로 복귀
        if move_to(placing_pos + np.array([0, 0, 0.2])):
            print("모든 시퀀스 종료")
            task_phase = 8

simulation_app.close()