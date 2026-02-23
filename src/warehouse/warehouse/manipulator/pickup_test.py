# 시뮬레이션 시작
from isaacsim import SimulationApp

# GUI가 열리는 모드({"headless": False})로 시뮬레이션 앱 시작
simulation_app = SimulationApp({"headless": False})


# usd 월드 파일 불러오기

from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units

import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
import numpy as np
from scipy.spatial.transform import Rotation as R

world = World(
    stage_units_in_meters=1.0
)
world.scene.clear()


#bring usd file
#bs = BaseSample()
#self = "/home/rokey/isaacsim/exts/isaacsim.examples.interactive/isaacsim/examples/interactive/hello_world/back.usd"

background_usd = "/home/rokey/conveyer_test.usd"

add_reference_to_stage(usd_path=background_usd, prim_path="/World/Background")

# add ground
world.scene.add_default_ground_plane()

robots = world.scene.get_object("ur10_conveyor")
print(robots)



# robot 씬 추가

from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper

add_reference_to_stage(usd_path=background_usd, prim_path="/World/Background")

# USD 안의 특정 prim 경로를 RigidPrim으로 래핑
gripper = SurfaceGripper(
            end_effector_prim_path="/World/Background/ur10_palletizing/ur10_long_suction/ee_link", 
            surface_gripper_path="/World/Background/ur10_palletizing/ur10_long_suction/ee_link/SurfaceGripper"
        )
ur10 = world.scene.add(
            SingleManipulator(
                prim_path="/World/Background/ur10_palletizing/ur10_long_suction",
                name="ur10_long_suction", end_effector_prim_path="/World/Background/ur10_palletizing/ur10_long_suction/ee_link", gripper=gripper
            )
        )

# box 씬 추가
from omni.isaac.core.prims import RigidPrim

cube = world.scene.add(
    RigidPrim(
        prim_path="/World/Background/box",
        name="box"
    )
)


# 로봇 불러오기 + 컨트롤러 생성

robot = world.scene.get_object("ur10_long_suction")
print(robot) 

my_controller = PickPlaceController(
            name="pick_place_controller", 
            gripper=robot.gripper, 
            robot_articulation=robot
        )

'''
# 액션 생성
box = world.scene.get_object("box")
print(box.get_world_pose())

placing = np.array([-0.35668085634140223, -0.16320017128181238, 1])
_end_effector_offset = np.array([0, 0, 0.02])

actions = my_controller.forward(
            picking_position=box.get_world_pose(),
            placing_position=placing,
            current_joint_positions=robot.get_joint_positions(),
            end_effector_offset=_end_effector_offset,
        )
        '''
# 외부 루프 (총 4 사이클 실행)
for i in range(4):
    print("running cycle: ", i)
    
    # 사이클 1, 3: "moving"
    if i == 1 or i == 3:
        print("moving")
        
    
    # 사이클 2: "stopping"
    if i == 2:
        print("stopping")
        
    
    # 내부 루프 (각 사이클마다 100번의 시뮬레이션 스텝 실행)
    for j in range(100):
        # 시뮬레이션 1스텝 진행 (물리 + 렌더링)
        world.step(render=True)
