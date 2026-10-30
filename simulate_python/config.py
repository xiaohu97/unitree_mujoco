ROBOT = "go2" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1" 
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml" # Robot scene
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 1 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer

# 感知版 walk 的高程图（unitree_rl_lab Velocity_Rough 状态订阅 rt/perceptive/height_map），见 height_map_publisher.py
ENABLE_HEIGHT_MAP = False
HEIGHT_MAP_TOPIC = "rt/perceptive/height_map"
HEIGHT_MAP_BODY = "torso_link"  # g1 / h1 的躯干 body，与训练里 RayCaster 挂的 link 一致
HEIGHT_MAP_RATE = 10  # Hz
