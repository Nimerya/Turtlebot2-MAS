# coding :utf-8
# deployed with Python 3.6.0


import RobotWorld.vrep

sensors = ["gyro_link_visual", "kinect_depth", "kinect_rgb"]  # gyroscope, kinect depth and rgb
wheels = ["wheel_right_joint", "wheel_left_joint"]
bumpers = ["bumper_front_joint", "bumper_right_joint", "bumper_left_joint"]
leds = ["status_led", "led_1", "led_2"]

myWorld = RobotWorld.World(sensors, wheels, host='127.0.0.1', port=19999)
myWorld1 = RobotWorld.World(sensors, wheels, host='127.0.0.1', port=20000)

myRobotBrain = RobotWorld.RobotBrain()

while True:
    pass
    #result = myWorld.sense(sensors)
    #action = myRobotBrain.think(result)
    #print(action)
    #myWorld.act(action)
