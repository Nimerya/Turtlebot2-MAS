# coding :utf-8
# deployed with Python 3.6.0

import multiprocessing
import RobotWorld


def job(data):
    print(data['port'], "started")
    myWorld = RobotWorld.World(data['sensors'], data['wheels'], data['host'], data['port'])
    myRobotBrain = RobotWorld.RobotBrain()

    while True:
        pass
        #result = myWorld.sense(sensors)
        #action = myRobotBrain.think(result)
        #print(action)
        #myWorld.act(action)


bumpers = ["bumper_front_joint", "bumper_right_joint", "bumper_left_joint"]
leds = ["status_led", "led_1", "led_2"]

# list of all the data.
dataList = [
    {'sensors': ["gyro_link_visual", "kinect_depth", "kinect_rgb"], # gyroscope, kinect depth and rgb.
     'wheels': ["wheel_right_joint", "wheel_left_joint"],
     'host':'127.0.0.1',
     'port': 19999
     },
    {'sensors': ["gyro_link_visual#0", "kinect_depth#0", "kinect_rgb#0"],  # gyroscope, kinect depth and rgb.
     'wheels': ["wheel_right_joint#0", "wheel_left_joint#0"],
     'host': '127.0.0.1',
     'port': 20000
     }
]

pool = multiprocessing.Pool(processes=2)  # start two processes.
pool.map(job, dataList)  # we map the process to the input.
pool.close()
pool.join()




