
try:
    import multiprocessing
    import RobotWorld
except ImportError:
    print('--------------------------------------------------------------')
    print('import exception')
    print('--------------------------------------------------------------')
    print('')


def job(data):
    print(data['port'], 'started')
    myWorld = RobotWorld.World(data['sensors'], data['wheels'], data['signals'], data['host'], data['port'])
    myRobotBrain = RobotWorld.RobotBrain()

    while True:
        # result = myWorld.sense()
        # myWorld.go(5)
        myWorld.turn(0, 0, 90)
        exit(0)
        pass
        # action = myRobotBrain.think(result)
        # print(action)
        # myWorld.act(action)


# list of all the data.
dataList = [
    {
        'sensors': {
                    'gyro': 'gyro_link_visual',
                    'kinect_depth': 'kinect_depth',
                    'kinect_rgb': 'kinect_rgb'
                   },  # gyroscope, kinect depth and rgb.
        'wheels': {'wheel_right': 'wheel_right_joint',
                   'wheel_left': 'wheel_left_joint'},
        'signals': {'gyro_signal': 'gyro_signal'},
        'host': '127.0.0.1',
        'port': 19999
    },
    {
        'sensors': {
                    'gyro': 'gyro_link_visual#0',
                    'kinect_depth': 'kinect_depth#0',
                    'kinect_rgb': 'kinect_rgb#0'
                },  # gyroscope, kinect depth and rgb.
        'wheels': {'wheel_right': 'wheel_right_joint#0',
                   'wheel_left': 'wheel_left_joint#0'},
        'signals': {'gyro_signal': 'gyro_signal#0'},
        'host': '127.0.0.1',
        'port': 20000
    }
]

#pool = multiprocessing.Pool(processes=len(dataList))  # start two processes.
#pool.map(job, dataList)  # we map each process to the input.
#pool.close()
#pool.join()

# other elements

# bumpers = ['bumper_front_joint', 'bumper_right_joint', 'bumper_left_joint']
# leds = ['status_led', 'led_1', 'led_2']
