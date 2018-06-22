try:
    import multiprocessing
    import RobotWorld
    import Terminal
    import time

except ImportError:
    print('--------------------------------------------------------------')
    print('import exception')
    print('--------------------------------------------------------------')
    print('')


def job(data):
    print(data['port'], 'Starting...')
    try:
        terminal = Terminal.Terminal(data['port'])
        time.sleep(5)
        world = RobotWorld.World(data['sensors'], data['wheels'], data['signals'], data['host'], data['port'], terminal)
        brain = RobotWorld.RobotBrain()
    except Exception as e:
        print(data['port'], 'Exception: ', e)

    while True:
        # result = world.sense()
        # world.go(5)
        world.turn(0, 1, 90)
        world.go(5)
        time.sleep(10)
        exit(0)
        # exit(0)
        # action = brain.think(result)
        # print(action)
        # world.act(action)


host = '172.27.24.104'
# host = '192.168.0.2'

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
        'host': host,
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
        'host': host,
        'port': 20000
    }
]

pool = multiprocessing.Pool(processes=len(dataList))  # start processes
pool.map(job, dataList)  # we map each process to the input.
pool.close()
pool.join()


# other elements
# bumpers = ['bumper_front_joint', 'bumper_right_joint', 'bumper_left_joint']
# leds = ['status_led', 'led_1', 'led_2']
