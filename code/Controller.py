try:
    import multiprocessing
    import redis
    import RobotWorld
    import Terminal
    import time
    import subprocess

except ImportError as e:
    print('--------------------------------------------------------------')
    print('import exception ', e)
    print('--------------------------------------------------------------')


# host = '192.168.43.185'
host = '192.168.0.2'
#'127.0.0.1'

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


def job(data):
    print(data['port'], 'Starting...')
    try:
        terminal = Terminal.Terminal(data['port'])
        time.sleep(1)
        world = RobotWorld.World(data['sensors'], data['wheels'], data['signals'], data['host'], data['port'], terminal)
        brain = RobotWorld.Brain(world, data['port'], terminal)
    except Exception as e:
        print(data['port'], 'Exception: ', e)
        exit(1)

    while True:
        environment = world.sense()
        action = brain.think(environment)
        world.act(action)


def main():
    pool = multiprocessing.Pool(processes=len(dataList))  # start processes
    pool.map(job, dataList)  # we map each process to the input.
    pool.close()
    pool.join()


if __name__ == "__main__":
    main()

