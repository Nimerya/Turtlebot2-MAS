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

# ip of the vrep simulator
host = '192.168.0.2'

# list of dictionaries containing the names of the handles of the unit's sensors/parts
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
        'plate': 'plate_top_visual',
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
        'plate': 'plate_top_visual#0',
        'host': host,
        'port': 20000
    }
]


def job(data):
    """
    function that encapsulates each unit's
    :param data: dictionary containing the data that will be used in the world and brain classes
    :return: nothing
    """
    print(data['port'], 'Starting...')
    try:
        # spawn a terminal for logging
        terminal = Terminal.Terminal(data['port'])
        # wait for the terminal to spawn
        time.sleep(1)
        # init the world obj
        world = RobotWorld.World(data['sensors'], data['wheels'], data['signals'], data['plate'],
                                 data['host'], data['port'], terminal)
        # init the brain obj
        brain = RobotWorld.Brain(world, data['port'], terminal)
    except Exception as e:
        print(data['port'], 'Exception: ', e)
        exit(1)

    # cycle
    while True:
        # sense the environment
        environment = world.sense()
        # compute an action
        action = brain.think(environment)
        # do that action
        world.act(action)


def main():
    """
    main: spawns a process for each declared unit
    :return: none
    """
    pool = multiprocessing.Pool(processes=len(dataList))  # start processes
    pool.map(job, dataList)  # we map each process to the input.
    pool.close()
    pool.join()


if __name__ == "__main__":
    main()

