try:
    import vrep
    import math
    import random
    import struct
    import time

except ImportError:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')


class World:
    """
    Robot simulator class to communicate with the simulation environment.
    """

    def __init__(self, sensors, wheels, signals, host='127.0.0.1', port=19999, terminal = None):
        """
        Initialize the connection with V-REP.
        :param host: host number.
        :param port: port number.
        """
        assert (sensors, dict)  # check if the variable is a dict.
        assert (wheels, dict)  # check if the variable is a dict.
        assert (signals, dict)  # check if the variable is a dict.

        self._host = host
        self._port = port
        self._term = terminal

        vrep.simxFinish(-1)  # just in case, close all opened connections.
        self._clientID = vrep.simxStart(self._host, self._port, True, True, 5000, 5)  # Connect to V-REP.
        if self._clientID == -1:  # connection error
            terminal.write('{} : Connection to the server was not possible'.format(self._port))
        self._operation_mode = vrep.simx_opmode_blocking  # fire and forget.
        self.wheels_handles = {}
        self.sensors_handles = {}
        self.signals = signals

        self._term.write('{} : Fetching wheels handles...'.format(self._port))
        for w in wheels:  # initialize the robot.
            res, handle = vrep.simxGetObjectHandle(self._clientID, wheels[w], self._operation_mode)
            if res == vrep.simx_return_ok:
                self.wheels_handles[w] = handle
            else:
                self._term.write('{} : Wheels handle error: {}'.format(self._port, res))
                exit(1)

        self._term.write('{} : Fetching sensors handles...'.format(self._port))
        for s in sensors:  # initialize the robot.
            res, handle = vrep.simxGetObjectHandle(self._clientID, sensors[s], self._operation_mode)
            if res == vrep.simx_return_ok:
                self.sensors_handles[s] = handle
            else:
                self._term.write(self._port, '{} : Sensors handle error: {}'.format(self._port, res))
                exit(1)

        self._term.write("{} : SUCCESSFULLY FETCHED ALL HANDLES".format(port))

    def sense(self):
        """
        Sense the world and return data.
        :return: data.
        """
        out = []  # list.
        # reading the gyroscope.
        data = vrep.simxGetStringSignal(self._clientID, self.signals['gyro_signal'], self._operation_mode)
        val = struct.unpack("f", bytearray(data[1][:4]))
        self._term.write("{} : gyro = {0:.4f}".format(self._port, val))
        return out

    def stop(self):
        """
        Stop the robot.
        """
        self._term.write('{} : stopped'.format(self._port))
        # the second parameter is the velocity.
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_right"], 0, self._operation_mode)
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_left"], 0, self._operation_mode)

    def turn(self, speedr, speedl, angle):
        """
        Turn the robot.
        giving speed to the left wheel makes the robot to turn right and vice-versa
        :param speedr: velocity of the right wheel.
        :param speedl: velocity of the left wheel.
        :param angle: turning angle.
        """
        self._term.write('{} : turning, angle = {}'.format(self._port, angle))
        z = 0
        while z < angle:
            time.sleep(1)
            # the second parameter is the velocity.
            vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_right"], speedr,
                                            self._operation_mode)
            vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_left"], speedl,
                                            self._operation_mode)
            gyro_data = vrep.simxGetStringSignal(self._clientID, self.signals['gyro_signal'], self._operation_mode)
            gyro_data_unpacked_x = (struct.unpack("f", bytearray(gyro_data[1][:4]))[0] * 180) / math.pi
            gyro_data_unpacked_y = (struct.unpack("f", bytearray(gyro_data[1][4:8]))[0] * 180) / math.pi
            gyro_data_unpacked_z = (struct.unpack("f", bytearray(gyro_data[1][8:12]))[0] * 180) / math.pi
            self._term.write('-------------------------------------------------------\n'
                  '{} : X-Gyro = {} dps\n        Y-Gyro = {} dps\n        Z-Gyro = {} dps'
                  .format(self._port, round(gyro_data_unpacked_x, 2), round(gyro_data_unpacked_y, 2),
                          round(gyro_data_unpacked_z, 2)))

            z += abs(gyro_data_unpacked_z)
            self._term.write('{} : cumulative angle = {}'.format(self._port, z))
        self._term.write('{} : turn completed'.format(self._port))

    def go(self, speed):
        """
        The robot go forward.
        :param speed: velocity of both wheels.
        """
        self._term.write('{} : going, speed = {}'.format(self._port, speed))
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_right"], speed,
                                        self._operation_mode)
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_left"], speed,
                                        self._operation_mode)


class RobotBrain(object):

    def __init__(self):
        self._state = None

    def think(self, sensor_reading):
        """
        It decides what do.
        :param sensor_reading: result of the sense.
        :return: an action.
        """
        # self._state = self.perception(sensor_reading)
        # action = self.decision()
        return None  # action

    def perception(self, sensor_reading):
        """
        Read state and build a world representation.
        :param sensor_reading: result of the sense
        :return: state
        """
        return None

    def decision(self):
        """
        The state contains the world representation.
        :return: a decision
        """
        return None
