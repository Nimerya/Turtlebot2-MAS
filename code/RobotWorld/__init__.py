# coding :utf-8
# deployed with Python 3.6.0

"""
TO DO
    Write a RobotWorld Python 3.6.x wrapper library with classes that encapsulates the V-REP Remote API
"""

try:
    import vrep
    import math
    import random
    import struct

except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')


class World(object):
    """
    Robot simulator class to communicate with the simulation environment.
    """

    def __init__(self, sensors, wheels, host='127.0.0.1', port=16123):
        """
        Initialize the connection with V-REP.
        :param host: host number.
        :param port: port number.
        """
        assert (sensors, dict)  # check if the variable is a dict.
        assert (wheels, dict)  # check if the variable is a dict.

        self._host = host
        self._port = port
        vrep.simxFinish(-1)  # just in case, close all opened connections.
        self._clientID = vrep.simxStart(self._host, self._port, True, True, 5000, 5)  # Connect to V-REP.
        if self._clientID == -1:  # connection error
            print(self._port, ': Connection to the server was not possible')
        self._operation_mode = vrep.simx_opmode_blocking  # fire and forget.
        self.wheels_handles = {}
        self.sensors_handles = {}
        self.signals = {}  # TODO ADD SIGNALS

        print(self._port, ': Fetching wheels handles...')
        for w in wheels:  # initialize the robot.
            res, handle = vrep.simxGetObjectHandle(self._clientID, wheels[w], self._operation_mode)
            if res == vrep.simx_return_ok:
                self.wheels_handles[w] = handle
            else:
                print(self._port, ': Wheels handle error: ', res)
                exit(1)

        print(self._port, ': Fetching sensors handles...')
        for s in sensors:  # initialize the robot.
            res, handle = vrep.simxGetObjectHandle(self._clientID, sensors[s], self._operation_mode)
            if res == vrep.simx_return_ok:
                self.sensors_handles[s] = handle
                print(handle)
            else:
                print(self._port, ': Sensors handle error: ', res)
                exit(1)

        print(self._port, ": SUCCESSFULLY FETCHED ALL HANDLES")

    def sense(self):
        """
        Sense the world and return data.
        :return: data.
        """
        out = []  # list.
        data = vrep.simxGetStringSignal(self._clientID, "a", self._operation_mode) #TODO ADD SIGNAL
        val = struct.unpack("f", bytearray(data[1][:4]))
        print("gyro: %.4f" % val)

        #distanceLC = vrep.simRead(self._clientID, self.sensors_handles["gyro"], self._operation_mode)
        #print(distanceLC)


        # r,state, point,handle,vector = vrep.simxReadProximitySensor(self._clientID, proximity_sensor_handle, operation_mode)
        ##r => Result
        ##state  => detectionState: the detection state (false=no detection)
        ##point => detectedPoint: the detected point coordinates (relative to the sensor reference frame)
        ##handle => detectedObjectHandle: the handle of the detected object
        ##vector => detectedSurfaceNormalVector: the normal vector (normalized) of the detected surface. Relative to the sensor reference frame
        # if res1 != vrep.simx_return_ok:
        #    print('Simulator problem')
        # elif r == vrep.simx_return_ok:
        #    if state == False:
        #        print('No detection')
        #        return False # No detection
        #    out.append(r)
        #    out.append(state)
        #    out.append(point)
        #    out.append(handle)
        #    out.append(vector)
        #    print(out)
        return out

    def act(self, action):
        """
        Receive a command, send an "order".
        :param action: command.
        :return: a command to the robot.
        """

        assert isinstance(action, str)  # check if the variable is a string.

        if action == 'Continue':
            pass

    def stop(self):
        """
        Stop the robot.
        """
        # the second parameter is the velocity.
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_right"], 0, self._operation_mode)
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_left"], 0, self._operation_mode)

    def turn(self, vel1, vel2, angle):
        """
        Turn the robot.
        :param vel1: velocity of the first wheel.
        :param vel2: velocity of the second wheel.
        """
        x = 0
        while x < angle:
            # the second parameter is the velocity.
            vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_right"], vel1,
                                        self._operation_mode)
            vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_left"], vel2,
                                        self._operation_mode)
            data = vrep.simxGetFloatSignal(self._clientID, "a", self._operation_mode)  # TODO ADD SIGNAL
            val = struct.unpack("f", bytearray(data[1][:4]))
            # val = vrep.simxUnpackFloats(data)
            x = x+float(val[0])
            print(x)

    def go(self, vel):
        """
        The robot go forward.
        :param vel: velocity of both wheels.
        """
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_right"], vel,
                                        self._operation_mode)
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_left"], vel,
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
        self._state = self.perception(sensor_reading)
        action = self.decision()
        return action

    def perception(self, sensor_reading):
        """
        Read state and build a world representation.
        :param sensor_reading: result of the sense
        :return: state
        """

        pass

    def decision(self):
        """
        The state contains the world representation.
        :return: a decision
        """
        pass
