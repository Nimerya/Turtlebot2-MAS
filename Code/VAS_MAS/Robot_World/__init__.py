# coding :utf-8
# deployed with Python 3.6.0

"""
TO DO
    Write a RobotWorld Python 3.5.x wrapper library with classes that encapsulates the V-REP Remote API
    ....
"""

try:
    import vrep
    import math
    import random
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
    Robot simulator class to comunicate with the simulation environment

    """

    def __init__(self, sensor, host='127.0.0.1', port=16123):
        """
        Initialize the connection with V-REP.
        :param host: host number.
        :param port: port number.
        """
        self._host = host
        self._port = port
        vrep.simxFinish(-1)  # just in case, close all opened connections.
        self._clientID = vrep.simxStart(self._host, self._port, True, True, 5000, 5)  # Connect to V-REP
        if self._clientID == -1:  # connection error
            print("Connection to the server was not possible")
        operation_mode = vrep.operation_mode
        # res1, leftJointHandle = vrep.simxGetObjectHandle(self._clientID, "dr12_leftJoint_", operationMode)
        # res2, rightJointHandle = vrep.simxGetObjectHandle(self._clientID, "dr12_rightJoint_", operationMode)
        # vrep.simxSetJointTargetVelocity(self._clientID, leftJointHandle, 100*math.pi/180,operationMode)
        # vrep.simxSetJointTargetVelocity(self._clientID, rightJointHandle, 100*math.pi/180, operationMode)

    def sense(self, sensor_name):
        """
        Sense the world and return data.
        :param sensor_name: name of the sensor.
        :return: data.
        """

        out = []  # list.
        assert isinstance(sensor_name, str)  # check if the variable is a list.
        # res1, proximity_sensor_handle = vrep.simxGetObjectHandle(self._clientID, sensor_name, operation_mode)
        ## I have a pointer to the sensor.
        #
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
        #    return out

        return out

    def act(self, action):
        """
        Receive a command, send an "order".
        :param action: command.
        :return: a command to the robot.
        """

        assert isinstance(action, str)  # check if the variable is a string.
        param = []
        param.append(-100 * math.pi / 180)
        param.append(-50 * math.pi / 180)

        if action == 'Move':
            operationMode = vrep.simx_opmode_blocking
            res1, leftJointHandle = vrep.simxGetObjectHandle(self._clientID, "dr12_leftJoint_", operationMode)
            res2, rightJointHandle = vrep.simxGetObjectHandle(self._clientID, "dr12_rightJoint_", operationMode)
            if (vrep.simxSetJointTargetVelocity(self._clientID, leftJointHandle, param[0],
                                                operationMode)) == vrep.simx_return_ok and (
            vrep.simxSetJointTargetVelocity(self._clientID, rightJointHandle, param[1],
                                            operationMode)) == vrep.simx_return_ok:
                print('')
            else:
                print('problems')
        if action == 'Continue':
            operationMode = vrep.simx_opmode_blocking
            res1, leftJointHandle = vrep.simxGetObjectHandle(self._clientID, "dr12_leftJoint_", operationMode)
            res2, rightJointHandle = vrep.simxGetObjectHandle(self._clientID, "dr12_rightJoint_", operationMode)
            vrep.simxSetJointTargetVelocity(self._clientID, leftJointHandle, 100 * math.pi / 180, operationMode)
            vrep.simxSetJointTargetVelocity(self._clientID, rightJointHandle, 100 * math.pi / 180, operationMode)
        return True


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
