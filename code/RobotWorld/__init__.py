try:
    import vrep
    import math
    import random
    import struct
    import time
    import array
    import numpy as np
    import redis

except ImportError as e:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print(e)


class World(object):
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

        out = {}  # list.
        result, resolution, data = vrep.simxGetVisionSensorDepthBuffer(self._clientID,
                                                                       self.sensors_handles['kinect_depth'],
                                                                       self._operation_mode)
        if result != vrep.simx_return_ok:  # checking the reading result.
            exit(result)

        out['depth'] = self.get_depth(data)  # appending the distance depth.

        result, resolution, image = vrep.simxGetVisionSensorImage(self._clientID,
                                                                  self.sensors_handles['kinect_rgb'],
                                                                  0,
                                                                  vrep.simx_opmode_blocking)

        out['vision'] = self.get_vision(resolution, image)

        self._term.write("{}".format(out))

        return out

    def get_depth(self, matrix):
        """
        Get the depth camera matrix and return it.
        :param matrix: the entire matrix
        :return: the clean matrix.
        """
        # 640*480 resolution
        depth = 100
        for i in range(210, 430):
            for j in range(480):
                if matrix[i*220+j] < depth:
                    depth = matrix[i*220+j]  # update matrix.
        return depth

    def get_vision(self, resolution, image):
        """
        Compute the position of red/green object in the scene.
        :param resolution: Resolution of the camera.
        :param image: Seen image.
        :return: Return a tuple containing the detected color and the position.
        """

        image = np.array(image, dtype=np.uint8)
        section = int(resolution[0]/3)  # x section: partitioning on the x
        sectiony = int(resolution[1]/3)  # y section: partitioning on the Y
        detected_color = "NONE"
        position = "NONE"
        colors = 3
        # the len of the image is 921600 = 640x480x3 => this means that each 3 values is a pixel
        # red = 200,41,41
        # green = 72,233,72

        pos_from_left = None
        pos_from_right = None

        for x in range(resolution[0]-3):
            for y in range(sectiony, sectiony*2):
                r = image[colors * (y * resolution[0] + x)]
                g = image[colors * (y * resolution[0] + x) + 1]
                b = image[colors * (y * resolution[0] + x) + 2]
                if (r == 0 or g == 0 or b == 0) and pos_from_right is None:  # if black
                    continue
                elif (r == 0 or g == 0 or b == 0) and pos_from_right is not None:  # pos from right is final, i can exit
                    avg = (pos_from_left + pos_from_right) / 2  # the center of mass.

                    if avg < section:
                        position = "LEFT"
                    elif avg > section * 2:
                        position = "RIGHT"
                    else:
                        position = "CENTER"

                    return detected_color, position

                if g > 190:  # we detect green.
                    detected_color = "GREEN"

                    if pos_from_left is None:  # we save the first x position of green.
                        pos_from_left = x

                    pos_from_right = x  # we save the last x position of green.

                elif r > 190:  # we detect red.
                    detected_color = "RED"

                    if pos_from_left is None:  # we save the first x position of red.
                        pos_from_left = x

                    pos_from_right = x  # we save the last x position of red.

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

    def act(self, action):
        """
        translates abstracted actions to elementary actiolns
        :param action: string resembling the action to perform
        :return:
        """
        if action == "TURN_RIGHT":
            self.turn(0, 2, 45)
        else:
            self.go(5)


class Brain(object):

    def __init__(self, port, terminal):
        self._state = None
        self._load = "full"  # the robot have no package on the top.
        
        self._port = port  # robot port.
        self._agent_name = "turtlebot_{}".format(self._port)  # name of the agent.
        self._topic = "fromMAS"  # communication topic (from DALI to me).
        self._term = terminal

        self._to_linda = redis.Redis()  # is the channel object to the proxy.
        self._from_linda = redis.Redis(host='127.0.0.1', port=6379)  # channel from linda (proxy).
        self._sub = self._from_linda.pubsub()
        self._sub.subscribe(self._topic)

        self._term.write("subbed to topic: {}".format(self._topic))

        # publish
        # self._to_linda.publish("LINDAchannel", self._agent_name+"helloWorld")

        # sub + read
        # from_linda = redis.Redis(host='127.0.0.1', port=6379)  # channel from linda
        # sub = from_linda.pubsub()
        # sub.subscribe('fromMAS')
        #
        # print('listening for messages from MAS...')
        # for item in sub.listen():
        #    print(item)
        #    if item['type'] == 'message':
        #        msg = item['data']
        #        print(msg)

    def think(self, sensor_reading):
        """
        It decides what do.
        :param sensor_reading: result of the sense.
        :return: an action.
        """
        self._state, result = self.perception(sensor_reading)
        if result == 0:  # the world is changed, we have to call DALI.
            action = self.decision()
        else:  # the world no changed.
            action = self.ground_decision(sensor_reading)
        return action

    def perception(self, sensor_reading):
        """
        Read state and build a world representation.
        :param sensor_reading: result of the sense
        :return: state
        """

        predicate = []  # we build the predicate.
        result = None
        # 0 if the new state if different from the previous one, 1 otherwise

        predicate.append(sensor_reading['vision'][0].lower())  # all to lower case.
        predicate.append(sensor_reading['vision'][1].lower())  # all to lower case.
        predicate.append(sensor_reading['depth'].lower())  # all to lower case.
        predicate.append(self._load.lower())

        if self.compare_states(predicate, self._state):  # the world changed.
            result = 0
        else:
            result = 1

        return self._state, result

    def decision(self):
        """
        The state contains the world representation.
        :return: a decision
        """
        action = ""

        message = "vision("+self._state[0]+","+self._state[1]+")."
        # publishing.
        self._to_linda.publish("LINDAchannel", self._agent_name+message)

        # publishing.
        message = "depth("+self._state[2]+")."
        self._to_linda.publish("LINDAchannel", self._agent_name+message)

        # publishing.
        message = "load("+self._state[3]+")."
        self._to_linda.publish("LINDAchannel", self._agent_name + message)

        print('listening for decision from MAS...')
        for item in self._sub.listen():
            if item['type'] == 'message':
                action = item['data']
                print(action)
        return action

    def ground_decision(self, sensor_reading):
        """
        The robot has to avoid collisions.
        :return: a decision.
        """

        if sensor_reading['depth'] > 0.25:
            return "TURN_RIGHT"
        return "GO"

    def compare_states(self, new_state, old_state):

        if old_state[0] != new_state[0] or old_state[1] != new_state[1] or old_state[3] != new_state[3]:
            return True
        else:
            if(new_state[2] - old_state[2]) > 0.2:  # todo: check threshold
                return True
            else:
                return False
