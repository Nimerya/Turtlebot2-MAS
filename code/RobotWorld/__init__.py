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
        self._turning_speed = 1
        self._host = host
        self._port = port
        self._term = terminal
        self._load = "EMPTY"

        vrep.simxFinish(-1)  # just in case, close all opened connections.
        self._clientID = vrep.simxStart(self._host, self._port, True, True, 5000, 5)  # Connect to V-REP.
        if self._clientID == -1:  # connection error
            terminal.write('Connection to the server was not possible')
        self._operation_mode = vrep.simx_opmode_blocking  # fire and forget.
        self.wheels_handles = {}
        self.sensors_handles = {}
        self.signals = signals

        self._term.write('Fetching wheels handles...')
        for w in wheels:  # initialize the robot.
            res, handle = vrep.simxGetObjectHandle(self._clientID, wheels[w], self._operation_mode)
            if res == vrep.simx_return_ok:
                self.wheels_handles[w] = handle
            else:
                self._term.write('Wheels handle error: {}'.format(res))
                exit(1)

        self._term.write('Fetching sensors handles...')
        for s in sensors:  # initialize the robot.
            res, handle = vrep.simxGetObjectHandle(self._clientID, sensors[s], self._operation_mode)
            if res == vrep.simx_return_ok:
                self.sensors_handles[s] = handle
            else:
                self._term.write(self._port, 'Sensors handle error: {}'.format(res))
                exit(1)

        self._term.write("successfully fetched all handles")

    def sense(self):
        """
        Sense the world and return data.
        :return: data.
        """

        out = {}

        result, resolution, data = vrep.simxGetVisionSensorDepthBuffer(self._clientID,
                                                                       self.sensors_handles['kinect_depth'],
                                                                       self._operation_mode)
        if result != vrep.simx_return_ok:  # checking the reading result.
            exit(result)

        out['depth'] = self.get_depth(data)  # appending the distance depth.

        result_vision, resolution, image = vrep.simxGetVisionSensorImage(self._clientID,
                                                                         self.sensors_handles['kinect_rgb'],
                                                                         0,
                                                                         vrep.simx_opmode_blocking)

        result_blob, t0, t1 = vrep.simxReadVisionSensor(self._clientID,
                                                        self.sensors_handles['kinect_rgb'],
                                                        vrep.simx_opmode_blocking)

        out['vision'] = self.get_vision(resolution, image, t1)

        out['load'] = self._load

        self._term.write("sensed: {}".format(out))

        return out

    def get_depth(self, matrix):
        """
        Get the depth camera matrix and return it.
        :param matrix: the entire matrix
        :return: the clean matrix.
        """
        # 640*480 resolution
        depth = 100
        # look only at the central vertical slice
        for i in range(210, 430):
            for j in range(480):
                if matrix[i*220+j] < depth:
                    depth = matrix[i*220+j]  # update matrix.
        return round(depth, 5)

    def get_vision(self, resolution, image, blob_data):
        """
        blob_data[0]=blob count
        blob_data[1]=n=value count per blob
        blob_data[2]=blob 1 size
        blob_data[3]=blob 1 orientation
        blob_data[4]=blob 1 position x
        blob_data[5]=blob 1 position y
        blob_data[6]=blob 1 width
        blob_data[7]=blob 1 height
        ...
        :return:
        """
        color = "NONE"
        position = "NONE"

        blob_data = blob_data[1]

        if blob_data[0] == 0:
            return color, position

        blob_size = blob_data[2]

        # get color
        color = self.get_blob_color(resolution, image)
        if color == "NONE":
            return color, position, ('size', round(blob_size, 5))

        if blob_size >= 0.7:
            return color, "NEAR", ('size', round(blob_size, 5))

        if 0.35 < blob_data[4] < 0.65:
            return color, "CENTER", ('size', round(blob_size, 5))

        if 0.0 < blob_data[4] < 0.35:
            return color, "LEFT", ('size', round(blob_size, 5))

        if 0.65 < blob_data[4] < 1:
            return color, "RIGHT", ('size', round(blob_size, 5))

        return color, position, ('size', round(blob_size, 5))

    @staticmethod
    def get_blob_color(resolution, image):
        """
        Compute the position of red/green object in the scene.
        :param resolution: Resolution of the camera.
        :param image: Seen image.
        :return: Return a the color of the object or none.
        """

        image = np.array(image, dtype=np.uint8)
        detected_color = "NONE"
        colors = 3
        # the len of the image is 921600 = 640x480x3 => this means that each 3 values is a pixel
        # red = 200,41,41
        # green = 72,233,72

        for x in range(resolution[0]-3):
            y = int(resolution[1]/2) # I only look at the line of pixels in the middle
            r = image[colors * (y * resolution[0] + x)]
            g = image[colors * (y * resolution[0] + x) + 1]
            b = image[colors * (y * resolution[0] + x) + 2]
            if r == 0 or g == 0 or b == 0:  # if black
                continue

            if g > 190:  # we detect green.
                detected_color = "GREEN"

            elif r > 190:  # we detect red.
                detected_color = "RED"

        return detected_color

    def stop(self):
        """
        Stop the robot.
        """
        self._term.write('stopped')
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
        self._term.write('turning, angle = {}'.format(angle))
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
            #self._term.write('-------------------------------------------------------\n'
            #      '{} : X-Gyro = {} dps\n        Y-Gyro = {} dps\n        Z-Gyro = {} dps'
            #      .format(self._port, round(gyro_data_unpacked_x, 2), round(gyro_data_unpacked_y, 2),
            #              round(gyro_data_unpacked_z, 2)))

            z += abs(gyro_data_unpacked_z)
            #self._term.write('cumulative angle = {}'.format(z))
        self._term.write('turn completed')

    def go(self, speed):
        """
        The robot go forward.
        :param speed: velocity of both wheels.
        """
        self._term.write('going, speed = {}'.format(speed))
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_right"], speed,
                                        self._operation_mode)
        vrep.simxSetJointTargetVelocity(self._clientID, self.wheels_handles["wheel_left"], speed,
                                        self._operation_mode)

    # TODO loadup
    def loadup(self, ):
        self.stop()
        time.sleep(3)
        self._load = "FULL"
        return

    # TODO unload
    def unload(self):
        self.stop()
        time.sleep(3)
        self._load = "EMPTY"
        return

    def act(self, action):
        """
        translates abstracted actions to elementary actiolns
        :param action: string resembling the action to perform
        :return:
        """
        try:
            separator = action.index(':')
        except Exception as e:
            if action == 'stop':
                self.stop()
                return
            if action == 'unload':
                self.unload()
                return
            if action == 'loadup':
                self.loadup()
                return
        
        value = int(action[separator+1:])
        action = action[:separator]

        if action == 'go':
            self.go(value)
            return
        
        if action == 'right':
            self.turn(0, self._turning_speed, value)
            return
        
        if action == 'left':
            self.turn(self._turning_speed, 0, value)
            return


class Brain(object):

    def __init__(self, world, port, terminal):
        self._depth_treshold = 0.1
        self._world = world
        self._state = None
        self._dali_depth = ""

        self._port = port  # robot port.
        self._agent_name = "turtlebot_{}".format(self._port)  # name of the agent.
        self._topic = "fromMAS"  # communication topic (from DALI to me).
        self._term = terminal

        self._to_linda = redis.Redis()  # is the channel object to the proxy.
        self._from_linda = redis.Redis(host='127.0.0.1', port=6379)  # channel from linda (proxy).
        self._sub = self._from_linda.pubsub()
        self._sub.subscribe(self._topic)
        self._previous_action = None

        self._term.write("subbed to topic: {}".format(self._topic))

    def think(self, sensor_reading):
        """
        It decides what to do.
        :param sensor_reading: result of the sense.
        :return: an action.
        """
        self._state, changed = self.perception(sensor_reading)
        # the world is changed of if the unit is facing the wrong direction -> call DALI.
        if changed:
            # stop the unit while DALi is computing
            self._world.act('stop')
            action = self.decision()
            self._previous_action = action
        else:  # the world did not change
            action = self.ground_decision()
        return action

    def perception(self, sensor_reading):
        """
        Read state and build a world representation.
        :param sensor_reading: result of the sense
        :return: state
        """
        new_state = {'color': sensor_reading['vision'][0].lower(),
                     'position': sensor_reading['vision'][1].lower(),
                     'depth': sensor_reading['depth'],
                     'load': sensor_reading['load'].lower()}  # we build the new_state.

        # the world changed
        if self._state is None:
            self._state = new_state.copy()
            self._dali_depth = self._state['depth']
            # to trigger the first DALI reasoning
            changed = True
        else:
            changed = self.compare_states(self._state, new_state)

        return new_state, changed

    def decision(self):
        """
        The state contains the world representation.
        :return: a decision from DALI
        """

        self._dali_depth = self._state['depth']

        if self._state['depth'] > self._depth_treshold:
            depth = "far"
        else:
            depth = "near"

        vision = "vision({},{}).".format(self._state['color'], self._state['position'])
        depth = "depth({}).".format(depth)
        load = "load({}).".format(self._state['load'])
        name = "agentname('{}:').".format(str(self._port))

        meta = ":- dynamic vision/2. :- dynamic depth/1. :- dynamic load/1. :- dynamic agentname/1."

        message = "{} {} {} {} {}".format(meta, vision, depth, load, name)

        self._to_linda.publish("LINDAchannel", self._agent_name + ':' + message)

        self._term.write('listening for decision from MAS...')
        for item in self._sub.listen():
            if item['type'] == 'message':
                msg = item['data'].decode('utf-8')
                separator = msg.index(':')
                name = int(msg[:separator])
                # if the action is not for me
                if name != self._port:
                    continue
                action = msg[separator+1:]
                self._term.write('received action: {}'.format(action))
                return action

    def ground_decision(self):
        """
        The robot has to avoid collisions.
        :return: a decision.
        """
        if self._state['depth'] <= self._depth_treshold:
            # the unit is near something, call DALI
            return self.decision()
        # if the state is not changed and I'm not colliding then repeat the previous action
        return self._previous_action

    def compare_states(self, old_state, new_state):
        return old_state['color'] != new_state['color'] or \
               old_state['position'] != new_state['position'] or \
               old_state['load'] != new_state['load'] or \
               (abs(new_state['depth'] - self._dali_depth) >= 0.05)
