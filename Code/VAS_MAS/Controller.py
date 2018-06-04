# coding :utf-8
# deployed with Python 3.6.0


import Robot_World

sensor = ['']  # TODO
myWorld = Robot_World.World(sensor, host='127.0.0.1', port=19999)
myRobotBrain = Robot_World.RobotBrain()

while True:
    result = myWorld.sense(sensor)
    action = myRobotBrain.think(result)
    print(action)
    myWorld.act(action)
