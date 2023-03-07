# -*- coding:utf-8 -*-
# !/usr/bin/env python
"""redballcontroller controller."""

from controller import Robot, Supervisor, Display
from numpy import cos, sin, abs
import numpy as np
import transformations as tf
from copy import deepcopy
import sys
# import threading
import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

##把所有从webots中读到的物体格式进行归一化
class ObstacleArray(object):
    class Obstacle:
        shape_ = 0  # 障碍物种类
        xyz_ = [0, 0, 0]  # 障碍物位置
        scalexyz_ = [0, 0, 0]  # 障碍物尺寸

        def __init__(self, shape, xyz, scalexyz):
            self.shape_ = shape
            self.xyz_ = xyz
            self.scalexyz_ = scalexyz

    obstaclearray = []

    def __init__(self):
        self.obstaclearray = []

    def addNewObstacle(self, shape, xyz, scalexyz):
        obst = self.Obstacle(shape, xyz, scalexyz)
        self.obstaclearray.append(obst)

    def clearFirstObstacle(self):
        self.obstaclearray.pop(0)

    def clearLastObstacle(self):
        self.obstaclearray.pop(-1)

    def clearAllObstacles(self):
        del self.obstaclearray[:]
        obstaclearray = [Obstacle(0, [0, 0, 0], [0, 0, 0])]

    def getAllobstacles(self):
        return deepcopy(self.obstaclearray)


supervisor = Supervisor()
# do this once only
orign_node = supervisor.getFromDef("ROBOTORIGIN")

def getRobotDEFposeofwebots(supervisor_instant, str):
    "This function for get webots DEF pose"

    node = supervisor_instant.getFromDef(str)
    if node is None:
        sys.stderr.write("No" + str + "DEF node found in the current world file\n")
        sys.exit(1)
    transfield = node.getField("translation")
    rotatfield = node.getField("rotation")
    posiball = transfield.getSFVec3f()
    angleball = rotatfield.getSFRotation()
    pose = [posiball[0], posiball[1], posiball[2], angleball[0] * sin(angleball[3] / 2),
            angleball[1] * sin(angleball[3] / 2), angleball[2] * sin(angleball[3] / 2), cos(angleball[3] / 2)]

    return deepcopy(pose)


def getBALLMoveDEFposeofwebots(supervisor_instant, str):
    "This function for get webots Ball DEF pose"

    node = supervisor_instant.getFromDef(str)
    if node is None:
        sys.stderr.write("No" + str + "DEF node found in the current world file\n")
        sys.exit(1)
    transfield = node.getField("translation")
    rotatfield = node.getField("rotation")
    posiball = transfield.getSFVec3f()
    angleball = rotatfield.getSFRotation()
    pose = [posiball[0], posiball[1], posiball[2], angleball[0] * sin(angleball[3] / 2),
            angleball[1] * sin(angleball[3] / 2), angleball[2] * sin(angleball[3] / 2), cos(angleball[3] / 2)]
    # print(pose)
    childfiled = node.getField("children")  ##print('childfiled',childfiled)
    shapenode = childfiled.getMFNode(0)  ##print('shapenode', shapenode.getId())
    geometryfield = shapenode.getField("geometry")  ##对于node下面有node的情况，父node要先获得子node对应名称的field
    geonode = geometryfield.getSFNode()  ##,再从对应名称的field获得对应的node
    rfield = geonode.getField("radius")
    radius = rfield.getSFFloat()

    return deepcopy(pose), [deepcopy(radius)], transfield, rotatfield


def getBALLDEFposeofwebots(supervisor_instant, str, obstlist):
    "This function for get webots Ball DEF pose"

    node = supervisor_instant.getFromDef(str)
    if node is None:
        sys.stderr.write("No" + str + "DEF node found in the current world file\n")
        sys.exit(1)
    transfield = node.getField("translation")
    rotatfield = node.getField("rotation")
    posiball = transfield.getSFVec3f()
    angleball = rotatfield.getSFRotation()
    pose = [posiball[0], posiball[1], posiball[2], angleball[0] * sin(angleball[3] / 2),
            angleball[1] * sin(angleball[3] / 2), angleball[2] * sin(angleball[3] / 2), cos(angleball[3] / 2)]
    # print(pose)
    childfiled = node.getField("children")  ##print('childfiled',childfiled)
    shapenode = childfiled.getMFNode(0)  ##print('shapenode', shapenode.getId())
    geometryfield = shapenode.getField("geometry")  ##对于node下面有node的情况，父node要先获得子node对应名称的field
    geonode = geometryfield.getSFNode()  ##,再从对应名称的field获得对应的node
    rfield = geonode.getField("radius")
    radius = rfield.getSFFloat()

    obstlist.addNewObstacle(2, posiball, [radius * 2] * 3)

    return deepcopy(pose), [deepcopy(radius)], transfield, rotatfield


def getWallDEFposeofwebots(supervisor_instant, str, obstlist):
    '''
    This function for get webots wall DEF pose
    All Wall in webots must be consistent
    '''

    node = supervisor_instant.getFromDef(str)
    if node is None:
        sys.stderr.write("No" + str + "DEF node found in the current world file\n")
        sys.exit(1)
    transfield = node.getField("translation")
    rotatfield = node.getField("rotation")
    sizefield = node.getField("size")
    posiball = transfield.getSFVec3f()
    angleball = rotatfield.getSFRotation()
    size = sizefield.getSFVec3f()
    pose = [posiball[0], posiball[1], posiball[2], angleball[0] * sin(angleball[3] / 2),
            angleball[1] * sin(angleball[3] / 2), angleball[2] * sin(angleball[3] / 2), cos(angleball[3] / 2)]
    # print(pose)
    pose[2] = pose[2] + size[1] / 2  # 这个坐标已经是机器人体坐标系下面的了，所以要在pose z方向加长度
    pose.append(size[0])
    pose.append(size[1])
    pose.append(size[2])
    obstlist.addNewObstacle(1, pose[0:3], [size[2], size[0], size[1]])
    return deepcopy(pose)


def getTableDEFposeofwebots(supervisor_instant, str, obstlist):
    '''
        This function for get webots Table DEF pose and body frame in the center of table face
    '''

    node = supervisor_instant.getFromDef(str)
    if node is None:
        sys.stderr.write("No" + str + "DEF node found in the current world file\n")
        sys.exit(1)
    transfield = node.getField("translation")
    rotatfield = node.getField("rotation")
    sizefield = node.getField("size")
    feetsizefield = node.getField("feetSize")
    thicksizefield = node.getField("frameThickness")
    posiball = transfield.getSFVec3f()
    angleball = rotatfield.getSFRotation()
    size = sizefield.getSFVec3f()
    feetsize = feetsizefield.getSFVec2f()
    thicksize = thicksizefield.getSFFloat()
    pose = [posiball[0], posiball[1], posiball[2], angleball[0] * sin(angleball[3] / 2),
            angleball[1] * sin(angleball[3] / 2), angleball[2] * sin(angleball[3] / 2), cos(angleball[3] / 2)]
    print(pose)
    pose[2] = pose[2] + size[1] - thicksize / 2
    # print(pose)
    for i in size:
        pose.append(i)
    for i in feetsize:
        pose.append(i)
    pose.append(thicksize)
    obstlist.addNewObstacle(1, pose[0:3], [size[2], size[0], thicksize])

    return deepcopy(pose)


def getCYLINDERDEFposeofwebots(supervisor_instant, str, cylindersize, obstlist):
    "This function for get webots Ball DEF pose"

    node = supervisor_instant.getFromDef(str)
    if node is None:
        sys.stderr.write("No" + str + "DEF node found in the current world file\n")
        sys.exit(1)
    transfield = node.getField("translation")
    rotatfield = node.getField("rotation")
    posiball = transfield.getSFVec3f()
    angleball = rotatfield.getSFRotation()
    pose = [posiball[0], posiball[1], posiball[2], angleball[0] * sin(angleball[3] / 2),
            angleball[1] * sin(angleball[3] / 2), angleball[2] * sin(angleball[3] / 2), cos(angleball[3] / 2)]
    # print(pose)
    cylindernode = supervisor_instant.getFromDef(cylindersize)
    childfiled = cylindernode.getField("children")  ##print('childfiled',childfiled)
    shapenode = childfiled.getMFNode(0)  ##print('shapenode', shapenode.getId())
    geometryfield = shapenode.getField("geometry")  ##对于node下面有node的情况，父node要先获得子node对应名称的field
    geonode = geometryfield.getSFNode()  ##,再从对应名称的field获得对应的node
    rfield = geonode.getField("radius")
    radius = rfield.getSFFloat()

    hfield = geonode.getField("height")
    height = hfield.getSFFloat()

    pose.append(height)
    pose.append(radius)

    obstlist.addNewObstacle(3, pose[0:3], [radius * 2, radius * 2, height])
    return deepcopy(pose)


def getstaticmarker(tempmarkid, i, rospy, frameidstr, nsstr):
    tempmarker = Marker()
    tempmarker.header.frame_id = frameidstr
    tempmarker.header.stamp = rospy.Time.now()
    tempmarker.ns = nsstr
    tempmarker.id = tempmarkid
    tempmarker.type = i.shape_
    tempmarker.action = 0
    tempmarker.pose.position.x = i.xyz_[0]
    tempmarker.pose.position.y = i.xyz_[1]
    tempmarker.pose.position.z = i.xyz_[2]
    # tempmarker.pose.position.z = i.xyz_[2] - 0.3  ##0.3 urdf模型机器人腰坐标系不在原点
    tempmarker.pose.orientation.x = 0
    tempmarker.pose.orientation.y = 0
    tempmarker.pose.orientation.z = 0
    tempmarker.pose.orientation.w = 1
    tempmarker.scale.x = i.scalexyz_[0]
    tempmarker.scale.y = i.scalexyz_[1]
    tempmarker.scale.z = i.scalexyz_[2]
    tempmarker.color.r = 0.8
    tempmarker.color.g = 1.0
    tempmarker.color.b = 0
    tempmarker.color.a = 1.0
    tempmarker.lifetime = rospy.Duration()

    return deepcopy(tempmarker)

def getdynamicmarker(tempmarkid, i, rospy, frameidstr, nsstr):
    tempmarker = Marker()
    tempmarker.header.frame_id = frameidstr
    tempmarker.header.stamp = rospy.Time.now()
    tempmarker.ns = nsstr
    tempmarker.id = tempmarkid
    tempmarker.type = i.shape_
    tempmarker.action = 0
    tempmarker.pose.position.x = i.xyz_[0]
    tempmarker.pose.position.y = i.xyz_[1]
    tempmarker.pose.position.z = i.xyz_[2]
    # tempmarker.pose.position.z = i.xyz_[2] - 0.3  ##0.3 urdf模型机器人腰坐标系不在原点
    tempmarker.pose.orientation.x = 0
    tempmarker.pose.orientation.y = 0
    tempmarker.pose.orientation.z = 0
    tempmarker.pose.orientation.w = 1
    tempmarker.scale.x = i.scalexyz_[0]
    tempmarker.scale.y = i.scalexyz_[1]
    tempmarker.scale.z = i.scalexyz_[2]
    tempmarker.color.r = 1.0
    tempmarker.color.g = 0
    tempmarker.color.b = 0
    tempmarker.color.a = 1.0
    tempmarker.lifetime = rospy.Duration()

    return deepcopy(tempmarker)

#supervisor.setLabel(0, "hello world", 0, 0, 0.1, 0xff0000, 0, "Arial")
# print(1)
###动态障碍物球体
node = supervisor.getFromDef("REDBALL")
if node is None:
    sys.stderr.write("No" + str + "DEF node found in the current world file\n")
    sys.exit(1)
transfield = node.getField("translation")
rotatfield = node.getField("rotation")
posiball = transfield.getSFVec3f()
angleball = rotatfield.getSFRotation()
print("posiball", posiball)
print("angleball", angleball)


poseball, poseballr, trans_fieldball, _ = getBALLMoveDEFposeofwebots(supervisor, "REDBALL")


trball = tf.quaternion_matrix(poseball[3:7])
trball[0:3, 3] = poseball[0:3]

###机器人
posewlk = getRobotDEFposeofwebots(supervisor, "WalkerX")
trwlk = tf.quaternion_matrix(posewlk[3:7])
trwlk[0:3, 3] = posewlk[0:3]

print('poseball', poseball)
print('trball')
print(trball)

print('posewlk', posewlk)
print('trwlk')
print(trwlk)

trwlk2 = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

trwlkinv = tf.inverse_matrix(trwlk)

poseball2wlk = [0] * 7

trball2wlk = trwlk2.dot(trwlkinv.dot(trball))
poseball2wlk[0:3] = trball2wlk[0:3, 3]
poseball2wlk[3:7] = tf.quaternion_from_matrix(trball2wlk)

print('poseball2wlk', poseball2wlk)

# print(point_field.getCount())
# ptcount = point_field.getCount()

##静态障碍物已经全部建立在机器人torso坐标系下面了
##添加障碍物 ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3
obstlist = ObstacleArray()
# stopball1, stopball1r, _, _ = getBALLDEFposeofwebots(supervisor, "BLUEBALLSTOP", obstlist)
stopball2, stopball2r, _, _ = getBALLDEFposeofwebots(supervisor, "REDBALLSTOP", obstlist)
# wall1info = getWallDEFposeofwebots(supervisor, "WALL1", obstlist)
# wall2info = getWallDEFposeofwebots(supervisor, "WALL2", obstlist)
tableinfo = getTableDEFposeofwebots(supervisor, "TABLE", obstlist)
caninfo = getCYLINDERDEFposeofwebots(supervisor, "CAN", "CANSIZE", obstlist)
goalinfo = getCYLINDERDEFposeofwebots(supervisor, "GOAL", "CANSIZE", obstlist)
obstlist.addNewObstacle(2, poseball2wlk[0:3], [poseballr[0] * 2] * 3)  ## rviz里面显示的是直径，但是fcl是半径

DEBUG = 0
TIME_STEP = 1
# get init position
speed = 0.2
Radius = 0.1
theta = 0.0
rate = 1.0 / TIME_STEP * 1000
index = 0
coordvalue = 0
MAXIMUM_NUMBER_OF_COORDINATES = 100

if DEBUG == 0:
    tmp = 0
    tempmarkid = 0
    if __name__ == '__main__':
        # 启动之后等个数秒再开legmotion

        print ('test')
        try:
            rospy.init_node('webotstest', anonymous=True)
        except rospy.exceptions.ROSException as e:
            print("Node has already been initialized, do nothing")

        # 发布静态障碍物到rviz 	//添加障碍物 ARROW=0, CUBE=1, SPHERE=2, CYLINDER=3
        obstarraymarker = MarkerArray()
        for listindex, listvalue in enumerate(obstlist.obstaclearray):
            print listindex, listvalue.xyz_
            obstarraymarker.markers.append(
                getstaticmarker(listindex, listvalue, rospy, "/torso_base_link", "obstacles_visualization"))

        rospy.init_node('webotstest', anonymous=True)

        rosrate = rospy.Rate(1000)

        env_pub = rospy.Publisher("/static_obstacles", MarkerArray, queue_size=1)
        dynamic_pub = rospy.Publisher("/dynamic_obstacles", MarkerArray, queue_size=1)

        goalball = [0] * 3
        while not rospy.is_shutdown():
            goalball[0] = poseball[0] + Radius * abs(cos(theta))
            goalball[1] = poseball[1] + Radius * sin(theta)
            goalball[2] = poseball[2] + Radius * cos(theta)

            theta = theta + speed / Radius / rate

            if supervisor.step(TIME_STEP) != -1:
                trans_fieldball.setSFVec3f(goalball)

            trball[0:3, 3] = goalball

            trball2wlk = trwlk2.dot((trwlkinv.dot(trball)))
            poseball2wlk[0:3] = trball2wlk[0:3, 3]
            poseball2wlk[3:7] = tf.quaternion_from_matrix(trball2wlk)


            obstlist.clearLastObstacle()
            staticobstarraymarker = MarkerArray()
            for listindex, listvalue in enumerate(obstlist.obstaclearray):
                staticobstarraymarker.markers.append(
                    getstaticmarker(listindex, listvalue, rospy, "/torso_base_link", "static_obstacles"))
            env_pub.publish(staticobstarraymarker)

            dynamicobstarraymarker = MarkerArray()
            obstlist.addNewObstacle(2, poseball2wlk[0:3], [poseballr[0] * 2] * 3)  ##更新障碍物列表里面动态障碍物

            dynamicobstarraymarker.markers.append(getdynamicmarker(len(obstlist.obstaclearray)-1, obstlist.obstaclearray[-1], rospy, "/torso_base_link", "dynamic_obstacles"))
            dynamic_pub.publish(dynamicobstarraymarker)

        sys.exit()
        supervisor.simulationQuit()
