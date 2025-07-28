import threading
import time
import ntcore
from wpimath.geometry import Pose2d, Rotation2d

from constants import constants
from lidarHitboxingMap import lidarHitboxMap
from lidarLib.lidarMeasurement import lidarMeasurement
from lidarHitboxNode import lidarHitboxNode
from lidarLib.translation import translation


class publisher:
    def __init__ (self, teamNumber, autoConnect=True):
        self.publisher=None

        self.teamNumber=teamNumber
        self.autoConnect=autoConnect

        if self.autoConnect:
            self.loop = threading.Thread(target=self.connectionTester, daemon=True)
            self.loop.start()

    def setUpTables(self):
        

        self.publishFolder=  self.publisher.getTable("lidar")
        self.individualPointTopic = self.publishFolder.getStructArrayTopic("individualReadings", Pose2d)
        self.individualPointPublisher = self.individualPointTopic.publish()

        self.hitboxTopic = self.publishFolder.getStructArrayTopic("detectedHitboxes", Pose2d)
        self.hitboxPublisher = self.hitboxTopic.publish()

        self.poseTopic = self.publisher.getStructTopic("robotPose", Pose2d)
        self.poseSubscriber = self.poseTopic.subscribe(Pose2d())
        

        self.nodeWidthTopic = self.publishFolder.getFloatTopic("NodeWidth")
        self.nodeWidthPublisher = self.nodeWidthTopic.publish()
        self.nodeWidthPublisher.set(constants.mapNodeSizeMeters)

        self.lidarPoseTopic = self.publishFolder.getStructArrayTopic("lidarPoses", Pose2d)
        self.lidarPosePublisher = self.lidarPoseTopic.publish()


    def connect(self, port:str=None, teamNumber:int=None, name="lidar", startAsServer=False, saveConnectionIfSuccessful=True)->bool:
        connecter:ntcore.NetworkTableInstance = ntcore.NetworkTableInstance.getDefault()
        if port == teamNumber:
            raise ValueError("Must give a team number or a port when trying to connect to network tables. values given were port:", port,". TeamNumber:", teamNumber)

        if teamNumber:
            connecter.setServerTeam(teamNumber)

        else:
            connecter.setServer(port)
        
        
        if startAsServer:
            connecter.startServer()
        else:
            connecter.startClient4(name)


        if connecter.isConnected() and saveConnectionIfSuccessful:
            self.publisher=connecter
            self.setUpTables()


        return connecter.isConnected()
    

    def connectionTester(self):
        while True:
            if self.connect(teamNumber=self.teamNumber) or self.connect(port="127.0.0.1"):
                break
            time.sleep(4)
        print("Connected on port", self.publisher.getConnections()[0].remote_ip)
                

    def getPose(self)->Pose2d:
        return self.poseSubscriber.get()

    def getPoseAsTran(self)->Pose2d:
        return translation.fromPose2d(self.getPose())
    
    def publishPointsFromPoses(self, poses:list[Pose2d]):
        
        self.individualPointPublisher.set(poses)
    
    def publishPointsFromLidarMeasurements(self, measurements:list[lidarMeasurement]):
        poses:list[Pose2d] = []
        for measurement in measurements:
            poses.append(Pose2d(measurement.getX(), measurement.getY(), Rotation2d()))

        self.publishPointsFromPoses(poses)
    

    def publishHitboxesFromPoses(self, poses:list[Pose2d]):
        
        self.hitboxPublisher.set(poses)
        

    def publishHitboxesFromHitboxMap(self, map:lidarHitboxMap):
        poses:list[Pose2d] = []
        for node in map.getAs1DList():
            node:lidarHitboxNode
            if (not node.isOpen):
                poses.append(Pose2d(node.x, node.y, Rotation2d()))
        
        self.publishHitboxesFromPoses(poses)

    def updateNodeWith(self, nodeWidth:int):
        self.nodeWidthPublisher.set(nodeWidth)


    def publishLidarPosesFromPose(self, poses:Pose2d):
        self.lidarPosePublisher.set(poses)

    def publishLidarPosesFromTrans(self, trans:list[translation]):
        poses:Pose2d = []
        for translation in trans:
            poses.append(Pose2d(translation.x, translation.y, translation.rotation))

        self.publishLidarPosesFromPose(poses)

    def isConnectedToSim(self)->bool:
        return self.publisher.isConnected() and self.publisher.getConnections()[0].remoteIP == "127.0.0.1"

    def isConnected(self)->bool:
        return self.publisher and self.publisher.isConnected()