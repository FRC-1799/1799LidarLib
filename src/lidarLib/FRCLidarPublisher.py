from math import floor
import threading
import time
import ntcore
from wpimath.geometry import Pose2d, Rotation2d

from lidarLib.constants import constants
from lidarLib.lidarMap import lidarMap
from lidarLib.lidarMeasurement import lidarMeasurement
from lidarLib.translation import translation

class publisher:
    def __init__ (self, teamNumber:int, autoConnect:bool=True):
        self.publisher:ntcore.NetworkTableInstance=None # type: ignore

        self.teamNumber=teamNumber
        self.autoConnect=autoConnect

        if self.autoConnect:
            self.loop = threading.Thread(target=self.connectionTester, daemon=True)
            self.loop.start()
        self.nodeWidth:float =0

        

    def setUpTables(self):
        

        self.publishFolder=  self.publisher.getTable("lidar")
        self.individualPointTopic = self.publishFolder.getStructArrayTopic("lidarReadings", Pose2d)
        self.individualPointPublisher = self.individualPointTopic.publish()


        self.poseTopic = self.publisher.getStructTopic("robotPose", Pose2d)
        self.poseSubscriber = self.poseTopic.subscribe(Pose2d())
        

        self.nodeWidthTopic = self.publishFolder.getFloatTopic("NodeWidth")
        self.nodeWidthPublisher = self.nodeWidthTopic.publish()
        self.updateNodeWidth(constants.mapNodeSizeMeters)

        self.lidarPoseTopic = self.publishFolder.getStructArrayTopic("lidarPoses", Pose2d)
        self.lidarPosePublisher = self.lidarPoseTopic.publish()


    def connect(self, port:str=None, teamNumber:int=None, name:str="lidar", startAsServer:bool=False, saveConnectionIfSuccessful:bool=True)->bool: # type: ignore
        connecter:ntcore.NetworkTableInstance = ntcore.NetworkTableInstance.getDefault()
        if port == teamNumber: # type: ignore
            raise ValueError("Must give a team number or a port when trying to connect to network tables. values given were port:", port,". TeamNumber:", teamNumber)

        if teamNumber:
            connecter.setServerTeam(teamNumber)

        else:
            connecter.setServer(port)
        
        
        if startAsServer:
            pass#connecter.startServer()
        else:
            connecter.startClient4(name)


        if connecter.isConnected() and saveConnectionIfSuccessful:
            self.publisher=connecter
            self.setUpTables()


        return connecter.isConnected()
    

    def connectionTester(self):
        while True:
            if self.teamNumber!=0:        
                if self.connect(teamNumber=self.teamNumber) or self.connect(port="127.0.0.1"):
                    break
            else:
                if self.connect(port="127.0.0.1"):
                    break

            time.sleep(4)
        print("Connected on port", self.publisher.getConnections()[0].remote_ip)
                

    def getPose(self)->Pose2d:
        # TODO
        # if self.poseSubscriber.get()==Pose2d():
        #     raise Warning("LidarLib received a pose of 0,0,0. This likely means that the lidar lib is not getting a valid pose input which stops it from accurately tracking")

        # if self.poseSubscriber.getLastChange()< self.publisher.addTimeSyncListener() self.publisher.getServerTimeOffset()-0.2 and not self.isConnectedToSim():
        #     raise Warning("robot pose data (at position \"robotPose\"is not being consistently updated. This can cause lidar data to be inaccurate")
        

        return self.poseSubscriber.get()

    def getRobotPoseAsTrans(self)->translation:
        """Returns the pose of the robot as a translation object."""
 
        return translation.fromPose2d(self.getPose())

    
    
    def publishLidarReadingFromPose(self, poses:list[Pose2d]):
        """Publishes a lidar scan from a list of pose2d objects."""
        self.__publishLidarReadings(poses)
    
    def publishPointsFromLidarMeasurements(self, measurements:list[lidarMeasurement]):
        """Publishes a lidar scan from a list of lidar measurement objects."""
        poses:list[Pose2d] = []
        for measurement in measurements:
            poses.append(Pose2d(measurement.getX(), measurement.getY(), Rotation2d()))

        self.__publishLidarReadings(poses)
           

    def publishHitboxesFromLidarMap(self, map:lidarMap):
        """Publishes a lidar scan from a lidar Map object"""
        poses:list[Pose2d] = []
        for measurement in map.getPoints():
            measurement:lidarMeasurement
           
            poses.append(Pose2d(measurement.x, measurement.y, Rotation2d())) # type: ignore
        
        self.__publishLidarReadings(poses)

    def __publishLidarReadings(self, map:list[Pose2d]):
        """Internal function to publish lidar readings. manages the internal grid so that a manageable amount of data is published to network tables"""
        poseDict:dict[float, dict[float, bool]] = {}
        for pose in map:
            if poseDict.get(floor(pose.x/self.nodeWidth)*self.nodeWidth)==None:
                 poseDict[floor(pose.x/self.nodeWidth)*self.nodeWidth]={}
            poseDict[floor(pose.x/self.nodeWidth)*self.nodeWidth][floor(pose.y/self.nodeWidth)*self.nodeWidth] = True

        publishPoses:list[Pose2d] = []
        for x in poseDict.keys():
            for y in poseDict[x].keys():
                publishPoses.append(Pose2d(x, y, Rotation2d()))

        self.individualPointPublisher.set(publishPoses)



    def updateNodeWidth(self, nodeWidth:float):
        """Changes the published node width value. This tells other network table connections how big the points on the grid used to publish are."""
        self.nodeWidth=nodeWidth
        self.nodeWidthPublisher.set(nodeWidth)


    def publishLidarPosesFromPose(self, poses:list[Pose2d]):
        """
            Publishes the poses of lidar units from given pose2d objects.
            If you would like to enter translation objects instead use the publishLidarPosesFromTrans method instead
        """
        self.lidarPosePublisher.set(poses)

    def publishLidarPosesFromTrans(self, trans:list[translation]):
        """
            Publishes the poses of lidar units from given translation objects.
            If you would like to enter pose2d objects instead use the publishLidarPosesFromPose method instead
        """
        poses:list[Pose2d] = []
        for translation in trans:
            poses.append(Pose2d(translation.x, translation.y, translation.rotation))

        self.publishLidarPosesFromPose(poses)

    def isConnectedToSim(self)->bool:
        """
            Returns wether the lidar is connected to a simulation. 
            If the lidar is not connected at all it will return false. Because of this isConnectedToNonSim should be used if attempting to determine if the lib is connected to a robot.
        """
        return self.publisher.isConnected() and self.publisher.getConnections()[0].remote_ip == "127.0.0.1"
        
    def isConnectedToNonSim(self)->bool:
        return self.publisher.isConnected() and not self.isConnectedToSim()

    def isConnected(self)->bool:
        """Returns wether or not the publisher is connected to a network table"""
        return self.publisher and self.publisher.isConnected()