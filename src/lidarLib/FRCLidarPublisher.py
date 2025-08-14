from math import floor
import threading
import time
import ntcore
from wpimath.geometry import Pose2d, Rotation2d

from lidarLib.lidarMap import lidarMap
from lidarLib.lidarMeasurement import lidarMeasurement
from lidarLib.translation import translation

class publisher:
    """
        <h2>Class to manage publishing lidar data to network tables.</h2>
        This class only works with FRC network tables and will not work with other networking systems. 
    """
    def __init__ (self, teamNumber:int, autoConnect:bool=True):
        """
            <h2>Creates a new publisher object.</h2>
            Team number should be a FRC team number to be used in publishing. If set to 0 the publisher will only try to connect to simulated robots (127.0.0.1).
            Otherwise the publisher will try to connect to real robots under the team number as well as simulated robots.
            \n
            Auto connect determines wether or not the publisher will manage connecting to network tables on its own. 
            If set to false the user will need to connect manually by calling the connect method and making sure that it actually manages to connect.
        """
        self.publisher:ntcore.NetworkTableInstance=None # type: ignore

        self.teamNumber=teamNumber
        self.autoConnect=autoConnect

        if self.autoConnect:
            self.loop = threading.Thread(target=self.__connectionTester, daemon=True)
            self.loop.start()
        self.nodeWidth:float =0

        self.__DEFAULT_NODE_WIDTH = 0.3

        

    def setUpTables(self):
        """<h2>Configures all the network tables to be used in publishing.</h2>"""
        

        self.publishFolder=  self.publisher.getTable("lidar")
        self.individualPointTopic = self.publishFolder.getStructArrayTopic("lidarReadings", Pose2d)
        self.individualPointPublisher = self.individualPointTopic.publish()


        self.poseTopic = self.publisher.getStructTopic("robotPose", Pose2d)
        self.poseSubscriber = self.poseTopic.subscribe(Pose2d())
        

        self.nodeWidthTopic = self.publishFolder.getFloatTopic("NodeWidth")
        self.nodeWidthPublisher = self.nodeWidthTopic.publish()
        self.updateNodeWidth(self.__DEFAULT_NODE_WIDTH)

        self.lidarPoseTopic = self.publishFolder.getStructArrayTopic("lidarPoses", Pose2d)
        self.lidarPosePublisher = self.lidarPoseTopic.publish()


    def connect(self, port:str=None, teamNumber:int=None, name:str="lidar", startAsServer:bool=False, saveConnectionIfSuccessful:bool=True)->bool: # type: ignore
        """
            <h2>Attempts to Connect to network tables.</h2>

            Port should be a string ipv4 address ex. 127.0.0.1 \n
            Team number should be an integer frc team number ex. 1799.\n 
            Ether a team number or a port (but not both) should be given. If both are given the port will be discarded and only the team number will be used.    

            Start as server determines wether the publisher should try and connect to an existing server or make one itself.

            Save Connection determines wether the system should save a connection if successful or forget it. 
            If this parameter is set too false the publisher will still connect to network tables but will forget the connection and disconnect as soon as this function finishes. 
        """
        connecter:ntcore.NetworkTableInstance = ntcore.NetworkTableInstance.getDefault()
        if port == teamNumber: # type: ignore
            raise ValueError("Must give a team number or a port when trying to connect to network tables. Values given were port:", port,". TeamNumber:", teamNumber)

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
    

    def __connectionTester(self):
        """
            <h2>Function to manage the auto connect feature.</h2>
        """
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
        """<h2>Returns the pose published to \"robotPose\" on network tables. </h2>"""
        # TODO
        # if self.poseSubscriber.get()==Pose2d():
        #     raise Warning("LidarLib received a pose of 0,0,0. This likely means that the lidar lib is not getting a valid pose input which stops it from accurately tracking")

        # if self.poseSubscriber.getLastChange()< self.publisher.addTimeSyncListener() self.publisher.getServerTimeOffset()-0.2 and not self.isConnectedToSim():
        #     raise Warning("robot pose data (at position \"robotPose\"is not being consistently updated. This can cause lidar data to be inaccurate")
        

        return self.poseSubscriber.get()

    def getRobotPoseAsTrans(self)->translation:
        """<h2>Returns the pose of the robot (read from \"robotPose\" on network tables) as a translation object.</h2>"""
 
        return translation.fromPose2d(self.getPose())

    
    
    def publishLidarReadingFromPose(self, poses:list[Pose2d]):
        """
            <h2>Publishes a lidar scan from a list of pose2d objects.</h2>
            Data will be published as a list of pose2d objects in a grid to \"lidar/lidarReadings\".'
            Instead of publishing every reading the library will only publish the list of places on a grid in which at least one reading was present.
            This system exists so that a manageable amount of data (approx. 600 readings per second) is publishes instead of the original data set captured by the lidar (up to 16000 per lidar).
            The width of squares on the grid can be changed with the updateNodeWidth function.
            However keep in mind that while a smaller grid size does increase the accuracy of the data it also puts more strain on network tables.
        """

        self.__publishLidarReadings(poses)
    
    def publishPointsFromLidarMeasurements(self, measurements:list[lidarMeasurement]):
        """
            <h2>Publishes a lidar scan from a list of lidar measurement objects.</h2>
            Data will be published as a list of pose2d objects in a grid to \"lidar/lidarReadings\".'
            Instead of publishing every reading the library will only publish the list of places on a grid in which at least one reading was present.
            This system exists so that a manageable amount of data (approx. 600 readings per second) is publishes instead of the original data set captured by the lidar (up to 16000 per lidar).
            The width of squares on the grid can be changed with the updateNodeWidth function.
            However keep in mind that while a smaller grid size does increase the accuracy of the data it also puts more strain on network tables.
        """
        poses:list[Pose2d] = []
        for measurement in measurements:
            poses.append(Pose2d(measurement.getX(), measurement.getY(), Rotation2d()))

        self.__publishLidarReadings(poses)
           

    def publishHitboxesFromLidarMap(self, map:lidarMap):
        """
            <h2>Publishes a lidar scan from a lidar Map object</h2>
            Data will be published as a list of pose2d objects in a grid to \"lidar/lidarReadings\".'
            Instead of publishing every reading the library will only publish the list of places on a grid in which at least one reading was present.
            This system exists so that a manageable amount of data (approx. 600 readings per second) is publishes instead of the original data set captured by the lidar (up to 16000 per lidar).
            The width of squares on the grid can be changed with the updateNodeWidth function.
            However keep in mind that while a smaller grid size does increase the accuracy of the data it also puts more strain on network tables.
        """
        poses:list[Pose2d] = []
        for measurement in map.getPoints():
            measurement:lidarMeasurement
           
            poses.append(Pose2d(measurement.x, measurement.y, Rotation2d())) # type: ignore
        
        self.__publishLidarReadings(poses)

    def __publishLidarReadings(self, map:list[Pose2d]):
        """
            <h2>Internal function to publish lidar readings.</h2> 
            Manages the internal grid so that a manageable amount of data is published to network tables.
        """
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
        """
            <h2>Changes the node width value to the given value in meters. </h2> 
            This Value manages the size of the grid lidar readings are published. 
        """
        self.nodeWidth=nodeWidth
        self.nodeWidthPublisher.set(nodeWidth)


    def publishLidarPosesFromPose(self, poses:list[Pose2d]):
        """
            <h2>Publishes the poses of lidar units from given pose2d objects.</h2>
            If you would like to enter translation objects instead use the publishLidarPosesFromTrans method instead.
        """
        self.lidarPosePublisher.set(poses)

    def publishLidarPosesFromTrans(self, trans:list[translation]):
        """
            <h2>Publishes the poses of lidar units from given translation objects.</h2>
            If you would like to enter pose2d objects instead use the publishLidarPosesFromPose method instead.
        """
        poses:list[Pose2d] = []
        for translation in trans:
            poses.append(Pose2d(translation.x, translation.y, translation.rotation))

        self.publishLidarPosesFromPose(poses)

    def isConnectedToSim(self)->bool:
        """
            <h2>Returns wether the lidar is connected to a simulation.</h2>
            If the lidar is not connected at all it will return false. Because of this isConnectedToNonSim should be used if attempting to determine if the lib is connected to a real robot.
        """
        return self.publisher.isConnected() and self.publisher.getConnections()[0].remote_ip == "127.0.0.1"
        
    def isConnectedToNonSim(self)->bool:
        """
            <h2>Returns wether the lidar is connected to a real robot.</h2>
            If the lidar is not connected at all it will return false. Because of this isConnectedToSim should be used if attempting to determine if the lib is connected to a simulated robot.
        """
        return self.publisher.isConnected() and not self.isConnectedToSim()

    def isConnected(self)->bool:
        """<h2>Returns wether or not the publisher is connected to a network table.</h2>"""
        return self.publisher and self.publisher.isConnected()