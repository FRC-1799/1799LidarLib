import json
from math import acos, asin, cos, pi, sin
import os
import threading
import time

import serial
from importlib import resources
from lidarLib.translation import translation
from lidarLib.Lidar import Lidar
from lidarLib.LidarConfigs import lidarConfigs
from serial.tools import list_ports

from lidarLib.lidarProtocol import RPLIDAR_CMD_GET_HEALTH, RPLIDAR_DESCRIPTOR_LEN, RPLIDAR_MAX_MOTOR_PWM, RPLIDAR_SYNC_BYTE1, RPLIDAR_SYNC_BYTE2, RPlidarCommand, RPlidarConnectionError, RPlidarHealth, RPlidarProtocolError, RPlidarResponse
from lidarLib.renderLib.renderMachine import initMachine
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame


def getInput(toPrint:str = None, shouldStrip:bool = True, shouldLower:bool = True):
    response = input(toPrint + "\n")
    if shouldStrip: response=response.strip()
    if shouldLower: response=response.lower()
    return response

def standardQuestion(question:str, yesStr:str = None, noStr:str = None, helpStr:str = None, invalidStr:str = None, addAnswerString:bool=True)->bool:

    while True:
        response = getInput(question+((" (y/n" + ("/help)" if helpStr else")")) if addAnswerString else ""))

        if response=='y':
            if yesStr: print(yesStr) 
            print()
            return True
        elif response == 'n':
            if noStr: print(noStr)
            print()
            return False
        elif response == 'help':
            if helpStr: print(helpStr)
            else: print("Sorry help is not currently available on this question")
            print()

        else:
            
            print("Sorry ", response, " is not a valid response. Try y, n", (", or help. " if helpStr else "."), "\n", sep="")


def handleQuickstartProjects(path:str, configFilePath:str):
    projects=[]
    for item in os.listdir(path):

        if os.path.isdir(item):
            continue

        if ".json" not in item:
            continue

        try:
            with open(item, 'r') as file:
                data:dict = json.load(file)
                if data.get("type", "lol")=="projectConfig":
                    projects.append(item)
        except:
            pass

    for project in projects:
        if standardQuestion(
            "Lidar project detected on at " + project + ". Would you like to add this config file to that project?",
            helpStr= "Project config files are used to set up systems will multiple lidar objects like the FRCQuickstartLidarProject included in this library."
        ):
            with open(project, 'r+') as file:
                data:dict = json.load(file)
                file.seek(0)
                file.truncate()
                if "lidarConfigs" in data:
                    data["lidarConfigs"].append(configFilePath)
                else:
                    data["lidarConfigs"] = [configFilePath]
                
                json.dump(data, file, indent=4)    

    if len(projects)==0:
        if standardQuestion(
            "No lidar project file was found. Would you like to make one?",
            helpStr= "Project config files are used to set up systems will multiple lidar objects like the FRCQuickstartLidarProject included in this file."
        ):
            while True:
                projectFileName = input("Please give a filename for the project(Without the path)")
                if "/" in projectFileName or ".json" not in projectFileName:
                    print("Sorry", projectFileName, "is not a valid filename. Please make sure not to include a path and remember to add the .json")
                    continue

                break
            
            while True:
                teamNumber = input("Please enter an FRC team number if you are in an FRC team. If you would like to skip dont enter anything").strip().lower()
                if teamNumber=='':
                    if standardQuestion("Are you sure you would like to proceed without a team number. Without one the builtin FRCQuickstartLidarProject will not be able to function"):
                        teamNumber=False
                        break

                    else:continue

                try:
                    int(teamNumber)
                    break
                except:
                    print("Sorry", teamNumber, "could not be turned into an integer")


            with open(projectFileName, 'w') as file:
                data = {
                    "lidarConfigs" : [configFilePath],
                    "type" : "projectConfig"
                }
                if teamNumber:
                    data["teamNumber"] = teamNumber
                
                json.dump(data, file, indent=4)    

def demoRun(configPath:str):
    configs:lidarConfigs = lidarConfigs.configsFromJson(configPath)
    lidar:Lidar = Lidar(configs)

    if not configs.autoConnect:
        lidar.connect()

    configs.printConfigs()
    print(lidar.getSampleRate())
    print(lidar.getScanModeTypical())
    print(str(lidar.getScanModes()))
    for item in lidar.getScanModes():
        print(item)

    if not configs.autoStart:
        if configs.mode=="normal":
            lidar.startScan()
        else:
            lidar.startScanExpress()


    time.sleep(1)

    renderer, pipe = initMachine()
    
    while pipe.isConnected():
        pipe.send(lidar.getLastMap())
        time.sleep(0.1)

    
    lidar.disconnect()



class lidarConfigurationTool:

    def __init__(self):
        self.defaults = lidarConfigs.defaultConfigs
        self.configFile:lidarConfigs=None
        # self.configFile = lidarConfigs(port="lol")

        # #opening
        self.opening()
        

        self.verboseCheck()

        # #find lidar
        self.findLidar()


        # #baudrate
        self.findBaudRate()

        #get trans
        self.getTransOrDeadband(True)
        

        #deadband
        # self.configFile = lidarConfigs(port="lol")
        # self.configFile.x, self.configFile.y, self.configFile.r = 0, 0, 180
        self.getTransOrDeadband(False)

        #verbose checks
        if self.verbose:
            #speed
            self.getSpeed()

            #auto start + connect
            self.getAutoConnectAndStart()
            
            #debug 
            self.getDebug()

            #defaultMode TODO

        #path
        self.getPath()

        #name
        self.getName()
        
        #file write
        self.configFile.writeToJson(os.path.join(self.path, self.filename))

        #quickstart tool??????
        handleQuickstartProjects(self.path, os.path.join(self.path, self.filename))

        #test
        if standardQuestion(
            "The lidar lib would now like to run a scan(with a render) to make sure everything worked correctly. " +
            "This will involve connecting to and scanning with the lidar. Is this ok?"):
            demoRun(os.path.join(self.path, self.filename))

        print("Congratulations on setting up a lidar module!!!!")



    def opening(self):
        standardQuestion(
            "Welcome the Wired lib lidar Config tool. If at any time you are confused type \"help\" and more information will be provided. Make sense?",
            None,
            "Try typing \"help\" for help"
            "throughout the tool the character \'y\' will be used for yes and the character \'n\' will be used for no. If these responses are not working make sure your typing the right character and then restart the tool."
        )


    def verboseCheck(self):
        self.verbose = standardQuestion(
            "Would you like to run in verbose mode (only recommended for people with significant experience)?",
            helpStr="Verbose mode provides the option to edit more volatile configs that may cause crashes if used wrong and should be defaulted most of the time."
        )

    def findLidar(self):
        trash = getInput("Please plug in EXACTLY 1 Slamtec lidar to be configured. Press enter to continue")
    


        for bus in list_ports.comports():
            if bus.vid == self.defaults["vendorID"]:
                if self.configFile==None:
                    self.configFile = lidarConfigs(vendorID=(bus.vid), productID=(bus.pid), serialNumber=bus.serial_number)
                else:
                    print("Detected multiple lidar like devices. Please make sure that there is only one lidar plugged in. If the issue continues please try on a different device.")
                    
                    if standardQuestion(
                        "Would you like to enter the productID, vendorID, and serial number manually?", 
                        helpStr="It is possible for the config tool to mistake other devices for lidars. If this it the case you may choose to manually the values here."
                        ):
                        self.enterSerialValuesManual()
                        return
 
                    self.configFile=None
                    self.findLidar()
                    return
                
        if self.verbose:
            print("Lidar found with vendorID:", hex(self.configFile.vendorID), " productID:", hex(self.configFile.productID), "serialPort:", self.configFile.serialNumber)
            if not standardQuestion("Does this look right?"):
                self.enterSerialValuesManual()
                if not self.configFile:
                    self.findLidar()
                    return

        else:
            print("Lidar Found!!!")




    def enterSerialValuesManual(self):
        if not standardQuestion(
                "Would you like to continue setting the productID, vendorID, and serial number of the lidar manually?",
                helpStr=
                    "This function allows you to manually set the vendorID, productID, and serial number of the lidar."+
                    " This can be useful if the Config tool is having trouble finding the lidar."
        ):return
        

        vid = None
        while True:
            vid = getInput("please enter the vendor ID in format 0xXXXX where X (but not x) is replaced with a hexadecimal number or press enter to use default")
            print()
            
            if len(vid)==0:
                vid=self.defaults["vendorID"]

            else:    
                try:
                    vid = int(vid, 16)

                except:
                    print("Vendor ID", vid, "Could not be converted into a hexadecimal integer. Likely it was formatted incorrectly\n")
                    continue

            if vid<0 or vid>0xffff:
                print("Vendor ID", vid, "Was not in valid range 0x0000 to 0xffff")
                continue


            if standardQuestion("VendorID will be set to " + hex(vid) + ". Does this look correct?"):
                break  
        
        
        pid = None
        while True:
            pid = getInput("please enter the product ID in format 0xXXXX where X (but not x) is replaced with a hexadecimal number.")
            print()

            try:
                pid = int(pid, 16)

            except:
                print("Product ID", pid, "Could not be converted into a hexadecimal integer. Likely it was formatted incorrectly\n")
                self.enterSerialValuesManual()
                continue
            
            if pid<0 or pid>0xffff:
                print("Product ID", pid, "Was not in valid range 0x0000 to 0xffff")
                continue

            if standardQuestion("ProductID will be set to " + hex(pid) + ". Does this look correct?"):
                break


        serialNumber = None
        while True:
            serialNumber = getInput("please enter the serial number as a string (Make sure NOT to include a 0x at the beginning).")
            print()


            if standardQuestion("Product ID will be set to " + serialNumber + ". Does this look correct?"):
                break

        
        if not standardQuestion(
            "This will set the Vendor ID to " + hex(vid) +
            ", the product id to " + hex(pid) +
            ", and the serial number to " + serialNumber +
            ". Does this look correct?"):
            if standardQuestion("Are you sure? saying yes will restart the manual process from the beginning."):
                self.enterSerialValuesManual()
                return

           


        self.configFile = lidarConfigs(vendorID=vid, productID=pid, serialNumber=serialNumber)
        print("Values set!!!\n")



    def findBaudRate(self):
        baudrate = self.baudRateAutoTest()
        if baudrate=="Unknown":
            if not standardQuestion(
            
                question=
                    "The config tool could not automatically determine the baudrate of connected lidar. "
                    "Would you like to manually enter the baudrate (otherwise the system will try again)?",
                helpStr= 
                    "Baudrate tells the system how fast the lidar will send packages. " +
                    "If your lidar model has a 5pin to usb adapter it may have a switch on the side to change Baudrate with the numbers written on it.\n" +
                    "Otherwise check the specific models documentation to find the baudrate. "
            ): 
                self.findBaudRate()
                return
            
            
            baudrate=None
            while not baudrate:
                baudrate = getInput("Please enter the baudrate as a decimal integer or enter \"help\" for help.")
                if baudrate == "help":
                    print(
                        "Baudrate tells the system how fast the lidar will send packages.", 
                        "If your lidar model has a 5pin to usb adapter it may have a switch on the side to change Baudrate with the numbers written on it.\n" +
                        "Otherwise check the specific models documentation to find the baudrate."
                    )
                    baudrate=None
                else:
                    try:
                        baudrate = int(baudrate)
                    except:
                        print("Could not convert", baudrate, "to a decimal integer. Please try again")
                        baudrate=None
                print()
                if baudrate and not standardQuestion(
                    "This will set the baudrate to " + str(baudrate) + ". Does this look correct?",
                    helpStr= 
                        "Baudrate tells the system how fast the lidar will send packages. "
                        "If your lidar model has a 5pin to usb adapter it may have a switch on the side to change Baudrate with the numbers written on it.\n" +
                        "Otherwise check the specific models documentation to find the baudrate. "
                    ):baudrate = None




        self.configFile.baudrate=baudrate


                

            

    def baudRateAutoTest(self):
        

        for bus in list_ports.comports():
            if bus.serial_number == self.configFile.serialNumber and bus.vid == self.configFile.vendorID and bus.pid == self.configFile.productID:
                port = bus.device
        
        if not port:
            raise ValueError("Could not find port associated with lidar")

        serialPort = serial.Serial(port)
        serialPort.timeout = 0.5
        for baudrate in [115200, 256000]:
            serialPort.baudrate = baudrate
            serialPort.write(RPlidarCommand(RPLIDAR_CMD_GET_HEALTH, None).raw_bytes)
            count=0
            while serialPort.in_waiting<7:
                time.sleep(0.01)
                count+=0.01
                if count>1:
                    break
            if count>1: continue
            
            descriptor = RPlidarResponse(serialPort.read(RPLIDAR_DESCRIPTOR_LEN))

                    
            if descriptor.sync_byte1 != RPLIDAR_SYNC_BYTE1[0] or descriptor.sync_byte2 != RPLIDAR_SYNC_BYTE2[0]:
                continue


            
            while serialPort.in_waiting<descriptor.data_length:
                time.sleep(0.1)
                print("loop")
            data = serialPort.read(descriptor.data_length)
            
            health = RPlidarHealth(data)
            if health.status!=0:
                continue

            print(baudrate)
            return baudrate
        
        
        return 'Unknown'
    




    def getTransOrDeadband(self, isTrans:bool):
        if isTrans:
            print(
                "Next you will need to enter the offset of the lidar in relation to the robot.",
                "This concept can be confusing so we have provided a (slightly scuffed) GUI.\n"
                "All numbers should be in meters accept for the rotation which should be provided in degrees.", 
                "As a reminder FRC coordinates are jank and so X and Y may be swapped from what you are used to.\n" 
                "if you do not wish to use this feature simply do not provide any values\n"
                "Once you have entered all coordinates press enter to continue."
            )
        else:
            print(
                "Next you will need to enter the deadband of the lidar in relation to the robot.",
                "This is a range of degree values that will be thrown out by the lidar.\n"
                "This is useful to block out areas occupied by your robot.",   
                "This concept can be confusing so we have provided another (slightly scuffed) GUI.\nThe grayed out angles are the ones that will be discarded.",
                "All numbers should be in degrees." 
                "As a reminder FRC coordinates are jank and so X and Y may be swapped from what you are used to.", 
                "If you do not wish to use this feature simply do not provide any values.\n"
                "Once you have entered the correct deadband press enter to continue."
            )
        input()


        x, y, r, deadbandStart, deadbandEnd = 0,0,0,0,0

        pygame.init()
        


        displayWidth = 500
        displayHeight = 400

        gameDisplay = pygame.display.set_mode((displayWidth,displayHeight + 100))

        pygame.display.set_caption('Translation demo')
        
        xInputBox = InputBox(displayWidth/2 - 170, 380, 320, 32, "x(in meters)=")
        yInputBox = InputBox(displayWidth/2 - 170, 420, 320, 32, "y(in meters)=")
        rInputBox = InputBox(displayWidth/2 - 170, 460, 320, 32, "r(in degrees)=")
        deadbandStartBox  = InputBox(displayWidth/2 - 170, 420, 320, 32, "Start of the deadband=")
        deadbandEndBox = InputBox(displayWidth/2 - 170, 460, 320, 32, "End of the deadband=")

        transInputBoxes = [xInputBox, yInputBox, rInputBox]
        deadInputBoxes = [deadbandStartBox, deadbandEndBox]


        white = (255,255,255)

        clock = pygame.time.Clock()
        
    
        directory = os.path.dirname(os.path.realpath(__file__))
        print(directory)
        robotImg = pygame.image.load(os.path.join(directory, 'robot.png'))
        lidarImg = pygame.image.load(os.path.join(directory, 'lidar.png'))
        imageRect = lidarImg.get_rect(center=(displayWidth // 2, displayHeight // 2))




    
        shouldContinue=True
        while shouldContinue:


            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    shouldContinue = False


                if isTrans:
                    for box in transInputBoxes:
                        box.handleEvent(event)
                else:
                    for box in deadInputBoxes:
                        box.handleEvent(event)


                if event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
                    shouldContinue=False

                    if isTrans:
                        for box in transInputBoxes:
                            if not box.hasValidText:
                                shouldContinue=True
                                break
                    else:
                        for box in deadInputBoxes:
                            if not box.hasValidText:
                                shouldContinue=True
                                break
                    
            for box in transInputBoxes:
                box.update()
            if isTrans:
                x, y, r = xInputBox.getText(), yInputBox.getText(), rInputBox.getText()%360
            else:
                deadbandStart, deadbandEnd = deadbandStartBox.getText()%360, deadbandEndBox.getText()%360
                x, y, r = self.configFile.localTrans.x, self.configFile.localTrans.y, self.configFile.localTrans.r

            gameDisplay.fill(white)
            gameDisplay.blit(robotImg, ((displayWidth-332)/2, (displayHeight-332)/2))
            lidarRenderX, lidarRenderY =y*400+displayWidth/2, x*-400+displayHeight/2

            if isTrans:
                for box in transInputBoxes:
                    box.draw(gameDisplay)
            else:
                for box in deadInputBoxes:
                    box.draw(gameDisplay)
                
                if deadbandStartBox.text!='' and deadbandEndBox.text!='':
                    alphaDisplay = pygame.Surface((displayWidth, displayHeight), pygame.SRCALPHA)
                    if deadbandEnd>deadbandStart:
                        pygame.draw.polygon(alphaDisplay, [128, 128, 128, 128], (
                                (lidarRenderX, lidarRenderY),
                                (lidarRenderX+400000*cos((deadbandStart-90)*pi/180),lidarRenderY+400000*sin((deadbandStart-90)*pi/180)),
                                (lidarRenderX+400000*cos(((deadbandStart+deadbandEnd)/2-90)*pi/180),lidarRenderY+400000*sin(((deadbandStart+deadbandEnd)/2-90)*pi/180)),
                                (lidarRenderX+400000*cos((deadbandEnd-90)*pi/180),lidarRenderY+400000*sin((deadbandEnd-90)*pi/180))
                        ))
                        # print(
                        #     (lidarRenderX+4000*cos((deadbandStart-90)*pi/180),   lidarRenderY+4000*sin((deadbandStart-90)*pi/180)),
                        #     (lidarRenderX+4000*cos(((deadbandStart+deadbandEnd)/2-90)*pi/180),  lidarRenderY+4000*sin(((deadbandStart+deadbandEnd)/2-90)*pi/180)),
                        #     (lidarRenderX+4000*cos((deadbandEnd-90)*pi/180),  lidarRenderY+4000*sin((deadbandEnd-90)*pi/180))
                        # )
                        # print(deadbandStart, deadbandEnd)
                    else:
                        pygame.draw.polygon(alphaDisplay, [128, 128, 128, 128], (
                                (lidarRenderX, lidarRenderY),
                                (lidarRenderX+400000*cos((deadbandEnd-90)*pi/180),lidarRenderY+400000*sin((deadbandEnd-90)*pi/180)),
                                (lidarRenderX+400000*cos(((deadbandStart+deadbandEnd)/2+90)*pi/180),lidarRenderY+400000*sin(((deadbandStart+deadbandEnd)/2+90)*pi/180)),
                                (lidarRenderX+400000*cos((deadbandStart-90)*pi/180),lidarRenderY+400000*sin((deadbandStart-90)*pi/180))
                        ))
                    gameDisplay.blit(alphaDisplay, (0,0))


            rotateLidar = pygame.transform.rotate(lidarImg, -r)
            gameDisplay.blit(rotateLidar, (lidarRenderX-16, lidarRenderY-16))

            pygame.display.flip()
            pygame.time.Clock().tick(60)

        pygame.quit()

        if isTrans:
            if not standardQuestion("This will set the lidar offset to x(meters) : " + str(x) + " y(meters) : " + str(y) + " rotation(degrees) :" + str(r) + "?"):
                self.getTransOrDeadband(True)
                return
            else:
                self.configFile.localTrans = translation.fromCart(x, y, r)
           

        
        else:
            if not standardQuestion("This will set the lidar deadband to the range from " + str(deadbandStart) + " to " + str(deadbandEnd) + "?"):
                self.getTransOrDeadband(False)
                return
            else:
                self.configFile.deadband = (deadbandStart, deadbandEnd)



    def getSpeed(self):
        speed = getInput("Please enter the speed you would like the lidar to run at as an integer or leave default for recommended speed")
        if speed!='':
            try: speed = int(speed)
            except:
                print("sorry", speed, "could not be turned into a int")
                self.getSpeed()
                return
            
            if speed<0 or speed>RPLIDAR_MAX_MOTOR_PWM:
                print("Sorry", speed, "is outside of the valid range 0 to", RPLIDAR_MAX_MOTOR_PWM)
                self.getSpeed()
                return
            
            self.configFile.defaultSpeed=speed
            
        else:
            return
        
    def getAutoConnectAndStart(self):
        self.configFile.autoConnect=standardQuestion(
            "Would you like the lidar object to automatically connect when it is created(recommended yes)?",
            helpStr=
                "This setting will allow the lidar to automatically connect when the lidar object is initialized. " +
                "Otherwise the connection will have to be made manually using the connect method. " +
                "Generally it is a good idea for the lidar to connect automatically so it is suggested to turn this setting on."
        )

        self.configFile.autoStart=standardQuestion(
            "Would you like the lidar object to automatically start when it is connected(recommended no)?",
            helpStr=
                "This setting will allow the lidar to automatically start when the lidar object is connected(including when it is connected via auto connect). "+
                "Otherwise the lidar will need to be started via the startScan or startExpress methods. " +
                "This setting is defaulted to true."
        )
    
    def getDebug(self):
        self.configFile.debugMode=standardQuestion(
            "Would you like to create the lidar in debug mode (recommended no)?",
            helpStr=
                "Debug mode prints additional information about the lidar. " +
                "This is recommended for projects that dive deep into the lidars internals or for handling issues with the driver itself. " +
                "However the volume of text can slow down the library slightly and be confusing so we recommend turning it off when it isn't actively being used (including when on robots)."
        )

    def getPath(self):
        path=None
        while True:
            path = input("Please enter the path to the directory where you would like to store to config file(do not include the config file's name itself)")
            if not os.path.exists(path):
                print("Sorry", path, "does not exist.")
                continue
            if os.path.isfile(path):
                print("Sorry that path :", path, "appears to be a file and not a directory")
                continue

            if standardQuestion(
                "Are you sure you want to make the config in the directory " + path + "?",
                helpStr= 
                    "This path is the directory/folder in which the config file will be placed. " +
                    "You do not need to include the file name of the config file, we will get there in a bit. " +
                    "While you can include the path in relation to the location this script was run from we recommend giving the entire path from root if you are having issues."
            ):
                self.path=path
                return

    def getName(self):

        while True:
            name = input(
                "Please enter a name for the lidar. "+
                "This should be a unique from all other lidar names and  we recommend making it descriptive of the lidar (aka \"front left lidar\" not \"bob\")"
            ).strip()
            print()
            if name=='':
                print("Please provide a name")
                continue
            if ".json" in name:
                if not standardQuestion(
                    ".json was included in the lidar name. "
                    "The lidars name is different from the name of its config file(although they should be similar) so this .json is likely a mistake.\n"
                    "Are you sure you would like to have .json in the name?"):
                    continue
                    

            if standardQuestion("This will set the lidars name to " + name + ". Does this look good?"):
                break

        self.configFile.name=name

        if not standardQuestion("Would you like to name the config file " + name + ".json?"):
            while True:
                fileName = input("Please give a filename(Without the path)")
                if "/" in fileName or ".json" not in fileName:
                    print("Sorry", fileName, "is not a valid filename. Please make sure not to include a path and remember to add the .json")
                    continue

                self.filename=fileName
                break
        else:
            self.filename=name+".json"

        print()



class InputBox:


    def __init__(self, x, y, w, h, text=''):
        pygame.font.init() #

        self.font = pygame.font.SysFont(None, 22)
        self.colorActive = pygame.Color('dodgerblue2')
        self.colorInactive = pygame.Color('lightskyblue3')
        self.invalid = pygame.Color(200, 0, 0)

        self.hasValidText=False

        self.rect = pygame.Rect(x, y, w, h)
        self.color = self.colorInactive
        self.displayText = text
        self.text=''
        self.txt_surface = self.font.render(text, True, self.color)
        self.active = False



    def handleEvent(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            # If the user clicked on the input_box rect.
            if self.rect.collidepoint(event.pos):
                # Toggle the active variable.
                self.active = not self.active
            else:
                self.active = False
            # Change the current color of the input box.
            self.color = self.colorActive if self.active else self.colorInactive
        if event.type == pygame.KEYDOWN:
            if self.active:
                if event.key == pygame.K_BACKSPACE:
                    self.text = self.text[:-1]
                else:
                    if event.unicode.isdigit() or event.unicode=='.' or (event.unicode=='-' and len(self.text)==0):
                        self.text += event.unicode
                # Re-render the text.
                self.txt_surface = self.font.render(self.displayText + self.text, True, self.color)

    def update(self):
        # Resize the box if the text is too long.
        width = max(200, self.txt_surface.get_width()+10)
        self.rect.w = width

    def draw(self, screen):
        # Blit the text.
        screen.blit(self.txt_surface, (self.rect.x+5, self.rect.y+5))
        # Blit the rect.
        pygame.draw.rect(screen, self.color if self.hasValidText else self.invalid, self.rect, 2)

    def getText(self):
        try:
            self.hasValidText=True
            return float(self.text)
        
        except:
            
            self.hasValidText=self.text==''
            return 0
        



if __name__ == '__main__':
    
    lidarConfigurationTool()

