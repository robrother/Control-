#!/usr/bin/env python
import roslib; roslib.load_manifest('bebop_driver')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty  
from std_msgs.msg import Int8
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged

COMMAND_PERIOD = 20 #ms
class DroneController(object):
    def __init__(self):
        # Holds the current drone status
        self.status = -1
        # Allow the controller to publish to the /bebop/takeoff, land and reset topics
        self.pubLand    = rospy.Publisher('/bebop/land',Empty,queue_size=1000)
        self.pubTakeoff = rospy.Publisher('/bebop/takeoff',Empty,queue_size=1000)
        self.pubCommandPilot = rospy.Publisher('/bebop/cmd_vel',Twist,queue_size=1000)
        self.pubCommandCamera = rospy.Publisher('/bebop/camera_control',Twist,queue_size=1000)
        
        # Setup regular publishing of control packets
        self.command = Twist()
        self.command2 = Twist()

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand)

    def SendTakeoff(self):
        # Send a takeoff message to the bebop driver Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
        #if(self.status == DroneStatus.Landed):
        self.pubTakeoff.publish(Empty())

    def SendLand(self):
        # Send a landing message to the bebop driver Note we send this in all states, landing can do no harm
        self.pubLand.publish(Empty())

    def SendEmergency(self):
        # Send an emergency (or reset) message to the bebop driver
        self.pubReset.publish(Empty())

    def SetCommandPilot(self,roll,pitch,yaw_velocity,z_velocity):
        # Called by the main program to set the current command
        self.command.linear.x  = pitch
        self.command.linear.y  = roll
        self.command.linear.z  = z_velocity
        self.command.angular.z = yaw_velocity
        self.pubCommandPilot.publish(self.command)
        
    def SendCommand(self,event):
        # The previously set command is then sent out periodically if the drone is flying
        #if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
        self.pubCommandPilot.publish(self.command)
        self.pubCommandCamera.publish(self.command)

    def SetCommandCamera(self, tilt=0, cam_pan=0):
        self.command2.angular.y  = tilt
        self.command2.angular.z  = cam_pan
        self.pubCommandCamera.publish(self.command2)

    def callback(self, cmd):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo(cmd)
        print self.MsgBat
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(cmd.linear.x, cmd.linear.y, cmd.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(cmd.angular.x, cmd.angular.y, cmd.angular.z))
        drone.SetCommandPilot(cmd.linear.y, cmd.linear.x, cmd.angular.z, cmd.linear.z )
        drone.SetCommandCamera(cmd.angular.x, cmd.angular.y)
        #drone.SendCommand()
        
        #comm.SendCommand(env)
        #pubLand    = rospy.Publisher('/bebop/land',Empty,queue_size=1000)
        #pubTakeoff = rospy.Publisher('/bebop/takeoff',Empty,queue_size=1000)
        #pubCommandPilot = rospy.Publisher('/bebop/cmd_vel',Twist,queue_size=1000)
        #command = Twist()
        #self.command.linear.x  = msg.linear.x
        #self.command.linear.y  = msg.linear.y
        #self.command.linear.z  = msg.linear.z
        #self.command.angular.z = msg.angular.z
        #pubCommand.publish(command)

    def takeoff(self, toff):
        #rospy.Subscriber("/keyboard/takeoff", Int8, takeoff)
        rospy.loginfo(toff.data)
        #if toff.data== True:
        if toff.data == 1:
            drone.SendTakeoff()
            rospy.loginfo("TAKE OFF")

    def land(self, ld):
        #rospy.Subscriber("/keyboard/land", Int8, land)
        rospy.loginfo(ld.data)
        #if ld.data== True:
        if ld.data == 1:
            drone.SendLand()
            rospy.loginfo("LAND")
            #print "LAND"

    def battery_status(self, data):	
		#self.statusBar().showMessage(data.percent)
		#rospy.loginfo("NIVEL DE BATERIA %s", str(data.percent))
		self.MsgBat = "Bateria: "+ str(data.percent) + "%"
		print self.MsgBat

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/keyboard/cmd_vel", Twist, drone.callback)
    rospy.Subscriber("/keyboard/takeoff", Int8, drone.takeoff)
    rospy.Subscriber("/keyboard/land", Int8, drone.land)
    rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged, drone.battery_status)    
    
    #pubReset   = rospy.Publisher('/bebop/reset',Empty) 
    # Allow the controller to publish to the /cmd_vel topic and thus control the drone Setup regular publishing of control packets
    rospy.spin()

if __name__ == '__main__':
    drone = DroneController()
    listener()