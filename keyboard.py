#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import Empty  

# Finally the GUI libraries
from PySide import QtCore, QtGui

GUI_UPDATE_PERIOD = 100 #ms

# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_W
	PitchBackward    = QtCore.Qt.Key.Key_S
	RollLeft         = QtCore.Qt.Key.Key_A
	RollRight        = QtCore.Qt.Key.Key_D
	YawLeft          = QtCore.Qt.Key.Key_Q
	YawRight         = QtCore.Qt.Key.Key_E
	IncreaseAltitude = QtCore.Qt.Key.Key_Up
	DecreaseAltitude = QtCore.Qt.Key.Key_Down
	Takeoff          = QtCore.Qt.Key.Key_T
	Land             = QtCore.Qt.Key.Key_Space
	Hovering         = QtCore.Qt.Key.Key_H
	Release          = QtCore.Qt.Key.Key_R

	CamUp			= QtCore.Qt.Key.Key_I
	CamDown			= QtCore.Qt.Key.Key_K
	CamRigth		= QtCore.Qt.Key.Key_J
	CamLeft			= QtCore.Qt.Key.Key_L

class MainWindow(QtGui.QMainWindow):
	def __init__(self):
		super(MainWindow,self).__init__()
		self.setWindowTitle('Bebop interfaz')
		#self.setWindowIcon(QtGui.QIcon('3dr.jpg')) 

		# suscripcion para recibir la info de la bateria
		#self.subNavdata = rospy.Subscriber('bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged, self.callback) 
		#self.MsgBat = "Iniciando..."
		#self.statusBar().showMessage(self.MsgBat)
		
		#self.FlyingStateChanged = rospy.Subscriber('/bebop/states/ARDrone3/PilotingState/FlyingStateChanged', Ardrone3PilotingStateFlyingStateChanged, self.flying_status)
		#timer for update  GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.update)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)  

		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.tilt = 0
		self.cam_pan = 0
		self.z_velocity = 0 
		self.arm = False
		self.toff = False
		self.ld = False
		self.override = False

		self.pubCommandP = rospy.Publisher('/keyboard/cmd_vel',Twist,queue_size=10)
		self.pubLand    = rospy.Publisher('/keyboard/land',Int8,queue_size = 10)
		self.pubTakeoff = rospy.Publisher('/keyboard/takeoff',Int8,queue_size = 10)
		self.pubOverride = rospy.Publisher('/keyboard/override',Int8,queue_size = 10)
		self.keyb = Twist()

	# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
	def keyPressEvent(self, event):
		self.override = 1
		self.pubOverride.publish(self.override)
		key = event.key()
		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		#if not event.isAutoRepeat():
		if not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.Hovering:
				self.keyb.linear.x  = 0.0
				self.keyb.linear.y  = 0.0
				self.keyb.linear.z  = 0.0
				self.keyb.angular.z = 0.0
				self.pubCommandP.publish(self.keyb)
				rospy.loginfo("Hovering")

			elif key == KeyMapping.Takeoff:
				self.toff = not self.toff
				if self.toff == True:
					#self.pubTakeoff.publish(Empty())
					self.pubTakeoff.publish(1)
					self.toff = False
				rospy.loginfo("TAKING OFF")
				
			elif key == KeyMapping.Land:
				self.ld = not self.ld
				if self.ld == True:
					self.pubLand.publish(1)
					self.ld = False
				rospy.loginfo("LANDING")
	
			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:    
					self.yaw_velocity += 0.3
					rospy.loginfo("YAW_LEFT")

				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -0.3
					rospy.loginfo("YAW_RIGHT")

				elif key == KeyMapping.PitchForward:
					self.pitch += 0.3
					rospy.loginfo("FORWARD")

				elif key == KeyMapping.PitchBackward:
					self.pitch += -0.3
					rospy.loginfo("BACKWARD")

				elif key == KeyMapping.RollLeft:
					self.roll += 0.3
					rospy.loginfo("LEFT")

				elif key == KeyMapping.RollRight:
					self.roll += -0.3
					rospy.loginfo("RIGTH")

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 0.3
					rospy.loginfo("UP")

				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -0.3
					rospy.loginfo("DOWN")

				elif key == KeyMapping.CamUp:
					self.tilt += 5
				elif key == KeyMapping.CamDown:
					self.tilt += -5
				elif key == KeyMapping.CamLeft: 
					self.cam_pan += 5
				elif key == KeyMapping.CamRigth: 
					self.cam_pan += -5
					

			self.keyb.linear.x  = self.pitch
			self.keyb.linear.y  = self.roll
			self.keyb.linear.z  = self.z_velocity
			self.keyb.angular.x = self.tilt
			self.keyb.angular.y = self.cam_pan
			self.keyb.angular.z = self.yaw_velocity
			self.pubCommandP.publish(self.keyb)
			rospy.loginfo(self.roll)
			rospy.loginfo(self.pitch)
			rospy.loginfo(self.z_velocity)
			rospy.loginfo(self.yaw_velocity)

		else:
			self.pubCommandP.publish(self.keyb)

			
	def keyReleaseEvent(self,event):
		key = event.key()
		
		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		#if not event.isAutoRepeat():
		if not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 0.3

			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -0.3
			    
			elif key == KeyMapping.PitchForward:
				self.pitch -= 0.3

			elif key == KeyMapping.PitchBackward:
				self.pitch -= -0.3

			elif key == KeyMapping.RollLeft:
				self.roll -= 0.3

			elif key == KeyMapping.RollRight:
				self.roll -= -0.3

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 0.3

			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -0.3

			elif key == KeyMapping.Release:
				self.override = 0
				self.pubOverride.publish(self.override)

			self.keyb.linear.x  = self.pitch
			self.keyb.linear.y  = self.roll
			self.keyb.linear.z  = self.z_velocity
			self.keyb.angular.x = self.tilt
			self.keyb.angular.y = self.cam_pan
			self.keyb.angular.z = self.yaw_velocity
			self.pubCommandP.publish(self.keyb) 

if __name__=='__main__':
	import sys
	rospy.init_node('keyboard')
	app = QtGui.QApplication(sys.argv)
	# Firstly we setup a ros node, so that we can communicate with the other packages
	#rospy.init_node('bebopKeyController')
	# Now we construct our Qt Application and associated controllers and windows
	#t1 = SoloCamera()
	#t1.start()
	#controller = BasicBebopController()
	#rate = rospy.Rate(10)
	display = MainWindow()
	display.show()
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
