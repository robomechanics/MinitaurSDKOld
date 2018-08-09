'''
MIT License (modified)

Copyright (c) 2018 Ghost Robotics
Authors:
Avik De <avik@ghostrobotics.io>
Tom Jacobs <tom.jacobs@ghostrobotics.io>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARIfastsinG FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

''' How to use:

See main.py

'''

import numpy as np

''' ROS interface for ethernet example '''
bROS = False
try:
	import rospy
	import numpy as np
	import tf
	import std_msgs
	from sensor_msgs.msg import Imu, BatteryState, JointState, Joy
	from std_msgs.msg import UInt32
	from nav_msgs.msg import Odometry
	from geometry_msgs.msg import Twist
	bROS = True
except:
	print('No ROS found; continuing.')

def toContinue():
	if bROS:
		return not rospy.is_shutdown()
	else:
		return True

# ROS variables
robotname = "robot0"
publish_time = 0
pubs = []
subs = []

# Commands received from ROS
linear_x = 0
angular_z = 0
behaviorId = 1
behaviorMode = 1

def initROS():
	global pubs, subs, robotname

	# Get robot name param from ROS
	if bROS:
		try:
			if rospy.has_param('robotname'):
				robotname = rospy.get_param('robotname')
		except:
			print("ROS master not running.")

		# Create publishers and subscribers
		pubs = [
			rospy.Publisher(robotname + '/state/imu', Imu, queue_size=10),
			rospy.Publisher(robotname + '/state/batteryState', BatteryState, queue_size=10),
			rospy.Publisher(robotname + '/state/behaviorId', UInt32, queue_size=10),
			rospy.Publisher(robotname + '/state/behaviorMode', UInt32, queue_size=10),
			rospy.Publisher(robotname + '/state/joint', JointState, queue_size=10),
			rospy.Publisher(robotname + '/state/joystick', Joy, queue_size=10),
		]
		subs = [
			rospy.Subscriber(robotname + '/command/cmd_vel', Twist, cmd_vel_callback),
			rospy.Subscriber(robotname + '/command/behaviorId', UInt32, behaviorId_callback),
			rospy.Subscriber(robotname + '/command/behaviorMode', UInt32, behaviorMode_callback),
		]

		# Init ROS node
		rospy.init_node('ethernet_robot_control')

def getCommands():
	global linear_x, angular_z, behaviorId, behaviorMode
	return linear_x, angular_z, behaviorId, behaviorMode

def cmd_vel_callback(data):
	global linear_x, angular_z
	rospy.loginfo("Received:\n %s", data)
	linear_x = data.linear.x
	angular_z = data.angular.z

def behaviorId_callback(data):
	global behaviorId
	rospy.loginfo("Received:\n %s", data)
	behaviorId = data.data

def behaviorMode_callback(data):
	global behaviorMode
	rospy.loginfo("Received:\n %s", data)
	behaviorMode = data.data

def publishState(state, ros_pub_dec, numDoF):
	# Publish our robot state to ROS topics /robotname/state/* periodically
	global publish_time
	publish_time += 1
	if bROS and publish_time > ros_pub_dec:
		publish_time = 0

		# Construct /robotname/state/imu ROS message
		msg = Imu()
		msg.linear_acceleration.x = state['imu/linear_acceleration'][0]
		msg.linear_acceleration.y = state['imu/linear_acceleration'][1]
		msg.linear_acceleration.z = state['imu/linear_acceleration'][2]
		msg.angular_velocity.x = state['imu/angular_velocity'][0]
		msg.angular_velocity.y = state['imu/angular_velocity'][1]
		msg.angular_velocity.z = state['imu/angular_velocity'][2]
		msg.orientation_covariance = state['imu/orientation_covariance']
		msg.linear_acceleration_covariance = np.empty(9)
		msg.linear_acceleration_covariance.fill(state['imu/linear_acceleration_covariance'])
		msg.angular_velocity_covariance = np.empty(9)
		msg.angular_velocity_covariance.fill(state['imu/angular_velocity_covariance'])
		roll, pitch, yaw = state['imu/euler'] # Convert from euler to quaternion ufastsing ROS tf
		quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		msg.orientation.x = quaternion[0]
		msg.orientation.y = quaternion[1]
		msg.orientation.z = quaternion[2]
		msg.orientation.w = quaternion[3]
		pubs[0].publish(msg)

		# Construct /robotname/state/batteryState
		msg = BatteryState()
		msg.current = state['battery/current']
		msg.voltage = state['battery/voltage']
		#num_cells = 8
		num_cells = 4
		if state['battery/cell_count'] > 0:
			num_cells = state['battery/cell_count']
		def percentage(total_voltage, num_cells):
			# Linearly interpolate charge from voltage 
			# https://gitlab.com/ghostrobotics/SDK/uploads/6878144fa0e408c91e481c2278215579/image.png
			charges =  [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
			voltages = [3.2, 3.5, 3.6, 3.65, 3.8, 4.2]
			return np.interp(total_voltage / num_cells, voltages, charges)
		if msg.percentage < 0:
			msg.percentage = percentage(msg.voltage, num_cells)
		pubs[1].publish(msg)

		# Construct /robotname/state/behaviorId
		msg = UInt32()
		msg.data = state['behaviorId']
		pubs[2].publish(msg)

		# Construct /robotname/state/behaviorMode
		msg = UInt32()
		msg.data = state['behaviorMode']
		pubs[3].publish(msg)

		# Construct /robotname/state/joint
		msg = JointState()
		msg.name = []
		msg.position = []
		msg.velocity = []
		msg.effort = []
		for j in range(numDoF):
			msg.name.append(str(j))
			msg.position.append(state['joint/position'][j])
			msg.velocity.append(state['joint/velocity'][j])
			msg.effort.append(state['joint/effort'][j])
		pubs[4].publish(msg)

		# Construct /robotname/state/joystick
		msg = Joy()
		msg.axes = state['joy/axes']
		msg.buttons = state['joy/buttons']
		pubs[5].publish(msg)

		# Current robot twist received back from robot
		#state['twist/linear']
		#state['twist/angular']
