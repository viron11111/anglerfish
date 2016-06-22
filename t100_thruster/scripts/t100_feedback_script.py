#!/usr/bin/python

#Anglerfish has 6 T100 Thrusters with Blue ESCs.  Each thruster had its firmware manually updated 
#(using Arduino and AVRdude in Ubuntu) to acquire a unique I2C address.
#See BlueRobotics T100 instructions on how to program ESCs.
#I2C address' are: 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E
#since launch file doesn't support hex: 41, 42, 43, 44, 45, 46

import sys
import smbus
import time
import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Header
from std_msgs.msg import String
from t100_thruster.msg import t100_thruster_feedback

bus = smbus.SMBus(1)


class read_registers():
	#Temperature calculator
	def temperature_calculate(self):

		#read temp registers
		temp1_read = bus.read_byte_data(self.T100_ADDR, self.T100_TEMPERATURE_1)
		temp2_read = bus.read_byte_data(self.T100_ADDR, self.T100_TEMPERATURE_2)

                temp_reg = temp1_read << 8 | temp2_read

		# THERMISTOR SPECIFICATIONS
		# resistance at 25 degrees C
		THERMISTORNOMINAL = 10000      
		# temp. for nominal resistance (almost always 25 C)
		TEMPERATURENOMINAL =  25   
		# The beta coefficient of the thermistor (usually 3000-4000)
		BCOEFFICIENT = 3900
		# the value of the 'other' resistor
		SERIESRESISTOR = 3300 
		
		if temp_reg == 0:
			rospy.logwarn("temp_reg zero error: %d %s" % (temp_reg, self.T100_NAME))
			temp_reg = 30000
			#rospy.logwarn(self.T100_NAME)
		resistance = SERIESRESISTOR/(65535/float(temp_reg)-1)
		#rospy.logwarn(resistance)
		steinhart = resistance / THERMISTORNOMINAL  # (R/Ro)
		steinhart = math.log(steinhart)                 # ln(R/Ro)
		steinhart /= BCOEFFICIENT                  # 1/B * ln(R/Ro)
		steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15) # + (1/To)
		steinhart = 1.0 / steinhart                 # Invert
		steinhart -= 273.15  
		self.thruster_temp = steinhart 

	#voltage calculation
	def voltage(self):
		#read voltage reg
		volt_1 = bus.read_byte_data(self.T100_ADDR, self.T100_VOLTAGE_1)
		volt_2 = bus.read_byte_data(self.T100_ADDR, self.T100_VOLTAGE_2)

	        voltage_raw = volt_1 << 8 | volt_2		
		self.actual_voltage = (voltage_raw*.0004921)

	#current calculation taken from BlueRobotics
	def current(self):
		#read current registers, bit shift
		curr_1 = bus.read_byte_data(self.T100_ADDR, self.T100_CURRENT_1)
		curr_2 = bus.read_byte_data(self.T100_ADDR, self.T100_CURRENT_2)

	        current_raw = curr_1 << 8 | curr_2		
		self.actual_current = ((current_raw-32767)*.001122)

	#take pulse count over time, calculate RPM
	def RPM(self):

		#read pulse reg to calculate RPM
		pulse1_read = bus.read_byte_data(self.T100_ADDR, self.T100_PULSE_COUNT_1)
		pulse2_read = bus.read_byte_data(self.T100_ADDR, self.T100_PULSE_COUNT_2)
		
		#bit shift 2 bytes
	        pulse_count_reg = pulse1_read << 8 | pulse2_read				

		self.actual_rpm = (float(pulse_count_reg)/((time.clock()-self.rpmTimer)*120))*60

		self.rpmTimer = time.clock()


	def thrust(self, force):
		if force.data > 2.36:
			#rospy.logwarn("Max forward thrust = 2.36 kgf.  Input, %0.2f kgf changed to 2.36 kgf", force.data)
			force.data = 2.36
			
		elif force.data < -1.82:
			#rospy.logwarn("Max reverse thrust = -1.82 kgf.  Input, %0.2f kgf changed to -1.82 kgf", force.data)
			force.data = -1.82
		
		if force.data < 0.0000:
			#rospy.logwarn("desired force: %d" % force.data)
			output = (force.data/-1.82)*-32767
			#rospy.logwarn("output: %d" % output)
		elif force.data > 0.0000:
			output = (force.data/2.36)*32767
		else:
			output = 0.0

		output = int(output)

		self.signal = output

                #bus.write_byte_data(self.T100_ADDR, self.T100_THROTTLE_1, 0)
                #bus.write_byte_data(self.T100_ADDR, self.T100_THROTTLE_2, 0)

	        bus.write_byte_data(self.T100_ADDR, self.T100_THROTTLE_1, output>>8)
	        bus.write_byte_data(self.T100_ADDR, self.T100_THROTTLE_2, output)

		if (force.data > 0.000 and self.actual_rpm == 0.0) or (force.data < 0.000 and self.actual_rpm == 0.0):
			self.spin_warn_trigger = time.time()*1000
			self.trigger = 1

                if self.actual_rpm > 0.0:
                        self.spin_warn_time = time.time()*1000
			self.trigger = 0

		if (self.spin_warn_time - self.spin_warn_trigger) > 500 and force.data != 0.000 and self.trigger==1:
			rospy.logerr("NO SPIN: %s" % self.T100_NAME)
			

	def stop_motor():
	        bus.write_byte_data(rospy.get_param('~register'), 0x00, 0)
	        bus.write_byte_data(rospy.get_param('~register'), 0x01, 0)	

	def __init__(self):
		#ROS params for definging node for each thruster
		self.T100_ADDR = rospy.get_param('~register')
		self.T100_NAME = rospy.get_param('~name')
		self.T100_OUTPUT = rospy.get_param('~force')

		self.spin_warn_time = time.time()*1000	
		self.spin_warn_trigger = time.time()*1000	
		self.trigger = 0

		#T100 registers used for RPM, voltage, temp, current (2 bytes each)
		self.T100_THROTTLE_1	  = 0x00
		self.T100_THROTTLE_2      = 0x01
		self.T100_PULSE_COUNT_1   = 0x02
		self.T100_PULSE_COUNT_2   = 0x03
		self.T100_VOLTAGE_1       = 0x04
		self.T100_VOLTAGE_2       = 0x05
		self.T100_TEMPERATURE_1   = 0x06
		self.T100_TEMPERATURE_2   = 0x07
		self.T100_CURRENT_1       = 0x08
		self.T100_CURRENT_2       = 0x09
		#T100 register for detecting if T100 is alive (bool)
		self.T100_ALIVE           = 0x0A

		bus.write_byte_data(self.T100_ADDR, self.T100_THROTTLE_1, 0)
		bus.write_byte_data(self.T100_ADDR, self.T100_THROTTLE_2, 0)
		time.sleep(.05)

		self.signal = 0

		self.ROV_pub = rospy.Publisher(self.T100_NAME, t100_thruster_feedback, queue_size=1)
		rospy.Subscriber(self.T100_OUTPUT, Float32, self.thrust)

		t100 = t100_thruster_feedback()

		rate = rospy.Rate(15)	

		self.rpmTimer = 0.0

		while not rospy.is_shutdown():
			
			#Check to see if thruster is alive
			if bus.read_byte_data(self.T100_ADDR, self.T100_ALIVE):

				#return RPM
				self.RPM()
				
				#return actual temp
				self.temperature_calculate()

				#return actual voltage
				self.voltage()

				#return actual current
				self.current()

				t100.t100_header = Header(
					stamp = rospy.get_rostime(),
					frame_id = str(self.T100_ADDR)
				)
				
				t100.signal = self.signal
				t100.temperature = self.thruster_temp
				t100.voltage = self.actual_voltage
				t100.current = self.actual_current 
				t100.rpm = self.actual_rpm
				t100.alive = "CONNECTED"

			else:
				t100.alive = "DISCONNECTED"			

			self.ROV_pub.publish(t100)
			rate.sleep()
	rospy.on_shutdown(stop_motor)

def main(args):
	rospy.init_node('t100_feedback_sensors', anonymous=False)

	read_registers()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main(sys.argv)
