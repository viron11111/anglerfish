#!/usr/bin/python

import sys
import smbus
import time
import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Header
from bmp180.msg import bmp180_msg

# BMP085 default address.
BMP180_I2CADDR           = 0x77

# Operating Modes
BMP085_ULTRALOWPOWER     = 0
BMP085_STANDARD          = 1
BMP085_HIGHRES           = 2
BMP085_ULTRAHIGHRES      = 3

# BMP085 Registers
BMP085_CAL_AC1           = 0xAA  # R   Calibration data (16 bits)
BMP085_CAL_AC2           = 0xAC  # R   Calibration data (16 bits)
BMP085_CAL_AC3           = 0xAE  # R   Calibration data (16 bits)
BMP085_CAL_AC4           = 0xB0  # R   Calibration data (16 bits)
BMP085_CAL_AC5           = 0xB2  # R   Calibration data (16 bits)
BMP085_CAL_AC6           = 0xB4  # R   Calibration data (16 bits)
BMP085_CAL_B1            = 0xB6  # R   Calibration data (16 bits)
BMP085_CAL_B2            = 0xB8  # R   Calibration data (16 bits)
BMP085_CAL_MB            = 0xBA  # R   Calibration data (16 bits)
BMP085_CAL_MC            = 0xBC  # R   Calibration data (16 bits)
BMP085_CAL_MD            = 0xBE  # R   Calibration data (16 bits)
BMP085_CONTROL           = 0xF4
BMP085_TEMPDATA          = 0xF6
BMP085_PRESSUREDATA      = 0xF6

# Commands
BMP085_READTEMPCMD       = 0x2E
BMP085_READPRESSURECMD   = 0x34

class measure_internal_pressure(object):

    # Global variables
    address = 0x77
    mode = 1 # TODO: Add a way to change the mode

    # BMP180 registers
    CONTROL_REG = 0xF4
    DATA_REG = 0xF6

    # Calibration data registers
    CAL_AC1_REG = 0xAA
    CAL_AC2_REG = 0xAC
    CAL_AC3_REG = 0xAE
    CAL_AC4_REG = 0xB0
    CAL_AC5_REG = 0xB2
    CAL_AC6_REG = 0xB4
    CAL_B1_REG = 0xB6
    CAL_B2_REG = 0xB8
    CAL_MB_REG = 0xBA
    CAL_MC_REG = 0xBC
    CAL_MD_REG = 0xBE

    # Calibration data variables
    calAC1 = 0
    calAC2 = 0
    calAC3 = 0
    calAC4 = 0
    calAC5 = 0
    calAC6 = 0
    calB1 = 0
    calB2 = 0
    calMB = 0
    calMC = 0
    calMD = 0

    def __init__(self):
        self.bmp_pub = rospy.Publisher("BMP180_pressure_sensor", bmp180_msg, queue_size=1)
        self.bus = smbus.SMBus(1)
        self.read_calibration_data()

        rate = rospy.Rate(10) #Hz

        bmp = bmp180_msg()

        while not rospy.is_shutdown():
            
            #self.get_reading()    
            
            bmp.header = Header(
                stamp = rospy.get_rostime(),
                frame_id = 'BMP180'
            )
            
            self.bmp_pub.publish(bmp)
            rate.sleep()

    def read_signed_16_bit(self, register):
        """Reads a 16-bit signed value and returns it.
        register -- the register to read from.
        Returns the read value.
        """
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        if high > 127:
            high -= 256
        
        return (high << 8) + low

    # Reads a 16-bit unsigned value from the given register and returns it
    def read_unsigned_16_bit(self, register):
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        return (high << 8) + low

    def read_calibration_data(self):
        """Reads and stores the raw calibration data."""
        self.calAC1 = self.read_signed_16_bit(self.CAL_AC1_REG)
        self.calAC2 = self.read_signed_16_bit(self.CAL_AC2_REG)
        self.calAC3 = self.read_signed_16_bit(self.CAL_AC3_REG)
        self.calAC4 = self.read_unsigned_16_bit(self.CAL_AC4_REG)
        self.calAC5 = self.read_unsigned_16_bit(self.CAL_AC5_REG)
        self.calAC6 = self.read_unsigned_16_bit(self.CAL_AC6_REG)
        self.calB1 = self.read_signed_16_bit(self.CAL_B1_REG)
        self.calB2 = self.read_signed_16_bit(self.CAL_B2_REG)
        self.calMB = self.read_signed_16_bit(self.CAL_MB_REG)
        self.calMC = self.read_signed_16_bit(self.CAL_MC_REG)
        self.calMD = self.read_signed_16_bit(self.CAL_MD_REG)

    def read_raw_temp(self):
        """Reads the raw (uncompensated) temperature from the sensor."""
        self._device.write8(BMP085_CONTROL, BMP085_READTEMPCMD)
        time.sleep(0.005)  # Wait 5ms
        raw = self._device.readU16BE(BMP085_TEMPDATA)
        self._logger.debug('Raw temp 0x{0:X} ({1})'.format(raw & 0xFFFF, raw))
        return raw

    def read_raw_pressure(self):
        """Reads the raw (uncompensated) pressure level from the sensor."""
        self._device.write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (self._mode << 6))
        if self._mode == BMP085_ULTRALOWPOWER:
            time.sleep(0.005)
        elif self._mode == BMP085_HIGHRES:
            time.sleep(0.014)
        elif self._mode == BMP085_ULTRAHIGHRES:
            time.sleep(0.026)
        else:
            time.sleep(0.008)
        msb = self._device.readU8(BMP085_PRESSUREDATA)
        lsb = self._device.readU8(BMP085_PRESSUREDATA+1)
        xlsb = self._device.readU8(BMP085_PRESSUREDATA+2)
        raw = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - self._mode)
        self._logger.debug('Raw pressure 0x{0:04X} ({1})'.format(raw & 0xFFFF, raw))
        return raw

    def read_temperature(self):
        """Gets the compensated temperature in degrees celsius."""
        UT = self.read_raw_temp()
        # Datasheet value for debugging:
        #UT = 27898
        # Calculations below are taken straight from section 3.5 of the datasheet.
        X1 = ((UT - self.cal_AC6) * self.cal_AC5) >> 15
        X2 = (self.cal_MC << 11) // (X1 + self.cal_MD)
        B5 = X1 + X2
        temp = ((B5 + 8) >> 4) / 10.0
        self._logger.debug('Calibrated temperature {0} C'.format(temp))
        return temp

    def read_pressure(self):
        """Gets the compensated pressure in Pascals."""
        UT = self.read_raw_temp()
        UP = self.read_raw_pressure()
        # Datasheet values for debugging:
        #UT = 27898
        #UP = 23843
        # Calculations below are taken straight from section 3.5 of the datasheet.
        # Calculate true temperature coefficient B5.
        X1 = ((UT - self.cal_AC6) * self.cal_AC5) >> 15
        X2 = (self.cal_MC << 11) // (X1 + self.cal_MD)
        B5 = X1 + X2
        self._logger.debug('B5 = {0}'.format(B5))
        # Pressure Calculations
        B6 = B5 - 4000
        self._logger.debug('B6 = {0}'.format(B6))
        X1 = (self.cal_B2 * (B6 * B6) >> 12) >> 11
        X2 = (self.cal_AC2 * B6) >> 11
        X3 = X1 + X2
        B3 = (((self.cal_AC1 * 4 + X3) << self._mode) + 2) // 4
        self._logger.debug('B3 = {0}'.format(B3))
        X1 = (self.cal_AC3 * B6) >> 13
        X2 = (self.cal_B1 * ((B6 * B6) >> 12)) >> 16
        X3 = ((X1 + X2) + 2) >> 2
        B4 = (self.cal_AC4 * (X3 + 32768)) >> 15
        self._logger.debug('B4 = {0}'.format(B4))
        B7 = (UP - B3) * (50000 >> self._mode)
        self._logger.debug('B7 = {0}'.format(B7))
        if B7 < 0x80000000:
            p = (B7 * 2) // B4
        else:
            p = (B7 // B4) * 2
        X1 = (p >> 8) * (p >> 8)
        X1 = (X1 * 3038) >> 16
        X2 = (-7357 * p) >> 16
        p = p + ((X1 + X2 + 3791) >> 4)
        self._logger.debug('Pressure {0} Pa'.format(p))
        return p

    def read_altitude(self, sealevel_pa=101325.0):
        """Calculates the altitude in meters."""
        # Calculation taken straight from section 3.6 of the datasheet.
        pressure = float(self.read_pressure())
        altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa, (1.0/5.255)))
        self._logger.debug('Altitude {0} m'.format(altitude))
        return altitude

    def read_sealevel_pressure(self, altitude_m=0.0):
        """Calculates the pressure at sealevel when given a known altitude in
        meters. Returns a value in Pascals."""
        pressure = float(self.read_pressure())
        p0 = pressure / pow(1.0 - altitude_m/44330.0, 5.255)
        self._logger.debug('Sealevel pressure {0} Pa'.format(p0))
        return p0

def main(args):
	rospy.init_node('ms5837_pressure_sensor', anonymous=True)

	measure_internal_pressure()

        try:
		rospy.spin()
        except rospy.ROSInterruptException:
		print "Shutting down"
                pass
	


if __name__ == '__main__':
	main(sys.argv)
