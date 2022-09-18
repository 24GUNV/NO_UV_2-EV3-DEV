## File for all of the turning functions

# Libraries
from inspect import currentframe
import math
from tracemalloc import start
from ratio import Ratio
from CurrentPower import CurrentPower

# Class for the turning functions
class Turning():
	left_drive = None
	right_drive = None
	boost = 10 # The speed that the robot starts moving at

	# Initialization functions
	# Input is Motor Object from pybricks
	def __init__(self, left_drive, right_drive):
		self.left_drive = left_drive
		self.right_drive = right_drive

	def Start_Smooth(V0, VMax, CEnc, a):
		if V0 < 0:
			return max(-math.sqrt(V0 * V0 - 2 * a * CEnc) + VMax, VMax)
		else:
			return max(math.sqrt(V0 * V0 + 2 * a * CEnc) + VMax, VMax)

	def Stop_Smooth(V0, VMin, CEnc, a):
		if V0 < 0:
			return min(-math.sqrt(V0 * V0 - 2 * a * CEnc) + VMin, VMin)
		else:
			return min(math.sqrt(V0 * V0 + 2 * a * CEnc) + VMin, VMin)
	
	# Function for partial turn towards the left
	# maxPowerRight, maxPowerLeft, minPowerLeft from -100 to 100 (%)
	# degrees = motor degrees turned
	def SmoothStart_Left(self, maxPowerRight: int, maxPowerLeft: int, minPowerLeft: int, degree: int):
		# Some mathematic calculations
		ratio = math.abs(maxPowerRight / maxPowerLeft)
		sign = math.abs(maxPowerRight * maxPowerLeft - 1) - math.abs(maxPowerLeft * maxPowerRight)
		e_old = 0

		# Inilializing speed variables
		pastSpeedR = self.right_drive.angle()
		pastSpeedL = self.left_drive.angle()
		speedR = 0
		speedL = 0

		isum = 0
		enc = enc * 2

		# # Starting the motors to move
		# init_L = max(self.boost, pastSpeedL)
		# init_R = max(self.boost, pastSpeedR)
		# self.left_drive.dc(init_L, wait=False)
		# self.right_drive.dc(init_R, wait=False)
		# CurrentPower.LMotor = init_L
		# CurrentPower.RMotor = init_R

		# Gonna turn until more than this degree
		while math.abs(speedR) + math.abs(speedL) < degree:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratio.Arc_ki * e
			
			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratio.Arc_kp * e + Ratio.Arc_kd * (e - e_old) + isum
			e_old = e

			CurrentPower.LMotor = self.Start_Smooth(minPowerLeft, maxPowerLeft, speedL, self.boost)
			CurrentPower.RMotor = maxPowerRight / maxPowerLeft * CurrentPower.LMotor # Sets the right motor to the ratio with the left motor

			# Drives the motors
			self.right_drive.DC(CurrentPower.RMotor - u * sign)
			self.left_drive.DC(CurrentPower.LMotor - u)

	# Function to slow it down to a stop while turning left
	def SmoothStop_Left(self, minPowerR, minPowerL, maxPowerL, degree):
		# Some mathematic calculations
		ratio = math.abs(minPowerR / minPowerL)
		sign = math.abs(minPowerR * minPowerL - 1) - math.abs(minPowerR * minPowerL)
		e_old = 0
		boost = (maxPowerL * maxPowerL - minPowerL * minPowerL)

		# Inilializing speed variables
		pastSpeedR = self.right_drive.angle()
		pastSpeedL = self.left_drive.angle()
		speedR = 0
		speedL = 0

		isum = 0
		enc = enc * 2

		while math.abs(speedR) + math.abs(speedL) < degree:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratio.Arc_ki * e
			
			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratio.Arc_kp * e + Ratio.Arc_kd * (e - e_old) + isum
			e_old = e

			CurrentPower.LMotor = self.Stop_Smooth(maxPowerL, minPowerL, speedL, boost)
			CurrentPower.RMotor = minPowerR / minPowerL * CurrentPower.LMotor # Sets the right motor to the ratio with the left motor

			# Drives the motors
			print(CurrentPower.LMotor, CurrentPower.RMotor)
			self.right_drive.DC(CurrentPower.RMotor - u * sign)
			self.left_drive.DC(CurrentPower.LMotor - u)

	
	# Function for partial turn towards the right
	# maxPowerRight, maxPowerLeft, minPowerLeft from -100 to 100 (%)
	# degrees = motor degrees turned
	def SmoothStart_Right(self, maxPowerLeft: int, maxPowerRight: int, minPowerRight: int, degree: int):
		# Some mathematic calculations
		ratio = math.abs(maxPowerLeft / maxPowerRight)
		sign = math.abs(maxPowerLeft * maxPowerRight - 1) - math.abs(maxPowerRight * maxPowerLeft)
		e_old = 0

		# Inilializing speed variables
		pastSpeedR = self.right_drive.angle()
		pastSpeedL = self.left_drive.angle()
		speedR = 0
		speedL = 0

		isum = 0
		enc = enc * 2

		# # Starting the motors to move
		# init_L = max(self.boost, pastSpeedL)
		# init_R = max(self.boost, pastSpeedR)
		# self.left_drive.dc(init_L, wait=False)
		# self.right_drive.dc(init_R, wait=False)
		# CurrentPower.LMotor = init_L
		# CurrentPower.RMotor = init_R

		# Gonna turn until more than this degree
		while math.abs(speedR) + math.abs(speedL) < degree:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratio.Arc_ki * e
			
			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratio.Arc_kp * e + Ratio.Arc_kd * (e - e_old) + isum
			e_old = e

			CurrentPower.RMotor = self.Start_Smooth(minPowerRight, maxPowerRight, speedR, self.boost)
			CurrentPower.LMotor = maxPowerLeft / maxPowerRight * CurrentPower.RMotor # Sets the right motor to the ratio with the left motor

			# Drives the motors
			self.right_drive.DC(CurrentPower.RMotor - u)
			self.left_drive.DC(CurrentPower.LMotor - u * sign)

	
	# Function to slow it down to a stop while turning left
	def SmoothStop_Right(self, minPowerL, minPowerR, maxPowerR, degree):
		# Some mathematic calculations
		ratio = math.abs(minPowerL / minPowerR)
		sign = math.abs(minPowerL * minPowerR - 1) - math.abs(minPowerL * minPowerR)
		e_old = 0
		boost = (maxPowerR * maxPowerR - minPowerR * minPowerR)

		# Inilializing speed variables
		pastSpeedR = self.right_drive.angle()
		pastSpeedL = self.left_drive.angle()
		speedR = 0
		speedL = 0

		isum = 0
		enc = enc * 2

		while math.abs(speedR) + math.abs(speedL) < degree:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratio.Arc_ki * e
			
			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratio.Arc_kp * e + Ratio.Arc_kd * (e - e_old) + isum
			e_old = e

			CurrentPower.RMotor = self.Stop_Smooth(maxPowerR, minPowerR, speedR, boost)
			CurrentPower.LMotor = minPowerL / minPowerR * CurrentPower.RMotor # Sets the right motor to the ratio with the left motor

			# Drives the motors
			print(CurrentPower.LMotor, CurrentPower.RMotor)
			self.right_drive.DC(CurrentPower.RMotor - u)
			self.left_drive.DC(CurrentPower.LMotor - u * sign)

	# Function for smooth turning
	# AKA stationary turning
<<<<<<< Updated upstream:turn.py
	def SmoothAll(self, sPowerL, sPowerR, ePowerL, enc):
		ratio = math.abs(sPowerL / sPowerR)
		sign = math.abs(sPowerL * sPowerR - 1) - math.abs(sPowerL * sPowerR)
		e_old = 0
		
		# Inilializing speed variables
		pastSpeedR = self.right_drive.speed()
		pastSpeedL = self.left_drive.speed()
		speedR = 0
		speedL = 0

		isum = 0
		enc = enc * 2
		benc = enc - enc / (ratio + 1)
		excess = benc - (20000 - sPowerL * sPowerL - ePowerL * ePowerL) / 2 / self.boost

		if excess <= 0:
			maxPowerL = math.sqrt(benc * self.boost + sPowerL * sPowerL / 2 + ePowerL * ePowerL / 2)
			startEnc = (maxPowerL * maxPowerL - sPowerL * sPowerL) / 2 / self.boost
		else:
			maxPowerL = 100
			startEnc = (10000 - sPowerL * sPowerL) / 2 / self.boost + excess
		
		maxPowerL_sign = sPowerL / math.abs(sPowerL)
		maxPowerL = maxPowerL * maxPowerL_sign
		startEnc = startEnc * maxPowerL_sign

		while math.abs(speedR) + math.abs(speedL) < enc:
			speedR = self.right_drive.speed() - pastSpeedR
			speedL = self.left_drive.speed() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratio.Arc_ki * e

			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratio.Arc_kp * e + Ratio.Arc_kd * (e - e_old) + isum
			e_old = e

			if math.abs(speedL) <= startEnc * maxPowerL_sign:
				CurrentPower.LMotor = self.Start_Smooth(sPowerL, maxPowerL, speedL, self.boost)
			else:
				CurrentPower.LMotor = self.Stop_Smooth(maxPowerL, ePowerL, speedL - startEnc, self.boost)
			
			# Actually running the motors itself
			self.left_drive.DC(CurrentPower.LMotor - u * sign)
			self.right_drive.DC(CurrentPower.RMotor - u)
		
=======
	def SmoothAll(self):

		raise NotImplementedError
>>>>>>> Stashed changes:EV3/turn.py
