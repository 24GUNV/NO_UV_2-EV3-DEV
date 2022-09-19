## File for all of the turning functions

# Libraries
import math
from ratio import Ratio
Ratios = Ratio()

# Class for the turning functions
class Turning():
	left_drive = None
	right_drive = None
	boost = 10 # The speed that the robot starts moving at
	CurrentPower = None # Class for the current power

	# Initialization functions
	# Input is Motor Object from pybricks
	def __init__(self, left_drive, right_drive, currentPower):
		self.left_drive = left_drive
		self.right_drive = right_drive
		self.CurrentPower = currentPower

	def Start_Smooth(self, V0, VMax, CEnc, a):
		if V0 < 0:
			return max(-math.sqrt(V0 * V0 - 2 * a * CEnc) + VMax, VMax)
		else:
			return max(math.sqrt(V0 * V0 + 2 * a * CEnc) + VMax, VMax)

	def Stop_Smooth(self, V0, VMin, CEnc, a):
		if V0 < 0:
			return min(-math.sqrt(V0 * V0 - 2 * a * CEnc) + VMin, VMin)
		else:
			return min(math.sqrt(V0 * V0 + 2 * a * CEnc) + VMin, VMin)
	
	# Function for partial turn towards the left
	# maxPowerRight, maxPowerLeft, minPowerLeft from -100 to 100 (%)
	# degrees = motor degrees turned
	def SmoothStart_Left(self, maxPowerRight: int, maxPowerLeft: int, minPowerLeft: int, degree: int):
		# Some mathematic calculations
		ratio = abs(maxPowerRight / maxPowerLeft)
		sign = abs(maxPowerRight * maxPowerLeft - 1) - abs(maxPowerLeft * maxPowerRight)
		e_old = 0

		# Inilializing speed variables
		pastSpeedR = self.right_drive.angle()
		pastSpeedL = self.left_drive.angle()
		speedR = 0
		speedL = 0

		isum = 0
		degree = degree * 2

		# # Starting the motors to move
		# init_L = max(self.boost, pastSpeedL)
		# init_R = max(self.boost, pastSpeedR)
		# self.left_drive.dc(init_L, wait=False)
		# self.right_drive.dc(init_R, wait=False)
		# CurrentPower.LMotor = init_L
		# CurrentPower.RMotor = init_R

		# Gonna turn until more than this degree
		while abs(speedR) + abs(speedL) < degree:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratios.Arc_ki * e
			
			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratios.Arc_kp * e + Ratios.Arc_kd * (e - e_old) + isum
			e_old = e

			self.CurrentPower.LMotor = self.Start_Smooth(minPowerLeft, maxPowerLeft, speedL, self.boost)
			self.CurrentPower.RMotor = maxPowerRight / maxPowerLeft * self.CurrentPower.LMotor # Sets the right motor to the ratio with the left motor

			# Drives the motors
			self.right_drive.dc(self.CurrentPower.RMotor - u * sign)
			self.left_drive.dc(self.CurrentPower.LMotor - u)

	# Function to slow it down to a stop while turning left
	def SmoothStop_Left(self, minPowerR, minPowerL, maxPowerL, degree):
		# Some mathematic calculations
		ratio = abs(minPowerR / minPowerL)
		sign = abs(minPowerR * minPowerL - 1) - abs(minPowerR * minPowerL)
		e_old = 0
		boost = (maxPowerL * maxPowerL - minPowerL * minPowerL)

		# Inilializing speed variables
		pastSpeedR = self.right_drive.angle()
		pastSpeedL = self.left_drive.angle()
		speedR = 0
		speedL = 0

		isum = 0
		enc = enc * 2

		while abs(speedR) + abs(speedL) < degree:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratios.Arc_ki * e
			
			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratios.Arc_kp * e + Ratios.Arc_kd * (e - e_old) + isum
			e_old = e

			self.CurrentPower.LMotor = self.Stop_Smooth(maxPowerL, minPowerL, speedL, boost)
			self.CurrentPower.RMotor = minPowerR / minPowerL * self.CurrentPower.LMotor # Sets the right motor to the ratio with the left motor

			# Drives the motors
			print(self.CurrentPower.LMotor, self.CurrentPower.RMotor)
			self.right_drive.dc(self.CurrentPower.RMotor - u * sign)
			self.left_drive.dc(self.CurrentPower.LMotor - u)

	
	# Function for partial turn towards the right
	# maxPowerRight, maxPowerLeft, minPowerLeft from -100 to 100 (%)
	# degrees = motor degrees turned
	def SmoothStart_Right(self, maxPowerLeft: int, maxPowerRight: int, minPowerRight: int, degree: int):
		# Some mathematic calculations
		ratio = abs(maxPowerLeft / maxPowerRight)
		sign = abs(maxPowerLeft * maxPowerRight - 1) - abs(maxPowerRight * maxPowerLeft)
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
		while abs(speedR) + abs(speedL) < degree:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratios.Arc_ki * e
			
			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratios.Arc_kp * e + Ratios.Arc_kd * (e - e_old) + isum
			e_old = e

			self.CurrentPower.RMotor = self.Start_Smooth(minPowerRight, maxPowerRight, speedR, self.boost)
			self.CurrentPower.LMotor = maxPowerLeft / maxPowerRight * self.CurrentPower.RMotor # Sets the right motor to the ratio with the left motor

			# Drives the motors
			self.right_drive.dc(self.CurrentPower.RMotor - u)
			self.left_drive.dc(self.CurrentPower.LMotor - u * sign)

	
	# Function to slow it down to a stop while turning left
	def SmoothStop_Right(self, minPowerL, minPowerR, maxPowerR, degree):
		# Some mathematic calculations
		ratio = abs(minPowerL / minPowerR)
		sign = abs(minPowerL * minPowerR - 1) - abs(minPowerL * minPowerR)
		e_old = 0
		boost = (maxPowerR * maxPowerR - minPowerR * minPowerR)

		# Inilializing speed variables
		pastSpeedR = self.right_drive.angle()
		pastSpeedL = self.left_drive.angle()
		speedR = 0
		speedL = 0

		isum = 0
		enc = enc * 2

		while abs(speedR) + abs(speedL) < degree:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratios.Arc_ki * e
			
			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratios.Arc_kp * e + Ratios.Arc_kd * (e - e_old) + isum
			e_old = e

			self.CurrentPower.RMotor = self.Stop_Smooth(maxPowerR, minPowerR, speedR, boost)
			self.CurrentPower.LMotor = minPowerL / minPowerR * self.CurrentPower.RMotor # Sets the right motor to the ratio with the left motor

			# Drives the motors
			print(self.CurrentPower.LMotor, self.CurrentPower.RMotor)
			self.right_drive.dc(self.CurrentPower.RMotor - u)
			self.left_drive.dc(self.CurrentPower.LMotor - u * sign)

	# Function for smooth turning
	# AKA stationary turning
	def SmoothAll_L(self, sPowerL, sPowerR, ePowerL, enc):
		ratio = abs(sPowerL / sPowerR)
		sign = abs(sPowerL * sPowerR - 1) - abs(sPowerL * sPowerR)
		e_old = 0
		
		# Inilializing speed variables
		pastSpeedR = self.right_drive.angle()
		pastSpeedL = self.left_drive.angle()
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
		
		maxPowerL_sign = sPowerL / abs(sPowerL)
		maxPowerL = maxPowerL * maxPowerL_sign
		startEnc = startEnc * maxPowerL_sign

		while abs(speedR) + abs(speedL) < enc:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratios.Arc_ki * e

			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratios.Arc_kp * e + Ratios.Arc_kd * (e - e_old) + isum
			e_old = e

			if abs(speedL) <= startEnc * maxPowerL_sign:
				self.CurrentPower.LMotor = self.Start_Smooth(sPowerL, maxPowerL, speedL, self.boost)
			else:
				self.CurrentPower.LMotor = self.Stop_Smooth(maxPowerL, ePowerL, speedL - startEnc, self.boost)
			
			# Actually running the motors itself
			self.left_drive.dc(self.CurrentPower.LMotor - u * sign)
			self.right_drive.dc(self.CurrentPower.RMotor - u)
		
	
	def SmoothAll_R(self, sPowerL, sPowerR, ePowerR, enc):
		ratio = abs(sPowerL / sPowerR)
		sign = abs(sPowerL * sPowerR - 1) - abs(sPowerL * sPowerR)
		e_old = 0
		
		# Inilializing speed variables
		pastSpeedR = self.right_drive.angle()
		pastSpeedL = self.left_drive.angle()
		speedR = 0
		speedL = 0

		isum = 0
		enc = enc * 2
		benc = enc - enc / (ratio + 1)
		excess = benc - (20000 - sPowerL * sPowerL - ePowerR * ePowerR) / 2 / self.boost

		if excess <= 0:
			maxPowerL = math.sqrt(benc * self.boost + sPowerL * sPowerL / 2 + ePowerR * ePowerR / 2)
			startEnc = (maxPowerL * maxPowerL - sPowerL * sPowerL) / 2 / self.boost
		else:
			maxPowerL = 100
			startEnc = (10000 - sPowerL * sPowerL) / 2 / self.boost + excess
		
		maxPowerL_sign = sPowerL / abs(sPowerL)
		maxPowerL = maxPowerL * maxPowerL_sign
		startEnc = startEnc * maxPowerL_sign

		while abs(speedR) + abs(speedL) < enc:
			speedR = self.right_drive.angle() - pastSpeedR
			speedL = self.left_drive.angle() - pastSpeedL
			e = speedR * sign + speedL * ratio
			isum = isum + Ratios.Arc_ki * e

			# DC motor cant drive more than 100%
			if isum > 100:
				isum = 100
			elif isum < -100:
				isum = -100

			# Math stuff
			u = Ratios.Arc_kp * e + Ratios.Arc_kd * (e - e_old) + isum
			e_old = e

			if abs(speedL) <= startEnc * maxPowerL_sign:
				self.CurrentPower.LMotor = self.Start_Smooth(sPowerL, maxPowerL, speedL, self.boost)
			else:
				self.CurrentPower.LMotor = self.Stop_Smooth(maxPowerL, ePowerR, speedL - startEnc, self.boost)
			
			# Actually running the motors itself
			self.left_drive.dc(self.CurrentPower.LMotor - u * sign)
			self.right_drive.dc(self.CurrentPower.RMotor - u)
		