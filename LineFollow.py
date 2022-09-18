from CurrentPower import CurrentPower
import math
import time
from ratio import Ratio

class LineFollow:
    left_sensor = None
    right_sensor = None
    left_motor = None
    right_motor = None

    # Cross Variables
    borderXCross = 30
    borderLCross= 5

    def __init__(self, left_sensor, right_sensor, left_motor, right_motor):
        self.left_sensor = left_sensor
        self.right_sensor = right_sensor
        self.left_motor = left_motor
        self.right_motor = right_motor      

    # Mapping a range of 255 to 100 scale
    def mapping255(value, min, max):
        return (value - min) / (max - min) * 255
    
    # Mapping a range of 100
    def mapping100(value, min, max):
        return (value - min) / (max - min) * 100
    
    def avgAngle(self):
        return (self.left_motor.angle() - self.right_motor.angle()) / 2
    
    # Values for having it follow a black line
    def SET_DEFAULT(self):
        self.isInverse = 1
        self.isR1 = 1
        self.isG1 = 1
        self.isB1 = 1
        self.isR2 = 1
        self.isG2 = 1
        self.isB2 = 1

        self.rCross = 1
        self.gCross = 1
        self.bCross = 1
        
        # To be measured
        # self.minR1 =
        # self.minG1 = 
        # self.minB1 = 

        # self.maxR1 =  
        # self.maxG1 = 
        # self.maxB2 = 

        # self.minR2 =
        # self.minG2 = 
        # self.minB2 = 

        # self.maxR2 =  
        # self.maxG2 = 
        # self.maxB2 = 

    def readRGB():
        return 0

    def getS1():
        return 0
    
    def getS2():
        return 0
    
    def getCrossesS1():
        return 0
    
    def getCrossesS2():
        return 0

    def Smooth_Start(V0, VMax, CEnc, a):
        return min(math.sqrt(V0 * V0 + 2 * a * CEnc), VMax)


    # Function will make the robot follow a line, until it crosses a certain amount of crosses
    def XCROSS_smoothstart(self, minPower, maxPower, crosses):
        s1 = 100
        s2 = 100
        flag = 1
        e_old = 0
        count = 0
        CurrentPower.LineFollower = minPower
        kp = math.sqrt(maxPower / Ratio.R_Sensor_power) * Ratio.R_Sensor_kp
        kd = Ratio.R_Sensor_kd / Ratio.R_Sensor_kp * kp
        ki = Ratio.R_Sensor_ki / Ratio.L_Sensor_kp * kp
        isum = 0
        c1 = 0
        c2 = 0
        currentAngle = self.avgAngle()

        while count < crosses:
            self.readRGB()
            s1 = self.getS1()
            s2 = self.getS2()

            e = (s1 - s2) * self.isInverse
            isum = isum + ki * e
            if isum > 100:
                isum = 100
            elif isum < -100:
                isum = -100
            
            u = kp * e + (e - e_old) * kd + isum
            e_old = e
            currentAngle = self.avgAngle()
            CurrentPower.LineFollower = self.Smooth_Start(minPower, maxPower, currentAngle, 14, CurrentPower.LineFollower)
            vb = (-u - CurrentPower.LineFollower)
            vc = CurrentPower.LineFollower - u
            if vb > 100:
                vb = 100
            elif vb < -100:
                vb = -100
            if vc > 100:
                vc = 100
            elif vc < -100:
                vc = -100
            self.left_motor.DC(vb)
            self.right_motor.DC(vc)
            c1 = self.getCrossesS1()
            c2 = self.getCrossesS2()

            if c1 < self.borderXCross and c2 < self.borderXCross:
                if flag == 0:
                    flag = 1
                    count += 1
                else:
                    flag = 0
            else:
                flag = 0
        time.sleep(1)

