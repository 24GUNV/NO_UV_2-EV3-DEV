# File for random utility functions to be used


# Function to sense far colors
def sense_color_far(color_sensor):
    hsv = color_sensor.hsv(surface =True)
    if hsv.s < 0.22:
        if hsv.v >= 25:
            color = 6
        else:
            color = 5
    
    else:
        if (hsv.h >= 300 and hsv.h <= 360) or hsv < 20:
            color = 1
        elif hsv.h >= 20 and hsv.h < 90:
            color = 2
        elif hsv.h >= 90 and hsv.h < 210:
            color = 3
        else:
            color = 4
    
    return color