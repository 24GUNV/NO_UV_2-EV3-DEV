# File for random utility functions to be used


# Function to sense far colors
def sense_color_far(color_sensor):
    colors = color_sensor.rgb()
    hsv = RGB_to_HSV(colors[0], colors[1], colors[2])
    if hsv[1] < 0.22:
        if hsv[2] >= 25:
            color = 6
        else:
            color = 5
    
    else:
        if (hsv[0] >= 300 and hsv[0] <= 360) or hsv < 20:
            color = 1
        elif hsv[0] >= 20 and hsv[0] < 90:
            color = 2
        elif hsv[0] >= 90 and hsv[0] < 210:
            color = 3
        else:
            color = 4
    
    return color

# Given R G and B colors, returns the hue saturation and value (hsv)
def RGB_to_HSV(r, g, b):
    min_num = min(min(r, g), b)
    max_num = max(max(r, g), b)

    if max_num == min_num:
        h = 0
    elif max == r and g == b:
        h = 60 * (g - b) / (max_num - min_num)
    elif max_num == r and g < b:
        h = 60 * (g - b) / (max_num - min_num) + 360
    elif max_num == g:
        h = 60 * (b - r) / (max_num - min_num) + 120
    elif max_num == b:
        h = 60 * (r - g) / (max_num - min_num) + 240
    
    l = (max_num + min_num) / 2

    if max_num == 0:
        s = 0
    else:
        s = 1 - min_num / max_num

    v = max_num

    return (h, s, v)