## File for all of the turning functions


# Class for the turning functions
class Turning():
    left_drive = None
    right_drive = None

    # Initialization functions
    # Input is Motor Object from pybricks
    def __init__(self, left_drive, right_drive):
        self.left_drive = left_drive
        self.right_drive = right_drive
    
    
