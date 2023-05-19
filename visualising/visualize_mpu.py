from ursina import *
from math import atan2, degrees

def get_angles():

    with open("log.txt", 'r') as f:
        
        if f.readlines[-1] == 's':
            accel = eval(f.readlines[-4])
            gyro = eval(f.readlines[-4])
            mag = eval(f.readlines[-4])
            angles = eval(f.readlines()[-1])
            self.stats.text = self.get_stats(angles[0], angles[1], angles[2])

    return [accel, gyro, mag]

angles = get_angles()


app = Ursina()
camera.y = 1
class Block(Entity):

    def __init__(self, **kwargs):
        super().__init__()
        self.model='cube'
        self.color = color.red
        self.scale_x = 3
        self.scale_y = 0.5
        self.scale_z = 3
        self.texture = 'fav.jpg'

        for key, value in kwargs.items():
            setattr(self, key, value)

        self.stats = Text(text = self.get_stats(0, 0, 0))
        self.stats.x = -0.6


    def get_stats(self, roll, pitch, yaw):
        return f"Roll: {roll}\nPitch: {pitch}\nYaw: {yaw}"

    def update(self):
        self.rotation_x = -angles[0]
        self.rotation_y = -angles[2]
        self.rotation_z = -angles[1]


MPU = Block()
app.run()



