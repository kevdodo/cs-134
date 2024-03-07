import numpy as np

class CameraProcess():
    def __init__(self, msg):
        self.hsvImageMap = {}
        self.camD = np.array(msg.d).reshape(5)
        self.camK = np.array(msg.k).reshape((3,3))
        self.camw = msg.width
        self.camh = msg.height
        self.depthImage = None

    def average_index_of_ones(self, arr):
        indices = np.argwhere(arr)

        # Calculate the average index for x and y separately
        avg_index_x = int(np.median(indices[:, 0]))
        avg_index_y = int(np.median(indices[:, 1]))
        return avg_index_x, avg_index_y

    
    def ir_depth(self, color):

        dists = self.depthImage[self.hsvImageMap[color] != 0.0]
    
        return np.nanmedian(dists[dists < 1500.0])
    
    def getPriorityDonut(self, color):
        camera_scale = 1000
        v, u = self.average_index_of_ones(self.hsvImageMap[color])
        zc = self.ir_depth(color)
        fx = self.camK[0, 0]
        fy = self.camK[1, 1]
        cx = self.camK[0, 2]
        cy = self.camK[1, 2]
        x_bar = (u - cx) / fx
        y_bar = (v - cy) / fy

        xc, yc = x_bar * zc / camera_scale, y_bar * zc / camera_scale

        zc = zc / camera_scale
        return xc, yc, zc
    
    def get_xy_bar(self, color):
        v, u = self.average_index_of_ones(self.hsvImageMap[color])
        fx = self.camK[0, 0]
        fy = self.camK[1, 1]
        cx = self.camK[0, 2]
        cy = self.camK[1, 2]
        x_bar = (v - cx) / fx
        y_bar = (u - cy) / fy
        return x_bar, y_bar