import numpy as np
from sklearn.cluster import KMeans

DISK_COLOR_MAP = {1:"red", 2: "orange", 3:"yellow", 4:"green", 5: "blue"} 

class CameraProcess():
    def __init__(self, msg):
        self.hsvImageMap = {}
        self.camD = np.array(msg.d).reshape(5)
        self.camK = np.array(msg.k).reshape((3,3))
        self.camw = msg.width
        self.camh = msg.height
        self.depthImage = None
        self.justPlaced = None

    def average_index_of_ones(self, arr):
        indices = np.argwhere(arr)

        # Calculate the average index for x and y separately
        avg_index_y = int(np.average(indices[:, 0]))
        avg_index_x = int(np.average(indices[:, 1]))
        return avg_index_x, avg_index_y
    
    def getTopColors(self):
        
        heights = []
        for color_idx in range(1, 6):
            heights.append((self.ir_depth(DISK_COLOR_MAP[color_idx]), color_idx))
        return sorted(heights, key=lambda x : x[0])

    
    def ir_depth(self, color):

        dists = self.depthImage[self.hsvImageMap[color] != 0.0]

        return np.nanmedian(dists)
    
    def transform_idx(self, u, v, color):
        """Transforms the v and u values to world frame 
        given color (for depth) """
        # TODO: Implement so that IR depth gets pixels within some reasonable amount

        # colors will be on the same depth lowkey
        CAMERA_SCALE = 1000

        zc = self.ir_depth(color)
        fx = self.camK[0, 0]
        fy = self.camK[1, 1]
        cx = self.camK[0, 2]
        cy = self.camK[1, 2]
        x_bar = (u - cx) / fx
        y_bar = (v - cy) / fy

        xc, yc = x_bar * zc / CAMERA_SCALE, y_bar * zc / CAMERA_SCALE

        zc = zc / CAMERA_SCALE
        return xc, yc, zc    
    
    def getDonutLoc(self, color):

        u, v = self.average_index_of_ones(self.hsvImageMap[color])
        return self.transform_idx(u, v, color)
        # zc = self.ir_depth(color)
        # fx = self.camK[0, 0]
        # fy = self.camK[1, 1]
        # cx = self.camK[0, 2]
        # cy = self.camK[1, 2]
        # x_bar = (u - cx) / fx
        # y_bar = (v - cy) / fy

        # xc, yc = x_bar * zc / camera_scale, y_bar * zc / camera_scale

        # zc = zc / camera_scale
        # return xc, yc, zc

    # def getPriorityDonut(self, color):
    #     if same:
    #         return self.getDonutLoc(self.justPlaced), True
    #     for i in range(1, 6):
    #         color = DISK_COLOR_MAP[i]
    #         if color != self.justPlaced and len(np.flatnonzero(self.hsvImageMap[color])) > 50:
    #             self.justPlaced = color
    #             return self.getDonutLoc(color), True
    #     return None, None, None, False
    
    def get_xy_bar(self, color):
        u, v = self.average_index_of_ones(self.hsvImageMap[color])
        fx = self.camK[0, 0]
        fy = self.camK[1, 1]
        cx = self.camK[0, 2]
        cy = self.camK[1, 2]
        x_bar = (v - cx) / fx
        y_bar = (u - cy) / fy
        return x_bar, y_bar
    
    def getNonZeroIndices(self, color):
        return np.argwhere(self.hsvImageMap[color] != 0)

    def getPegLoc(self, col):
        """Returns the base of the peg location"""

        if col == 'black':
            # TODO: Kmeans is too computationally expensive
            idxs = np.argwhere(self.hsvImageMap['black'])# self.getNonZeroIndices('black')
            kmeans = KMeans(n_clusters=2, random_state=0, max_iter=20, n_init="auto")
            # should be the indices of Nonzero
            kmeans.fit(idxs)

            cluster_centers = kmeans.cluster_centers_

            peg_locs = []
            for cluster_center in cluster_centers:
                peg_locs.append(self.transform_idx(cluster_center[0], cluster_center[1], 'black'))
            return peg_locs
        return self.getDonutLoc(col)

    
    # def ir_depth_coords(self, color, v, u):
        

    #     # TODO Not jank solution
    #     # depth of all places not zero


    #     # transform index

    #     pass
    # def getDonutLoc(self, color):
    #     camera_scale = 1000
    #     u, v = self.average_index_of_ones(self.hsvImageMap[color])
    #     zc = self.ir_depth(color)
    #     fx = self.camK[0, 0]
    #     fy = self.camK[1, 1]
    #     cx = self.camK[0, 2]
    #     cy = self.camK[1, 2]
    #     x_bar = (u - cx) / fx
    #     y_bar = (v - cy) / fy

    #     xc, yc = x_bar * zc / camera_scale, y_bar * zc / camera_scale

    #     zc = zc / camera_scale
    #     return xc, yc, zc
    
    # def get_xy_bar(self, color):
    #     v, u = self.average_index_of_ones(self.hsvImageMap[color])
    #     fx = self.camK[0, 0]
    #     fy = self.camK[1, 1]
    #     cx = self.camK[0, 2]
    #     cy = self.camK[1, 2]
    #     x_bar = (v - cx) / fx
    #     y_bar = (u - cy) / fy
    #     return x_bar, y_bar
    

    
    # def getDonutLoc(self, color):
    #     u, v = self.average_index_of_ones(self.hsvImageMap[color])
    #     return self.transform_idx(u, u, color)


    #     # zc = self.ir_depth(color)
    #     # fx = self.camK[0, 0]
    #     # fy = self.camK[1, 1]
    #     # cx = self.camK[0, 2]
    #     # cy = self.camK[1, 2]
    #     # x_bar = (u - cx) / fx
    #     # y_bar = (v - cy) / fy

    #     # xc, yc = x_bar * zc / camera_scale, y_bar * zc / camera_scale

    #     # zc = zc / camera_scale
    #     # return xc, yc, zc
    
    # def getNonZeroIndices(self, color):
    #     bin_image = self.hsvImageMap[color]
    #     return np.argwhere(bin_image)
    
        
        
    #     # return self.getDonutLoc(self, 'black')
    
    # def get_xy_bar(self, color):
    #     v, u = self.average_index_of_ones(self.hsvImageMap[color])
    #     fx = self.camK[0, 0]
    #     fy = self.camK[1, 1]
    #     cx = self.camK[0, 2]
    #     cy = self.camK[1, 2]
    #     x_bar = (v - cx) / fx
    #     y_bar = (u - cy) / fy
    #     return x_bar, y_bar