import numpy as np
DISK_COLOR_MAP = {1:"red", 2: "orange", 3:"yellow", 4:"green", 5: "blue"} 

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
        # print("dist shape", self.depthImage.shape)
        shape = self.depthImage.shape
        dists = self.depthImage[self.hsvImageMap[color] != 0.0]

        dists = dists[dists < 1500.0]
        dists = dists[dists > 10]

        # Get the minimum value
        return np.nanmedian(dists)
    
    def getDonutColors(self):
        heights = []
        for color_idx in range(1, 6):
            color = DISK_COLOR_MAP[color_idx]
            if len(np.flatnonzero(self.hsvImageMap[color])) > 1000:
                heights.append((self.ir_depth(color), color_idx))
        return sorted(heights, key=lambda x : x[0])
    
    def getTopColors(self):
        heights = []
        for color_idx in range(1, 6):
            color = DISK_COLOR_MAP[color_idx]
            if len(np.flatnonzero(self.hsvImageMap[color])) > 1000:
                heights.append((self.ir_depth(color), color_idx))
        return sorted(heights, key=lambda x : x[0])
        
    # def kevin_tweaked(self, color):

    #     shape = self.depthImage.shape

    #     dists = np.copy(self.depthImage)

    #     dists[self.hsvImageMap[color] != 0.0] = 0.0
    #     print('was', len(np.flatnonzero(dists)))

    #     dists[dists > 1500.0] = 0.0
    #     print('hahah', len(np.flatnonzero(dists)))

    #     dists[dists < 10] = 0.0



    #     flattened_dists = sorted(dists.flatten()[np.flatnonzero(dists)])

    #     print('whataskldfj;alskdnf', len(flattened_dists))
    #     twPT = flattened_dists[:int(.2 * len(flattened_dists))] 
    #     # for i in range(shape[0]):
    #     #     for j in range(shape[1]):
    #     #         if dists[i][j] in twPT:
    #     #             coordsDist.append(dists[i, j])
    #     #             coords.append([i, j])

    #     # print('wut', np.array([coords]).shape)

    #     # return [np.median(np.array([coords])[:, 0]), np.median(np.array([coords])[:, 1])], np.median(np.array([coordsDist]))

    #     valid = np.isin(dists, twPT)
    #     print('valid', valid)

    #     print('bruhhh', len(np.flatnonzero(valid)))

    #     coordinates = np.where(valid)

    #     print('coord shape', np.array(coordinates).shape)

    #     # Calculate the median of the coordinates
    #     median_coordinates = np.median(coordinates, axis=1)
    #     return [median_coordinates[0], median_coordinates[1]], np.median(twPT)
    #     # print(f"The median coordinates are {median_coordinates}")
    #     # coordinates = np.where(np.isin(dists, twPT))

    #     # print('coord shape', coordinates)

    #     # return [np.median(coordinates, axis=0),np.median(coordinates, axis=1)] , np.median(dists[coordinates]) 
    # # def josh_tweaked(self, color):
    # #     shape = self.depthImage.shape

    # #     v, u = self.average_index_of_ones(self.hsvImageMap[color])
    # #     dists = np.copy(self.depthImage)
    # #     dists[self.hsvImageMap[color] != 0.0] = 0.0
    # #     dists[dists < 1500.0] = 0.0
    # #     dists[dists > 10] = 0.0


    # #     while v < shape[0] and dists[v, u] != 0:
    # #         v += 1

    # #     return v, u, dists[v, u]

    def getDonutLoc(self, color):
        camera_scale = 1000

        v, u = self.average_index_of_ones(self.hsvImageMap[color])
        # u, v, zc = self.josh_tweaked(color)
        # u, v, zc = self.final_tweak(color)

        # print(u, v, zc)

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
    
    def getPriorityDonut2(self, color):
        camera_scale = 1000

        # v, u = self.average_index_of_ones(self.hsvImageMap[color])
        # u, v, zc = self.josh_tweaked(color)
        u, v, zc = self.final_tweak(color)

        # print(u, v, zc)

        # zc = self.ir_depth(color)

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