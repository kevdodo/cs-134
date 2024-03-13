import numpy as np

DISK_COLOR_MAP = {5:"red", 4: "orange", 3:"yellow", 2:"green", 1: "blue", -1:"black"} 
COLOR_TO_DISK_MAP = {"blue":1, "green":2, "yellow":3, "orange":4, "red":5, "black": -1}

import cv2

import copy

class CameraProcess():
    def __init__(self, msg):
        self.camD = np.array(msg.d).reshape(5)
        self.camK = np.array(msg.k).reshape((3,3))
        self.camw = msg.width
        self.camh = msg.height
        self.depthImage = None
        self.hsvImageMap = {}

        self.prev_img = {}

    def average_index_of_ones(self, color):
        arr = self.filter_binary_color(color)
        # arr = self.hsvImageMap[color]
        indices = np.argwhere(arr)

        print(len(np.flatnonzero(indices)))

        # Calculate the average index for x and y separately
        avg_index_x = int(np.nanmedian(indices[:, 0]))
        avg_index_y = int(np.nanmedian(indices[:, 1]))
        return avg_index_x, avg_index_y

    
    def ir_depth(self, color):
        # print("dist shape", self.depthImage.shape)
        shape = self.depthImage.shape
        dists = self.depthImage[self.hsvImageMap[color] != 0.0]

        dists = dists[dists < 1000.0]
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
    
    def getTopColors(self, p, R):
        heights = []
        for color_idx in range(1, 6):
            color = DISK_COLOR_MAP[color_idx]
            if len(np.flatnonzero(self.hsvImageMap[color])) > 1000:
                xc, yc, zc = self.getDonutLoc(color)

                donut_loc = np.array(p + R @ 
                        np.array([xc, zc, -yc]).reshape((3,1)))
                
                height = donut_loc[2, 0]
                
                height = height - ((1/.35)*donut_loc[0,0])/68

                heights.append((height, color_idx))
        # print(sorted(heights, key=lambda x : x[0], reverse=True))
        return sorted(heights, key=lambda x : x[0], reverse=True)

    def onFloor(self, color):
        _, _, height = self.getDonutLoc(color)
        return height > .12 

        # heights = []
        # for color_idx in range(1, 6):
        #     color = DISK_COLOR_MAP[color_idx]
        #     if len(np.flatnonzero(self.hsvImageMap[color])) > 1000:
        #         heights.append((self.ir_depth(color), color_idx))
        # return sorted(heights, key=lambda x : x[0])
    
    def get_game_state(self, p, R):
        shape = self.hsvImageMap['blue'].shape

        left_to_right_colors =  [None, None, None]
        curr_idx = 0

        num_cols = shape[1] // 3
        while curr_idx < 3:
            colors = []
            for color in COLOR_TO_DISK_MAP:
                image_map = self.hsvImageMap[color]
                if len(np.flatnonzero(image_map)) > 50:

                    xc, yc, zc = self.getDonutLoc(color)
                    # self.get_logger().info(f"priority donut camera {[xc, yc, zc]}")
                
                    donut_position = np.array(p + R @ 
                            np.array([xc, zc, -yc]).reshape((3,1))).reshape((3,1))
                    
                    # gets the correct partition
                    partitioned_map = image_map[:, curr_idx*num_cols : (curr_idx+1) * num_cols]
                    # print(curr_idx, color, len(np.flatnonzero(partitioned_map)))
                    if len(np.flatnonzero(partitioned_map)) > 50:
                        colors.append([donut_position[2, 0], color])
            valid_colors = sorted(colors, key = lambda x : x[0], reverse=True )

            print(valid_colors)
            # left_to_right_colors.append(valid_colors[0][1])
            if len(valid_colors) > 0:
                col = valid_colors[0][1]
                if col != 'black':
                    left_to_right_colors[curr_idx] = col
                else:
                    if len(valid_colors) == 1:
                        left_to_right_colors[curr_idx] = col
                    else:
                        col = valid_colors[1][1]
                        left_to_right_colors[curr_idx] = col
            curr_idx += 1
        return left_to_right_colors

    def filter_binary_color(self, color):
        shape = self.depthImage.shape

        cond = self.hsvImageMap[color] == 0.0
        # print('cond', len(np.flatnonzero(cond)))

        dists = np.copy(self.depthImage)
        # print('nonzeros1', len(np.flatnonzero(dists)))

        dists[cond] = 0.0
        # print('nonzeros1', len(np.flatnonzero(dists)))

        dists[dists > 1500.0] = 0.0
        # print('nonzeros2', len(np.flatnonzero(dists)))
        dists[dists < 10] = 0.0
        # print('nonzeros3', len(np.flatnonzero(dists)))

        dists_cond = dists == 0

        map = np.copy(self.hsvImageMap[color])

        map[dists_cond] = 0.0

        if len(np.flatnonzero(map)) == 0:
            return self.prev_img[color]
        # print('nonzeros', len(np.flatnonzero(map)))

        self.prev_img[color] = map

        return map
    

    def get_contours(self, color):
        binary = self.hsvImageMap[color]

        binary = self.filter_binary_color(color)
        # Erode and Dilate. Definitely adjust the iterations!

        iter = 4
        binary = cv2.erode(binary, None, iterations=iter)
        binary = cv2.dilate(binary, None, iterations=2*iter)
        binary = cv2.erode(binary, None, iterations=iter)


        # Find contours in the mask and initialize the current
        # (x, y) center of the ball
        (contours, hierarchy) = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all contours on the original image for debugging.
        # cv2.drawContours(frame, contours, -1, self.blue, 2)

        # Only proceed if at least one contour was found.  You may
        # also want to loop over the contours...
        if len(contours) > 0:
            # Pick the largest contour.
            for contour in contours:
                #contour = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                # cv2.drawContours(frame,[box],0,(0,0,255),2)
# 
                # center_x = int(np.average([x[0] for x in box]))
                # center_y = int(np.average([x[1] for x in box]))


                ((ur, vr), radius) = cv2.minEnclosingCircle(contour)

                return ur, vr


                    # [[x, y]] = self.transform_box(frame, trans_mat, [[ur, vr]], annotateImage=True)
                    # new_point = Point()
                    # new_point.x = float(x)
                    # new_point.y = float(y)
                    # new_point.z = float(0.0)
                    # # cv2.circle(frame, (ur, vr), int(radius), self.yellow,  2)
                    # # cv2.circle(frame, (ur, vr), 5,           self.red,    -1)
                    # self.pubpoint.publish(new_point)


    def getDonutLoc(self, color):
        camera_scale = 1000

        v, u = self.average_index_of_ones(color)


        uv = np.array([u, v], dtype=np.double).reshape((1,1,2))

        # print("uv", uv, uv.shape)

        # print(u, v, zc)

        coords = cv2.undistortPoints(uv, self.camK, self.camD)
        # print("coords shape", coords.shape)
        x_bar = coords[0][0][0]

        y_bar = coords[0][0][1]

        zc = self.ir_depth(color)

        # fx = self.camK[0, 0]
        # fy = self.camK[1, 1]
        # cx = self.camK[0, 2]
        # cy = self.camK[1, 2]
        # x_bar = (u - cx) / fx
        # y_bar = (v - cy) / fy


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
        v, u = self.average_index_of_ones(color)

        uv = np.array([v, u], dtype=np.double).reshape((1,1,2))

        # print("uv", uv, uv.shape)

        # print(u, v, zc)

        coords = cv2.undistortPoints(uv, self.camK, self.camD)
        # fx = self.camK[0, 0]
        # fy = self.camK[1, 1]
        # cx = self.camK[0, 2]
        # cy = self.camK[1, 2]
        # x_bar = (v - cx) / fx
        # y_bar = (u - cy) / fy
        return coords[0][0][0], coords[0][0][1]