import utils
from robot import Robot
import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
plt.style.use('ggplot')



class sensor():

    # the x, y of sensor should be the position of robot, we can define other values w.r.t the actual env size.
    def __init__(self, env, x_dim, y_dim, rotate_spd=0.1, x=500, y=500):
        self.rotate_spd = rotate_spd # degree per sample
        self.sensor_x = x
        self.sensor_y = y
        self.env = env
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.theta = 0
        self.sensor_data = np.zeros(int(360/self.rotate_spd))  # for 360 degrees we can get 3600 samples by default.
        self.detected_info = [{} for i in range(len(self.sensor_data))]

    def one_circle(self):
        for deg in range(len(self.sensor_data)):
            info_dic = {}
            theta = deg / 10
            # print the theta every 30 degrees
            if theta % 30 == 0:
                print(theta)
            closest_dist, closest_intsct_x, closest_intsct_y, x_range, y_range = self.rangefinder_distance(theta)
            info_dic[0] = closest_dist
            info_dic[1] = (closest_intsct_x, closest_intsct_y)
            info_dic[2] = (x_range, y_range)
            self.detected_info[deg] = info_dic
        sensor_1 = np.zeros(len(self.detected_info))
        # sensor_2 = np.zeros(len(self.detected_info))
        # the two sensors have 60 degrees in between
        for i in range(len(sensor_1)):
            sensor_1[i] = self.detected_info[i][0]  # get the closest distance values and store in this array
        sensor_2 = np.roll(sensor_1, 600)  # rotate 60 degrees

        plt.figure(figsize=(25, 30))
        plt.scatter(self.env[0], self.env[1], c='k', linewidth=2)
        plt.scatter(self.sensor_x, self.sensor_y, c='r', marker='*')
        plt.xlim([0, self.x_dim])
        plt.ylim([0, self.y_dim])
        plt.title('Environment created from figure and robot position')
        plt.xlabel('X')
        plt.ylabel('Y')



        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(sensor_1)
        plt.xlabel('Scanning time')
        plt.ylabel('Sensor_1 distance measurement')

        plt.subplot(2, 1, 2)
        plt.plot(sensor_2)
        plt.xlabel('Scanning time')
        plt.ylabel('Sensor_2 distance measurement')

        plt.show()









    def rangefinder_distance(self, theta):
        """
        :return:
        distance: distance between laser rangefinder and object it's facing.
        x_intersect = X coord of intersection ----- closest_intsct_x
        y_intersect = Y coord of intersection ----- closest_Intsct_y
        x_range: range of X values of sensor beam
        y_range: range of Y values of sensor beam
        """
        env_x = self.env[0]
        env_y = self.env[1]

        sensor_angle = theta % 360
        if sensor_angle == 0:
            x_max = self.x_dim
            x_range = np.linspace(self.sensor_x, x_max, self.x_dim - self.sensor_x + 1)
            y_range = self.sensor_y * np.ones(len(x_range))
            idx = np.nonzero(env_x >= self.sensor_x)
            env_x = env_x[idx]
            env_y = env_y[idx]

        elif  sensor_angle >= 0 and sensor_angle < 90 :
            slope = np.tan(theta * np.pi / 180)
            y_intersect = self.sensor_y - slope * self.sensor_x
            x_max = min((self.y_dim - y_intersect)/slope, self.x_dim)
            x_range = np.linspace(self.sensor_x, x_max, self.x_dim - self.sensor_x + 1)
            y_range = slope * x_range + y_intersect

        elif sensor_angle == 90: # facing straight up
            y_range = np.arange(self.sensor_y, self.y_dim + 1)
            x_range = self.sensor_x * np.ones(len(y_range))


        elif sensor_angle > 90 and sensor_angle <= 180:
            slope = np.tan(theta * np.pi / 180)
            y_intersect = self.sensor_y - slope * self.sensor_x
            x_min = max((self.y_dim - y_intersect)/slope, 1)
            x_range = np.linspace(x_min, self.sensor_x, self.sensor_x)   #debug
            y_range = slope * x_range + y_intersect
            idx_x = np.nonzero(env_x <= self.sensor_x)
            idx_y = np.nonzero(env_y >= self.sensor_y)
            idx = np.intersect1d(idx_x, idx_y)
            env_x = env_x[idx]
            env_y = env_y[idx]

        elif sensor_angle > 180 and sensor_angle < 270:
            slope = np.tan(theta * np.pi / 180)
            y_intersect = self.sensor_y - slope * self.sensor_x
            x_min = max((1 - y_intersect) / slope, 1)
            x_range = np.linspace(x_min, self.sensor_x, self.sensor_x)   #debug
            y_range = slope * x_range + y_intersect
            idx_x = np.nonzero(env_x <= self.sensor_x)
            idx_y = np.nonzero(env_y <= self.sensor_y)
            idx = np.intersect1d(idx_x, idx_y)
            env_x = env_x[idx]
            env_y = env_y[idx]

        elif sensor_angle == 270: # sensor facing straight down
            y_range = np.arange(1, self.sensor_y + 1)
            x_range = self.sensor_x * np.ones(len(y_range))

        else:
            slope = np.tan(theta * np.pi / 180)
            y_intersect = self.sensor_y - slope * self.sensor_x
            x_max = min((1 - y_intersect)/slope, self.x_dim)
            x_range = np.linspace(self.sensor_x, x_max, self.x_dim - self.sensor_x + 1)
            y_range = slope * x_range + y_intersect
            idx_x = np.nonzero(env_x >= self.sensor_x)
            idx_y = np.nonzero(env_y <= self.sensor_y)
            idx = np.intersect1d(idx_x, idx_y)
            env_x = env_x[idx]
            env_y = env_y[idx]

        intsct_xs, intsct_ys = [], []
        x_used  = []

        i = 0
        while i < len(x_range):
            cur_x = round(x_range[i])
            x_used.append(cur_x)
            # debug
            cur_y = round(y_range[i])
            x_idx_in_env = np.nonzero(env_x == cur_x)
            y_idx_in_env = np.nonzero(env_y == cur_y)
            x_in_env = env_x[x_idx_in_env]
            y_in_env = env_y[y_idx_in_env]

            if len(x_in_env) == 0 or len(y_in_env) == 0:
                i = i + 1
                continue

            x_diff = abs(x_in_env - cur_x)
            y_diff = abs(y_in_env - cur_y)

            # # use for test
            # print(len(x_diff))

            min_diff_x = min(x_diff)
            min_diff_y = min(y_diff)
            min_diff_x_idx = np.argmin(x_diff)
            min_diff_y_idx = np.argmin(y_diff)


            # diff_flag indicates whether the nearest index comes from x axis.
            if min_diff_x < min_diff_y:
                min_diff = min_diff_x
                min_diff_idx = min_diff_x_idx
                real_diff = x_in_env - cur_x
                diff_flag = 1
            else:
                min_diff = min_diff_y
                min_diff_idx = min_diff_y_idx
                real_diff = y_in_env - cur_y
                diff_flag = 0

            # ??
            step = round(real_diff[min_diff_idx]/10)
            if step > 0:
                i = i + step
            else:
                i = i + 1
                # if the difference is no more than 1, then we just include the current location.
                if min_diff <=1 :
                    # if the minimum is selected along x axis.
                    if diff_flag:
                        intsct_ys.append(cur_y)
                        intsct_xs.append(x_in_env[min_diff_x_idx])
                    # if the minimum is selected along y axis.
                    else:
                        intsct_ys.append(y_in_env[min_diff_y_idx])
                        intsct_xs.append(cur_x)
        dist = []
        for i in range(len(intsct_xs)):
            # compute the distance between sensor and detected object
            distance = sqrt((intsct_xs[i] - self.sensor_x) **2 + (intsct_ys[i] - self.sensor_y) ** 2)
            dist.append(distance)
            print(distance)

        dist = np.asarray(dist)

        closest_dist = min(dist)
        closest_idx = np.argmin(dist)

        # convert to np array and we can index with values returned by np.argmin or np.argmax.
        intsct_xs = np.asarray(intsct_xs)
        intsct_ys = np.asarray(intsct_ys)

        closest_intsct_x = intsct_xs[closest_idx]
        closest_intsct_y = intsct_ys[closest_idx]

        return closest_dist, closest_intsct_x, closest_intsct_y, x_range, y_range
















if __name__ == "__main__":

    # x = np.asarray([1, 2, 2 , 3, 5, 6])
    # idx = np.nonzero(x > 2)
    # print(np.argmin(x))
    # print(np.zeros(5))
    # x = np.asarray([1,2,3,4,5,6])
    # y = np.roll(x, 2)
    # print(y)

    filename = 'square_map.png'

    env, x_dim, y_dim = utils.read_env_from_img(filename)

    ss = sensor(env, x_dim, y_dim)
    ss.one_circle()


    # print(env[0])
    # print(set(env[1]))
    #
    # if 1:
    #     plt.figure()
    #     plt.scatter(env[0], env[1], c='k')
    #     # plt.scatter(sensor_x, sensor_y, '*r')
    #     plt.xlim([0, x_dim])
    #     plt.ylim([0, y_dim])
    #     plt.title('Environment created from figure and robot position')
    #     plt.xlabel('X')
    #     plt.ylabel('Y')
    #
    #     plt.show()
    #






