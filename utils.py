import cv2
import matplotlib.pyplot as plt
import numpy as np

def read_env_from_img(img_file):
    gray_img = cv2.imread(img_file, 0)  #read as gray scale image
    gray_img[gray_img < 122] = 0 # dark pixel to 0
    gray_img[gray_img > 123] = 255 # bright pixel to 255

    gray_img = np.flipud(gray_img)
    y_dim, x_dim = gray_img.shape

    valid_x, valid_y = [], []
    for i in range(y_dim):
        for j in range(x_dim):
            if gray_img[i][j] == 0:
                valid_x.append(j)
                valid_y.append(i)
    valid_x = np.asarray(valid_x)
    valid_y = np.asarray(valid_y)
    env = (valid_x, valid_y)
    return env, x_dim, y_dim


# img_file = 'square_map.png'
#
# x, y = read_env_from_img(img_file)
# print(set(x))
# print(set(y))

if __name__ == '__main__':
    img_file = 'square_map.png'

    env, x, y = read_env_from_img(img_file)

    print(len(env[0]))
