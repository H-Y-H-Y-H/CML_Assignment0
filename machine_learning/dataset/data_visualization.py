import numpy as np
import math
import cv2


def draw_angled_rec(loc, img, color, thickness):
    y0, x0, width, height, angle = loc
    b = math.cos(angle) * 0.5
    a = math.sin(angle) * 0.5
    pt0 = (int(x0 - a * height - b * width),
           int(y0 + b * height - a * width))
    pt1 = (int(x0 + a * height - b * width),
           int(y0 - b * height - a * width))
    pt2 = (int(2 * x0 - pt0[0]), int(2 * y0 - pt0[1]))
    pt3 = (int(2 * x0 - pt1[0]), int(2 * y0 - pt1[1]))

    cv2.line(img, pt0, pt1, color, thickness)
    cv2.line(img, pt1, pt2, color, thickness)
    cv2.line(img, pt2, pt3, color, thickness)
    cv2.line(img, pt3, pt0, color, thickness)


def map_to_img(data_input):
    default_scale = 530 / 0.34

    data_input[0] = data_input[0] * default_scale
    data_input[1] = data_input[1] * default_scale + 320
    data_input[2:4] *= default_scale
    data_input[4] -= 1.57
    data_input[4] *= -1
    return data_input


if __name__ == "__main__":

    objects_num = 15  # from 4-15
    label_or_input = 'label'  # or "label"
    dataset = np.loadtxt('%s/num_%d.txt' % (label_or_input, objects_num))

    show_example_id = 0
    sample0 = dataset[show_example_id]
    bg = np.ones((480, 640, 3))
    color = (0, 0, 0)
    thickness = 3
    objects_num = len(sample0) // 5

    for i in range(objects_num):
        loc = sample0[i * 5:i * 5 + 5]
        loc = map_to_img(loc)
        draw_angled_rec(loc, bg, color, thickness)

    cv2.imshow('Image', bg)
    cv2.waitKey(0)
