#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import skimage
import numpy as np


def histogram(image, bins, min_range=0, max_range=255):
    if len(image.shape) == 3:
        h1, e1 = histogram(image[:, :, 0], bins)
        h2, e2 = histogram(image[:, :, 1], bins)
        h3, e3 = histogram(image[:, :, 2], bins)
        h = np.concatenate((h1, h2, h3))
        e = np.concatenate((e1, e2, e3))
    else:
        h, e = np.histogram(image.ravel(), bins, (min_range, max_range))
    return h, e


def spatial_histogram(image, w_cells, h_cells, bins, min_range=0, max_range=255):
    w_step = image.shape[0]//w_cells
    h_step = image.shape[1]//h_cells
    sp_hist = np.array([])
    for w in range(w_cells):
        for h in range(h_cells):
            b = image[h * h_step:(h + 1)*h_step, w * w_step:(w + 1)*w_step, :]
            h, _ = histogram(b, bins, min_range, max_range)
            sp_hist = np.concatenate((sp_hist, h))
    return sp_hist


def spatial_histogram_bw(image, w_cells, h_cells, bins, min_range=0, max_range=255):
    w_step = image.shape[0]//w_cells
    h_step = image.shape[1]//h_cells
    sp_hist = np.array([])
    for w in range(w_cells):
        for h in range(h_cells):
            b = image[h * h_step:(h + 1)*h_step, w * w_step:(w + 1)*w_step]
            h, _ = histogram(b, bins, min_range, max_range)
            sp_hist = np.concatenate((sp_hist, h))
    return sp_hist


def spatial_histogram_hsv(image, w_cells, h_cells, bins):
    image = bgr_to_hsv(image)
    w_step = image.shape[0]//w_cells
    h_step = image.shape[1]//h_cells
    sp_hist = np.array([])
    for w in range(w_cells):
        for h in range(h_cells):
            b = image[h * h_step:(h + 1)*h_step, w * w_step:(w + 1)*w_step, :]
            h1, _ = histogram(b[:, :, 0], bins, 0, 179)
            h2, _ = histogram(b[:, :, 1], bins, 0, 255)
            h3, _ = histogram(b[:, :, 2], bins, 0, 255)
            h = np.concatenate((h1, h2, h3))
            sp_hist = np.concatenate((sp_hist, h))
    return sp_hist


def spatial_nbhs(image, w_cells, h_cells):
    image = bgr_to_hsv(image)
    w_step = image.shape[0]//w_cells
    h_step = image.shape[1]//h_cells
    sp_hist = np.array([])
    for w in range(w_cells):
        for h in range(h_cells):
            b = image[h * h_step:(h + 1)*h_step, w * w_step:(w + 1)*w_step, :]
            h1, _ = histogram(b[:, :, 0], 36, 0, 179)
            h2, _ = histogram(b[:, :, 1], 32, 0, 255)
            h = np.concatenate((h1, h2))
            sp_hist = np.concatenate((sp_hist, h))
    return sp_hist


def spatial_histogram_luv(image, w_cells, h_cells, bins):
    image = bgr_to_luv(image)
    w_step = image.shape[0]//w_cells
    h_step = image.shape[1]//h_cells
    sp_hist = np.array([])
    for w in range(w_cells):
        for h in range(h_cells):
            b = image[h * h_step:(h + 1)*h_step, w * w_step:(w + 1)*w_step, :]
            h1, _ = histogram(b[:, :, 0], bins, 0, 100)
            h2, _ = histogram(b[:, :, 1], bins, -134, 220)
            h3, _ = histogram(b[:, :, 2], bins, -140, 122)
            h = np.concatenate((h1, h2, h3))
            sp_hist = np.concatenate((sp_hist, h))
    return sp_hist


def spatial_histogram_ohta(image, w_cells, h_cells, bins):
    image = bgr_to_ohta(image)
    w_step = image.shape[0]//w_cells
    h_step = image.shape[1]//h_cells
    sp_hist = np.array([])
    for w in range(w_cells):
        for h in range(h_cells):
            b = image[h * h_step:(h + 1)*h_step, w * w_step:(w + 1)*w_step, :]
            h1, _ = histogram(b[:, :, 0], bins, 0, 255)
            h2, _ = histogram(b[:, :, 1], bins, -128, 128)
            h3, _ = histogram(b[:, :, 2], bins, -128, 128)
            h = np.concatenate((h1, h2, h3))
            sp_hist = np.concatenate((sp_hist, h))
    return sp_hist


def bgr_to_hsv(bgr_image):
    return cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)


def bgr_to_luv(bgr_image):
    return cv2.cvtColor(bgr_image, cv2.COLOR_BGR2LUV)


def bgr_to_ohta(bgr_image):
    I1 = np.sum(bgr_image, axis=2)//3
    I2 = np.subtract(bgr_image[:, :, 2], bgr_image[:, :, 0])//2
    I3 = (-np.sum(bgr_image[:, :, [0, 2]], axis=2) + 2*bgr_image[:, :, 1])//4
    return np.dstack((I1, I2, I3))


if __name__ == "__main__":
    c = np.random.randint(0, 255, [100, 100])
    a = np.dstack((c, c, c)).astype(np.uint8)
    g = bgr_to_ohta(a)
    print(g)
    print(g.shape)
    # b = spatial_histogram(a, 4,4,8)
