# -*- coding: utf-8 -*-
"""
Check the formula used to compute the depth map from a rotation, translation
and plane (parameterized by normal + distance).
"""

import cv2
from matplotlib import pyplot as plt
from math import tan, pi
import numpy as np

np.set_printoptions(suppress=True)


def skew(v):
    """Returns the skew-symmetric matrix of a vector"""
    return np.array([[0, -v[2], v[1]],
                    [v[2], 0, -v[0]],
                    [-v[1], v[0], 0]], dtype=np.float64)


def calibrationMatrixFromHFOV(hfov_deg, W, H):
    f = 0.5 * W / tan(0.5 * hfov_deg * pi / 180.0)
    K = np.array([[f, 0,  0.5 * W],
                  [0, f, 0.5 * H],
                  [0, 0, 1]]).astype(np.float64)
    K_inv = np.linalg.inv(K)
    return K, K_inv

    
def computeDepthmapAnalytic(R_01, t_01, n, d, K1, width, height):
    K1_inv = np.linalg.inv(K1)
    depth = np.zeros((height,width), dtype=np.float64)
    for x in range(width):
        for y in range(height):
            X1 = np.array([x,y,1]).reshape((3,1))
            X1 = K1_inv.dot(X1)
            
            z = -(d+n.T.dot(t_01))/(n.T.dot(R_01).dot(X1))
            depth[y,x] = z[0,0]
            
    return depth


if __name__ == "__main__":

    # Index 1 refers to cam (destination image)
    # Index 0 refers to world (source image)
    
    plt.close('all')
    
    img = cv2.imread('../textures/carpet.jpg', 0).astype(np.float32)
    img = img.astype(np.float32) / 255.0
    
    hfov_plane_deg = 130.0
    hfov_camera_deg = 90
    
    H0, W0 = img.shape
    K0, K0_inv = calibrationMatrixFromHFOV(hfov_plane_deg, W0, H0)
    H1, W1 = 260, 346
    K1, K1_inv = calibrationMatrixFromHFOV(hfov_camera_deg, W1, H1)
    
    K2, K2_inv = K1, K1_inv
    W2, H2 = W1, H1
    
    n = np.array([-0.12,-0.05,1.0]).reshape((3,1))
    n = n / np.linalg.norm(n)
    d = -1.0
    
    w_01 = (np.array([15.0, 5.0, -10.0]) * pi / 180.0).reshape((3,1)).astype(np.float64)
    t_01 = np.array([-1.0,0.4,-0.1]).reshape((3,1))
    R_01, _ = cv2.Rodrigues(w_01)
    
    R_10 = R_01.T
    t_10 = -R_01.T.dot(t_01)
    
    R = R_10
    t = t_10
    C = -R.T.dot(t)
    Hn_10 = R-1/d*t.dot(n.T)
    
    Hn_01 = np.linalg.inv(Hn_10)
    Hn_01_analytic = (np.eye(3) - 1.0/(d+n.T.dot(C))*C.dot(n.T)).dot(R.T) # analytic inverse
    
    print('Test analytic inverse of H_01: {}'.format(np.allclose(Hn_01, Hn_01_analytic)))
    
    H_10 = K1.dot(Hn_10).dot(K0_inv)
    H_01 = np.linalg.inv(H_10)
    
    warped = cv2.warpPerspective(img, H_01, dsize=(W1,H1), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
    depth = computeDepthmapAnalytic(R_01, t_01, n, d, K1, W1, H1)
    
    plt.figure()
    plt.imshow(depth)
    plt.colorbar()
    plt.title('Analytical depth map')

    x1,y1 = np.random.randint(0,W1), np.random.randint(0,H1)
    X1 = np.array([x1,y1,1]).reshape((3,1))
    X = K1_inv.dot(X1)
    
    X0 = H_01.dot(X1)
    X0[...] /= X0[2]
    
    plt.figure()
    plt.subplot(121)
    plt.imshow(warped, cmap='gray')
    plt.scatter(X1[0], X1[1])
    plt.title('Warped image')
    
    plt.subplot(122)
    plt.imshow(img, cmap='gray')
    plt.scatter(X0[0], X0[1], color='b')
    plt.title('Source image')    
    
    # Check that the predicted depth indeed works to project X1 on image 0
    z1 = depth[y1,x1]
    P1 = z1 * K1_inv.dot(X1)
    P0 = R_01.dot(P1) + t_01
    X0_depth = K0.dot(P0)
    X0_depth[...] /= X0_depth[2]
    
    print('Test reprojection with analytical depth: {}'.format(np.allclose(X0, X0_depth)))
    