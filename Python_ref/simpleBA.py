from __future__ import print_function
import urllib
import bz2
import os
import numpy as np
import math
from scipy.sparse import lil_matrix
import time
from scipy.optimize import least_squares
#%matplotlib inline
import matplotlib.pyplot as plt

'''
BASE_URL = "http://grail.cs.washington.edu/projects/bal/data/ladybug/"
FILE_NAME = "problem-49-7776-pre.txt.bz2"
URL = BASE_URL + FILE_NAME

if not os.path.isfile(FILE_NAME):
    urllib.request.urlretrieve(URL, FILE_NAME)
'''

#FILE_NAME = "../data/problem-49-7776-pre.txt"
#FILE_NAME = "../data/problem-16-22106-pre.txt"
FILE_NAME = "../data/toy_ex_test_data.txt"


##### Parameter setting start #####
RANDOM_SAVE = 'LOAD'; # 'SAVE' for random variable gen & save, 'LOAD' for random variable fixe & load
MODE = 'LM' # 'LM' for Levenberg-Marquardt method, 'GN' for Gauss-Newton method
#NPOSES = 4; # fix this for now
#NPTS = 50;
NUM_ITERATIONS = 30


# generate noisy initial guess poses
ROTATION_NOISE_STD = 0.7/180 * math.pi
POSITION_NOISE_STD = 0.5;

# generate noisy initial guess points
POINT_STD = [0.01, 0.01, 0.01]

# std deviation on image noise
FOCAL_LENGTH = 500
IMAGE_NOISE_STD = 0.3 / FOCAL_LENGTH

OUTLIER_PROB = 0.1;    # probability of a _bad_ outlier
OUTLIER_IMAGE_NOISE_STD = 30 / FOCAL_LENGTH
##### Parameter setting end #####





def read_bal_data(file_name):
    file = open(file_name, 'r');
    n_cameras, n_points, n_observations = map(int, file.readline().split())

    camera_indices = np.empty(n_observations, dtype=int)
    point_indices = np.empty(n_observations, dtype=int)
    points_2d = np.empty((n_observations, 2))

    for i in range(n_observations):
        camera_index, point_index, x, y = file.readline().split()
        camera_indices[i] = int(camera_index)
        point_indices[i] = int(point_index)
        points_2d[i] = [float(x), float(y)]

    camera_params = np.empty(n_cameras * 9)
    for i in range(n_cameras * 9):
        camera_params[i] = float(file.readline())
    camera_params = camera_params.reshape((n_cameras, -1))

    points_3d = np.empty(n_points * 3)
    for i in range(n_points * 3):
        points_3d[i] = float(file.readline())
    points_3d = points_3d.reshape((n_points, -1))

    return camera_params, points_3d, camera_indices, point_indices, points_2d


def rotate(points, rot_vecs):
    """Rotate points by given rotation vectors.
    
    Rodrigues' rotation formula is used.
    """
    theta = np.linalg.norm(rot_vecs, axis=1)[:, np.newaxis]
    with np.errstate(invalid='ignore'):
        v = rot_vecs / theta
        v = np.nan_to_num(v)
    dot = np.sum(points * v, axis=1)[:, np.newaxis]
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    return cos_theta * points + sin_theta * np.cross(v, points) + dot * (1 - cos_theta) * v


def project(points, camera_params):
    """Convert 3-D points to 2-D by projecting onto images."""
    points_proj = rotate(points, camera_params[:, :3])
    points_proj += camera_params[:, 3:6]
    points_proj = -points_proj[:, :2] / points_proj[:, 2, np.newaxis]
    f = camera_params[:, 6]
    k1 = camera_params[:, 7]
    k2 = camera_params[:, 8]
    n = np.sum(points_proj**2, axis=1)
    r = 1 + k1 * n + k2 * n**2
    points_proj *= (r * f)[:, np.newaxis]
    return points_proj


def fun(params, n_cameras, n_points, camera_indices, point_indices, points_2d):
    """Compute residuals.
    
    `params` contains camera parameters and 3-D coordinates.
    """
    camera_params = params[:n_cameras * 9].reshape((n_cameras, 9))
    points_3d = params[n_cameras * 9:].reshape((n_points, 3))
    points_proj = project(points_3d[point_indices], camera_params[camera_indices])
    return (points_proj - points_2d).ravel()


def bundle_adjustment_sparsity(n_cameras, n_points, camera_indices, point_indices):
    m = camera_indices.size * 2
    n = n_cameras * 9 + n_points * 3
    A = lil_matrix((m, n), dtype=int)

    i = np.arange(camera_indices.size)
    for s in range(9):
        A[2 * i, camera_indices * 9 + s] = 1
        A[2 * i + 1, camera_indices * 9 + s] = 1

    for s in range(3):
        A[2 * i, n_cameras * 9 + point_indices * 3 + s] = 1
        A[2 * i + 1, n_cameras * 9 + point_indices * 3 + s] = 1

    return A




cam_pose_estimate, points_world_estimate, camera_indices, point_indices, points_image_noisy = read_bal_data(FILE_NAME)

NPOSES = cam_pose_estimate.shape[0]
NPTS = points_world_estimate.shape[0]
NOBS = points_image_noisy.shape[0]

print("NPOSES: {}".format(NPOSES))
print("NPTS: {}".format(NPTS))
print("NOBS: {}".format(NOBS))

print("cam_pose_estimate: {}".format(cam_pose_estimate.shape))
print("points_world_estimate: {}".format(points_world_estimate.shape))
print("points_image_noisy: {}".format(points_image_noisy.shape))

numJRows = 2*NPTS*NPOSES;
numJCols = NPTS*3+NPOSES*6;
J = np.empty((numJRows, numJCols), dtype=float)
