import math, time, pickle, os
import cv2
import scipy.optimize
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from scipy import stats
import argparse
import os
import scipy.spatial.distance as distance
from scipy.optimize import curve_fit

def display_npy(filename, point_cloud=False, obj=True):
    """
    Displays the numpy files

    Parameters
    ----------
    filename : str
        The name of the file to display
    """
    bound = 400
    ts = []
    xs = []
    ys = []
    zs = []
    result = []
    with open(filename, "r") as f:
        for i, line in enumerate(f):
            x, y, z, _, _, _ = line.split(",")
            x, y, z = float(x), float(y), float(z)
            result.append([x, y, z])
            ts.append(i)
            xs.append(x)
            ys.append(y)
            zs.append(z)
    npy = np.array(result)
    # print(xs)
    def func(t, a, b, c):
        return a * t **2 + b * t + c

    approx = []
    for i in range(len(ts)):
        x_popt = curve_fit(func, 
                           ts[max(0, i - 2) : min(len(ts), i + 3)], 
                           xs[max(0, i - 2) : min(len(ts), i + 3)])[0]
        y_popt = curve_fit(func, 
                           ts[max(0, i - 2) : min(len(ts), i + 3)], 
                           ys[max(0, i - 2) : min(len(ts), i + 3)])[0]
        z_popt = curve_fit(func, 
                           ts[max(0, i - 2) : min(len(ts), i + 3)], 
                           zs[max(0, i - 2) : min(len(ts), i + 3)])[0]

        approx.append([func(ts[i], *x_popt), func(ts[i], *y_popt), func(ts[i], *z_popt)])
        
    # print(approx_x)
    npy = np.array(approx)
    # print(e_result)
    last_dim = npy.shape[-1]

    if point_cloud:
        fig = plt.figure(num='point cloud')
        ax = fig.add_axes([0, 0, 1, 1], projection='3d')
        ax.set_autoscale_on(False)

        ax.view_init(30, -45)
        ax.set_xlabel('x')
        ax.set_xlim3d(-bound, bound)
        ax.set_ylabel('z')
        if obj:
            ax.set_ylim3d(-bound, bound)
        else:
            ax.set_ylim3d(bound, 3 * bound)
        ax.set_zlabel('y')
        ax.set_zlim3d(-20, 20)
        xyz = np.reshape(npy, (-1, 3))
        sct, = ax.plot(xyz[:,0], xyz[:,2], -xyz[:,1], 'o', markersize=0.2, color='b')

        # print(distance.pdist(xyz).max())
        
        plt.show()
    else:
        npy = npy - np.min(npy)
        npy = npy / np.max(npy)
        plt.imshow(npy)
        plt.show()

def display_png(filename):
    """
    Displays the png files

    Parameters
    ----------
    filename : str
        The name of the file to display
    """
    im = mpimg.imread(filename + '.png')
    if len(im.shape) < 3:
        plt.imshow(im, cmap='gray', vmin=0, vmax=np.max(im))
    else:
        plt.imshow(cv2.cvtColor(im, cv2.COLOR_RGB2BGR))
    plt.show()

def display_ply(filename):
    """
    Displays the ply files

    Parameters
    ----------
    filename : str
        The name of the file to display
    """
    pcl = PyntCloud.from_file(filename + '.ply')
    pcl.plot()

# if _name__=='__main_':

    """
    Example command line arguments

    python display_file.py -test -t 44 -s noshape -d gray    Displays test_images/gray_gp44_noshape.png
    python display_file.py -n 10 -f 0 -d npy                 Displays realsense_output/10/000000_npy.npy
    python display_file.py -n 10 -f obj -d obj               Displays realsense_output/10/obj_obj.npy
    """

parser = argparse.ArgumentParser()
parser.add_argument('-test', action='store_true', default=False, help=
    'Show test images. Default: False'
)
parser.add_argument('-num', '-n', type=int, default=0, help=
    'Clip number. Default: 0'
)
parser.add_argument('-frame', '-f', type=str, default='undeformed', help=
    'Frame name. Options: undeformed, obj, any number. Default: undeformed'
)
parser.add_argument('-throw', '-t', type=int, default=44, help=
    'Throw value. Options: 44, 20, 01. Default: 44'
)
parser.add_argument('-shape', '-s', type=str, default='noshape', help=
    'Options: noshape, square, rectangle, triangle, circle. Default: noshape'
)
parser.add_argument('-display', '-d', type=str, default='pcl', help=
    'Options: gray, color, depth, normals, pcl, obj, ply. Default: color'
)
args = parser.parse_args()

if args.test:
    file_directory = 'test_images/'
    if args.display == 'gray':
        display_png(file_directory + ('gray_gp%02d_%s' % (args.throw, args.shape)))
    else:   
        display_npy(file_directory + ('depth_gp%02d_%s' % (args.throw, args.shape)) )   
else:
    if args.frame.isdigit():
        file_directory = 'realsense_output/%d/%06d' % (args.num, int(args.frame))
    else:
        file_directory = 'realsense_output/%d/%s' % (args.num, args.frame)

    if args.display == 'color':
        display_png(file_directory + '_color')
    elif args.display == 'depth':
        display_png(file_directory + '_depth')
    elif args.display == 'normals':
        display_npy(file_directory + '_normals')
    elif args.display == 'pcl':
        display_npy('ROAR_Sim/data/easy_map_waypoints.txt', point_cloud=True)
    elif args.display == 'obj':
        display_npy(file_directory + '_obj', point_cloud=True, obj=True)
    else:
        display_ply(file_directory + '_ply')