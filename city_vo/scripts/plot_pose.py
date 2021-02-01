'''
    plot poses via reading from poses_vo.csv
'''


__author__ = "Masoud S. Bahraini"
__copyright__ = "Copyright 2021, City, University of London"
__credits__ = ["Masoud S. Bahraini"]
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "Masoud S. Bahraini"
__email__ = "sotoodeh.bahraini@city.ac.uk"
__status__ = "Production"


import csv
from collections import defaultdict
import numpy as np
import math as m
import matplotlib.pyplot as plt

columns = defaultdict(list)  # each value in each column is appended to a list
# each value in each column is appended to a list
columns_gt = defaultdict(list)
columns_vo_FM = defaultdict(list)


def main():
    with open('/home/ros/Documents/eem404_bag/pose_eem_vo_cg41.csv', 'r') as csvfile:
        # read rows into a dictionary format
        csv_reader = csv.DictReader(csvfile)
        # read a row as {column1: value1, column2: value2,...}
        for row in csv_reader:
            for (k, v) in row.items():  # go over each column name and value
                # append the value into the appropriate list based on column name k
                columns[k].append(v)
        x = list(columns['pose_z'])
        x_floats = []
        for item in x:
            x_floats.append(float(item))
        y = list(columns['pose_x'])
        y_floats = []
        for item in y:
            y_floats.append(float(item))
    vo = np.array(np.column_stack((x_floats, y_floats)))
    with open('vo_poses.csv', 'r') as csvfile:
        # read rows into a dictionary format
        csv_reader = csv.DictReader(csvfile)
        # read a row as {column1: value1, column2: value2,...}
        for row in csv_reader:
            for (k, v) in row.items():  # go over each column name and value
                # append the value into the appropriate list based on column name k
                columns_vo_FM[k].append(v)
        x = list(columns_vo_FM['pose_z'])
        x_floats = []
        for item in x:
            x_floats.append(float(item))
        y = list(columns_vo_FM['pose_x'])
        y_floats = []
        for item in y:
            y_floats.append(float(item))
    vo_FM = np.array(np.column_stack((x_floats, y_floats)))
    with open('/home/ros/Documents/eem404_bag/zed_node-pose.csv', 'r') as csvfile:
        # read rows into a dictionary format
        csv_reader = csv.DictReader(csvfile)
        # read a row as {column1: value1, column2: value2,...}
        for row in csv_reader:
            for (k, v) in row.items():  # go over each column name and value
                # append the value into the appropriate list based on column name k
                columns_gt[k].append(v)
        x = list(columns_gt['.pose.position.x'])
        x_floats = []
        for item in x:
            x_floats.append(float(item))
        y = list(columns_gt['.pose.position.y'])
        y_floats = []
        for item in y:
            y_floats.append(float(item))
    gt = np.array(np.column_stack((x_floats, y_floats)))
    plt.plot(-vo[:, 0], -vo[:, 1], label='VO', linewidth=2.0, color='red')
    plt.plot(-vo_FM[:, 0], -vo_FM[:, 1], label='VO_FM',
             linewidth=2.0, color='blue')
    plt.plot(gt[:, 0], gt[:, 1], label='GT', linewidth=2.0,
             color='black', linestyle='dashed')
    plt.axis('equal')
    plt.xlabel('x(m)')
    plt.ylabel('y(m)')
    plt.title('GT-CG41')
    plt.legend(loc='upper center', shadow=True, fontsize='x-large')
    plt.show()


if __name__ == "__main__":
    main()
