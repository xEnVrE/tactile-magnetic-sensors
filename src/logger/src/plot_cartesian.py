import matplotlib.pyplot as plt
import csv
import numpy as np
import math
import sys

def read_data(prefix, postfix):

    data = []

    with open(prefix + '_' + postfix + '.txt', newline='') as csv_data:
        for row in csv_data:
            data.append([float(num_string.rstrip()) for num_string in row.split(sep = " ") if num_string != ''])

    return np.array(data)

def plot_y(axes, x_data, y_data, label_txt):

    return axes.plot(x_data, y_data, label=label_txt)

def main():
    prefix = "data"

    # data
    time = read_data(prefix, "time")

    # data
    data = read_data(prefix, "right_arm_state")
    x_1 = data[:, 0]
    y_1 = data[:, 1]
    z_1 = data[:, 2]

    print(x_1)

    # make plot
    fig, ax = plt.subplots(1,3)

    x_1_plot, = plot_y(ax[0], time, x_1, "x")
    ax[0].legend(handles = [x_1_plot])

    y_1_plot, = plot_y(ax[1], time, y_1, "y")
    ax[1].legend(handles = [y_1_plot])

    z_1_plot, = plot_y(ax[2], time, z_1, "z")
    ax[2].legend(handles = [z_1_plot])

    plt.show()

if __name__ == "__main__":
    main()
