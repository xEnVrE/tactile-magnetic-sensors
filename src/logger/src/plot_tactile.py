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
    # user inputs the finger number starting from 1
    sens_id = int(sys.argv[1]) - 1

    # data
    time = read_data(prefix, "time")

    # data
    data = read_data(prefix, "tactile_3d")
    x_1 = data[:, 0 + (sens_id * 2) * 3]
    y_1 = data[:, 1 + (sens_id * 2) * 3]
    z_1 = data[:, 2 + (sens_id * 2) * 3]

    x_2 = data[:, 0 + ((sens_id * 2) + 1) * 3]
    y_2 = data[:, 1 + ((sens_id * 2) + 1) * 3]
    z_2 = data[:, 2 + ((sens_id * 2) + 1) * 3]

    # make plot
    fig, ax = plt.subplots(2,3)

    x_1_plot, = plot_y(ax[0, 0], time, x_1, "x sens 1")
    ax[0, 0].legend(handles = [x_1_plot])

    y_1_plot, = plot_y(ax[0, 1], time, y_1, "y sens 1")
    ax[0, 1].legend(handles = [y_1_plot])

    z_1_plot, = plot_y(ax[0, 2], time, z_1, "z sens 1")
    ax[0, 2].legend(handles = [z_1_plot])

    x_2_plot, = plot_y(ax[1, 0], time, x_2, "x sens 2")
    ax[1, 0].legend(handles = [x_2_plot])

    y_2_plot, = plot_y(ax[1, 1], time, y_2, "y sens 2")
    ax[1, 1].legend(handles = [y_2_plot])

    z_2_plot, = plot_y(ax[1, 2], time, z_2, "z sens 2")
    ax[1, 2].legend(handles = [z_2_plot])

    plt.show()

if __name__ == "__main__":
    main()
