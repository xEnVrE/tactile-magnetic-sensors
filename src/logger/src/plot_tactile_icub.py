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
    # user inputs the finger name
    finger_name = sys.argv[1]
    if finger_name == "index":
        sens_id = 0
    elif finger_name == "middle":
        sens_id = 1
    elif finger_name == "ring":
        sens_id = 2
    elif finger_name == "little":
        sens_id = 3
    elif finger_name == "thumb":
        sens_id = 4

    # data
    time = read_data(prefix, "time")

    # data
    data = read_data(prefix, "tactile_comp")
    t_0 = data[:,   0 + (sens_id * 12)]
    t_1 = data[:,   1 + (sens_id * 12)]
    t_2 = data[:,   2 + (sens_id * 12)]
    t_3 = data[:,   3 + (sens_id * 12)]
    t_4 = data[:,   4 + (sens_id * 12)]
    t_5 = data[:,   5 + (sens_id * 12)]
    t_6 = data[:,   6 + (sens_id * 12)]
    t_7 = data[:,   7 + (sens_id * 12)]
    t_8 = data[:,   8 + (sens_id * 12)]
    t_9 = data[:,   9 + (sens_id * 12)]
    t_10 = data[:, 10 + (sens_id * 12)]
    t_11 = data[:, 11 + (sens_id * 12)]

    # make plot
    fig, ax = plt.subplots(2,6)

    t_0_plot, = plot_y(ax[0, 0], time, t_0, "t0")
    ax[0, 0].legend(handles = [t_0_plot])

    t_1_plot, = plot_y(ax[0, 1], time, t_1, "t1")
    ax[0, 1].legend(handles = [t_1_plot])

    t_2_plot, = plot_y(ax[0, 2], time, t_2, "t2")
    ax[0, 2].legend(handles = [t_2_plot])

    t_3_plot, = plot_y(ax[0, 3], time, t_3, "t3")
    ax[0, 3].legend(handles = [t_3_plot])

    t_4_plot, = plot_y(ax[0, 4], time, t_4, "t4")
    ax[0, 4].legend(handles = [t_4_plot])

    t_5_plot, = plot_y(ax[0, 5], time, t_5, "t5")
    ax[0, 5].legend(handles = [t_5_plot])

    t_6_plot, = plot_y(ax[1, 0], time, t_6, "t6")
    ax[1, 0].legend(handles = [t_6_plot])

    t_7_plot, = plot_y(ax[1, 1], time, t_7, "t7")
    ax[1, 1].legend(handles = [t_7_plot])

    t_8_plot, = plot_y(ax[1, 2], time, t_8, "t8")
    ax[1, 2].legend(handles = [t_8_plot])

    t_9_plot, = plot_y(ax[1, 3], time, t_9, "t9")
    ax[1, 3].legend(handles = [t_9_plot])

    t_10_plot, = plot_y(ax[1, 4], time, t_10, "t10")
    ax[1, 4].legend(handles = [t_10_plot])

    t_11_plot, = plot_y(ax[1, 5], time, t_11, "t11")
    ax[1, 5].legend(handles = [t_11_plot])

    plt.show()

if __name__ == "__main__":
    main()
