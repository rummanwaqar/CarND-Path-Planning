#!/usr/bin/env python

import sys
import csv
import math
import matplotlib.pyplot as plt


def read_csv(filename):
    data = []
    with open(filename, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=' ')
        for row in csv_reader:
            data.append([float(i) for i in row])
    return data


def get_xy(s, d, map):
    map_x = [i[0] for i in map]
    map_y = [i[1] for i in map]
    map_s = [i[2] for i in map]
    prev_wp = -1
    while map_s[prev_wp] < s < len(map_s) - 1:
        prev_wp += 1
    next_wp = (prev_wp + 1) % len(map_s)
    heading_wp = math.atan2(map_y[next_wp] - map_y[prev_wp],
                            map_x[next_wp] - map_y[prev_wp])

    seg_s = s - map_s[prev_wp]
    seg_x = map_x[prev_wp] + seg_s * math.cos(heading_wp)
    seg_y = map_y[prev_wp] + seg_s * math.sin(heading_wp)

    perp_heading = heading_wp - math.pi / 2
    x = seg_x + d * math.cos(perp_heading)
    y = seg_y + d * math.sin(perp_heading)
    return x,y


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage ./plot_map.py map.csv')
        exit(0)

    highway = read_csv(sys.argv[1])
    x = [i[0] for i in highway]
    y = [i[1] for i in highway]
    i = [i for i in range(len(highway))]
    plt.plot(x, y)
    plt.scatter(x, y, c=i, s=10)
    plt.colorbar().set_label('Time')
    plt.show()
