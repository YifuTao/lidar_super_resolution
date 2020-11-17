#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
# SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
#
# newercollege_lidar_angles.py - Spit out the elevation and azimuth angles to feed into supereight

from typing import List



_width = 1024

_elevation_angles = [
         17.74400,  17.12000,  16.53600,  15.98200,  15.53000,  14.93600,  14.37300,  13.82300,
         13.37300,  12.78600,  12.23000,  11.68700,  11.24100,  10.67000,  10.13200,   9.57400,
          9.13800,   8.57700,   8.02300,   7.47900,   7.04600,   6.48100,   5.94400,   5.39500,
          4.96300,   4.40100,   3.85900,   3.31900,   2.87100,   2.32400,   1.78300,   1.23800,
          0.78600,   0.24500,  -0.29900,  -0.84900,  -1.28800,  -1.84100,  -2.27500,  -2.92600,
         -3.37800,  -3.91000,  -4.45700,  -5.00400,  -5.46000,  -6.00200,  -6.53700,  -7.09600,
         -7.55200,  -8.09000,  -8.62900,  -9.19600,  -9.65700, -10.18300, -10.73200, -11.28900,
        -11.77000, -12.29700, -12.85400, -13.41500, -13.91600, -14.44200, -14.99700, -15.59500]

_azimuth_angle_offsets = [
        3.10200, 0.9290, -1.2370, -3.37800, 3.06300, 0.9120, -1.2170, -3.33200,
        3.04500, 0.9220, -1.1850, -3.28500, 3.04200, 0.9340, -1.1670, -3.25400,
        3.03000, 0.9420, -1.1390, -3.22700, 3.03400, 0.9580, -1.1210, -3.18600,
        3.04800, 0.9840, -1.0840, -3.15800, 3.05900, 0.9980, -1.0660, -3.13400,
        3.08500, 1.0270, -1.0420, -3.11100, 3.11700, 1.0500, -1.0660, -3.07100,
        3.14900, 1.0800, -0.9890, -3.05800, 3.19000, 1.1130, -0.9580, -3.03700,
        3.23000, 1.1440, -0.9310, -3.02700, 3.27400, 1.1900, -0.8990, -3.02200,
        3.32700, 1.2360, -0.8810, -3.00600, 3.39300, 1.2750, -0.8600, -3.00400]

_azimuth_pixel_offsets = 16 * [0, 6, 12, 18]



def generate_azimuth_angles(): #-> List[float]:
    offset = 360.0 / _width
    counter_offsets = [offset * x for x in _azimuth_pixel_offsets]
    return [x + y for x, y in zip(_azimuth_angle_offsets, counter_offsets)]



# def to_eigen_variable(coeffs: List[float], var_name: str): #-> str:
def to_eigen_variable(coeffs, var_name):

    s = 'const Eigen::VectorXf ' + var_name + ' = (Eigen::VectorXf(' + str(len(coeffs)) + ') << '
    c = ', '.join([str(x) for x in coeffs])
    return s + c + ').finished();'



if __name__ == "__main__":
    try:
        print(to_eigen_variable(_elevation_angles, 'elevation_angles'))
        print(to_eigen_variable(generate_azimuth_angles(), 'azimuth_angles'))
    except KeyboardInterrupt:
        pass

