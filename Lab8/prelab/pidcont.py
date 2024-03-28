#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np


def pid_controller(
    d,  # error
    ed,  # last error
    edi,  # integral accumulator value
    kP=0,
    kI=0,
    kD=0,
):

    kp = kP
    ki = kI
    kd = kD

    edi = edi + ed  # error integration

    # calculates angular speed clamped to [-1.86,1.86]
    w = max(
        min(
            1.86,
            kp * d + ki * edi + kd * (d - ed),
        ),
        -1.86,
    )

    # calculates velocity as an exponential function of angular speed (for non-zero radius turning?)
    v = 0.15 * np.exp(-100 * abs(w))  # Max speed set as 0.15

    ed = d  # previous error

    # print(d, w, v)
    return v, w, ed, edi
    #######################################################################################
