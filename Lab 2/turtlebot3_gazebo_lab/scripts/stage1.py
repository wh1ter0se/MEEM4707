#!/usr/bin/env python
# license removed for brevity

import math
import numpy as np
import matplotlib.pyplot as plt
import sys
from path_planning import *

def stage1(est_pos,X_map,Y_map):

	##############@TODO############
	rx,ry=path(est_pos,X_map,Y_map)

	#write your code below to generate v and w to follow the path rx,ry
	v_cmd=0
	w_cmd=0

	###############################
	return v_cmd,w_cmd
