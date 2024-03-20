from collections import deque
import time
import numpy as np
import matplotlib.pyplot as plt

# Parameters
Zeta = 10.0  # attractive potential gain
Eta = 100.0  # repulsive potential gain
q_star = 2  # additional distance from the obstacles
show_animation = True
OSCILLATIONS_DETECTION_LENGTH = 3


def get_motion_model():
    # dx, dy
    motion = [
        [1, 0],
        [0, 1],
        [-1, 0],
        [0, -1],
        [-1, -1],
        [-1, 1],
        [1, -1],
        [1, 1],
    ]
    return motion


def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH:
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False


def draw_env(ox, oy, rr, xw, yw):
    plt.axis("equal")
    ax = plt.gca()
    ax.set_xlim((0, xw))
    ax.set_ylim((0, yw))
    for j in range(len(ox)):
        circle = plt.Circle((ox[j], oy[j]), rr[j])
        ax.add_patch(circle)


def cal_attractive_potential(x, y, gx, gy):
    d = np.hypot(x - gx, y - gy)
    return 0.5 * Zeta * d**2


def cal_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i]) - rr[i]
    if dmin >= d:
        dmin = d
        minid = i

    # calculate repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid]) - rr[minid]
    if dq <= q_star:
        if dq <= 0.1:
            dq = 0.1
        return 0.5 * Eta * (1.0 / dq - 1.0 / q_star) ** 2
    else:
        return 0.0


def cal_potential_field(gx, gy, ox, oy, rr, sx, sy, xw, yw):
    # calculate final potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]
    for x in range(xw):
        for y in range(yw):
            ug = cal_attractive_potential(x, y, gx, gy)
            uo = cal_repulsive_potential(x, y, ox, oy, rr)
            pmap[x][y] = ug + uo
    return pmap


def potential_field_planning(sx, sy, gx, gy, ox, oy, rr, xw, yw):
    # calculate potential field
    pmap = cal_potential_field(gx, gy, ox, oy, rr, sx, sy, xw, yw)
    d = np.hypot(sx - gx, sy - gy)
    ix, iy = sx, sy
    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()
    if show_animation:
        draw_env(ox, oy, rr, xw, yw)
        plt.axis("square")
        plt.gca().set_xlim((0, xw))
        plt.gca().set_ylim((0, yw))
        plt.plot(ix, iy, "*k")
        plt.plot(gx, gy, "*m")
        plt.plot(ix, iy, ".r")
        print(ix, iy)
        plt.pause(0.01)

    while d >= 1:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        d = np.hypot(gx - ix, gy - iy)
        rx.append(ix)
        ry.append(iy)
        if oscillations_detection(previous_ids, ix, iy):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break

        if show_animation:
            draw_env(ox, oy, rr, xw, yw)
            plt.axis("square")
            plt.gca().set_xlim((0, xw))
            plt.gca().set_ylim((0, yw))
            plt.plot(ix, iy, "*k")
            plt.plot(gx, gy, "*m")
            plt.plot(ix, iy, ".r")
            print(ix, iy)
            plt.pause(0.01)

    print("Goal!!")
    return rx, ry


def main():
    print("potential_field_planning start")
    sx = 5.0  # start x position
    sy = 5.0  # start y positon
    gx = 35.0  # goal x position
    gy = 35.0  # goal y position
    obs_radius = [5, 4, 5, 3]  # osbtacle radius
    ox = [
        5.0,
        10.0,
        20.0,
        30.0,
    ]  # obstacle x center position list
    oy = [
        15.0,
        25.0,
        26.0,
        22.0,
    ]  # obstacle y center position list
    xw = 40  # x limit
    yw = 40  # y limit
    # path generation
    ix, iy = potential_field_planning(sx, sy, gx, gy, ox, oy, obs_radius, xw, yw)


if __name__ == "__main__":
    print("start!!")
    main()
    print("Done!!")
    while True:
        try:
            time.sleep(0.5)
        except:
            break
