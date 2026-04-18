#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import threading
import time

import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from scipy.ndimage import rotate

# ======================
# Robot State
# ======================
robot_x, robot_y = 0.0, 1.0
robot_theta = np.radians(90)
robot_theta_display = robot_theta
status = "stop"
boost = 350

lock = threading.Lock()
trail_points = []

# ======================
# Load Robot Image
# ======================
robot_img = mpimg.imread("/home/fadhil/auv_ws/src/auv_pkg/auv_pkg/IMG_2077.PNG")
robot_artist = None

# ======================
# Speed Map
# ======================
speed_map = {
    ("all", 350): 0.075,
    ("backward", 350): 0.078,
    ("sway_left", 0): 0.05,
    ("sway_right", 0): 0.05,
}

def calculate_speed(status, boost):
    return speed_map.get((status, boost), 0)

# ======================
# Plot Setup
# ======================
fig, ax = plt.subplots(figsize=(8, 8))

def setup_plot():
    ax.clear()
    ax.set_xlim(-14, 14)
    ax.set_ylim(0, 25)
    ax.set_aspect('equal')
    ax.set_xticks(np.arange(-14, 15, 1))
    ax.set_yticks(np.arange(0, 26, 1))
    ax.grid(True, linestyle='--', alpha=0.4)

    ax.axhline(12, linestyle='--', color='black', alpha=0.2)

        # Danger zones (Red-Black)
    ax.fill_between(
        [-14, 14], 24, 25,
        facecolor='red',
        edgecolor='black',
        alpha=0.6,
        hatch='////'
    )
    ax.text(
        0, 24.5, "DANGER ZONE",
        color='black',
        fontsize=12,
        ha='center',
        va='center',
        fontweight='bold'
    )

    ax.fill_betweenx(
        [0, 25], -14, -13,
        facecolor='red',
        edgecolor='black',
        alpha=0.6,
        hatch='////'
    )
    ax.text(
        -13.5, 12.5, "DANGER ZONE",
        color='black',
        fontsize=12,
        ha='center',
        va='center',
        rotation=90,
        fontweight='bold'
    )

    ax.fill_betweenx(
        [0, 25], 13, 14,
        facecolor='red',
        edgecolor='black',
        alpha=0.6,
        hatch='////'
    )
    ax.text(
        13.5, 12.5, "DANGER ZONE",
        color='black',
        fontsize=12,
        ha='center',
        va='center',
        rotation=270,
        fontweight='bold'
    )



    # Gate
    ax.add_patch(plt.Rectangle((-2.5, 12), 5, 0.4, color='orange'))
    ax.text(0, 11, "Gate", fontsize=10, ha='center', va='bottom', fontweight='bold')

    # Tarpaulin
    ax.add_patch(plt.Rectangle((-12, 22), 13, 2, color='green'))

    # Buckets
    ax.add_patch(plt.Circle((-10, 23), 0.8, color='blue'))
    ax.add_patch(plt.Circle((-7, 23), 0.8, color='red'))
    ax.add_patch(plt.Circle((-4, 23), 0.8, color='red'))
    ax.add_patch(plt.Circle((-1, 23), 0.8, color='red'))

    ax.text(-10, 21, "Blue", fontsize=10, color='black', ha='center', va='bottom', fontweight='bold')
    ax.text(-7, 21, "Red", fontsize=10, color='black', ha='center', va='bottom', fontweight='bold')
    ax.text(-4, 21, "Red", fontsize=10, color='black', ha='center', va='bottom', fontweight='bold')
    ax.text(-1, 21, "Red", fontsize=10, color='black', ha='center', va='bottom', fontweight='bold')
    ax.text(-5.5, 20, "Bucket", fontsize=10, color='black', ha='center', va='bottom', fontweight='bold')

    # Flares
    ax.add_patch(plt.Circle((0, 7.8), 0.6, color='orange'))
    ax.add_patch(plt.Circle((11, 20), 0.5, color='red'))
    ax.add_patch(plt.Circle((8, 18), 0.5, color='yellow'))
    ax.add_patch(plt.Circle((5, 16), 0.5, color='blue'))

    ax.text(0, 6, "Orange Flare", fontsize=9, ha='center', fontweight='bold')
    ax.text(11, 18.5, "Red Flare", fontsize=9, ha='center', fontweight='bold')
    ax.text(8, 16.5, "Yellow Flare", fontsize=9, ha='center', fontweight='bold')
    ax.text(5, 14.5, "Blue Flare", fontsize=9, ha='center', fontweight='bold')

# ======================
# Draw Robot Image
# ======================
def draw_robot_image(ax, x, y, theta):
    global robot_artist

    if robot_artist is not None:
        robot_artist.remove()

    # Gambar AUV menghadap KE ATAS → butuh offset 90°
    rotated_img = rotate(
        robot_img,
        angle=90 - np.degrees(theta),
        reshape=True,
        order=1
    )

    imagebox = OffsetImage(rotated_img, zoom=0.03)

    robot_artist = AnnotationBbox(
        imagebox,
        (x, y),
        frameon=False
    )

    ax.add_artist(robot_artist)

# ======================
# Robot Movement Loop
# ======================
def move_robot():
    global robot_x, robot_y, robot_theta
    while True:
        with lock:
            speed = calculate_speed(status, boost)

            if status == "all":
                robot_x += speed * np.cos(robot_theta)
                robot_y += speed * np.sin(robot_theta)

            elif status == "backward":
                robot_x -= speed * np.cos(robot_theta)
                robot_y -= speed * np.sin(robot_theta)

            elif status == "sway_left":
                robot_x += speed * np.cos(robot_theta + np.pi/2)
                robot_y += speed * np.sin(robot_theta + np.pi/2)

            elif status == "sway_right":
                robot_x += speed * np.cos(robot_theta - np.pi/2)
                robot_y += speed * np.sin(robot_theta - np.pi/2)

            elif status == "yaw_left":
                robot_theta += np.radians(5)

            elif status == "yaw_right":
                robot_theta -= np.radians(5)

        time.sleep(0.1)

# ======================
# Keyboard Input
# ======================
def keyboard_loop():
    global status
    print("""
Controls:
w = forward
s = backward
a = sway left
d = sway right
q = yaw left
e = yaw right
x = stop
""")
    while True:
        cmd = input("Command: ").strip().lower()
        with lock:
            if cmd == 'w': status = "all"
            elif cmd == 's': status = "backward"
            elif cmd == 'a': status = "sway_left"
            elif cmd == 'd': status = "sway_right"
            elif cmd == 'q': status = "yaw_left"
            elif cmd == 'e': status = "yaw_right"
            elif cmd == 'x': status = "stop"

# ======================
# Animation Update
# ======================
def update_plot(frame):
    setup_plot()
    with lock:
        trail_points.append((robot_x, robot_y))
        if len(trail_points) > 1:
            xs, ys = zip(*trail_points)
            ax.plot(xs, ys, 'b-', linewidth=1)

        draw_robot_image(ax, robot_x, robot_y, robot_theta)

# ======================
# Main
# ======================
if __name__ == "__main__":
    threading.Thread(target=move_robot, daemon=True).start()
    threading.Thread(target=keyboard_loop, daemon=True).start()

    ani = animation.FuncAnimation(fig, update_plot, interval=100)
    plt.show()
