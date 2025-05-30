import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import redis
import json
import time
import matplotlib.patches as patches
import numpy as np

# Redis connection
r = redis.Redis()

key_ball_position = "sai::real::inferred_ball_position"
key_ball_velocity = "sai::real::inferred_ball_velocity"
key_force_direction = "sai::real::desired_ball_force"
key_goal_position = "sai::real::ball_goal_position"
key_n_pi = "sai::real::n_pi"
key_gains = "sai::real::gains"

history_pos = []
memory = 20

# Interactive mode
plt.ion()
fig = plt.figure(figsize=(6, 7))  # Extra height for sliders
ax = fig.add_subplot(111)
fig.subplots_adjust(left=0, right=1, top=0.95, bottom=0.25)

# Black background
fig.patch.set_facecolor('black')
ax.set_facecolor('black')

# Hide axes
ax.set_xlim(-0.22, 0.22)
ax.set_ylim(-0.22, 0.22)
ax.set_aspect('equal')
ax.axis('off')

# Draw a reference circle
reference_circle = patches.Circle((0, 0), 0.2, edgecolor='white', facecolor='none', linestyle='-', linewidth=1.5)
ax.add_patch(reference_circle)

# Sliders
ax_kp = plt.axes([0.25, 0.1, 0.6, 0.03], facecolor='lightgray')
ax_kv = plt.axes([0.25, 0.05, 0.6, 0.03], facecolor='lightgray')

data_gains = r.get(key_gains)
if data_gains:
    kp_i = json.loads(data_gains)[0]
    kv_i = json.loads(data_gains)[1]

slider_kp = Slider(ax_kp, '', 0, 200, valinit=kp_i, valstep=1)
slider_kv = Slider(ax_kv, '', 0, 200, valinit=kv_i, valstep=1)

# Add static labels
fig.text(0.18, 0.1, 'Kp:', color='white', fontsize=10, ha='right', va='center')
fig.text(0.18, 0.05, 'Kv:', color='white', fontsize=10, ha='right', va='center')

# Add dynamic value text
kp_text = fig.text(0.87, 0.1, f"{slider_kp.val:.0f}", color='white', fontsize=10, ha='left', va='center')
kv_text = fig.text(0.87, 0.05, f"{slider_kv.val:.0f}", color='white', fontsize=10, ha='left', va='center')


# Callback functions with Redis update and text update
def update_kp(kp_val):
    kv_val = slider_kv.val
    gains_list = [kp_val, kv_val]
    r.set(key_gains, json.dumps(gains_list))
    kp_text.set_text(f"{kp_val:.0f}")
    kv_text.set_text(f"{kv_val:.0f}")

def update_kv(kv_val):
    kp_val = slider_kp.val
    gains_list = [kp_val, kv_val]
    r.set(key_gains, json.dumps(gains_list))
    kp_text.set_text(f"{kp_val:.0f}")
    kv_text.set_text(f"{kv_val:.0f}")

slider_kp.on_changed(update_kp)
slider_kv.on_changed(update_kv)
# slider_kv.on_changed(update_gains(slider_kp.val, slider_kv.val))


# Initialize plot elements
scat = ax.scatter([], [], s=100)
scatGoal = ax.scatter([], [], s=50)
velocity_text = ax.text(0, 0, "", color='yellow', fontsize=12, weight='bold')
force_arrow = None
velocity_arrow = None
rotation_axis = None

while True:
    data_position = r.get(key_ball_position)
    data_force = r.get(key_force_direction)
    data_goal_position = r.get(key_goal_position)
    data_rotation_axis = r.get(key_n_pi)
    data_velocity = r.get(key_ball_velocity)

    if data_position:
        pos = json.loads(data_position)
        history_pos.append((pos[0], pos[1]))
        if len(history_pos) > memory:
            history_pos.pop(0)

        xs, ys = zip(*history_pos)
        colors = [1 - (i / (len(xs) - 1)) if len(xs) > 1 else 1 for i in range(len(xs))]

        scat.remove()
        scat = ax.scatter(xs, ys, s=600, c=colors, cmap="Greys", edgecolors='white', linewidths=0.2)

    if data_goal_position:
        goal_pos = json.loads(data_goal_position)
        xGoal, yGoal = goal_pos[0], goal_pos[1]
        scatGoal.remove()
        scatGoal = ax.scatter(xGoal, yGoal, s=300, c='red', marker='x', linewidths=2)

    if data_rotation_axis:
        vec = json.loads(data_rotation_axis)
        if rotation_axis:
            rotation_axis.remove()

        dx, dy = vec[0], vec[1]
        magnitude = np.linalg.norm([dx, dy])
        dx_unit, dy_unit = (dx / magnitude, dy / magnitude) if magnitude > 0 else (0, 0)

        R = 0.2
        x0, y0 = -dx_unit * R, -dy_unit * R
        x1, y1 = dx_unit * R, dy_unit * R
        rotation_axis = ax.arrow(x0, y0, 1.02 * (x1 - x0), 1.02 * (y1 - y0),
                                 head_width=0.01, head_length=0.01,
                                 fc='deepskyblue', ec='deepskyblue', alpha=1.0)

    if data_force and data_position:
        force_dir = json.loads(data_force)
        if force_arrow:
            force_arrow.remove()

        start_x, start_y = pos[0], pos[1]
        dx, dy = force_dir[0], force_dir[1]
        magnitude = np.linalg.norm([dx, dy])
        max_arrow_length = 0.15
        max_magnitude = 0.5
        dx_scaled = dx * max_arrow_length / max_magnitude
        dy_scaled = dy * max_arrow_length / max_magnitude

        force_arrow = ax.arrow(start_x, start_y, dx_scaled, dy_scaled,
                               head_width=0.01, head_length=0.01,
                               fc='white', ec='white', alpha=1.0)

        if data_velocity:
            velocity_dir = json.loads(data_velocity)
            if velocity_arrow:
                velocity_arrow.remove()

            dx, dy = velocity_dir[0], velocity_dir[1]
            magnitude = np.linalg.norm([dx, dy])
            max_arrow_length = 0.15
            max_magnitude = 0.2
            dx_scaled = dx * max_arrow_length / max_magnitude
            dy_scaled = dy * max_arrow_length / max_magnitude
            transparency = 1.0 if magnitude > 0 else 0.0



            velocity_arrow = ax.arrow(start_x, start_y, dx_scaled, dy_scaled,
                                      head_width=0.01, head_length=0.01,
                                      fc='yellow', ec='yellow', alpha=transparency)

            velocity_text.set_text(f"{magnitude:.3f}")
            velocity_text.set_position((start_x + 0.02, start_y + 0.02))
            velocity_text.set_alpha(transparency)

    plt.pause(0.05)
