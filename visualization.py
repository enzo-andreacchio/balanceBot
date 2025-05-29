import matplotlib.pyplot as plt
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


history_pos = []
memory = 20

# Interactive mode
plt.ion()
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111)
fig.subplots_adjust(left=0, right=1, top=1, bottom=0)

# Black background
fig.patch.set_facecolor('black')
ax.set_facecolor('black')

# Hide axes
ax.set_xlim(-0.22, 0.22)
ax.set_ylim(-0.22, 0.22)
ax.set_aspect('equal')
ax.axis('off')

# Optional: Draw a reference circle
reference_circle = patches.Circle((0, 0), 0.2, edgecolor='white', facecolor='none', linestyle='-', linewidth=1.5, alpha=1)
ax.add_patch(reference_circle)

# Initialize scatter and arrow
scat = ax.scatter([], [], s=100)
force_arrow = None
velocity_arrow = None
rotation_axis = None
scatGoal = ax.scatter([], [], s=50)
velocity_text = ax.text(0, 0, "", color='yellow', fontsize=12, weight='bold')


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

        # Normalize the direction
        magnitude = np.linalg.norm([dx, dy])
        if magnitude > 0:
            dx_unit, dy_unit = dx / magnitude, dy / magnitude
        else:
            dx_unit, dy_unit = 0, 0  # fallback
        
         # Scale to radius
        R = 0.2
        x0, y0 = -dx_unit * R, -dy_unit * R
        x1, y1 = dx_unit * R, dy_unit * R

        # Draw the line (from one side of the circle to the other)
        rotation_axis = ax.arrow(x0, y0, 1.02*(x1-x0), 1.02*(y1-y0),
                            head_width=0.01, head_length=0.01,
                            fc='deepskyblue', ec='deepskyblue', alpha=1.0)
        



    if data_force and data_position:
        force_dir = json.loads(data_force)

        # Remove previous arrow
        if force_arrow:
            force_arrow.remove()

        # Draw new arrow
        start_x, start_y = pos[0], pos[1]
        dx, dy = force_dir[0], force_dir[1]


        magnitude = np.linalg.norm([dx, dy])

        max_arrow_length = 0.15
        max_magnitude = 0.5
        
        dx_scaled = dx*max_arrow_length/max_magnitude
        dy_scaled = dy*max_arrow_length/max_magnitude

            
        force_arrow = ax.arrow(start_x, start_y, dx_scaled, dy_scaled,
                            head_width=0.01, head_length=0.01,
                            fc='white', ec='white', alpha=1.0)
        

        if data_velocity and data_position:
            velocity_dir = json.loads(data_velocity)

            # Remove previous arrow
            if velocity_arrow:
                velocity_arrow.remove()

            # Draw new arrow
            start_x, start_y = pos[0], pos[1]
            dx, dy = velocity_dir[0], velocity_dir[1]


            magnitude = np.linalg.norm([dx, dy])

            print("{:.3f}".format(magnitude))


            arrow_length = 0.1
        
            dx_scaled = dx*arrow_length/magnitude
            dy_scaled = dy*arrow_length/magnitude

            transparency = 0.0
            if magnitude > 0.1:
                transparency = 1.0
            
            velocity_arrow = ax.arrow(start_x, start_y, dx_scaled, dy_scaled,
                                    head_width=0.01, head_length=0.01,
                                    fc='yellow', ec='yellow', alpha=transparency)

            velocity_text.set_text(f"{magnitude:.3f}")
            # position it slightly offset from the ball
            velocity_text.set_position((start_x + 0.02, start_y + 0.02))
            velocity_text.set_alpha(transparency)


            


    plt.pause(0.05)
