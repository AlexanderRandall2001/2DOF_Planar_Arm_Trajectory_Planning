from Robot_Arm_2DOF_IK import ik_2dof
from Robot_Arm_2DOF_FK import fk_2dof
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Compute joint angles at a single step

def cartesian_step(start_xy, target_xy, i, steps, lengths, mode):

    # Progress along trajectory (0=start, 1=end)

    s = i / steps

    # Choose time scaling function: cubic for smooth start/stop, linear for constant speed

    if mode == "cubic":
        h = 3*s**2 - 2*s**3
    elif mode == "linear":
        h = s

    # Interpolate end-effector position in Cartesian space

    x = start_xy[0] + h * (target_xy[0] - start_xy[0])
    y = start_xy[1] + h * (target_xy[1] - start_xy[1])

    # Convert interpolated position to joint angles using IK

    thetas = ik_2dof([x, y], lengths)[1]
    return thetas

# Full Cartesian trajectory

def trajectory_cartesian(start_thetas, target, lengths, steps, mode="cubic"):

    # Compute starting end-effector position

    joints, _ = fk_2dof(start_thetas, lengths)
    start_xy = joints[-1]

    traj = []

    # Generate trajectory step-by-step

    for i in range(steps + 1):
        traj.append(cartesian_step(start_xy, target, i, steps, lengths, mode))

    return traj

# Animate arm trajectory

def animate_2dof(traj, lengths, interval=50):

    fig, ax = plt.subplots()

    # Set axis limits based on total arm reach

    reach = sum(lengths)
    ax.set_xlim(-reach - 0.1, reach + 0.1)
    ax.set_ylim(-reach - 0.1, reach + 0.1)
    ax.set_aspect("equal")
    ax.grid(True)

    # Lines representing the arm links

    line, = ax.plot([], [], "-", lw=4, color='#66FF66')

    # Update the arm configuration for each frame

    def update(frame):

        thetas = traj[frame]
        joints, _ = fk_2dof(thetas, lengths)
        xs, ys = zip(*joints)
        line.set_data(xs, ys)
        return line,

    # Create the animation

    ani = FuncAnimation(
        fig,
        update,
        frames=len(traj),
        interval=interval,
        blit=True
    )
    plt.show()

# Example usage

start_thetas = [0.0, 0.0]
target = [1.2, 0.5]
lengths = [1.0, 0.8]
steps = 60
traj = trajectory_cartesian(start_thetas, target, lengths, steps, mode="cubic")


animate_2dof(traj, lengths)
