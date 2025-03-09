import time
import matplotlib.pyplot as plt

def cubic_trajectory(z_init, z_final, t_final, plot_duration, time_step=0.01):
    z_des = []
    z_dot_des = []
    time_stamps = []
    initial_time = time.time()

    t = time.time() - initial_time
    while t <= plot_duration:
        a2 = (3*(z_final - z_init)) / t_final**2
        a3 = (-2*(z_final - z_init)) / t_final**3

        if t < t_final:
            z_des.append(z_init + a2*t**2 + a3*t**3)
            z_dot_des.append(z_init + 2*a2*t + 3*a3*t**2)
        else:
            z_des.append(z_final)
            z_dot_des.append(0)

        time_stamps.append(t)

        # Wait for the next time step
        time.sleep(time_step)

        t = time.time() - initial_time

    return z_des, z_dot_des, time_stamps

def main(args=None):
    t_final = 20 # seconds
    z_init = 0 # meters
    z_final = 0.5 # meters
    plot_duration = 25 # seconds

    z_des, z_dot_des, time_stamps = cubic_trajectory(z_init, z_final, t_final, plot_duration)

    # Plot the results on the same figure
    plt.figure(figsize=(10, 6))

    # Plot desired position
    plt.plot(time_stamps, z_des, label="Desired Position (z_des)", color="blue", linewidth=2)

    # Plot desired velocity
    plt.plot(time_stamps, z_dot_des, label="Desired Velocity (z_dot_des)", color="red", linewidth=2)

    # Add labels, title, and legend
    plt.xlabel("Time (s)")
    plt.ylabel("Depth (m) / Velocity (m/s)")
    plt.title("Cubic Trajectory: Depth and Velocity vs Time")
    plt.grid(True)
    plt.legend()

    # Show the plot
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()