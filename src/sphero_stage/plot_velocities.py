import json
import matplotlib.pyplot as plt
from datetime import datetime

def plot_data(json_file_path):
    # Lists to store data for each variable
    times = []
    cohesion_vels = []
    align_vels = []
    separation_vels = []
    steer_vels = []
    obs_vels = []

    with open(json_file_path, 'r') as file:
        for line in file:
            # Parse each line as a JSON object
            data_point = json.loads(line)

            # Extract time
            timestamp = data_point.get('time', 0.0)
            times.append(datetime.fromtimestamp(timestamp))

            # Extract velocity data
            cohesion_vels.append(data_point.get('cohesion_vel', 0.0))
            align_vels.append(data_point.get('align_vel', 0.0))
            separation_vels.append(data_point.get('separation_vel', 0.0))
            steer_vels.append(data_point.get('steer_vel', 0.0))
            obs_vels.append(data_point.get('obs_vel', 0.0))

    # Plotting
    plt.figure(figsize=(10, 6))

    plt.plot(times, cohesion_vels, label='Cohesion Velocity')
    plt.plot(times, align_vels, label='Alignment Velocity')
    plt.plot(times, separation_vels, label='Separation Velocity')
    plt.plot(times, steer_vels, label='Steering Velocity')
    plt.plot(times, obs_vels, label='Obstacle Velocity')

    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title('Velocity vs. Time')
    plt.legend()
    plt.show()

# Replace 'your_json_file.json' with the actual path to your JSON file
plot_data("/home/mawais/catkin_ws/src/sphero_simulation/sphero_stage/resources/data/velocities.json")
