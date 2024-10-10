import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

# MQTT broker details
BROKER = "localhost"
PORT = 1883
TOPIC = "robot/wheel_velocities"
RESET_TOPIC = "robot/reset"
DESIRED_VELOCITY_TOPIC = "robot/target_velocity"

# Data lists
left_velocities = []
right_velocities = []
velocity_differences = []
desired_velocities = []

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code: {rc}")
    client.subscribe(TOPIC)
    client.subscribe(RESET_TOPIC)
    client.subscribe(DESIRED_VELOCITY_TOPIC)

def on_message(client, userdata, msg):
    if msg.topic == TOPIC:
        message = msg.payload.decode()
        try:
            left_velocity = round(float(message.split(",")[0].split(":")[1].strip()), 2)
            right_velocity = round(float(message.split(",")[1].split(":")[1].strip()), 2)
            print(f"Left Velocity: {left_velocity:.2f}, Right Velocity: {right_velocity:.2f}")
            left_velocities.append(left_velocity)
            right_velocities.append(right_velocity)
            velocity_differences.append(abs(left_velocity - right_velocity))
        except (IndexError, ValueError) as e:
            print(f"Error parsing message: {e}")

    elif msg.topic == RESET_TOPIC:
        print("Reset command received.")
        reset_plot()

    elif msg.topic == DESIRED_VELOCITY_TOPIC:
        message = msg.payload.decode()
        try:
           
            desired_velocity = float(message)
            desired_velocities.append(desired_velocity)
        except (IndexError, ValueError) as e:
            print(f"Error parsing desired velocity message: {e}")

def reset_plot():
    global left_velocities, right_velocities, velocity_differences, desired_velocities
    left_velocities.clear()
    right_velocities.clear()
    velocity_differences.clear()
    desired_velocities.clear()
    line_left.set_data([], [])
    line_right.set_data([], [])
    line_diff.set_data([], [])
    text_desired.set_text('')
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 5)
    plt.draw()

# MQTT Client Setup
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, 60)

# Matplotlib Setup
plt.style.use('fivethirtyeight')
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)

line_left, = ax.plot([], [], label='Left Velocity', color='blue')
line_right, = ax.plot([], [], label='Right Velocity', color='red')
line_diff, = ax.plot([], [], label='Velocity Difference', color='green')
text_desired = ax.text(0.05, 0.95, '', transform=ax.transAxes, fontsize=12, verticalalignment='top')

ax.set_xlim(0, 100)
ax.set_ylim(0, 5)
ax.set_xlabel('Messages Received')
ax.set_ylabel('Wheel Velocity')
ax.set_title('Live Wheel Velocities')
ax.legend()

def update(frame):
    rounded_left_velocities = [round(v, 2) for v in left_velocities]
    rounded_right_velocities = [round(v, 2) for v in right_velocities]
    rounded_velocity_differences = [round(v, 2) for v in velocity_differences]
    
    if desired_velocities:
        latest_desired_velocity = desired_velocities[0]
        print(latest_desired_velocity)
        text_desired.set_text(f'Desired Velocity: {latest_desired_velocity:.2f}')
    else:
        text_desired.set_text('Desired Velocity: N/A')

    line_left.set_data(range(len(rounded_left_velocities)), rounded_left_velocities)
    line_right.set_data(range(len(rounded_right_velocities)), rounded_right_velocities)
    line_diff.set_data(range(len(rounded_velocity_differences)), rounded_velocity_differences)

    ax.set_xlim(0, max(len(rounded_left_velocities), len(rounded_right_velocities), len(rounded_velocity_differences), 100))

    if rounded_left_velocities or rounded_right_velocities or rounded_velocity_differences:
        ax.set_ylim(0, max(max(rounded_left_velocities, default=0),
                           max(rounded_right_velocities, default=0),
                           max(rounded_velocity_differences, default=0),
                           5))

    return line_left, line_right, line_diff, text_desired

def reset(event):
    reset_plot()

# Button for resetting the plot
reset_ax = plt.axes([0.8, 0.05, 0.1, 0.075])
reset_button = Button(reset_ax, 'Reset')
reset_button.on_clicked(reset)

client.loop_start()
ani = animation.FuncAnimation(fig, update, interval=1000, cache_frame_data=False, blit=False)
plt.show()

try:
    client.loop_forever()
except KeyboardInterrupt:
    print("Exiting...")
client.loop_stop()
