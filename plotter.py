import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, Slider
import numpy as np
import threading


BROKER = "localhost"
PORT = 1883
VELOCITY_TOPIC = "robot/wheel_velocities"
RESET_TOPIC = "robot/reset"
START_TOPIC = "robot/start"
PID_TOPIC = "robot/pid"  


left_velocities = []
right_velocities = []


def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code: {rc}")
    client.subscribe(VELOCITY_TOPIC)

def on_message(client, userdata, msg):
    if msg.topic == VELOCITY_TOPIC:
        message = msg.payload.decode()
        try:
            left_velocity = float(message.split(",")[0].split(":")[1].strip())
            right_velocity = float(message.split(",")[1].split(":")[1].strip())
            print(f"Left Velocity: {left_velocity:.2f}, Right Velocity: {right_velocity:.2f}")
            left_velocities.append(left_velocity)
            right_velocities.append(right_velocity)
        except (IndexError, ValueError) as e:
            print(f"Error parsing message: {e}")


def reset_plot():
    global left_velocities, right_velocities
    left_velocities.clear()
    right_velocities.clear()
    line_left.set_data([], [])
    line_right.set_data([], [])
    ax.set_xlim(0, 100)
    plt.draw()


def reset(event):
    Kp = 0
    Ki = 0
    Kd = 0
    
    client.publish(PID_TOPIC, f"{Kp},{Ki},{Kd}")
    print(f"Updated PID Parameters - Kp: {Kp}, Ki: {Ki}, Kd: {Kd}")
    reset_plot()
    client.publish(RESET_TOPIC, "reset")  
    print("Published reset command")


def update_and_start(event):
    Kp = kp_slider.val
    Ki = ki_slider.val
    Kd = kd_slider.val
    
    client.publish(PID_TOPIC, f"{Kp},{Ki},{Kd}")
    print(f"Updated PID Parameters - Kp: {Kp}, Ki: {Ki}, Kd: {Kd}")
    
    client.publish(START_TOPIC, "start")
    print("Published start command")


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message


def start_mqtt():
    client.connect(BROKER, PORT, 60)
    client.loop_forever()

mqtt_thread = threading.Thread(target=start_mqtt, daemon=True)
mqtt_thread.start()


plt.style.use('fivethirtyeight')
fig, (ax, control_ax) = plt.subplots(nrows=2, sharex=False, figsize=(10, 10)) 


line_left, = ax.plot([], [], label='Left Velocity', color='blue')
line_right, = ax.plot([], [], label='Right Velocity', color='red')


ax.set_xlim(0, 100)
ax.set_ylim(0, 5)  
ax.set_xlabel('Messages Received')
ax.set_ylabel('Wheel Velocity')
ax.set_title('Live Wheel Velocities')
ax.legend()

control_ax.axis('off')


axcolor = 'lightgoldenrodyellow'
ax_kp = plt.axes([0.1, 0.35, 0.65, 0.03], facecolor=axcolor)
ax_ki = plt.axes([0.1, 0.3, 0.65, 0.03], facecolor=axcolor)
ax_kd = plt.axes([0.1, 0.25, 0.65, 0.03], facecolor=axcolor)

kp_slider = Slider(ax_kp, 'Kp', 0.0, 100, valinit=0.0)
ki_slider = Slider(ax_ki, 'Ki', 0.0, 100, valinit=0.0)
kd_slider = Slider(ax_kd, 'Kd', 0.0, 100, valinit=0.0)


start_ax = plt.axes([0.8, 0.35, 0.15, 0.075])
start_button = Button(start_ax, 'Update & Start')
start_button.on_clicked(update_and_start)


def update(frame):
    if left_velocities or right_velocities:
        line_left.set_data(range(len(left_velocities)), left_velocities)
        line_right.set_data(range(len(right_velocities)), right_velocities)

        
        ax.set_xlim(0, max(len(left_velocities), len(right_velocities), 100))

        
        ax.set_ylim(min(min(left_velocities, default=0), min(right_velocities, default=0)),
                    max(max(left_velocities, default=0), max(right_velocities, default=0)) + 1)

    return line_left, line_right


reset_ax = plt.axes([0.8, 0.25, 0.15, 0.075])
reset_button = Button(reset_ax, 'Reset')
reset_button.on_clicked(reset)

ani = animation.FuncAnimation(fig, update, interval=1000, blit=False)

plt.show()
