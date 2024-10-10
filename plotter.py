import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button


BROKER = "localhost"  
PORT = 1883           
TOPIC = "robot/wheel_velocities"  
RESET_TOPIC = "robot/reset"  


left_velocities = []
right_velocities = []


def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code: {rc}")
    
    client.subscribe(TOPIC)
    client.subscribe(RESET_TOPIC)


def on_message(client, userdata, msg):
    if msg.topic == TOPIC:
        
        message = msg.payload.decode()
        print(f"Received message '{message}' on topic '{msg.topic}'")
        
        
        try:
            left_velocity = round(float(message.split(",")[0].split(":")[1].strip()), 2)
            right_velocity = round(float(message.split(",")[1].split(":")[1].strip()), 2)
            
            
            left_velocities.append(left_velocity)
            right_velocities.append(right_velocity)

        except (IndexError, ValueError) as e:
            print(f"Error parsing message: {e}")

    elif msg.topic == RESET_TOPIC:
        print("Reset command received.")
        reset_plot()  

def reset_plot():
    
    global left_velocities, right_velocities
    left_velocities.clear()  
    right_velocities.clear()
    line_left.set_data([], [])
    line_right.set_data([], [])
    ax.set_xlim(0, 100)  
    ax.set_ylim(0, 5)    
    plt.draw()


client = mqtt.Client()


client.on_connect = on_connect
client.on_message = on_message


client.connect(BROKER, PORT, 60)


plt.style.use('fivethirtyeight')
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)  

line_left, = ax.plot([], [], label='Left Velocity', color='blue')
line_right, = ax.plot([], [], label='Right Velocity', color='red')
ax.set_xlim(0, 100)  
ax.set_ylim(0, 3)    
ax.set_xlabel('Messages Received')
ax.set_ylabel('Wheel Velocity')
ax.set_title('Live Wheel Velocities')
ax.legend()

def update(frame):
    line_left.set_data(range(len(left_velocities)), left_velocities)
    line_right.set_data(range(len(right_velocities)), right_velocities)

    ax.set_xlim(0, len(left_velocities) if left_velocities else 100)

    if left_velocities or right_velocities:
        ax.set_ylim(0, max(max(left_velocities), max(right_velocities), 5))  

    return line_left, line_right

def reset(event):
    reset_plot()  

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
