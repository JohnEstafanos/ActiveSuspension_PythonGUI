import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import threading
import time
import serial
import tk_tools
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math 


# Empty array of 16 states 
states = np.array([0.0] * 11); 
serialConnected = [False]
command = bytearray([0])


# State variables
speed_buffer = []
 
# Initialize GUI
root = tk.Tk()
root.geometry("800x800")

battery_voltage = tk.Label(root, text="Battery Voltage: 0.00")
battery_voltage.pack(anchor='w')

voltage_1 = tk.Label(root, text="Cell 1 Voltage: 0.00")
voltage_1.pack(anchor='w')

voltage_2 = tk.Label(root, text="Cell 2 Voltage: 0.00")
voltage_2.pack(anchor='w')

global gui_closed
gui_closed = [False]
 
# Create frame for 3D plot and bars
frame = tk.Frame(root)
frame.pack()
 
# Define the serial port and timeout duration (in seconds)
port = 'com4'
timeout_duration = 5  # 5 seconds timeout
 
# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=frame)
canvas.draw()
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

yaw_track = [0.0] * 5
pitch_track = [0.0] * 5 
roll_track = [0.0] * 5 
 
# # Sys-MoDEL Logo
# smLogo = (Image.open("Sys-MoDEL Black Logo.png"))
# smLogoResize = smLogo.resize((500, 200), Image.ANTIALIAS)
# smLogoResize = ImageTk.PhotoImage(smLogoResize)
# smLogoLabel = tk.Label(root, image=smLogoResize)
# smLogoLabel.place(anchor='nw', relx=0, rely=0)
 
# Create progress bars and labels
Encoderbars = []
encoder_names = ["Encoder Front Right", "Encoder Front Left", "Encoder Back Right", "Encoder Back Left"]
for i, name in enumerate(encoder_names):
    label = tk.Label(root, text=f'{name}:')
    label.pack()
    bar_frame = ttk.Frame(root)
    bar_frame.pack()
    bottom_label = tk.Label(bar_frame, text="0 mm")
    bottom_label.pack(side="left")
    bar = ttk.Progressbar(bar_frame, length=400, mode='determinate')
    bar.pack(side="left")
    top_label = tk.Label(bar_frame, text="25.4 mm")
    top_label.pack(side="left")
    Encoderbars.append(bar)
 

def stop_command():
    command[0] = 0
    print('Stop command sent')

def around_command():
    command[0] = 1
    print('Around command sent')

def bounce_command():
    command[0] = 2
    print('Bounce command sent')

def L2R_command():
    command[0] = 3
    print('Left-2-Right command sent')

def F2B_command():
    command[0] = 4
    print('Front-2-Back command sent')




# Create buttons
button_frame = tk.Frame(root)

stop_btn = tk.Button(button_frame, text="Stop", command=stop_command)
around_btn = tk.Button(button_frame, text="Around", command=around_command)
Bounce_btn = tk.Button(button_frame, text="Bounce", command=bounce_command)
L2R_btn = tk.Button(button_frame, text="Left-2-Right", command=L2R_command)
R2L_btn = tk.Button(button_frame, text="Front-2-Back", command=F2B_command)

stop_btn.pack()
around_btn.pack()
Bounce_btn.pack()
L2R_btn.pack()
R2L_btn.pack()

button_frame.pack(side='right', pady=50, padx=50, anchor='e')

# Create frame for steering gauge
steering_frame = tk.Frame(root)
steering_frame.pack(side='left', pady=50, padx=50, anchor='s')
 
# Create gauge for steering angle
steeringGauge = tk_tools.Gauge(steering_frame, width=600, height=250, min_value=-35, max_value=35, label='Steering Angle', unit='degrees', divisions=1)
steeringGauge.pack()
 
# Create frame for speed gauge
speed_frame = tk.Frame(root)
speed_frame.pack(side='right', pady=50, padx=50, anchor='s')
 
# Create gauge for vehicle speed
speedGauge = tk_tools.Gauge(speed_frame, width=600, height=250, min_value=-40, max_value=40, label='Vehicle Speed', unit='km/h', divisions=1)
speedGauge.pack()
 
# Create box vertices
box_vertices = np.array([[x, y, z] for x in (-1, 1) for y in (-2, 2) for z in (-0.5, 0.5)])
box_faces = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
 
# Define the origin of the coordinate frame
origin = np.array([0, 0, 0])

# Create a threading lock 
lock = threading.Lock()
 
# Define the end points of the axes in the coordinate frame
axes_end_points = np.array([[-2.5, 0, 0], [0, 2.5, 0], [0, 0, 2.5]])
 
# Function to calculate rotation matrix
def rotation_matrix(roll, pitch, yaw):
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)
 
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
 
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
 
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
 
    R = np.dot(Rz, np.dot(Ry, Rx))
 
    return R
 
# Function to stop the program
def stop_program():
    root.quit()     # stops mainloop
    root.destroy()  # this is necessary on Windows to prevent Fatal Python Error: PyEval_RestoreThread: NULL tstate
    gui_closed[0] = True
    is_port_available(port)
 
# Bind window closing event to stop_program function
root.protocol("WM_DELETE_WINDOW", stop_program)
 
def is_port_available(port):
    try:
        ser = serial.Serial(port)
        ser.close()
        return True
    except serial.SerialException:
        return False
 
def read_from_arduino(stateData:np.ndarray, serialActive:list[bool], actionCommand:bytearray, _gui_closed:bool):
    while not _gui_closed[0]:
        try:
            with serial.Serial(port, 115200, timeout=timeout_duration) as ser:
                while not _gui_closed[0]:
                    
                    ser.write(actionCommand[0].to_bytes(1, 'big'))
 
                    data = ser.readline().decode().strip()
                    if data:
                        print(data)

                    statesIn = data[1:-1]
 
                    try:
                        statesIn = np.array(statesIn.split(',')).astype(float)
                        with lock:
                            # Copy don't overwrite
                            for i in range(11):
                                stateData[i] = statesIn[i]

                            # if(math.isnan(states[9]) or math.isnan(states[10]) or math.isnan(states[11])):
                            #     continue
                            # elif(states[9]==0 and states[10]==180 and states[11]==0):
                            #     continue
                            # else: 
                            #     yaw_track.pop(0)
                            #     yaw_track.append(states[9])
                            #     pitch_track.pop(0)
                            #     pitch_track.append(states[10])
                            #     roll_track.pop(0)
                            #     roll_track.append(states[11])

                            #     states[9] = sum(yaw_track) / len(yaw_track)
                            #     states[10] = sum(pitch_track) / len(pitch_track)
                            #     states[11] = sum(roll_track) / len(roll_track)

                        with lock: 
                            serialActive[0] = True

                    except:
                        print('Arduino is not sending me numbers :()')

                        with lock: 
                            serialActive[0] = False

                    time.sleep(0.001)

 
        except:
            with lock: 
                serialActive[0] = False
            print(f"Connection to port {port} lost. Reconnecting...")
            time.sleep(1)  # Wait for a moment before trying to reconnect
 
def update_gui(stateData:np.ndarray, serialActive:list[bool], _gui_closed:bool):

    roll_local = 0
    pitch_local = 0   
    yaw_local = 0

    while not _gui_closed[0]:
        while(serialActive[0] == False):
            time.sleep(1)
            # print('GUI - Waiting for serial connection')
            if _gui_closed[0]:
                return


        with lock:
            states = stateData

        absEncCurrentPositionFR = states[0]
        absEncCurrentPositionFL = states[1]
        absEncCurrentPositionBR = states[2]
        absEncCurrentPositionBL = states[3]
        quadEncoderVel = states[4]
        pitch_local = states[5]
        roll_local = states[6]
        yaw_local = states[7]
        steeringAngle = states[8]
        cell_1_volt = states[9]
        cell_2_volt = states[10]

        encoder_readings = [absEncCurrentPositionFR, absEncCurrentPositionFL, absEncCurrentPositionBR,
                            absEncCurrentPositionBL]
    
        # Calculate rotation matrix
        R = rotation_matrix(roll_local, pitch_local, yaw_local)
        # print(R)

        # Apply rotation matrix to each vertex of the box
        rotated_vertices = np.dot(box_vertices, R.T)

        # Apply rotation matrix to each end point of the axes
        rotated_axes_end_points = np.dot(axes_end_points, R.T)

        # Update 3D plot
        ax.clear()
        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])
        ax.axis('off')
        for face in box_faces:
            vertices = [rotated_vertices[vertex] for vertex in face]
            ax.add_collection3d(Poly3DCollection([vertices], alpha=.25, linewidths=1, edgecolors='r', facecolors='cyan'))
        for i, color in enumerate(['r', 'g', 'b']):
            ax.plot([origin[0], rotated_axes_end_points[i, 0]], [origin[1], rotated_axes_end_points[i, 1]], [origin[2], rotated_axes_end_points[i, 2]], color)
        canvas.draw()

        # Update progress Encoderbars
        for bar, reading in zip(Encoderbars, encoder_readings):
            bar['value'] = reading * 3

        # Update steering gauge
        steeringGauge.set_value(steeringAngle)
    
        # Update the speed gauge with moving average of last 10 data points
        quadEncoderVelKmPerHour = (5.4e-5 * (quadEncoderVel / 7.5)) / 3600
        update_vehicle_speed_gauge(quadEncoderVel)

        voltage_1.config(text=f"Cell 1 Voltage: {cell_1_volt:.2f}")
        voltage_2.config(text=f"Cell 2 Voltage: {cell_2_volt:.2f}")
        battery_voltage.config(text=f"Battery Voltage: {(cell_1_volt + cell_2_volt):.2f}")
 
# Function to update vehicle speed gauge with moving average of last 10 data points
def update_vehicle_speed_gauge(quadEncoderVel):
    speed_buffer.append(quadEncoderVel)
    if len(speed_buffer) > 10:
        speed_buffer.pop(0)
    moving_average_speed = sum(speed_buffer) / len(speed_buffer)
    speedGauge.set_value(moving_average_speed)
 
# Start thread to read data from Arduino
serialCommThread = threading.Thread(target=read_from_arduino, args=(states, serialConnected, command, gui_closed))
GUIupdateThread = threading.Thread(target=update_gui, args=(states, serialConnected, gui_closed))
 
serialCommThread.start()
GUIupdateThread.start()


# Start GUI
root.mainloop()


serialCommThread.join()
GUIupdateThread.join()
 