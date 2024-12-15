import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

from config import *

df = pd.read_csv(ANGLES_DATA)
df['time'] = df['time'] / 1_000_000_000.0

# Set target FPS and calculate time step
fps = 60
time_step = 1 / fps

# Create a uniform time index
df['time'] = pd.to_datetime(df['time'], unit='s')
uniform_time_index = pd.date_range(start=df['time'].min(), 
                                   end=df['time'].max(), 
                                   freq=f'{int(1000 / fps)}L')  # Milliseconds per frame

# Interpolate quaternion values
df_resampled = df.set_index('time').reindex(uniform_time_index).interpolate('linear').reset_index()
df_resampled.rename(columns={'index': 'time'}, inplace=True)

# Define the box vertices
def create_box():
    return np.array([
        [-1, -1, -1],
        [ 1, -1, -1],
        [ 1,  1, -1],
        [-1,  1, -1],
        [-1, -1,  1],
        [ 1, -1,  1],
        [ 1,  1,  1],
        [-1,  1,  1]
    ])

# Apply rotation to the box vertices
def rotate_box(box, rotation_matrix):
    return box @ rotation_matrix.T

# Create a 3D animation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
box = create_box()
faces = [[0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7], [0, 1, 2, 3], [4, 5, 6, 7]]

# Initialize plot
plot_box = [ax.plot([], [], [], 'b')[0] for _ in faces]

def init():
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    return plot_box

def update(frame):
    time, w, x, y, z = frame
    quat = [w, x, y, z]
    rot = R.from_quat(quat).as_matrix()
    rotated_box = rotate_box(box, rot)
    
    for i, face in enumerate(faces):
        x = rotated_box[face, 0]
        y = rotated_box[face, 1]
        z = rotated_box[face, 2]
        plot_box[i].set_data(x, y)
        plot_box[i].set_3d_properties(z)
    return plot_box


frames = df_resampled[['time', 'gyro.w', 'gyro.x', 'gyro.y', 'gyro.z']].values
ani = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True)
plt.show()
