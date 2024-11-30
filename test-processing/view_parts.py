import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os, sys, math, random, time

SENSOR_DATA = 'recordings/with_donanta.csv'
ALGORITHM_DATA = SENSOR_DATA.replace('recordings/', 'analysis/').replace('.csv', '_algo.csv')
ANGLES_DATA = SENSOR_DATA.replace('recordings/', 'analysis/').replace('.csv', '_angles.csv')
STEPS_DATA = SENSOR_DATA.replace('recordings/', 'analysis/').replace('.csv', '_steps.csv')

steps = pd.read_csv(SENSOR_DATA)
# Convert ESP32 system time to local time so the graphs make more sense.
now = time.time()
steps['t'] = pd.to_datetime((steps['time'] / 1_000_000.0) + now, unit='s')
steps.set_index(steps['t'], inplace=True)
steps.drop(columns=['t'], inplace=True)
steps

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

steps['accel.x'].plot(ax=ax1)
steps['accel.y'].plot(ax=ax1)
steps['accel.z'].plot(ax=ax1)
ax1.xaxis.set_ticklabels([])
ax1.legend(loc='upper left')
ax1.grid()

steps['gyro.x'].plot(ax=ax2)
steps['gyro.y'].plot(ax=ax2)
steps['gyro.z'].plot(ax=ax2)
ax2.legend(loc='upper left')
ax2.grid()

ax1.set_title('Raw sensor data')
fig.set_figwidth(17)

plt.show()
