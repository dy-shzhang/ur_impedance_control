import pandas as pd
import os
import matplotlib.pyplot as plt

#

speed = pd.read_csv('speed.csv')
force = pd.read_csv('force.csv')
speed = speed.dropna()
force = force.dropna()

figure1 =plt.figure()

axes1 = figure1.add_subplot(3,1,1)
axes2 =figure1.add_subplot(3,1,2)
axes3 = figure1.add_subplot(3,1,3)

axes1.plot(speed[1])
axes2.plot(speed[2])
axes3.plot(speed[3])
