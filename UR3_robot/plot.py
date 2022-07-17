import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

def animate(i):
    data = np.load('ur3_errors.npy')
    
    if data.shape[1]>=10:
        plt.cla()
        axs[0, 0].plot(data[0, 10:], 'tab:blue')
        axs[0, 0].set_title('Joint1')
        axs[0, 1].plot(data[1, 10:], 'tab:orange')
        axs[0, 1].set_title('Joint2')
        axs[0, 2].plot(data[2, 10:], 'tab:green')
        axs[0, 2].set_title('Joint3')
        axs[1, 0].plot(data[3, 10:], 'tab:blue')
        axs[1, 0].set_title('Joint4')
        axs[1, 1].plot(data[4, 10:], 'tab:orange')
        axs[1, 1].set_title('Joint5')
        axs[1, 2].plot(data[5, 10:], 'tab:green')
        axs[1, 2].set_title('Joint6')
        
        plt.tight_layout()

fig, axs = plt.subplots(2, 3)

ani = FuncAnimation(fig, animate, interval=1000)

plt.tight_layout()
plt.show()
