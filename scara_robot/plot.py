import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

def animate(i):
    data = np.load('scara_errors.npy')

    plt.cla()
    axs[0, 0].plot(data[0], 'tab:blue')
    axs[0, 0].set_title('Joint1')
    axs[0, 1].plot(data[1], 'tab:orange')
    axs[0, 1].set_title('Joint2')
    axs[1, 0].plot(data[2], 'tab:green')
    axs[1, 0].set_title('Joint3')
    axs[1, 1].plot(data[3], 'tab:red')
    axs[1, 1].set_title('Joint4')

    plt.tight_layout()


fig, axs = plt.subplots(2, 2)

ani = FuncAnimation(fig, animate, interval=1000)

plt.tight_layout()
plt.show()
