from scipy.fft import fft
from plot_methods import plot_trajectory



def processor(points):
    x = fft(points)
    plt.plot()


def wrapper(time, ax, ay, az):
    processor(ax)
    


plot_trajectory('./processed/device_version_START/accel.csv', wrapper)