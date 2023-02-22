import matplotlib.pyplot as plt
import math
import numpy as np

if __name__ == "__main__":
    heap_time = [0, 0.0013964, 0.0334116, 0.65321, 9.625366]
    array_time = [0.00136, 0.070922, 7.5361742, 207.4, 750]  # figure out good estimate for 1000000
    ht = [i * 1 for i in heap_time]  # change scale value
    at = [i * 1 for i in array_time]
    n_2 = [i ** 2 for i in range(1, 6)]
    log = [i * math.log(i) for i in range(1, 6)]
    fig, ax = plt.subplots()
    x = np.linspace(0, 1000000, 5)

    plt.plot(x, ht, label='heap time')
    plt.plot(x, at, label='array time')
    # plt.plot(x, n_2, label='n^2')
    # plt.plot(x, log, label='log')

    plt.xlabel("Size of Graph")
    plt.ylabel("Time (s)")
    plt.title("Dijkstra's Algorithm Analysis")
    ax.set_xscale('log')
    plt.show()
