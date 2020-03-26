import matplotlib.pyplot as plt
import sys
import pickle

if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "data/constant_data_sawyer.pickle"
    data = pickle.load(open(filename, "rb"))
    print(data)
    for p in data.keys():
        for j in data[p].keys():
            for imu in data[p][j].keys():
                plt.plot(data[p][j][imu])
                plt.show()
