import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import re

def read_data(filename):
    with open(filename, "r") as file:
        raw_data = file.read().strip().split("Mode")[1:-2]

    ndata = len(raw_data)
    columns = list(map(lambda x: (x.split(" = ")[0]), raw_data[0].strip().split("\n")[1:]))
    columns = ["Mode", *columns]

    data_mode = ndata*[""]
    data_np = np.zeros((len(columns)-1, ndata))

    for i in range(ndata):
        spi_in_str = False
        current = raw_data[i]
        if ("SPI" in raw_data[i]):
            spi_in_str = True
            current = re.sub("SPI.*", "", raw_data[i], flags=re.S)
        if ("query" in raw_data[i]):
            current = re.sub("query.*\n", "", current)   
        if ("\na = " in raw_data[i]):
            current = re.sub("\na = .*", "\n", current, flags=re.S)
        if ("\nstate is" in raw_data[i]):
            current = re.sub("\nstate is.*", "\n", current, flags=re.S) 
        if ("\nstate is" in raw_data[i]):
            current = re.sub("\nstate is.*", "\n", current, flags=re.S) 
        try:  
            splitted_data = current.strip().split("\n")
            mode = splitted_data[0].strip()
            data_mode[i] = mode
            data_np[:,i] = list(map(lambda x: float(x.split(" = ")[-1]), splitted_data[1:]))

        except:
            print("Error parsing sample number ", i, "\n")
            print(raw_data[i])
            print(current)
            break

    data = pd.DataFrame(data_np.T, columns=columns[1:])
    data.insert(0,"time",np.cumsum(data["dt"]),False)
    data.insert(0,"Mode",data_mode,False)

    return data


for i in range(10):
    data = read_data("PF_test__8.txt")
    plt.cla()
    plt.plot(data["xpos"], data["ypos"])
    plt.pause(0.1)

plt.show()