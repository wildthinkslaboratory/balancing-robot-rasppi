import matplotlib.pyplot as plt
import json
import numpy as np
from utilities import import_data

indexes_to_plot = [0,1,2,3,4]
data = import_data("data.json")


# break our data into separate arrays for plotting
num_cols = len(data[0])
plot_data = []
for col in range(num_cols):
      A = [d[col] for d in data]
      plot_data.append(A)


for index in indexes_to_plot:
     plt.plot(plot_data[index])


# function to show the plot
plt.show()