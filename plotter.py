import matplotlib.pyplot as plt
import json
import numpy as np

datafile = 'data.json'

indexes_to_plot = [0]
# Open and read the JSON file
with open(datafile, 'r') as file:
    data = json.load(file)


# break our data into separate arrays for plotting
num_cols = len(data[0])
plot_data = []
for col in range(num_cols):
      A = [d[col] for d in data]
      print(np.var(A))
      print(np.std(A))
      plot_data.append(A)


for index in indexes_to_plot:
     plt.plot(plot_data[index])


# function to show the plot
plt.show()