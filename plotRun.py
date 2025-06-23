from utilities import import_data
import sys
import matplotlib.pyplot as plt


data = import_data('data.json')
# break our data into separate arrays for plotting
num_cols = len(data[0])
plot_data = []
for col in range(num_cols):
      A = [d[col] for d in data]
      plot_data.append(A)


plt.rcParams.update({'font.size': 12})
plt.rcParams.update({
"text.usetex": True,
})

ns=4
num_points = len(data)
if num_points > len(data):
     num_points = len(data)

state_names = ['$\\theta$ ','$\\dot{\\theta}$ ','u','$\\theta$ s', '$\\dot{\\theta}$ s']
#state_names = ['$x$ ','$\\dot{x}$ ','$\\theta$ ','$\\dot{\\theta}$ ']
for i in range(5):
    plt.plot(plot_data[i][:num_points],linewidth=2,label=state_names[i])
plt.xlabel('Time')
plt.ylabel('State')
plt.legend(loc='lower right')
plt.show()


nu = 1
fig, axs = plt.subplots(ns + nu)
fig.set_figheight(8)
for i in range(ns):
    axs[i].plot(plot_data[i][:num_points],linewidth=2)
    axs[i].set_ylabel(state_names[i])
for i in range(nu):
    axs[i+ns].plot(plot_data[i+ns][:num_points],linewidth=2)
    axs[i+ns].set_ylabel('u')
    
plt.xlabel('Time')
plt.show()
        
i_to_plot = [1,4]
for j in i_to_plot:
    plt.plot(plot_data[i][:num_points],linewidth=2,label=state_names[i])
plt.xlabel('Time')
plt.ylabel('State')
plt.legend(loc='lower right')
plt.show()


# # break our data into separate arrays for plotting
# num_cols = len(data[0])
# plot_data = []
# for col in range(num_cols):
#       A = [d[col] for d in data]
#       plot_data.append(A)


# for index in indexes_to_plot:
#      plt.plot(plot_data[index])


# function to show the plot
plt.show()
