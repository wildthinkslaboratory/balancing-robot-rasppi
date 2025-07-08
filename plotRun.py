from utilities import import_data
import sys
import matplotlib.pyplot as plt
import numpy as np


log = import_data('run_data/data.json')
time = log['time']
notes = log['notes']
constants = log['constants']
data = log['rundata']

# state_structure = constants['state_structure']
# state_names = constants['state_names']
# sensor_indexes = constants['sensor_indexes']
state_structure = [4, 1, 3]
state_names =  ['$x$ ','$\\dot{x}$ ','$\\theta$ ','$\\dot{\\theta}$ ', 'u', '$x$ s', '$\\theta$ s','$\\dot{\\theta}$ s']
sensor_indexes = [0, 2, 3]

# first we print out the run constants
print('Reviewing run from ', time)
for key in constants.keys():
    print(f"{key:10} |  {str(constants[key]):15}")

print('Notes: ' + notes)
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

# first we print out the state in one plot

plt.figure(figsize=(12, 6)) 
plt.title(time)
for i in range(state_structure[0]):
    d = [record[i] for record in data]
    plt.plot(d,linewidth=1,label=state_names[i])
plt.xlabel('Time')
plt.ylabel('State')
plt.legend(loc='lower right')
plt.show()


# next we do each state seperately with input
fig, axs = plt.subplots(state_structure[0] + state_structure[1])
fig.set_figheight(8)

for i in range(state_structure[0]):
    d = [record[i] for record in data]
    axs[i].plot(d,linewidth=1)
    axs[i].set_ylabel(state_names[i])
for i in range(state_structure[1]):
    index = i + state_structure[0]
    d = [record[index] for record in data]
    axs[index].plot(d,linewidth=1)
    axs[index].set_ylabel(state_names[index])
    
plt.xlabel('Time')
plt.show()

plt.figure(figsize=(12, 6)) 
plt.title(time)    
for i in range(state_structure[2]):
    state_i = sensor_indexes[i]
    sensor_i = state_structure[0] + state_structure[1] + i
    # plot estimated state
    est = [record[state_i] for record in data]
    plt.plot(est,linewidth=1,label=state_names[state_i])

    # plot sensor reading
    sensor = [record[sensor_i] for record in data]
    plt.plot(sensor,linewidth=1,label=state_names[sensor_i])

    plt.xlabel('Time')
    plt.legend(loc='lower right')
    plt.show()

