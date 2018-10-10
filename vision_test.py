f = open('results.txt', 'r')
import matplotlib.pyplot as plt

reward = []
steps = []
time = []
count = []
episode = []

total = f.readlines()
for line in total:
    line = line.split(',')
    if len(line) > 2:
        reward.append(float(line[0]))
        episode.append(int(line[1]))
        steps.append(float(line[2]))
        count.append(int(line[3]))
        time.append(float(line[4])/3600)




plt.plot(steps, count, c='r', label='reward')

plt.legend(loc='best')
plt.ylabel('count')
plt.xlabel('total training steps')
plt.grid()
plt.show()

