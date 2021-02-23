import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

#style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
plt.ylabel('lat /m')
plt.xlabel('lon /m')
plt.grid(True)
#plt.xlim((-83.499,-83.4991))
#plt.ylim((42.38415,42.38416))

#plt.ticklabel_format(style='plain', axis='x')
plt.gcf().axes[0].xaxis.get_major_formatter().set_scientific(False)
plt.axis('equal')

graph_data = open('lat_raw.txt','r').read()
lines = graph_data.split('\n')
yl = []
for line in lines:
    if len(line) > 1:
        y = line
        y_r = (float(y)-42.38396550)*111132.944444
        yl.append(float(y_r))

graph_data = open('lon_raw.txt','r').read()
lines = graph_data.split('\n')
xl = []
for line in lines:
    if len(line) > 1:
        x = line
        x_r = (float(x)-(-83.49858350))*82230.670304
        xl.append(float(x_r))
# graph_data = open('position_mat.dat','r').read()
# lines = graph_data.split('\n')
# xsc = []
# ysc = []
# flag=0
# flag_in=0
# for line in lines:
#     flag=flag+1
#     if len(line) > 1:
#         y, x = line.split(';')
#         x_r = (float(x)-(-83.499008179))*82230.670304
#         y_r = (float(y)-42.384140015)*111132.944444
#         flag_in=flag_in+1
#         if flag>0:#26000
#             #flag=0
#             if flag_in>80:
#                 flag_in=0
#                 xsc.append(float(x_r))
#                 ysc.append(float(y_r))
# ax1.plot(xs, ys, 'rx', xsc, ysc, 'g.')
# ax1.plot(xs, ys, 'rx')
#lat_data = open('lat_raw_new.txt','w').write()

ax1.plot(xl, yl, 'g.')
plt.show()
