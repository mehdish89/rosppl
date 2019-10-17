import numpy as np
import matplotlib.pyplot as plt
import math
import rosbag
import rospy
import json

import seaborn as sns

from std_msgs.msg import String

from scipy.stats import gamma, norm

M = 1
N = 2

plt.ion()
plt.show()

count = 0

h = [None, None]

dim=100

def log_normalize(y):
	y = np.log(y+1)
	y -= np.min(y)
	y /= np.max(y)
	return y

def plot_all(obj, str, id):
	global count

	if isinstance(obj, dict) and 'shape' in obj and 'scale' in obj:

		# if h[count % (M*N)] is None:
		# 	h[count % (M*N)] = np.zeros([dim,dim])

		# nh = np.roll(h[count % (M*N)].transpose(), -dim).transpose()
  		
		a = obj['shape']
		scale = obj['scale']

		

		# xstart = -4 # gamma.ppf(0.001, a, scale=scale)
		# xend =  2 # gamma.ppf(0.999, a, scale=scale)
		# x = np.exp(np.linspace(xstart, xend, dim))

		xstart = gamma.ppf(0.0001, a, scale=scale)
		xend = gamma.ppf(0.9999, a, scale=scale)

		# seaborn.plt.ylim(0, 20)
		
		x = np.linspace(xstart, xend, dim)

		y = (gamma.pdf(x, a, scale=scale))
		# y = log_normalize(y)

  		# nh[:, -1] = y


		# print nh.shape

		# x = np.log(x)

		ax=plt.subplot(M, N, count % (M*N) + 1)
		ax.set_title(str)
		plt.pause(0.002)
		# plt.yscale('log')
		# seaborn.heatmap(nh, cbar=False)
		plt.xscale('log')
		plt.yscale('log')
		# ax=plt.subplot(2*M, N, M+count % (M*N) + 1)
		ax.plot(x, y,
         'r-', lw=5, alpha=0.6, label='gamma pdf')
  		
		# h[count % (M*N)] = nh  		
  		count+=1

		plt.draw()
		return 

	# print obj

	
	
	if isinstance(obj, list):
		id-=1
		for val in obj:
			id += 1
			plot_all(val, str + '/' + id, id)
		return

	if isinstance(obj, dict): # or isinstance(obj, object):
		id-=1
		for key, val in obj.items():
			id += 1
			plot_all(val, str + '/' + key, id)
		return

	# print 'this is object ', obj, type(obj)
	print obj, str, id
	return 
	
	ax=plt.subplot(M, N, count % (M*N) + 1)
	count+=1
	ax.set_title(str)
	plt.pause(0.02)
	plt.scatter(rospy.Time.now().to_sec(), obj)
	plt.draw()
	

	

def plot_gamma(g):
	pass

dim =100

calls = 0

d = np.zeros([2, dim], dtype='float')

def callback(msg):
	global calls, d
	# calls+=1
	# if(calls%3!=0):
	# 	return

	params = json.loads(msg.data)

	shape = params['sig_r']['shape']
	scale = params['sig_r']['scale']

	x =  gamma.rvs(shape, scale=scale, size=1000)

	# d = np.zeros([2, dim], dtype='float')
	d *= 0.7

	xstart = -3
	xend = 1



	for i in x:
		y = math.log(i)
		if y>=xstart and y<xend:
			y = int(math.floor((y - xstart) * dim / (xend - xstart)))
			d[0, y]+=1

	shape = params['sig_b']['shape']
	scale = params['sig_b']['scale']

	x =  gamma.rvs(shape, scale=scale, size=1000)

	for i in x:
		y = math.log10(i)
		if y>=xstart and y<xend:
			y = int(math.floor((y - xstart) * dim / (xend - xstart)))
			d[1, y]+=1

	# plt.xscale('log')
	# print np.log(x)

	# d = np.log(d)


	g=sns.heatmap(d, cbar=False)
	plt.xticks(range(0,dim+1, dim/(xend-xstart)))
	g.set(xticklabels= ['1e'+str(x) for x in range(xstart,xend+1,1)])
	
	plt.pause(0.002)
	# plt.scatter(rospy.Time.now().to_sec(), obj)

	plt.draw()


	# ax = sns.distplot(x)



	
	
	# plot_all(params, '', 1)

#importing libraries
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# fig = plt.figure()
# #creating a subplot 
# ax1 = fig.add_subplot(1,1,1)

# def animate(i):
#     data = open('stock.txt','r').read()
#     lines = data.split('\n')
#     xs = []
#     ys = []
   
#     for line in lines:
#         x, y = line.split(',') # Delimiter is comma    
#         xs.append(float(x))
#         ys.append(float(y))
   
    
#     ax1.clear()
#     ax1.plot(xs, ys)

#     plt.xlabel('Date')
#     plt.ylabel('Price')
#     plt.title('Live graph with matplotlib') 
    
    



def listener():
    rospy.init_node('params_visualizer')
    rospy.Subscriber("/params", String, callback, queue_size=1)

    # ani = animation.FuncAnimation(fig, animate, interval=1000) 
    
    rospy.spin()

# plt.show()
listener()

# import pandas as pd

# mean, cov = [0, 1], [(1, .5), (.5, 1)]
# # data = np.random.multivariate_normal(mean, cov, 200)
# n=10
# data = np.zeros([n*10,2])
# for t in range(n):
# 	# data = np.random.multivariate_normal(mean, cov, 200)
# 	data[t*10:(t+1)*10, 1] = np.random.normal(t, 2, 10)
# 	data[t*10:(t+1)*10, 0] = t

# # print np.append(data, np.array([1,2]))
# df = pd.DataFrame(data, columns=["x", "y"])



# g = sns.jointplot(x="x", y="y", data=df, kind="kde", color="m")

# g.ax_joint.clear()

# g.x = g.x[::-1]

# cmap = sns.cubehelix_palette(as_cmap=True, dark=0, light=1, reverse=True)

# g.plot_joint(sns.kdeplot, cmap=cmap, n_levels=10, shade=True)
# # g.plot_joint(plt.scatter, c="w", s=30, linewidth=1, marker="+")
# g.ax_joint.collections[0].set_alpha(0)
# g.set_axis_labels("$X$", "$Y$");


# print g.x, g.y



# # g.kdeplot(df.x, df.y, cmap=cmap, n_levels=60, shade=True);

# # g.plot_joint(plt.scatter, c="w", s=30, linewidth=1, marker="+")
# plt.show()

# for a in range(10, 20):	
	


# plt.show()