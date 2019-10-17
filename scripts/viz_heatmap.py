import math
import csv
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from scipy.stats import gamma


# with open('params.csv', 'r') as readFile:
# 	reader = csv.reader(readFile)
# 	lines = list(reader)

# print lines

csv_file = open('params.csv', 'r')
params = pd.read_csv(csv_file)


def visualize(shape, scale, pos):
	plt.subplot(pos)

	dim = 1000
	size = len(shape)
	data = np.zeros([size, dim])
	
	j = 0	
	for sh, sc in zip(shape, scale):

		x =  gamma.rvs(sh, scale=sc, size=dim)

		xstart = -3
		xend = 1

		for i in x:
			if i<=0:
				continue
			y = math.log10(i)
			if y>=xstart and y<xend:
				y = int(math.floor((y - xstart) * dim / (xend - xstart)))
				data[j, y]+=1

		j += 1 

	data = np.log(data+1)

	g=sns.heatmap(data.transpose(), cbar=False, cmap="YlGnBu")
	plt.yticks(range(0,dim+1, dim/(xend-xstart)))
	g.set(yticklabels= ['1e'+str(x) for x in range(xstart,xend+1,1)])
	


	



# visualize(params.sig_v_shape, params.sig_v_scale, "221")
# visualize(params.sig_s_shape, params.sig_s_scale, "222")
# visualize(params.sig_r_shape, params.sig_r_scale, "223")
# visualize(params.sig_b_shape, params.sig_b_scale, "224")

visualize(params.L_shape, params.L_scale, "221")
visualize(params.H_shape, params.H_scale, "222")
visualize(params.b_shape, params.b_scale, "223")
visualize(params.a_shape, params.a_scale, "224")


plt.show()


csv_file.close()

