import matplotlib.pyplot as plt
import matplotlib.animation
import seaborn as sns
import numpy as np




def get_data(i=0):
    x,y = np.random.normal(loc=i,scale=3,size=(2, 260))
    x[0:10]=i

    # print x, y
    return x,y

x,y = get_data()
g = sns.JointGrid(x=x, y=y, size=10)
lim = (-10,10)

def prep_axes(g, xlim, ylim):
    g.ax_joint.clear()
    g.ax_joint.set_xlim(xlim)
    g.ax_joint.set_ylim(ylim)
    g.ax_marg_x.clear()
    g.ax_marg_x.set_xlim(xlim)
    g.ax_marg_y.clear()
    g.ax_marg_y.set_ylim(ylim)
    plt.setp(g.ax_marg_x.get_xticklabels(), visible=False)
    plt.setp(g.ax_marg_y.get_yticklabels(), visible=False)
    plt.setp(g.ax_marg_x.yaxis.get_majorticklines(), visible=False)
    plt.setp(g.ax_marg_x.yaxis.get_minorticklines(), visible=False)
    plt.setp(g.ax_marg_y.xaxis.get_majorticklines(), visible=False)
    plt.setp(g.ax_marg_y.xaxis.get_minorticklines(), visible=False)
    plt.setp(g.ax_marg_x.get_yticklabels(), visible=False)
    plt.setp(g.ax_marg_y.get_xticklabels(), visible=False)


def animate(i):
    g.x, g.y = get_data(i)
    # prep_axes(g, lim, lim)
    g.plot_joint(sns.kdeplot, cmap="Purples_d")
    # g.plot_marginals(sns.kdeplot, color="m", shade=True)

frames=np.sin(np.linspace(0,2*np.pi,17))*5
ani = matplotlib.animation.FuncAnimation(g.fig, animate, frames=frames, repeat=True, interval=100)



plt.show()


