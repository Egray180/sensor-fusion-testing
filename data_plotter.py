import matplotlib.pyplot as plt
import numpy as np 

def plot_in_one_figure(x, y, logx=False, logy=False,\
                       title='Figure', xlabel=None, ylabel=None,\
                       grid='on', legend=None,\
                       mpl_opt=''):
    '''
    Create a figure and plot x/y in this figure.
    Args:
        x: x axis data, array of size (n,) or (n,1)
        y: y axis data, array of size (n,m)
        title: figure title
        xlabel: x axis label
        ylabel: y axis label
        gird: if this is not 'off', it will be changed to 'on'
        legend: tuple or list of strings of length m.
    '''
    # create figure and axis
    fig = plt.figure(title)
    axis = fig.add_subplot(111)
    lines = []
    y = np.array(y)
    # if not x data, generate default x data
    if x is None:
        x = np.array(range(y.shape[0]))
    try:
        dim = y.ndim
        if dim == 1:
            if logx and logy:   # loglog
                line, = axis.loglog(x, y, mpl_opt)
            elif logx:          # semilogx
                line, = axis.semilogx(x, y, mpl_opt)
            elif logy:          # semilogy
                line, = axis.semilogy(x, y, mpl_opt)
            else:               # plot
                line, = axis.plot(x, y, mpl_opt)
            lines.append(line)
        elif dim == 2:
            for i in range(0, y.shape[1]):
                if logx and logy:   # loglog
                    line, = axis.loglog(x, y[:, i], mpl_opt)
                elif logx:          # semilogx
                    line, = axis.semilogx(x, y[:, i], mpl_opt)
                elif logy:          # semilogy
                    line, = axis.semilogy(x, y[:, i], mpl_opt)
                else:               # plot
                    line, = axis.plot(x, y[:, i], mpl_opt)
                lines.append(line)
        else:
            raise ValueError
    except:
        print('x-axis data len: ', x.shape)
        print('y-axis data shape: ', y.shape)
        raise ValueError('Check input data y.')
    # label
    if xlabel is not None:
        plt.xlabel(xlabel)
    if ylabel is not None:
        plt.ylabel(ylabel)
    # legend
    if legend is not None:
        plt.legend(lines, legend)
    # grid
    if grid.lower() != 'off':
        plt.grid()

def plot3d_in_one_figure(y, title='Figure', grid='on', legend=None, mpl_opt=''):
    '''
    Create a figure and plot 3d trajectory in this figure.
    Args:
        y: y axis data, np.array of size (n,3)
        title: figure title
        gird: if this is not 'off', it will be changed to 'on'
        legend: tuple or list of strings of length 3.
    '''
    # create figure and axis
    fig = plt.figure(title)
    axis = fig.add_subplot(111, projection='3d', aspect='equal')
    try:
        dim = y.ndim
        if dim == 2:    # y must be an numpy array of size (n,3), dim=2
            if y.shape[1] != 3:
                raise ValueError
            else:
                axis.plot(y[:, 0], y[:, 1], y[:, 2], mpl_opt)
        else:
            raise ValueError
    except:
        print(y.shape)
        raise ValueError('Check input data y.')
    # label
    if isinstance(legend, (tuple, list)):
        n = len(legend)
        if n != 3:
            legend = ['x', 'y', 'z']
    else:
        legend = ['x', 'y', 'z']
    axis.set_xlabel(legend[0])
    axis.set_ylabel(legend[1])
    axis.set_zlabel(legend[2])
    # grid
    if grid.lower() != 'off':
        plt.grid()