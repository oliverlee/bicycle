#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
from itertools import product
import numpy as np
import matplotlib as mpl
import matplotlib.pylab as plt
import seaborn as sns
import convert


def unit(value, degrees=True):
    if value.endswith('angle'):
        u = 'rad'
    elif value.endswith('rate'):
        u = 'rad/s'
    elif value.endswith('time'):
        u = 's'
    else:
        raise NotImplementedError(
                "No unit associated with '{}'.".format(value))

    if degrees:
        u = u.replace('rad', '°')
    return u


def plot_state(samples, filename=None, degrees=True):
    # get time from timestamp and sample time
    t = samples.bicycle.dt.mean() * samples.ts

    cols = 2
    rows = samples.x.shape[1] // cols
    fig, axes = plt.subplots(rows, cols, sharex=True)

    color = sns.color_palette('Paired', 10)
    state = ['roll angle', 'steer angle', 'roll rate', 'steer rate']

    for n, (i, j) in enumerate(product(range(rows), range(cols))):
        ax = axes[i, j]
        x = samples.x[:, n]
        x_state = state[n]
        x_unit = unit(x_state, degrees)
        x_hat = samples.kalman.x[:, n]
        if degrees and '°' in x_unit:
            x = np.rad2deg(x);
            x_hat = np.rad2deg(x_hat)

        ax.set_xlabel('{} [{}]'.format('time', unit('time')))
        ax.set_ylabel('{} [{}]'.format(x_state, x_unit))
        ax.plot(t, x, color=color[2*n + 1], label='true')

        if not x_hat.mask.all():
            ax.plot(t, x_hat, color=color[2*n], label='estimate')
            ax.legend()

    title = 'system state'
    if filename is not None:
        title += ' for file {}'.format(filename)
    fig.suptitle(title, size=mpl.rcParams['font.size'] + 2)
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(("Usage: {} <sample_log_file>\n\nPlot sample log " +
               "data.").format(__file__));
        print("\t<sample_log_file>\tFile containing samples in " +
                "flatbuffers binary format.")
        sys.exit(1)

    # set helvetica as plot font
    mpl.rcParams['font.family'] = 'Helvetica'
    mpl.rcParams['font.weight'] = 'light'
    mpl.rcParams['axes.labelweight'] = 'light'

    # use a muted color palette
    sns.set(palette='muted')

    samples = convert.load_sample_log(sys.argv[1])
    plot_state(samples, os.path.basename(sys.argv[1]))

    sys.exit(0)
