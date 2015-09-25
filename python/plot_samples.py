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

def _set_suptitle(fig, title, filename):
    if filename is not None:
        title += ' for file {}'.format(filename)
    fig.suptitle(title, size=mpl.rcParams['font.size'] + 2)


def plot_state(samples, filename=None, degrees=True, confidence=True):
    # TODO: Fill masked values with previous valid values
    # get time from timestamp and sample time
    t = samples.bicycle.dt.mean() * samples.ts

    cols = 2
    rows = samples.x.shape[1] // cols
    fig, axes = plt.subplots(rows, cols, sharex=True)

    color = sns.color_palette('Paired', 10)
    state = ['roll angle', 'steer angle', 'roll rate', 'steer rate']

    # We want C'*z so we have the measured state with noise at every timestep
    # (some states will be zero).
    # (C'*z)' = z'*C
    C = samples.bicycle.Cd[0] # C = Cd

    x_meas = np.dot(samples.z.transpose(0, 2, 1), C).transpose(0, 2, 1)
    for n, (i, j) in enumerate(product(range(rows), range(cols))):
        ax = axes[i, j]
        x_state = state[n]
        x_unit = unit(x_state, degrees)

        x = samples.x[:, n]
        x_hat = samples.kalman.x[:, n]
        z = x_meas[:, n]
        if degrees and '°' in x_unit:
            x = np.rad2deg(x)
            x_hat = np.rad2deg(x_hat)
            z = np.rad2deg(z)

        ax.set_xlabel('{} [{}]'.format('time', unit('time')))
        ax.set_ylabel('{} [{}]'.format(x_state, x_unit))
        ax.plot(t, x, color=color[2*n + 1], label='true', zorder=2)

        if not x_hat.mask.all():
            ax.plot(t, x_hat, color=color[2*n], label='estimate', zorder=3)
            if confidence:
                std_dev = np.sqrt(samples.kalman.P[:, n, n])
                if degrees and '°' in x_unit:
                    np.rad2deg(std_dev, out=std_dev) # in-place conversion
                # plot confidence interval of 2 standard deviations
                low = x_hat.ravel() - 2*std_dev
                high = x_hat.ravel() + 2*std_dev
                ax.fill_between(t, low, high, alpha=0.2, color=color[2*n])
            ax.legend()

        if not z.mask.all() and z.any():
            dark_color = sns.dark_palette(color[2*n + 1])[-3]
            ax.plot(t, z, color=dark_color, label='measurement', zorder=1)
            ax.legend()

    title = 'system state'
    _set_suptitle(fig, title, filename)
    return fig, axes


def plot_entries(samples, field, filename=None):
    # TODO: Fill masked values with previous valid values
    # get time from timestamp and sample time
    t = samples.bicycle.dt.mean() * samples.ts
    X = samples.__getattribute__(field)

    _, rows, cols = X.shape
    n = rows*cols
    if n > 6:
        color = sns.color_palette('husl', n_colors=n)
    else:
        color = sns.color_palette('muted', n)
    fig, axes = plt.subplots(rows, cols, sharex=True)

    vector_type = False
    if len(axes.shape) == 1:
        vector_type = True

    fields = field.split('.')
    for n, (i, j) in enumerate(product(range(rows), range(cols))):
        if vector_type:
            ax = axes[n]
            x = X[:, n]
        else:
            ax = axes[i, j]
            x = X[:, i, j]

        # small entries in Kalman gain K break plots
        small_indices = np.abs(x) < 10*np.finfo(x.dtype).eps
        if small_indices.any():
            # copy data (maybe we should modify in place?
            x = np.array(x)
            x[small_indices] = 0

        ax.set_xlabel('{} [{}]'.format('time', unit('time')))
        if vector_type:
            ax.set_title('{}[{}]'.format(fields[-1], n))
        else:
            ax.set_title('{}[{}, {}]'.format(fields[-1], i, j))
        ax.plot(t, x, color=color[n])

    title = ' '.join([f.title() for f in fields[:-1]] + fields[-1:])
    _set_suptitle(fig, title, filename)
    return fig, axes


def plot_error_covariance(samples, filename=None):
    # TODO: Fill masked values with previous valid values
    # get time from timestamp and sample time
    t = samples.bicycle.dt.mean() * samples.ts
    P = samples.kalman.P
    e = samples.x - samples.kalman.x

    # calculate outer product of error at each timestep
    E = e * e.transpose(0, 2, 1)

    _, rows, cols = P.shape
    color_calc = sns.husl_palette(rows*cols)
    color_true = sns.husl_palette(rows*cols, l=0.4)
    fig, axes = plt.subplots(rows, cols, sharex=True)

    for n, (i, j) in enumerate(product(range(rows), range(cols))):
        ax = axes[i, j]
        ax.set_xlabel('{} [{}]'.format('time', unit('time')))
        ax.set_title('P[{}, {}]'.format(i, j))
        ax.plot(t, P[:, i, j], color=color_calc[n], label='calculated')
        ax.plot(t, E[:, i, j], color=color_true[n], label='true')
        ax.legend()

    title = 'Kalman error covariance P'
    _set_suptitle(fig, title, filename)
    return fig, axes


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
