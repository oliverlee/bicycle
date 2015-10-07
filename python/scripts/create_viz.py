#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
from dtk.bicycle import *

sys.path.append('/Users/oliverlee/repos/bicycle/python/')
import convert
from bicycle_scene import BicycleScene


def main():
    pwd = os.getcwd()
    working_dir = os.path.dirname(os.path.realpath(__file__))

    params = benchmark_to_moore(benchmark_parameters())
    samples = convert.load_sample_log(
            '/Users/oliverlee/repos/bicycle/build/samples_full.bin')
    bicycle = BicycleScene(params, samples)

    os.chdir(working_dir)
    bicycle.run()
    os.chdir(pwd)


if __name__ == '__main__':
    main()

