#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
from pydy.viz import Server


resource_dir = '/Users/oliverlee/repos/bicycle/python/scripts/pydy-resources'
scene_file = ''
for f in os.listdir(resource_dir):
    if f.endswith('scene_desc.json') and f > scene_file:
        scene_file = f

server = Server(scene_file=scene_file, directory=resource_dir)
server.run_server()
