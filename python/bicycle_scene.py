#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Describe the bicycle system for visualization using Moore parameters."""
from collections import namedtuple
import numpy as np
from sympy import symbols, pi
from sympy.physics.mechanics import ReferenceFrame, Point
from pydy.viz.shapes import Cylinder
from pydy.viz.visualization_frame import VisualizationFrame
from pydy.viz.scene import Scene


Coordinates = namedtuple('Coordinates',
                         ['x', 'y', 'pitch', 'yaw', 'roll', 'steer'])
Parameters = namedtuple('Parameters', ['rr', 'rf', 'd1', 'd2', 'd3'])
Frames = namedtuple('Frames', ['N', 'V', 'C', 'E'])
Visual = namedtuple('Visual', ['D', 'C', 'E', 'F'])

class BicycleScene(object):

    def __init__(self, parameters=None, samples=None):
        ## coordinates
        # theta: pitch
        # psi: yaw
        # phi: roll
        # delta: steer
        # these correspond to x_aux[:] + x[0:3] in the auxiliary state and
        # state vectors
        x, y, theta, psi, phi, delta = symbols('x y θ ψ φ δ')
        # rr: rear radius
        # rf: front radius
        rr, rf = symbols('rr rf')
        # d1: orthogonal distance from rear wheel center to steer axis
        # d3: orthogonal distance from front wheel center to steer axis
        # d2: steer axis separation
        d1, d2, d3 = symbols('d1:4')

        ## reference frames
        # N: inertial frame
        # D: rear wheel frame
        # C: rear assembly frame - yaw, roll, pitch from inertial frame
        # E: front assembly frame
        # F: front wheel frame
        N = ReferenceFrame('N')
        C = N.orientnew('C', 'body', [psi, phi, theta], 'zxy')
        E = C.orientnew('E', 'axis', [delta, C.z]) # steer
        # Since it will not matter if circles (wheels) rotate, we can attach
        # them to the respective assembly frames
        # However, we need to rotate the rear/front assembly frames for a
        # cylinder shape to be attached correctly. We can simply rotate pi/2
        # about the z-axis for C and about the x-axis for E.
        Cy = C.orientnew('Cy', 'axis', [pi/2, C.z])
        Ey = C.orientnew('Ey', 'axis', [pi/2, E.x])

        ## points
        # define unit vectors from rear/front wheel centers to ground in the
        # wheel plane which will be used to describe points
        Dz = ((C.y ^ N.z) ^ C.y).normalize()
        Fz = ((E.y ^ N.z) ^ E.y).normalize()
        # origin of inertial frame
        O = Point('O')
        # rear wheel/ground contact point
        dn = O.locatenew('dn', x*N.x + y*N.y)
        # rear wheel center point
        do = dn.locatenew('do', -rr*Dz)
        # orthogonal projection of rear wheel center on steer axis
        ce = do.locatenew('ce', d1*C.x)
        # front wheel center point
        fo = ce.locatenew('fo', d2*C.z + d3*E.x)
        # front wheel/ground contact point
        fn = fo.locatenew('fn', rf*Fz)
        ## define additional points for visualization
        # rear assembly center
        cc = do.locatenew('cc', d1/2*C.x)
        # front assembly center
        ec = ce.locatenew('ec', d2/2*C.z)

        ## shapes
        Ds = Cylinder(name='rear wheel', radius=rr, length=0.01)
        Cs = Cylinder(name='rear assembly', radius=0.02, length=d1,
                      color='lightseagreen')
        Es = Cylinder(name='front assembly', radius=0.02, length=d2,
                      color='lightseagreen')
        Fs = Cylinder(name='front wheel', radius=rf, length=0.01)

        ## visualization frames
        Dv = VisualizationFrame('Rear Wheel', C, do, Ds)
        Cv = VisualizationFrame('Rear Assembly', Cy, cc, Cs)
        Ev = VisualizationFrame('Front Assembly', Ey, ec, Es)
        Fv = VisualizationFrame('Front Wheel', E, fo, Fs)

        # V: visualization frame
        V = N.orientnew('V', 'axis', [pi, N.x])

        self.coordinate = Coordinates(x, y, theta, psi, phi, delta)
        self.parameter = Parameters(rr, rf, d1, d2, d3)
        self.frame = Frames(N, V, C, E)
        self.shape = Visual(Ds, Cs, Es, Fs)
        self.point = Visual(do, cc, ec, fo)
        self.vframe = Visual(Dv, Cv, Ev, Fv)
        self.scene = Scene(V, dn) # scene moves with rear contact point
        self.scene.visualization_frames = list(self.vframe.__dict__.values())
        self.scene.states_symbols = list(self.coordinate.__dict__.values())

        if parameters is not None:
            self.set_parameters(parameters)
        if samples is not None:
            self.set_trajectory(samples)


    def set_parameters(self, parameter_dict):
        kv = list(parameter_dict.items())
        keys, values = zip(*kv)
        self.scene.constants = dict(zip(symbols(keys), values))

    def set_trajectory(self, samples):
        t = samples.ts * samples.bicycle.dt[0]
        self.scene.times = np.squeeze(t)
        self.scene.frames_per_second = 1/samples.bicycle.dt[0]
        self.scene.states_trajectories = np.squeeze(
                np.concatenate((samples.aux, samples.x[:, :3]), axis=1))

    def run(self):
        self.scene.display()
