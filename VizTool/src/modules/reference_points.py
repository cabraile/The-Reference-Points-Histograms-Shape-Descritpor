import numpy;
import math;

Simple = [
    numpy.array([1,0,0]),
    numpy.array([0,1,0]),
    numpy.array([0,0,1]),
    numpy.array([0,0,0])
];

A1 = [
    numpy.array([1,0,0]),
    numpy.array([0,1,0]),
    numpy.array([0,0,1]),
    numpy.array([-1,0,0]),
    numpy.array([0,-1,0]),
    numpy.array([0,0,-1]),
    numpy.array([0,0,0])
];

A2 = [
    numpy.array([0,0,0]) ,
    numpy.array([0,-1,0]),
    numpy.array([0,1,0])
]


for psi in numpy.arange(-45,90,45):
    for i in range(8):
        theta = math.pi * 45 * i / 180.0;
        x = math.sin(theta) * math.sin(psi);
        z = math.sin(theta) * math.cos(psi);
        y = math.cos(theta);
        A2.append(numpy.array([x,y,z]));

A3 = [
    numpy.array([-1,-1,-1]),
    numpy.array([-1,-1,0]),
    numpy.array([-1,-1,1]),
    numpy.array([-1,0,-1]),
    numpy.array([-1,0,0]),
    numpy.array([-1,0,1]),
    numpy.array([-1,1,-1]),
    numpy.array([-1,1,0]),
    numpy.array([-1,1,1]),
    numpy.array([0,-1,-1]),
    numpy.array([0,-1,0]),
    numpy.array([0,-1,1]),
    numpy.array([0,0,-1]),
    numpy.array([0,0,0]),
    numpy.array([0,0,1]),
    numpy.array([0,1,-1]),
    numpy.array([0,1,0]),
    numpy.array([0,1,1]),
    numpy.array([1,-1,-1]),
    numpy.array([1,-1,0]),
    numpy.array([1,-1,1]),
    numpy.array([1,0,-1]),
    numpy.array([1,0,0]),
    numpy.array([1,0,1]),
    numpy.array([1,1,-1]),
    numpy.array([1,1,0]),
    numpy.array([1,1,1])
];
