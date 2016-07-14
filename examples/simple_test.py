# -*- coding: utf-8 -*-
"""
Created on Fri Jul  8 08:44:19 2016

@author: nick
"""
from __future__ import division
import sys
sys.path.append('..')
from yapflm import FIS

fis = FIS('mamdani')
fis.addvar('input','size',[0,10])
fis.addvar('input','weight',[0,100])
fis.addvar('output','quality',[0,1])

fis.input[0].addmf('small','trimf',[0, 0, 10])
fis.input[0].addmf('large','trimf',[0, 10, 10])

fis.input[1].addmf('small','trimf',[0, 0, 100])
fis.input[1].addmf('small','trimf',[0, 100, 100])

fis.output[0].addmf('bad','trimf',[0, 0, 0.5])
fis.output[0].addmf('medium','trimf',[0, 0.5, 1])
fis.output[0].addmf('good','trimf',[0.5, 1, 1])

rule = [[0, 0, 0, 1, 0],
        [0, 1, 1, 1, 0],
        [1, 0, 1, 1, 0],
        [1, 1, 2, 1, 0]]
fis.addrule(rule)

for i in xrange(10000):
    fis.evalfis([2,25])