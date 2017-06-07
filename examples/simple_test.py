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
fis.addvar('input','size')
fis.addvar('input','weight')
fis.addvar('output','quality')

fis.input[0].addmf('small',[0, 0, 1])
fis.input[0].addmf('large',[0, 1, 1])

fis.input[1].addmf('small',[0, 0, 1])
fis.input[1].addmf('large',[0, 1, 1])

fis.output[0].addmf('bad',[0, 0, 0.5])
fis.output[0].addmf('medium',[0, 0.5, 1])
fis.output[0].addmf('good',[0.5, 1, 1])

rule = [[0, 0, 0, 1, 0],
        [0, 1, 1, 1, 0],
        [1, 0, 1, 1, 0],
        [1, 1, 2, 1, 0]]
fis.addrule(rule)
input_vals = [2,25]
input_scales = [10,100]
input_scaled = [a/b for a,b in zip(input_vals,input_scales)]
for i in xrange(10000):
    fis.evalfis(input_scaled)