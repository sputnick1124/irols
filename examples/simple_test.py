# -*- coding: utf-8 -*-
"""
Created on Fri Jul  8 08:44:19 2016

@author: nick
"""
from __future__ import division
import sys
sys.path.append('..')
from yapflm import FIS
from gfs import GFS
import time

# traditional FIS
fis = FIS()
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

# GFS create a blank template then fill it in with the copied params from 'fis'
#gfs = GFS(init=[22,3])
#gfs.input[0].name = 'size'
#gfs.input[1].name = 'weight'
#gfs.output[0].name = 'quality'
#
#for gfs_i,fis_i in zip(gfs.input,fis.input):
#    gfs_i.name = fis_i.name
#    for gfs_mf,fis_mf in zip(gfs_i.mf,fis_i.mf):
#        gfs_mf.params = fis_mf.params
#        gfs_mf.name = fis_mf.name
#        
#for gfs_o,fis_o in zip(gfs.output,fis.output):
#    gfs_o.name = fis_o.name
#    for gfs_mf,fis_mf in zip(gfs_o.mf,fis_o.mf):
#        gfs_mf.params = fis_mf.params
#        gfs_mf.name = fis_mf.name
#
#gfs.addrule(rule)

gfs1 = GFS([3,3,3],5)
gfs1.randomize()

gfs2 = GFS([3,3,3],5)
gfs2.randomize()

gfs3,gfs4 = gfs1.crossover(gfs2)

input_vals = [2,25]
input_scales = [10,100]
input_scaled = [a/b for a,b in zip(input_vals,input_scales)]
#print fis.evalfis(input_scaled)
t1 = time.time()
for i in xrange(int(1e4)):
    fis.evalfis(input_scaled)
tf = time.time()-t1
print("{} seconds".format(tf))