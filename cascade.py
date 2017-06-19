#!/usr/bin/env python

from gfs import GFS

class GenFuzzyCascade(object):
    def __init__(self,*gfs):
        self.gfs = gfs

    def crossover(self,other):
        c1,c2 = [],[]
        for p1,p2 in zip(self.gfs,other.gfs):
            c = p1.crossover(p2)
            c1.append(c[0])
            c2.append(c[1])

    def mutate(self):
        for gfs in self.gfs:
            gfs.mutate()

    def randomize(self):
        for gfs in self.gfs:
            gfs.randomize()