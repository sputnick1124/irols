#!/usr/bin/env python
from __future__ import print_function, division, absolute_import
import sys
sys.path.append('.')
from copy import deepcopy
from multiprocessing import Pool
from operator import itemgetter
from random import triangular


class GA(object):
    def __init__(self,popSize=100,genMax=200,numElite=5,numRecomb=50,
                 numMut=35,numRand=10,stagnation=10,cpus=4):
        popTrial = sum([numElite,numRecomb,numMut,numRand])
        if popTrial != popSize:
            popPer = popSize/popTrial
            numElite = int(numElite*popPer)
            numRecomb = int(numRecomb*popPer)
            numMut = int(numMut*popPer)
            numRand = popSize - (numElite + numRecomb + numMut)
            print('Incorrect population slicing. Numbers have been scaled to '
                  'fit popSize')
        self.popSize = popSize
        self.genMax = genMax
        self.numElite = numElite
        self.numRecomb = numRecomb
        self.numMut = numMut
        self.numRand = numRand
        self.stagnation = stagnation
        self.fitness_hist = []
        if cpus > 1:
            self.pool = Pool(cpus)
        else:
            self.pool = None

    def add_prototype(self,proto):
        self.proto = proto

    def add_fitness(self,fitness_fcn):
        self.fitness = fitness_fcn

    def init_population(self):
        self.pop_curr = [deepcopy(self.proto) for _ in range(self.popSize)]
        for ind in self.pop_curr:
            ind.randomize()

    def eval_population(self):
        if self.pool is not None:
            res = self.pool.map_async(self.fitness,self.pop_curr)
            fitnessvalues = zip(self.pop_curr,res.get())
        else:
            fitnessvalues = [(ind,self.fitness(ind)) for ind in self.pop_curr]
        sorted_fitness = zip(*sorted(fitnessvalues,key=itemgetter(1)))
        self.pop_curr = sorted_fitness[0]
        self.fitness_hist.append(sorted_fitness[1][0])

    def iter_generation(self,gen_curr=0):
        pop_new = []
        for i in range(self.numElite):
            pop_new.append(deepcopy(self.pop_curr[i]))
        for i in range(0,self.numRecomb + self.numMut,2):
            p1_ind = int(triangular(0,self.popSize,0))
            p2_ind = int(triangular(0,self.popSize,0))
            p1 = self.pop_curr[p1_ind]
            p2 = self.pop_curr[p2_ind]
            c1,c2 = p1.crossover(p2)
            if i >= self.numRecomb:
                c1.mutate(gen_curr/self.genMax)
                c2.mutate(gen_curr/self.genMax)
            pop_new.extend([c1,c2])
        for i in range(self.numRand):
            c1 = deepcopy(self.pop_curr[-i])
            c1.randomize()
            pop_new.append(c1)
        self.pop_curr = pop_new[:]

    def run(self):
        self.init_population()
        for gen in range(self.genMax):
#            print("kajsdnflkjnasdflkjasdff")
            print("Generation: {}, ".format(gen),end='')
            self.eval_population()
            print("Best Fitness: {}".format(self.fitness_hist[-1]))
#            if gen > 10 and\
#                abs(self.fitness_hist[-1] - sum(self.fitness_hist[-10:])/10.) < 0.000001:
#                return self.pop_curr[0]
            self.iter_generation()
        self.eval_population()
        print("Best Fitness: {}".format(self.fitness_hist[-1]))
        return self.pop_curr[0]

    def close(self):
#        self.pool.close()
        pass
