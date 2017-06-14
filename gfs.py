#!/usr/bin/env python
from __future__ import division
from yapflm import FIS, FuzzyVar, FuzzyRule
from operator import mul

from itertools import product
from random import random, randint
from copy import deepcopy

def in_range(x,interval):
    """
    Helper function to test inclusion of x in interval
    """
    return interval[0] <= x <= interval[1]

class GFS(FIS):
    """
    A genetic representation of a Fuzzy system. Contains methods to reproduce,
    mutate, and randomize its own parameters.
    """
    def __init__(self,in_mfs,out_mfs,name=''):
        super(GFS,self).__init__(name=name)
        if not hasattr(in_mfs,'__iter__'):
            self.in_mfs = [in_mfs]
        else:
            self.in_mfs = in_mfs
        if not hasattr(out_mfs,'__iter__'):
            self.out_mfs = [out_mfs]
        else:
            self.out_mfs = out_mfs
        self.num_in = len(self.in_mfs)
        self.num_out = len(self.out_mfs)
        self.num_rule = reduce(mul,self.in_mfs)
        
        self.init_vars()
        self.init_rules()

    def init_vars(self):
        for input_mfs in self.in_mfs:
            self.addvar(input_mfs,'input','')
        for output_mfs in self.out_mfs:
            self.addvar(output_mfs,'output','')

    def init_rules(self):
        self.rule = GenFuzzyRuleBase(in_mfs=self.in_mfs,out_mfs=self.out_mfs)

    def randomize(self):
        self.randomize_vars()
        self.randomize_rule_base()

    def randomize_vars(self):
        for inp in self.input:
            inp.randomize()
        for outp in self.output:
            outp.randomize()

    def randomize_rule_base(self):
        self.rule.randomize()

    def crossover(self,other):
        c1 = deepcopy(self)
        c2 = deepcopy(self)
        for i in range(len(self.input)):
            s1,s2 = self.input[i].crossover(other.input[i])
            c1.input[i].update(s1)
            c2.input[i].update(s2)
        for o in range(len(self.output)):
            s1,s2 = self.output[o].crossover(other.output[o])
            c1.output[o].update(s1)
            c2.output[o].update(s2)
        c1.rule,c2.rule = self.rule.crossover(other.rule)
        return c1,c2

    def mutate(self):
        pass

    def addvar(self,num_mfs,vartype,varname):
        if vartype in 'input':
            self.input.append(GenFuzzyVar(num_mfs,1))
        elif vartype in 'output':
            self.output.append(GenFuzzyVar(num_mfs,0))
        else:
            #Throw an invalid variable type exception
            pass

class GenFuzzyVar(FuzzyVar):
    def __init__(self,num_mfs,vartype=None,varrange=None):
        super(GenFuzzyVar,self).__init__(varname='',vartype=vartype)
        self.num_mfs = num_mfs
        self.num_params = num_mfs*3
        if varrange is None:
            self.varrange = (0,1)
        else:
            self.varrange = varrange
        bins = []
        interval = (self.varrange[1]-self.varrange[0])/(num_mfs-1)
        # make the 'buckets' in which the params need to reside
        bins.append((self.varrange[0],interval/2))
        for mf in range(1,num_mfs-1):
            bins.append(
            (self.varrange[0] + interval*mf - interval/2,
             self.varrange[0] + interval*mf + interval/2))
        bins.append((self.varrange[1] - interval/2, 1))
        # assign a bucket to each of the params
        self.buckets = [(self.varrange[0],)*2]*2 + [bins[1]]        
        for p in range(1,len(bins)-1):
            self.buckets.append(bins[p-1])
            self.buckets.append(bins[p])
            self.buckets.append(bins[p+1])
        self.buckets.append(bins[-2])
        self.buckets.extend([(self.varrange[1],)*2]*2)

    def serialize(self):
        return sum([mf.params for mf in self.mf],[])

    def update(self,params):
        if not len(self.mf):
            for mf in range(self.num_mfs):
                self.addmf(mfname='',mfparams=params[mf*3:mf*3+3])
        else:
            for ind,mf in enumerate(self.mf):
                mf.params = params[ind*3:ind*3+3]


    def randomize(self):
        params = [random()*(b-a)+a for a,b in self.buckets]
        self.update(params)

    def mutate(self,num_gene=2,perc=0):
        for g in range(num_gene):
            m = randint(2,self.num_params-2)
            r = random()
            cur = self.mf[m//3].params[m%3]
            tau = randint(0,1)
            if tau:
                delta = (self.buckets[m][1] - cur) * (1-r*(1-perc**1.5))
            else:
                delta = -(cur - self.buckets[m][0]) * (1-r*(1-perc**1.5))            
            self.mf[m//3].params[m%3] += delta

    def crossover(self,other):
        p1 = self.serialize()
        p2 = other.serialize()
        c1 = p1[:]
        c2 = p2[:]
        for p in range(2,self.num_params-2):
            xmin,xmax = min(p1[p],p2[p]),max(p1[p],p2[p])
            bmin,bmax = self.buckets[p]
            I = xmax - xmin
            R = bmax - bmin
            lamb = random()
            new_xmin = xmin - (I/R)*(xmin - bmin)
            new_xmax = xmax + (I/R)*(bmax - xmax)
            c1[p] = lamb*new_xmin + (1-lamb)*new_xmax
            c2[p] = (1-lamb)*new_xmin + lamb*new_xmax
        return c1,c2

class GenFuzzyRuleBase(object):
    def __init__(self,in_mfs=None,out_mfs=None,antecedents=None,consequents=None):
        self.rules = []
        if antecedents is not None:
            self.antecedents = antecedents
        else:
            self.init_antecedents(in_mfs)
        self.out_mfs = out_mfs
        self.weights = [1]*len(self.antecedents)
        self.connections = [1]*len(self.antecedents)
        if consequents is not None:
            self.consequents = consequents
            self.update()
        else:
            self.consequents = None

    def __iter__(self):
        return (rule for rule in self.rules)

    def __getitem__(self,key):
        return self.rules[key]
    
    def append(self,item):
        self.rules.append(item)

    def init_antecedents(self,in_mfs):
        ranges = map(range,in_mfs)
        self.antecedents = [list(x) for x in product(*ranges)]

    def update(self):
        if not len(self.rules):
            self.rules = [FuzzyRule(ant,con,w,c) for ant,con,w,c in zip(
                                            self.antecedents,
                                            self.consequents,
                                            self.weights,
                                            self.connections)]
        else:
            for rule,con in zip(self.rules,self.consequents):
                rule.consequent = con[:]
    
    def serialize(self):
        return self.consequents

    def randomize(self):
        self.consequents = []
        for ant in self.antecedents:
            self.consequents.append([randint(0,mf-1) for mf in self.out_mfs])
        self.update()

    def mutate(self,num_gen=2):
        up_down = (-1,1)
        for g in range(num_gen):
            tau = randint(0,1)
            m = randint(0,len(self.consequents))
            c = randint(0,len(self.out_mfs)-1)
            if self.consequents[m][c] == 0:
                self.consequents[m][c] += 1
            elif self.consequents[m][c] == self.out_mfs[c]-1:
                self.consequents[m][c] -= 1
            else:
                self.consequents[m][c] += up_down[tau]
        self.update()
            

    def crossover(self,other):
        """
        Two-point crossover
        """
        p1 = self.consequents[:]
        p2 = other.consequents[:]
        i,j  = randint(0,len(self.consequents)), randint(0,len(self.consequents))
        c1 = p1[:i] + p2[i:j] + p1[j:]
        c2 = p2[:i] + p1[i:j] + p2[j:]
        return GenFuzzyRuleBase(antecedents=self.antecedents,consequents=c1),\
               GenFuzzyRuleBase(antecedents=self.antecedents,consequents=c2)
