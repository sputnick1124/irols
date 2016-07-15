# -*- coding: utf-8 -*-
"""
Created on Tue Jun 28 21:05:15 2016

@author: nick
"""
from __future__ import print_function, division, absolute_import
import sys
sys.path.append('.')
from yapflm import FIS, FuzzyVar, MF, ParamError
from collections import deque
from itertools import cycle, product
from operator import itemgetter
from string import ascii_letters as let
import random
from multiprocessing import Pool

def prod(x):
    y = 1
    for _ in x:
        y *= _
    return y

def bitmaskarray(n,base=512,length=None):
    retval = []
    i = 1
    while n >= base:
        retval.append(n%base)
        n //= base
        i += 1
    retval.append(n)
    if length is not None:
        retarray = [0]*length
        retarray[:i] = retval
        return retarray[::-1]
    return retval[::-1]

def storebits(a,base=512):
    retval = 0
    for i,x in enumerate(a[::-1]):
        retval += x * (base**i)
    return retval

class GA(object):
    def __init__(self,popSize=100,genMax=200,numElite=15,numRecomb=50,
                 numMut=25,numRand=10,stagnation=10):
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
        self.numParents = popSize//5
        self.stagnation = stagnation

    def addSystem(self,system):
        if not any(hasattr(system,x) for x in ['encode','replicate','randomize']):
            return
        self.system = system
        self.ranges = system.getRanges()
        self.initPop()
        
    def evalGA(self):
        self.cur_gen = 0
        self.gen_fitnesses = []
        self.best_fit = []
        moving_fit_avg, most_fit = deque(maxlen=self.stagnation),[]
        multipool = Pool(4)
        while self.cur_gen < self.genMax:
            print("Generation %d" % self.cur_gen)
            #Evaluate and rank the population
            try:
                results = multipool.map_async(self.fitness,self.populations[self.cur_gen],
                                              chunksize = self.popSize//8)
                fitness_vals = zip(results.get(),range(self.popSize))
            except Exception as e:
                multipool.close()
                multipool.join()
                raise(e)
            pop_fitness,fitness_indices = zip(*sorted(fitness_vals,key=itemgetter(0)))
            self.best_fit.append(fitness_indices[0])
            most_fit.append(pop_fitness[0])
            moving_fit_avg.append(pop_fitness[0])
            if self.stagnation is not None and self.cur_gen >= self.stagnation:
                if abs(sum(moving_fit_avg)/self.stagnation - most_fit[-1]) < 1e-5:
                    break
            self.gen_fitnesses.append(fitness_vals)
            getElite = itemgetter(*fitness_indices[:self.numElite])
            elites = getElite(self.populations[self.cur_gen])
            parent_pool_indices = list(fitness_indices[self.numElite:])
            parents = []
            for i in xrange(self.numParents):
                choose = random.sample(parent_pool_indices,3)
                getContestants = itemgetter(*choose)
                winner = min(getContestants(fitness_vals,key=itemgetter(0)))[1]
                parents.append(self.populations[self.cur_gen][winner])
                del parent_pool_indices[parent_pool_indices.index(winner)]
            parents += elites
            children = [self.crossover(*random.sample(parents,2)) for i in xrange(self.numRecomb+self.numMut)]
            children[-self.numMut:] = map(self.mutate,children[-self.numMut:])
            randoms = tuple(self.system.randomize() for i in xrange(self.numRand))
            newPop = elites + tuple(children) + randoms
            self.populations.append(newPop)
            self.cur_gen += 1
        multipool.close()
        multipool.join()
        return self.populations[-1][0]
    
    def initPop(self):
        self.populations = [[self.system.randomize() for i in xrange(self.popSize)]]

    def evalPopulation(self,pop):
        return [(self.fitness(individual),i) for i,individual in enumerate(pop)]
    
    def addFitness(self,fitness_fun):
        self.fitness = fitness_fun
        
    def crossover(self,parent1,parent2,debug=None):
        x1 = parent1.encode()
        x2 = parent2.encode()
        xMin,xMax = zip(*[(min(a,b),max(a,b)) for a,b in zip(x1,x2)])
        child,param_num = [],0
        if debug:
            stuff = []
        for group in sum(self.ranges,[]):
            for i,param_lim in enumerate(group):
                ak,bk = param_lim
                alpha = abs((xMax[param_num]-xMin[param_num])/(bk-ak))
                I = min(xMin[param_num] - ak,bk - xMax[param_num])
                low = xMin[param_num] - I*alpha
                if debug:
                    stuff.append(
                    (param_num, round(I,5), round(alpha,5), round(low,5), round(xMax[param_num]+I*alpha,5))
                    )
                if i > 0:
                    if low >= child[param_num-1]:
                        pass
                    else:
                        low = child[param_num-1]
                newRange = (low,xMax[param_num] + I*alpha)
                if debug:
                    stuff += newRange
                child.append(random.uniform(*newRange))
#                child.append(sum(newRange)/2.)
#                newranges.append(newRange)
                param_num += 1
        if x1[param_num] and x2[param_num] and x1[param_num] != x2[param_num]:
            bitmask = (1<<self.system.rule_num_poss) - 1
            crossover_point = (1<<random.randint(0,self.system.rule_num_poss)) - 1
            new_rule_selection = (
                (x1[param_num] & (bitmask - crossover_point)) 
                                | 
                (x2[param_num] & crossover_point))
            if not new_rule_selection:
                new_rule_selection = (
                    (x2[param_num] & (bitmask - crossover_point)) 
                                    | 
                    (x1[param_num] & crossover_point))
#            print('cx',new_rule_selection,x1[param_num],x2[param_num],crossover_point)
            child.append(new_rule_selection)
        else:
            child.append(x1[param_num])
        if debug:
            return self.system.replicate(child),stuff
        return self.system.replicate(child)
    
    def mutate(self,child,num_genomes=3,b=1):
        coin_flips = [random.randint(0,1) for gen in xrange(num_genomes)]
        lambdas = [random.random() for i in xrange(num_genomes)]
        mut = child.encode()
        mut_genomes = random.sample(xrange(len(mut)),num_genomes)
        param_num = 0
        ranges = []
        for group in sum(self.ranges,[]):
            for i,param_lim in enumerate(group):
                if i == 0:
                    if len(mut)-1 == param_num:
                        ranges.append(param_lim)
                    else:
                        ranges.append((param_lim[0],mut[param_num+1]))
                elif i == len(group)-1:
                    ranges.append((mut[param_num-1],param_lim[1]))
                else:
                    ranges.append((mut[param_num-1],mut[param_num+1]))
                param_num += 1
        for genome,lamb,flip in zip(mut_genomes,lambdas,coin_flips):
            if genome == child.numParams:
#                print(len(mut),param_num,child.numParams,mut[-1])
                if mut[genome] is not None:
                    mut[genome] ^= (1<<random.randint(0,self.system.rule_num_poss-1))
                    if not mut[genome]:
                        mut[genome] ^= (1<<random.randint(0,self.system.rule_num_poss-1))
#                    print('mut',mut[genome])
                break
            if flip:
                mut[genome] = ranges[genome][0] +\
                                (mut[genome]-ranges[genome][0]) *\
                                (1 - lamb*(1 - self.cur_gen/self.genMax)**b)
            else:
                mut[genome] = mut[genome] +\
                                (ranges[genome][1] - mut[genome]) *\
                                (1 - lamb*(1 - self.cur_gen/self.genMax)**b)
        return self.system.replicate(mut)

class GFS(FIS):
    def __init__(self,init=None,inRange=None,outRange=None,
                  rules=None,rule_options=None,rule_selection=None,**fis_kwargs):
        super(GFS,self).__init__(**fis_kwargs)
        self._mfList = ['trimf','trapmf']
        if init is not None:
            self.init = [None]*4
            if inRange is None or outRange is None:
                r = (0,1)
                inRange, outRange = cycle([r]),cycle([r])
#                print("No range specified. Defaulting to [-1,1]")
            else:
                if hasattr(inRange[0],'__iter__'):
                    inRange = cycle(inRange)
                else:
                    inRange = cycle([inRange])
                if hasattr(outRange[0],'__iter__'):
                    outRange = cycle(outRange)
                else:
                    outRange = cycle([outRange])
            self.numInMFs = bitmaskarray(init[0],10)
            typeInMFs = bitmaskarray(init[1],512,len(self.numInMFs))
            for i,(inp,m) in enumerate(zip(self.numInMFs,typeInMFs)):
                self.addvar('input','input%d'%i,inRange.next())
                for j,mf in enumerate(bitmaskarray(m,2,inp)):
                    try:
                        self.input[-1].addmf('%s%d'%(let[j],i),self._mfList[mf])
                    except ParamError as e:
                        print("ParamError: {0}".format(e))
            self.numOutMFs = bitmaskarray(init[2],10)
            typeOutMFs = bitmaskarray(init[3],512,len(self.numOutMFs))
            for i,(outp,m) in enumerate(zip(self.numOutMFs,typeOutMFs)):
                self.addvar('output','output%d'%i,outRange.next())
                for j,mf in enumerate(bitmaskarray(m,2,outp)):
                    try:
                        self.output[-1].addmf('%s%d'%(let[j],i),self._mfList[mf])
                    except ParamError as e:
                        print("ParamError: {0}".format(e))
##            if rule_options:
#                self.rule_options = rule_options
##            if rule_selection:
#                self.rule_selection = rule_selection
#                if not hasattr(self,'rule_options'):
#                    print('You need to supply rule_options for me')
            if rules:
                self.addrule(rules)
            elif rule_selection is not None and rule_options:
                rules = [rule+(1,0) for i,rule in 
                zip(bitmaskarray(rule_selection,2,prod(self.numInMFs+self.numOutMFs))[::-1],
                                                            rule_options) 
                                                                        if i]
                self.rule_selection = rule_selection
                self.addrule(rules)
            else:
                print('no rules added')
        else:
            self.init = [None]*4
        self.numParams = self.decode(None)
        self.keep_rules = True

    def addvar(self,vartype,varname,varrange):
        if vartype in 'input':
            if self.init[0] is None:
                self.init[:2] = [0,0]
            else:
                self.init[0] *= 10
                self.init[1] *= 512
            self.input.append(GenFuzzyVar(varname,varrange,self,1))
            if len(self.rule) > 0:
                for rule in self.rule:
                    rule.antecedent += [0]
        elif vartype in 'output':
            if self.init[2] is None:
                self.init[2:] = [0,0]
            else:
                self.init[2] *= 10
                self.init[3] *= 512
            self.output.append(GenFuzzyVar(varname,varrange,self,0))
            if len(self.rule) > 0:
                for rule in self.rule:
                    rule.consequent += [0]
        else:
            #Throw an invalid variable type exception
            pass

    def config(self):
        if len(self.input) == 0 or len(self.output) == 0:
            return None
        self.init = []
        self.init.append(storebits([len(i.mf) for i in self.input],10))
        self.init.append(storebits([storebits([self._mfList.index(mf.type) 
                        for mf in inp.mf],2) for inp in self.input],512))
        self.init.append(storebits([len(outp.mf) for outp in self.output],10))
        self.init.append(storebits([storebits([self._mfList.index(mf.type) 
                        for mf in outp.mf],2) for outp in self.output],512))
        return self.init
        
    def replicate(self,encoded=None,rule_selection=None):
        inRanges = [inp.range for inp in self.input]
        outRanges = [outp.range for outp in self.output]
        if self.keep_rules:
            rules = tuple(r.encode() for r in self.rule)
            retFIS = GFS(init=self.init,inRange=inRanges,outRange=outRanges,
                         rules=rules)
        elif rule_selection:
            retFIS = GFS(init=self.init,inRange=inRanges,outRange=outRanges,
                         rule_options=self.rule_options,
                         rule_selection=rule_selection)
        elif encoded and len(encoded) == self.numParams+1:
#            print(encoded[-1])
            rule_selection = encoded.pop()
            inRanges = [var.range for var in self.input]
            outRanges = [var.range for var in self.output]
            retFIS = GFS(init=self.init,inRange=inRanges,outRange=outRanges,
                     rule_options=self.rule_options,rule_selection=rule_selection)
            retFIS.decode(encoded)
            return retFIS
        else:
            rules = tuple(r.encode() for r in self.rule)
            retFIS = GFS(init=self.init,inRange=inRanges,outRange=outRanges,
                         rules=rules)
        if encoded:
            retFIS.decode(encoded)
        else:
            retFIS.decode(self.encode())
        return retFIS #,self.check()

    def encode(self,rule=True):
        var = self.input + self.output
        ret = deque(sum((sum([mf.params for mf in v.mf],[]) for v in var),[]))
        if rule and hasattr(self,'rule_selection'):
            ret.append(self.rule_selection)
        return ret

    def decode(self,encoded):
        ret = False
        if encoded is None:
            count = 0
            ret = True
        elif not type(encoded) is deque:
            encoded = deque(encoded)
        var = self.input + self.output
        num_mf = [len(v.mf) for v in var]
        for i,num_in in enumerate(num_mf):
            for mf in xrange(num_in):
                params = var[i].mf[mf].params
                if encoded is None:
                    for _ in params:
                        count += 1
                else:
                    var[i].mf[mf].params = [encoded.popleft() for _ in params]
        if ret:
            return count

    def randomize(self,ret=False):
        # This only works for well-ordered param lists (like tri and trap)
        out = deque()
        for var in self.input + self.output:
            for mf in var.mf:
                [out.append(x) for x in 
                       sorted((random.uniform(*var.range) for p in mf.params))]
        if self.keep_rules:
            return self.replicate(out)
        if not hasattr(self,'rule_options'):
            self.randRules()
        elif not hasattr(self,'rule_num_poss'):
            self.randRules(False)
        rule_selects = random.sample(xrange(self.rule_num_poss),self.rule_num)
        rule_selection = 0;
        for i in rule_selects:
            rule_selection += (1<<i) 
#            print('rn',rule_selection)
#        rule_arg = [rule for i,rule in 
#        zip(bitmaskarray(rule_selection,2,self.rule_num_poss),self.rule_options) 
#                                                                          if i]
#        rules = [arg+(1,0) for arg in rule_arg]
        if ret:
            return out
        else:
#            print(rule_selection)
            return self.replicate(out,rule_selection)

    def _flatten(self,nested_iter):
        if hasattr(nested_iter,'__iter__'):
            for i in nested_iter:
                if hasattr(i,'__iter__'):
                    for j in self._flatten(i):
                        yield j
                else:
                    yield i
        else:
            yield nested_iter

    def ruleGen(self,mfs):
        ranges = map(range,mfs)
        return tuple(tuple(self._flatten(x)) for x in reduce(product,ranges))

    def fillBase(self,ins,outs):
        in_args = self.ruleGen(ins)
        if len(outs) > 1:
            out_args = cycle(self.ruleGen(outs))
            return tuple(inarg+outarg+(1,0) for inarg,outarg in zip(in_args,out_args))
        else:
            out_arg_samples = cycle(xrange(outs[0]))
            out_args = sorted([out_arg_samples.next() for i in xrange(prod(ins))])
            return tuple(inarg+(outarg,1,0) for inarg,outarg in zip(in_args,out_args))

    def randRules(self,gen_rules=True):
        self.numInMFs = bitmaskarray(self.init[0],10)
        self.numOutMFs = bitmaskarray(self.init[2],10)
        if gen_rules:
            self.rule_options = self.ruleGen(self.numInMFs + self.numOutMFs)
            rule_selects = [self.rule_options.index(r.antecedent+r.consequent) for r in self.rule]
            self.rule_selection = sum((1<<x) for x in rule_selects)
        self.rule_num = prod(self.numInMFs)
        self.rule_num_poss = prod(self.numInMFs + self.numOutMFs)
    
    def getRanges(self):
        ranges = []
        for var in self.input+self.output:
            ranges.append([tuple(var.range for p in mf.params) for mf in var.mf])
        if hasattr(self,'rule_num_poss'):
            ranges.append([([0,(1<<self.rule_num_poss)-1],)])
        return ranges

class GenFuzzyVar(FuzzyVar):
    def addmf(self,mfname,mftype,mfparams=None):
        try:
            mf = MF(mfname,mftype,mfparams,self)
        except ParamError as e:
            raise e
        mf.range = self.range
        mftypes = {'trimf':0, 'trapmf':1}
        self.mf.append(mf)
        if self.vartype is None:
            return
        elif self.vartype:
            numIn = len(self.parent.input)
            self.parent.init[0] += 10**(numIn - self.num - 1)
            
            typeMFs = bitmaskarray(self.parent.init[1])
            if len(typeMFs) <= numIn:
                typeMFs = [0]*(numIn-len(typeMFs)) + typeMFs
            typeMFs[self.num] = (typeMFs[self.num]<<1) + mftypes[mftype]
            self.parent.init[1] = storebits(typeMFs)
        else:
            numOut = len(self.parent.output)
            self.parent.init[2] += 10**(numOut - self.num - 1)
            
            typeMFs = bitmaskarray(self.parent.init[3])
            if len(typeMFs) <= numOut:
                typeMFs = [0]*(numOut-len(typeMFs)) + typeMFs
            typeMFs[self.num] = (typeMFs[self.num]<<1) + mftypes[mftype]
            self.parent.init[3] = storebits(typeMFs)
        