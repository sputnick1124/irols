# -*- coding: utf-8 -*-
"""
Created on Thu Jun  9 08:25:34 2016

@author: nick
"""
from __future__ import division , print_function
from math import exp
from collections import deque
from itertools import cycle, product
from string import ascii_letters as let
import random
  
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

class FIS(object):
    oper =      {'max'      :   max,
                 'min'      :   min,
                 'sum'      :   sum,
                 'prod'     :   prod}
                 
    def __init__(self,name='',fistype='mamdani',andMethod='min',orMethod='max',
                  impMethod='min',aggMethod='max',defuzzMethod='centroid',
                  init=None,inRange=None,outRange=None,
                  rules=None,rule_options=None,rule_selection=None):
        self.defuzz =    {'centroid' :   defuzzCentroid}
        self.input,self.output = [],[]
        self.name = name
        self.type = fistype
        self.andMethod = andMethod
        self.orMethod = orMethod
        self.impMethod = impMethod
        self.aggMethod = aggMethod
        self.defuzzMethod = defuzzMethod
        self.rule = []
        self._mfList = ['trimf','trapmf']
        if init is not None:
            self.init = [None]*4
            if inRange is None or outRange is None:
                r = (-1,1)
                inRange, outRange = cycle([r]),cycle([r])
#                print("No range specified. Defaulting to [-1,1]")
            else:
                inRange = cycle(inRange)
                outRange = cycle(outRange)
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
            if rules:
                self.addrule(rules)
                if rule_options and rule_selection:
                    self.rule_options = rule_options
                    self.rule_selection = rule_selection
            elif rule_options:
                self.rule_options = rule_options
                if not rule_selection:
                    rule_selection = range(prod(self.numInMFs+self.numOutMFs))
            elif rule_selection:
                if not rule_options:
                    self.randRules()
                else:
                    self.rule_options = rule_options
                rules = [rule for i,rule in 
                zip(bitmaskarray(rule_selection,2,prod(self.numInMFs+self.numOutMFs)),
                                                            self.rule_options) 
                                                                        if i]
                self.rule_selection = rule_selection
                self.addrule(rules)
            else:
                self.addrule(self.fillBase(self.numInMFs,self.numOutMFs))
        else:
            self.init = [None]*4

    def __str__(self):
        sys_atts = ['name','type','andMethod','orMethod',
                     'defuzzMethod','impMethod','aggMethod']
        s = ''
        for att in sys_atts:
            s += '{0:>13}: {1}\n'.format(att,self.__dict__[att])
        s += '{:>13}:\n'.format('input')
        for inp in self.input:
            s += inp.__str__('\t')
        s += '{:>13}:\n'.format('output')
        for outp in self.output:
            s += outp.__str__('\t')
        s += '{:>13}:\n'.format('rule')
        for rule in self.rule:
            s += rule.__str__('\t\t') + '\n'
        return s
    
    def __eq__(self,other):
        return self.__dict__ == other.__dict__
    
    def check(self):
        return all(var.check() for var in self.input + self.output)
    
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
        
    def replicate(self,encoded=None,rules=None,rule_options=None,
                  rule_selection=None):
        inRanges = [inp.range for inp in self.input]
        outRanges = [outp.range for outp in self.output]
        if rules is None:
            rules = tuple(r.encode() for r in self.rule)
        
        if encoded and len(encoded) == prod(self.numInMFs + self.numOutMFs)+1:
            rule_selection = encoded.pop()
        retFIS = FIS(init=self.init,inRange=inRanges,outRange=outRanges,
                     rules=rules,rule_options=rule_options,
                     rule_selection=rule_selection)
        if encoded is not None:
            rule_selection = retFIS.decode(encoded)
        else:
            retFIS.decode(self.encode())
        return retFIS,self.check()

    def encode(self,rule=True):
        var = self.input + self.output
        ret = deque(sum((sum([mf.params for mf in v.mf],[]) for v in var),[]))
        if rule and hasattr(self,'rule_selection'):
            ret.append(self.rule_selection)
        return ret

    def decode(self,encoded):
        if not type(encoded) is deque:
            encoded = deque(encoded)
        var = self.input + self.output
        num_mf = [len(v.mf) for v in var]
        for i,num_in in enumerate(num_mf):
            for mf in xrange(num_in):
                params = var[i].mf[mf].params
                var[i].mf[mf].params = [encoded.popleft() for _ in params]
        if encoded:
            return encoded

    def randomize(self,keep_rules=False,ret=False):
        # This only works for well-order param lists (like tri and trap)
        out = deque()
        for var in self.input + self.output:
            for mf in var.mf:
                [out.append(x) for x in 
                       sorted((random.uniform(*var.range) for p in mf.params))]
        if keep_rules:
            return self.replicate(out)
        if not hasattr(self,'rule_options'):
            self.randRules()
        else:
            self.randRules(False)
        rule_selects = random.sample(xrange(self.rule_num_poss),self.rule_num)
        rule_selection = 0;
        for i in rule_selects:
            rule_selection += (1<<i) 
        rule_arg = [rule for i,rule in 
        zip(bitmaskarray(rule_selection,2,self.rule_num_poss),self.rule_options) 
                                                                          if i]
        rules = [arg+(1,0) for arg in rule_arg]
        if ret:
            return out
        else:
            return self.replicate(out,rules,self.rule_options,rule_selection)

    def addvar(self,vartype,varname,varrange):
        if vartype in 'input':
            if self.init[0] is None:
                self.init[:2] = [0,0]
            else:
                self.init[0] *= 10
                self.init[1] *= 512
            self.input.append(FuzzyVar(varname,varrange,self,1))
            if len(self.rule) > 0:
                for rule in self.rule:
                    rule.antecedent += [0]
        elif vartype in 'output':
            if self.init[2] is None:
                self.init[2:] = [0,0]
            else:
                self.init[2] *= 10
                self.init[3] *= 512
            self.output.append(FuzzyVar(varname,varrange,self,0))
            if len(self.rule) > 0:
                for rule in self.rule:
                    rule.consequent += [0]
        else:
            #Throw an invalid variable type exception
            pass

    def rmvar(self,vartype,varindex):
        if vartype in 'input':
            if varindex > len(self.input):
                #throw invalid variable reference exception
                pass
            del self.input[varindex]
            if len(self.input) == 0:
                self.rule = []
                return
            if len(self.rule) > 0:
                for rule in self.rule:
                    del rule.antecedent[varindex]
        elif vartype in 'output':
            if varindex > len(self.output):
                #throw invalid variable reference exception
                pass
            del self.output[varindex]
            if len(self.output) == 0:
                self.rule = []
                return
            if len(self.rule) > 0:
                for rule in self.rule:
                    del rule.consequent[varindex]
        else:
            #Throw an invalid variable type exception
            pass

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
        self.rule_options = self.ruleGen(self.numInMFs + self.numOutMFs)
        self.rule_num = prod(self.numInMFs)
        self.rule_num_poss = prod(self.numInMFs + self.numOutMFs)

    def addrule(self,rules):
        numInput = len(self.input)
        numOutput = len(self.output)
        if not any(hasattr(rule,'__iter__') for rule in rules):
            rules = [rules]
        for rule in rules:
            if not len(rule) != sum([numInput,numOutput,2]):
                #Throw an incorrect number of in/outputs exception
                pass
            antecedent = rule[:numInput]
            consequent = rule[numInput:numInput+numOutput]
            weight = rule[-2]
            connection = rule[-1]
            self.rule.append(Rule(antecedent,consequent,weight,connection))
    
    def evalfis(self,x):
        if not hasattr(x,'__iter__'):
            x = [x]
        elif len(x) != len(self.input):
            #Throw an incorrect number of inputs exception
            pass
        numout = len(self.output)
        ruleout = []
        outputs = []
#        numrule = len(self.rule)
        andMethod = self.oper[self.andMethod]
        orMethod = self.oper[self.orMethod]
        impMethod = self.oper[self.impMethod]
        aggMethod = self.oper[self.aggMethod]
        defuzzMethod = self.defuzz[self.defuzzMethod]
        comb = [andMethod,orMethod]
        for rule in self.rule:
            ruleout.append([])
            ant = rule.antecedent
            con = rule.consequent
            weight = rule.weight
            conn = rule.connection
            mfout = [self.input[i].mf[a].evalmf(x[i]) 
                                    for i,a in enumerate(ant) if a is not None]
            # Generalize for multiple output systems. Easy
            for out in xrange(numout):
                outset = self.output[out].mf[con[out]].evalset()
                rulestrength = weight*comb[conn](mfout)
                ruleout[-1].append([impMethod([rulestrength,y]) for y in outset])
        for o in xrange(numout):
            ruletemp = [r[o] for r in ruleout]
            agg = [aggMethod([y[i] for y in ruletemp]) 
                                             for i in xrange(len(ruletemp[0]))]
            outputs.append(defuzzMethod(agg,self.output[o].range))
        return outputs if len(outputs)>1 else outputs[0]

class FuzzyVar(object):
    def __init__(self,varname,varrange,parent=None,vartype=None):
        self.name = varname
        self.range = varrange
        self.mf = []
        self.parent = parent
        if vartype:
            self.num = len(parent.input)
        elif vartype == 0:
            self.num = len(parent.output)
        self.vartype = vartype

    def __str__(self,indent=''):
        var_atts = ['name','range']
        s = ''
        for att in var_atts:
            s += indent + '{0:>10}: {1}\n'.format(att,self.__dict__[att])
        s += indent + '{:>10}:\n'.format('mf')
        for mf in self.mf:
            s += mf.__str__(indent+'\t') + '\n'
        return s
    
    def __eq__(self,other):
        local_dict = self.__dict__.copy()
        other_dict = other.__dict__.copy()
        local_dict.pop('parent')
        other_dict.pop('parent')
        return local_dict == other_dict
        
    def check(self):
        retval =  any(mf.evalmf(None) for mf in self.mf)
        print(self)
        print(retval)
        return retval
        
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

class MF(object):
    def __init__(self,mfname,mftype,mfparams,parent=None):
        self.name = mfname
        self.type = mftype
        
        mfdict = {'trimf'           :   (mfTriangle,3),
                  'trapmf'          :   (mfTrapezoid,4),
                  'trunctrilumf'    :   (mfTruncTriLeftUpper,4),
                  'trunctrillmf'    :   (mfTruncTriLeftLower,4),
                  'trunctrirumf'    :   (mfTruncTriRightUpper,4),
                  'trunctrirlmf'    :   (mfTruncTriRightLower,4),
                  'gaussmf'         :   (mfGaussian,2),
                  'gauss2mf'        :   (mfGaussian2,2)}
        
        self.mf = mfdict[self.type][0]
        if mfparams is not None:
            self.params = mfparams
        elif parent is not None:
            r = parent.range[1] - parent.range[0]
            p = mfdict[self.type][1]
            dr = r/(p-1)
            self.params = [parent.range[0]+i*dr for i in xrange(p)]
        else:
            self.params = [0]*mfdict[self.type][1]
        if not mfdict[self.type][1] == len(self.params):
            #Throw invalid param number exception
#            raise(Exception)
            pass
        try:
            self.mf(None,self.params)
#            self.params = mfparams
        except ParamError as e:
            raise e

    def __str__(self,indent=''):
        mf_atts = ['name','type','params']
        s = ''
        for att in mf_atts:
            s += indent + '{0:>10}: {1}\n'.format(att,self.__dict__[att])
        return s
    
    def __eq__(self,other):
        local_dict = self.__dict__.copy()
        other_dict = other.__dict__.copy()
        local_dict.pop('parent')
        other_dict.pop('parent')
        return local_dict == other_dict

    def evalmf(self,x):
        return self.mf(x,self.params)
        
    def evalset(self,points=101):
        if hasattr(self,'range'):
            a,b = self.range
        elif not 'gauss' in self.type:
            a,b = self.params[0],self.params[-1]
        else:
            a = self.params[1] - 6*self.params[0]
            b = -a
        #Fake linspace until I bring in numpy for the time being. Baby steps...
        dx = (b - a)/(points-1)
        xlinspace = [a + i*dx for i in xrange(points-1)] + [b]
        return [self.mf(x,self.params) for x in xlinspace]
        
class Rule(object):
    def __init__(self,antecedent,consequent,weight,connection):
        self.antecedent = antecedent
        self.consequent = consequent
        self.weight = weight
        self.connection = connection

    def __str__(self,indent=''):
        num_a = len(self.antecedent)
        num_c = len(self.consequent)
        ant = ' '.join('{%d!s:>4}'%i for i in xrange(num_a))
        con = ' '.join('{%d!s:>4}'%i for i in xrange(num_a,num_a+num_c))
        s = ant + ', ' + con + '  ({%d}) : {%d}'%(num_a+num_c,num_a+num_c+1)
        a = tuple(self.antecedent) + tuple(self.consequent) + \
                                     (self.weight,self.connection,)
        return indent + s.format(*a)
    
    def __eq__(self,other):
        return self.__dict__ == other.__dict__
    
    def encode(self):
        return self.antecedent + self.consequent + (self.weight, self.connection)

def mfTriangle(x,params):
    a,b,c = params
    check = [a==b,b==c,a<=x<=c]
    if x is None:
        err = [b<a,c<b]
        if any(err):
            #Throw an invalid param exception
            errs = ['b<a','c<b']
            e = [errs[i] for i,_ in enumerate(err) if _]
            raise ParamError(params,'Invalid params {}: {}'.format(e,params))
        else:
            return 0 #Everything looks good
    if not check[2]:
#        print('outside of range')
        return 0
    if check[0]:
        return (c-x)/(c-b)
    elif check[1]:
        return (x-a)/(b-a)
    else:
        return min((x-a)/(b-a),(c-x)/(c-b))
        
def mfTrapezoid(x,params):
    a,b,c,d = params
    check = [a==b,b==c,c==d,a<=x<=d]
    if x is None:
        err = [b<a,c<b,d<c]
        if any(err):
            #Throw an invalid param exception
            errs = ['b<a','c<b','d<c']
            e = [errs[i] for i,_ in enumerate(err) if _]
            raise ParamError(params,'Invalid params {}: {}'.format(e,params))
        else:
            return 0 #Everything looks good
    if not check[3]:
#            print('outside of range')
        return 0
    if check[1]:
        return mfTriangle(x,[a,b,d])
    if check[0]:
        return min(1,(d-x)/(d-c))
    if check[2]:
        return min((x-a)/(b-a),1)
    else:
        return min((x-a)/(b-a),1,(d-x)/(d-c))

def mfTruncTriLeftUpper(x,params):
    a,b,c,d = params
    if x is None:
        err = [b<a,c<b,d<c]
        if any(err):
            #Throw an invalid param exception
            errs = ['b<a','c<b','d<c']
            e = [errs[i] for i,_ in enumerate(err) if _]
            raise ParamError(params,'Invalid params {}: {}'.format(e,params))
        else:
            return 0 #Everything looks good
    if x<=c:
        return 1
    else:
        return mfTriangle(x,[a,b,d])

def mfTruncTriLeftLower(x,params):
    a,b,c,d = params
    if x is None:
        err = [b<a,c<b,d<c]
        if any(err):
            #Throw an invalid param exception
            errs = ['b<a','c<b','d<c']
            e = [errs[i] for i,_ in enumerate(err) if _]
            raise ParamError(params,'Invalid params {}: {}'.format(e,params))
        else:
            return 0 #Everything looks good
    if x<=b:
        return 0
    else:
        return mfTriangle(x,[a,c,d])

def mfTruncTriRightUpper(x,params):
    a,b,c,d = params
    if x is None:
        err = [b<a,c<b,d<c]
        if any(err):
            #Throw an invalid param exception
            errs = ['b<a','c<b','d<c']
            e = [errs[i] for i,_ in enumerate(err) if _]
            raise ParamError(params,'Invalid params {}: {}'.format(e,params))
        else:
            return 0 #Everything looks good
    if x>=b:
        return 1
    else:
        return mfTriangle(x,[a,c,d])

def mfTruncTriRightLower(x,params):
    a,b,c,d = params
    if x is None:
        err = [b<a,c<b,d<c]
        if any(err):
            #Throw an invalid param exception
            errs = ['b<a','c<b','d<c']
            e = [errs[i] for i,_ in enumerate(err) if _]
            raise ParamError(params,'Invalid params {}: {}'.format(e,params))
        else:
            return 0 #Everything looks good
    if x>=c:
        return 0
    else:
        return mfTriangle(x,[a,b,d])

def mfGaussian(x,params):
    sigma,c = params
    if sigma == 0:
        #Throw invalid param exception
        pass
    t = (x-c)/sigma
    return exp(-t*t/2)

def mfGaussian2():
    pass
 
def defuzzCentroid(agg,outrange):
    a,b = outrange
    points = len(agg)
    dx = (b-a)/(points - 1)
    totarea = sum(agg)
    if totarea == 0:
        print('Total area was zero. Using average of the range instead')
        return (a+b)/2
    totmom = 0
    for i,y in enumerate(agg):
        totmom += y*(a + i*dx)
    return totmom/totarea

class Error(Exception):
    pass

class ParamError(Error):
    def __init__(self,params,msg):
        self.params = params
        self.msg = msg
    def __str__(self):
        return repr(self.msg)