# -*- coding: utf-8 -*-
"""
Created on Thu Jun  9 08:25:34 2016

@author: nick
"""
from __future__ import division , print_function
import numpy as np
  
class FIS(object):
#    comboper =  {'max'      :   np.max,
#                 'min'      :   np.min}
#                 
#    oper =      {'max'      :   np.maximum,
#                 'min'      :   np.minimum,
#                 'sum'      :   np.sum,
#                 'prod'     :   np.prod}
    comboper =  {'max'      :   max,
                 'min'      :   min}
                 
    oper =      {'max'      :   max,
                 'min'      :   min,
                 'sum'      :   sum,
                 'prod'     :   np.prod}
                 
    def __init__(self,name='',fistype='mamdani',andMethod='min',orMethod='max',
                  impMethod='min',aggMethod='max',defuzzMethod='weightedMV'):
        self.defuzz =    {'centroid' :   defuzzCentroid,
                          'weightedMV':  defuzzWeightedMV} 
        self.input,self.output = [],[]
        self.name = name
        self.type = fistype
        self.andMethod = andMethod
        self.orMethod = orMethod
        self.impMethod = impMethod
        self.aggMethod = aggMethod
        self.defuzzMethod = defuzzMethod
        self.rule = []
        self._points = 101

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

    def addvar(self,vartype,varname,varrange):
        if vartype in 'input':
            self.input.append(FuzzyVar(varname,varrange,self,1))
            if len(self.rule) > 0:
                for rule in self.rule:
                    rule.antecedent += [0]
        elif vartype in 'output':
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

    def addrule(self,rules):
#        print(rules)
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
        andMethod = self.comboper[self.andMethod]
        orMethod = self.comboper[self.orMethod]
        impMethod = self.oper[self.impMethod]
        aggMethod = self.comboper[self.aggMethod]
        defuzzMethod = self.defuzz[self.defuzzMethod]
        comb = [andMethod,orMethod]
#        self.output_x = [np.linspace(*out.range,num=self._points) for out in self.output]
        for rule in self.rule:
            ruleout.append([])
            ant = rule.antecedent
            con = rule.consequent
            weight = rule.weight
            conn = rule.connection
            mfresult = [self.input[i].mf[a].evalmf(x[i]) 
                                    for i,a in enumerate(ant) if a is not None]
            # Generalize for multiple output systems. Easy
            rulestrength = weight*comb[conn](mfresult)
            for out in xrange(numout):
                outmf = self.output[out].mf[con[out]].evalmf(rulestrength)
                ruleout[-1].append(outmf)
        for o in xrange(numout):
#            ruletemp = [r[o] for r in ruleout]
#            agg = [aggMethod([y[i] for y in ruletemp]) for i in xrange(len(ruletemp[0]))]
#            agg = aggMethod(ruleout,axis=0)
            outs = [y[o] for y in ruleout]
            outputs.append(defuzzMethod(outs))
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
        return retval

    def addmf(self,mfname,mftype,mfparams=None):
        try:
            mf = MF(mfname,mftype,mfparams,self)
        except ParamError as e:
            raise e
        mf.range = self.range
        self.mf.append(mf)


class MF(object):
    def __init__(self,mfname,mftype,mfparams,parent=None):
        self.name = mfname
        self.type = mftype
        if parent.vartype == 1:
            mfdict = {'trimf'           :   (InputTriMF,3)}
        elif parent.vartype == 0:
            mfdict = {'trimf'           :   (OutputTriMF,3)}

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
            raise ParamError(params,'len({}) != %d'.format(params,mfdict[self.type][1]))
        self.mf = mfdict[self.type][0](mfparams)
        
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
        return self.mf(x)
        
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

class TriMF(object):
    def __init__(self,params):
        a,x_star,b = params
        err = [x_star < a, x_star > b]
        if any(err):
            #Throw an invalid param exception
            errs = ['b<a','c<b']
            e = [errs[i] for i,_ in enumerate(err) if _]
            raise ParamError(params,'Invalid params {}: {}'.format(e,params))
        if a != x_star:
            self.m1 = 1 / (x_star - a)
            self.b1 = -self.m1 * a
        else:
            self.m1, self.b1 = None, None
        if b != x_star:
            self.m2 = -1 / (b - x_star)
            self.b2 = 1 - (self.m2 * x_star)
        else:
            self.m2, self.b2 = None, None
        self.a, self.b = a, b
        self.x_star = x_star

class InputTriMF(TriMF):
    def __init__(self,*args,**kwargs):
        super(InputTriMF,self).__init__(*args,**kwargs)
        self.fns = [(lambda x: (self.m1 * x) + self.b1) if self.m1 else None,
                    (lambda x: (self.m2 * x) + self.b2) if self.m2 else None]
    
    def __call__(self,x):
        return self.fns[x >= self.x_star](x)
    
class OutputTriMF(TriMF):
    def __init__(self,*args,**kwargs):
        super(OutputTriMF,self).__init__(*args,**kwargs)
        self.fns = [(lambda y: (y - self.b1) / self.m1) if self.m1 else None,
                    (lambda y: (y - self.b2) / self.m2) if self.m2 else None]
    
    def __call__(self,y):
        start =  (self.a, 0)
        end   =  (self.b, 0)
        trunc1 = ((self.fns[0](y), y) if self.fns[0] else (self.a, y))
        trunc2 = ((self.fns[1](y), y) if self.fns[1] else (self.b, y))
        return [start, trunc1, trunc2, end]



def mfTrapezoid(x,params):
    a,b,c,d = params
    check = [a==b,b==c,c==d]
    if x is None:
        err = [b<a,c<b,d<c]
        if any(err):
            #Throw an invalid param exception
            errs = ['b<a','c<b','d<c']
            e = [errs[i] for i,_ in enumerate(err) if _]
            raise ParamError(params,'Invalid params {}: {}'.format(e,params))
        else:
            return 0 #Everything looks good
    flag = (type(x) is np.ndarray)
    if check[1]:
        return mfTriangle(x,[a,b,d])
    if check[0]:
        #TODO: account for edge case of square-cornered trapezoid
        retval = np.minimum(1,(d-x)/(d-c))
    elif check[2]:
        retval = np.minimum((x-a)/(b-a),1)
    else:
        retval = np.minimum(np.minimum((x-a)/(b-a),(d-x)/(d-c)),1)
    if flag:
        retval[np.bitwise_or(x<a,x>d)] = 0
    return retval

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
    flag = (type(x) is np.ndarray)
    if flag:
        retval = mfTriangle(x,[a,b,d])
        retval[x<=c] = 1
        return retval
    elif x<=c:
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
    flag = (type(x) is np.ndarray)
    if flag:
        retval = mfTriangle(x,[a,c,d])
        retval[x<=b] = 0
        return retval
    elif x<=b:
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
    flag = (type(x) is np.ndarray)
    if flag:
        retval = mfTriangle(x,[a,c,d])
        retval[x>=b] = 1
        return retval
    elif x>=b:
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
    flag = (type(x) is np.ndarray)
    if flag:
        retval = mfTriangle(x,[a,b,d])
        retval[x>=c] = 0
        return retval
    elif x>=c:
        return 0
    else:
        return mfTriangle(x,[a,b,d])

def mfGaussian(x,params):
    sigma,c = params
    if sigma == 0:
        #Throw invalid param exception
        pass
    t = (x-c)/sigma
    return np.exp(-t*t/2)

def mfGaussian2():
    pass

def defuzzCentroid(outs):
#    TODO: Make this work with new outmf structure
#    a, b = outrange
#    totarea = np.sum(agg)
#    if totarea == 0:
#        print('Total area was zero. Using average of the range instead')
#        return (a + b) / 2
#    totmom = np.sum(agg[0] * outx)
#    return totmom / totarea
    pass

def defuzzWeightedMV(outs):
    ht, hmv = 0, 0
    for y in outs:
        ht += y[1][1]
        hmv += y[1][1] * (y[2][0] + y[1][0]) / 2
    return hmv / ht

class Error(Exception):
    pass

class ParamError(Error):
    def __init__(self,params,msg):
        self.params = params
        self.msg = msg
    def __str__(self):
        return repr(self.msg)
