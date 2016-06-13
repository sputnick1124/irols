# -*- coding: utf-8 -*-
"""
Created on Thu Jun  9 08:25:34 2016

@author: nick
"""
from __future__ import division , print_function
import numpy as np
  
class FIS:
    oper =      {'max'      :   np.max,
                 'min'      :   np.min,
                 'sum'      :   np.sum,
                 'prod'     :   np.prod}
                 
    def __init__(self,name,fistype='mamdani',andMethod='min',orMethod='max',
                  impMethod='min',aggMethod='max',defuzzMethod='centroid'):
        self.defuzz =    {'centroid' :   self.defuzzCentroid} 
        self.input,self.output = [],[]
        self.name = name
        self.type = fistype
        self.andMethod = andMethod
        self.orMethod = orMethod
        self.impMethod = impMethod
        self.aggMethod = aggMethod
        self.defuzzMethod = defuzzMethod
        self.rule = []

    def __str__(self):
        sys_level = ['name','type','andMethod','orMethod',
                     'defuzzMethod','impMethod','aggMethod']
#        var_level = ['name','range','mf']
#        mf_level = ['name','type','params']
#        rule_level = []
        s = ''
        for att in sys_level:
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


    def addvar(self,vartype,varname,varrange):
        if vartype in 'input':
            self.input.append(FuzzyVar(varname,varrange))
        elif vartype in 'output':
            self.output.append(FuzzyVar(varname,varrange))
        else:
            #Throw an invalid variable type exception
            pass
    
    def addrule(self,rules):
        numInput = len(self.input)
        numOutput = len(self.output)
        if not any(isinstance(rule,list) for rule in rules):
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
        if type(x) is not np.ndarray:
            x = np.array(x)
        elif len(x) != len(self.input):
            #Throw an incorrect number of inputs exception
            pass
        numout = len(self.output)
	numrule = len(self.rule)
        andMethod = self.oper[self.andMethod]
        orMethod = self.oper[self.orMethod]
        impMethod = self.oper[self.impMethod]
        aggMethod = self.oper[self.aggMethod]
        defuzzMethod = self.defuzz[self.defuzzMethod]
        comb = [andMethod,orMethod]
        for rule in self.rule:
            ant = rule.antecedent
            con = rule.consequent
            weight = rule.weight
            conn = rule.connection
            mfout = [self.input[i].mf[a].evalmf(x[i]) for i,a in enumerate(ant) if a is not None]
            # Generalize for multiple output systems. Easy
            for out in range(numout):
                outset = self.output[out].mf[con[out]].evalset()
                rulestrength = weight*comb[conn](mfout)
                ruleout[-1].append([impMethod([rulestrength,y]) for y in outset])
        for o in range(numout):
            ruletemp = [r[o] for r in ruleout]
            agg = [aggMethod([y[i] for y in ruletemp]) for i in range(len(ruletemp[0]))]
            outputs.append(defuzzMethod(agg,o))
        return outputs if len(outputs)>1 else outputs[0]
        
    def defuzzCentroid(self,agg,out):
        a,b = self.output[out].range
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

class FuzzyVar:
    def __init__(self,varname,varrange):
        self.name = varname
        self.range = varrange
        self.mf = []

    def __str__(self,indent=''):
        var_atts = ['name','range']
        s = ''
        for att in var_atts:
            s += indent + '{0:>10}: {1}\n'.format(att,self.__dict__[att])
        s += indent + '{:>10}:\n'.format('mf')
        for mf in self.mf:
            s += mf.__str__(indent+'\t') + '\n'
        return s
        
    def addmf(self,mfname,mftype,mfparams):
        mf = MF(mfname,mftype,mfparams)
        mf.range = self.range
        self.mf.append(mf)

class MF:
    def __init__(self,mfname,mftype,mfparams):
        self.name = mfname
        self.type = mftype
        self.params = mfparams
        
        mfdict = {'trimf'           :   (self.mfTriangle,3),
                  'trapmf'          :   (self.mfTrapezoid,4),
                  'trunctrilumf'    :   (self.mfTruncTriLeftUpper,4),
                  'trunctrillmf'    :   (self.mfTruncTriLeftLower,4),
                  'trunctrirumf'    :   (self.mfTruncTriRightUpper,4),
                  'trunctrirlmf'    :   (self.mfTruncTriRightLower,4),
                  'gaussmf'         :   (self.mfGaussian,2),
                  'gauss2mf'        :   (self.mfGaussian2,2)}
        if not mfdict[self.type][1] == len(self.params):
            #Throw invalid param number exception
            pass
        
        self.mf = mfdict[self.type][0]
        
    def __str__(self,indent=''):
        mf_atts = ['name','type','params']
        s = ''
        for att in mf_atts:
            s += indent + '{0:>10}: {1}\n'.format(att,self.__dict__[att])
        return s

    def evalmf(self,x):
        return self.mf(x)
        
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
        xlinspace = [a + i*dx for i in range(100)] + [b]
        return [self.mf(x) for x in xlinspace]
    
    def mfTriangle(self,x,params=None):
        if params is not None:
            a,b,c = params
        else:
            a,b,c = self.params
        check = [b<a,c<b,a==b,b==c,a<=x<=c]
        if any(check[:2]):
            #Throw an invalid param exception
            pass
        if not check[4]:
#            print('outside of range')
            return 0
        if check[2]:
            return (c-x)/(c-b)
        elif check[3]:
            return (x-a)/(b-a)
        else:
            return min((x-a)/(b-a),(c-x)/(c-b))
            
    def mfTrapezoid(self,x,params=None):
        if params is not None:
            a,b,c,d = params
        else:
            a,b,c,d = self.params
        check = [b<a,c<b,d<c,a==b,b==c,c==d,a<=x<=d]
        if any(check[:3]):
            #Throw an invalid param exception
            pass
        if not check[6]:
#            print('outside of range')
            return 0
        if check[4]:
            return self.mfTriangle(x,[a,b,d])
        if check[3]:
            return min(1,(d-x)/(d-c))
        if check[5]:
            return min((x-a)/(b-a),1)
        else:
            return min((x-a)/(b-a),1,(d-x)/(d-c))
    
    def mfTruncTriLeftUpper(self,x,params=None):
        if params is not None:
            a,b,c,d = params
        else:
            a,b,c,d = self.params
        check = [b<a,c<b,d<c]
        if any(check):
            #Throw an inalid parameter exception
            pass
        if x<=c:
            return 1
        else:
            return self.mfTriangle(x,[a,b,d])

    def mfTruncTriLeftLower(self,x,params=None):
        if params is not None:
            a,b,c,d = params
        else:
            a,b,c,d = self.params
        check = [b<a,c<b,d<c]
        if any(check):
            #Throw an inalid parameter exception
            pass
        if x<=b:
            return 0
        else:
            return self.mfTriangle(x,[a,c,d])

    def mfTruncTriRightUpper(self,x,params=None):
        if params is not None:
            a,b,c,d = params
        else:
            a,b,c,d = self.params
        check = [b<a,c<b,d<c]
        if any(check):
            #Throw an inalid parameter exception
            pass
        if x>=b:
            return 1
        else:
            return self.mfTriangle(x,[a,c,d])

    def mfTruncTriRightLower(self,x,params=None):
        if params is not None:
            a,b,c,d = params
        else:
            a,b,c,d = self.params
        check = [b<a,c<b,d<c]
        if any(check):
            #Throw an inalid parameter exception
            pass
        if x>=c:
            return 0
        else:
            return self.mfTriangle(x,[a,b,d])

    def mfGaussian(self,x,params=None):
        if params is not None:
            sigma,c = params
        else:
            sigma,c = self.params
        if sigma == 0:
            #Throw invalid param exception
            pass
        t = (x-c)/sigma
        return exp(-t*t/2)
    
    def mfGaussian2(self):
        pass
    
class Rule:
    def __init__(self,antecedent,consequent,weight,connection):
        self.antecedent = antecedent
        self.consequent = consequent
        self.weight = weight
        self.connection = connection

    def __str__(self,indent=''):
        ant = '{} '*len(self.antecedent)
        con = '{} '*len(self.consequent)
        s = ant.strip() + ', ' + con + '({}) : {}'
        a = tuple(self.antecedent) + tuple(self.consequent) + \
                                     (self.weight,self.connection,)
        return indent + s.format(*a)
