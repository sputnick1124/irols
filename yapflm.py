from __future__ import division , print_function
import numpy as np
  
class FIS(object):
    """
    An object to hold a representation of a FIS. 
    """
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
        return self.__dict__ == other.__dict__ #traditional method

    def check(self):
        """
        Meta self-diagnosis helper function
        """
        return all(var.check() for var in self.input + self.output)

    def addvar(self,vartype,varname):
        if vartype in 'input':
            self.input.append(FuzzyVar(varname,self,1))
            if len(self.rule) > 0:
                for rule in self.rule:
                    rule.antecedent += [0]
        elif vartype in 'output':
            self.output.append(FuzzyVar(varname,self,0))
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
            self.rule.append(FuzzyRule(antecedent,consequent,weight,connection))
    
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
    def __init__(self,varname='',parent=None,vartype=None):
        self.name = varname
        self.mf = []
        self.parent = parent
        if parent is not None:
            if vartype == 1:
                self.num = len(parent.input)
            elif vartype == 0:
                self.num = len(parent.output)
            else:
                self.num = None
        self.vartype = vartype

    def __str__(self,indent=''):
        var_atts = ['name']
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

    def addmf(self,mfname,mfparams=None):
        try:
            mf = MF(mfname,mfparams,self)
        except ParamError as e:
            raise e
        self.mf.append(mf)


class MF(object):
    def __init__(self,mfname,mfparams,parent=None):
        self.name = mfname
        self.parent = parent
        if mfparams is not None:
#            print('i have params')
            self._params = mfparams
        else:
            p = 3
            dr = 1/(p-1)
            self._params = [0+i*dr for i in xrange(p)]
        if not len(self._params) == 3:
            #Throw invalid param number exception
            raise ParamError(self._params,'len({}) != 3'.format(self._params))
        if parent.vartype == 1:
            self.mf = InputTriMF(self.params)            
        elif parent.vartype == 0:
            self.mf = OutputTriMF(self.params)

    @property
    def params(self):
        return self._params
    @params.setter
    def params(self,val):
        self._params = val
        self.mf.get_slopes(val)
    
    def __str__(self,indent=''):
        mf_atts = ['name','_params']
        s = ''
        for att in mf_atts:
            s += indent + '{0:>10}: {1}\n'.format(att.strip('_'),self.__dict__[att])
        return s

    def __eq__(self,other):
        local_dict = self.__dict__.copy()
        other_dict = other.__dict__.copy()
        local_dict.pop('parent')
        other_dict.pop('parent')
        return local_dict == other_dict

    def evalmf(self,x):
        return self.mf(x)
        
class FuzzyRule(object):
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
    def __init__(self,params=None):
        self.slope = [None]*2
        self.y_int = [None]*2
        self.get_slopes(params)
    
    def get_slopes(self,params):
        a,x_star,b = params
        err = [x_star < a, x_star > b]
        if any(err):
            #Throw an invalid param exception
            errs = ['b<a','c<b']
            e = [errs[i] for i,_ in enumerate(err) if _]
            raise ParamError(params,'Invalid params {}: {}'.format(e,params))
        if a != x_star:
            self.slope[0] = 1 / (x_star - a)
            self.y_int[0] = -self.slope[0] * a
        else:
            self.slope[0], self.y_int[0] = None, None
        if b != x_star:
            self.slope[1] = -1 / (b - x_star)
            self.y_int[1] = 1 - (self.slope[1] * x_star)
        else:
            self.slope[1], self.y_int[1] = None, None
        self.a, self.b = a, b
        self.x_star = x_star
    
    def __eq__(self,other):
        return self.__dict__ == other.__dict__
        

class InputTriMF(TriMF):
    def __init__(self,*args,**kwargs):
        super(InputTriMF,self).__init__(*args,**kwargs)
        self.fn = infn
    
    def __call__(self,x):
        line = x >= self.x_star
        return self.fn(x,self.slope[line],self.y_int[line])
    
class OutputTriMF(TriMF):
    def __init__(self,*args,**kwargs):
        super(OutputTriMF,self).__init__(*args,**kwargs)
        self.fn = outfn
#        self.fns = [(lambda y: (y - self.b1) / self.m1) if self.m1 else None,
#                    (lambda y: (y - self.b2) / self.m2) if self.m2 else None]
    
    def __call__(self,y):
        start =  (self.a, 0)
        end   =  (self.b, 0)
        truncval1 = self.fn(y, self.slope[0], self.y_int[0])
        truncval2 = self.fn(y, self.slope[1], self.y_int[1])
        trunc1 = ((truncval1, y) if truncval1 else (self.a, y))
        trunc2 = ((truncval2, y) if truncval2 else (self.b, y))
        return [start, trunc1, trunc2, end]

def infn(x,m,b):
    y = ((m * x) + b) if m else None #y
    return y if y >= 0 else 0

def outfn(y,m,b):
    x = ((y - b) / m) if m else None #x
    return x if x >= 0 else 0


def defuzzCentroid(outs):
#    TODO: Make this work with new outmf structure if worth it (maybe not)
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
#        print(hmv,ht)
    return hmv / ht if ht and hmv else 0

class Error(Exception):
    pass

class ParamError(Error):
    def __init__(self,params,msg):
        self.params = params
        self.msg = msg
    def __str__(self):
        return repr(self.msg)
