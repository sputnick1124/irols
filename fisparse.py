#!/usr/bin/env python
from __future__ import division
import sys
from parse import parse
from yapflm import FIS
from time import time
from fisyaml import fis_to_yaml

class FISParser:
    def __init__(self,fisfile):
        with open(fisfile,'r') as infis:
            self.rawlines = infis.readlines()
        self.get_system()
        self.get_vars()
        self.get_rules()
    
    def get_system(self):
        end_sysblock = self.rawlines.index('\n')
        systemblock = self.rawlines[1:end_sysblock]
        fisargs = map(lambda x:parse('{arg}={val}',x),systemblock)
        fissys = {f['arg'].lower():f['val'].strip("'") for f in fisargs}
        self.fis = FIS(fissys['name'],fissys['type'],fissys['andmethod'],
                       fissys['ormethod'],fissys['impmethod'],
                       fissys['aggmethod'],fissys['defuzzmethod'])
        self.numinputs = int(fissys['numinputs'])
        self.numoutputs = int(fissys['numoutputs'])
        self.numrules = int(fissys['numrules'])
        self.start_varblocks = end_sysblock+1
        
    def get_var(self,vartype,varnum,start_line,end_line):
        varblock = self.rawlines[start_line:end_line]
        fisargs = map(lambda x:parse('{arg}={val}',x),varblock)
        fisvar = {f['arg'].lower():f['val'].strip("'") for f in fisargs}
        varrange = parse('[{:g}{:g}]',fisvar['range']).fixed
        varname = fisvar['name']
        self.fis.addvar(vartype,varname)
        for mf in range(int(fisvar['nummfs'])):
            mfargs = parse("{name}':'{type}',[{range}]",fisvar['mf%d'%(mf+1)])
            if not mfargs['type'] == 'trimf':
                raise TypeError('Type {type} is not supported'.format(mfargs['type']))
#                print(varnum)
            if 'input' in vartype:
                self.fis.input[varnum].addmf(mfargs['name'], 
                                        list(map(float,mfargs['range'].split())))
            elif 'output' in vartype:
                self.fis.output[varnum].addmf(mfargs['name'], 
                                        list(map(float,mfargs['range'].split())))
    
    def get_vars(self):
        start_ruleblock = self.rawlines.index('[Rules]\n')
        var_lines = []
        var_types = []
        flag = 0
        for i,line in enumerate(self.rawlines[self.start_varblocks-1:start_ruleblock]):
            if flag:
                flag = 0
                vt = parse('[{type}{num:d}]',line)
                var_types.append((vt['type'].lower(),vt['num']))
            if line == '\n':
                var_lines.append(i+self.start_varblocks-1)
                flag = 1
        for i,l in enumerate(var_lines[:-1]):
            if 'input' in var_types[i][0]:
                self.get_var('input',var_types[i][1]-1,l+2,var_lines[i+1])
            elif 'output' in var_types[i][0]:
                self.get_var('output',var_types[i][1]-1,l+2,var_lines[i+1])
                
    def get_rules(self):
        start_ruleblock = self.rawlines.index('[Rules]\n')
        ruleblock = self.rawlines[start_ruleblock+1:]
        antecedents = (('{a%d:d} '*self.numinputs) % tuple(range(self.numinputs))).strip()
        consequents = ('{c%d:d} '*self.numoutputs) % tuple(range(self.numoutputs))
        p = antecedents + ', ' + consequents + '({w:d}) : {c:d}'
        rules = []
        for rule in ruleblock:
            rp = parse(p,rule)
            r = []
            for inp in range(self.numinputs):
                rpar = rp['a%d'%inp]
                rval = rpar-1 if rpar else None
                r.append(rval)
            for outp in range(self.numoutputs):
                rpar = rp['c%d'%outp]
                rval = rpar-1 if rpar else None
                r.append(rval)
            r += [rp['w'],rp['c']-1]
            rules.append(r) 
        self.fis.addrule(rules)

def fisfile_to_yaml(fisfile,yamlfile):
    fp = FISParser(fisfile)
    fis_to_yaml(fp.fis,yamlfile)
        
if __name__ == "__main__":
    argv = sys.argv
    if len(argv) < 3:
        print("usage: {0} fisfile yamlfile".format(argv[0]))
        sys.exit(1)
    fisfile_to_yaml(argv[1],argv[2])
