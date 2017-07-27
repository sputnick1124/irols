#!/usr/bin/env python

from yapflm import FIS
import yaml

class FISYamlWriter(object):
    def __init__(self,fis):
        self.fis = fis
        self.fis_dict = {}
        
    def fisyaml_dict(self):
        self.fis_dict = {}
        self.fis_dict['name'] = self.fis.name
        self.fis_dict['type'] = self.fis.type
        self.fis_dict['andMethod'] = self.fis.andMethod
        self.fis_dict['orMethod'] = self.fis.orMethod
        self.fis_dict['aggMethod'] = self.fis.aggMethod
        self.fis_dict['impMethod'] = self.fis.impMethod
        self.fis_dict['defuzzMethod'] = self.fis.defuzzMethod
        self.fis_dict['input'] = [self.varyaml_dict(var) for var in self.fis.input]
        self.fis_dict['output'] = [self.varyaml_dict(var) for var in self.fis.output]
        self.fis_dict['rules'] = [self.ruleyaml_dict(rule) for rule in self.fis.rule]
    
    def varyaml_dict(self,var):
        var_dict = {}
        var_dict['name'] = var.name
        var_dict['mf'] = [self.mfyaml_dict(mf) for mf in var.mf]
        return var_dict

    def mfyaml_dict(self,mf):
        mf_dict = {}
        mf_dict['name'] = mf.name
        mf_dict['params'] = mf.params
        return mf_dict
    
    def ruleyaml_dict(self,rule):
        antecedent = rule.antecedent
        consequent = rule.consequent
        weight = rule.weight
        connection = rule.connection
        return antecedent + consequent + [weight] + [connection]
    
    def dump2yaml(self,fd):
        with open(fd,'w') as yaml_out:
            yaml_out.write(yaml.dump(self.fis_dict))

class FISYamlReader(object):
    def __init__(self,yaml_file):
        with open(yaml_file,'r') as yaml_in:
            raw_text = yaml_in.readlines()
        
        self.fis_dict = yaml.load(''.join(raw_text))
    
    def fis_from_yaml(self):
        fis = FIS(name=self.fis_dict['name'],
                  fistype=self.fis_dict['type'],
                  andMethod=self.fis_dict['andMethod'],
                  orMethod=self.fis_dict['orMethod'],
                  impMethod=self.fis_dict['impMethod'],
                  aggMethod=self.fis_dict['aggMethod'],
                  defuzzMethod=self.fis_dict['defuzzMethod'])
                  
        for var in self.fis_dict['input']:
            fis.addvar('input',var['name'])
            for mf in var['mf']:
                fis.input[-1].addmf(mf['name'],mf['params'])

        for var in self.fis_dict['output']:
            fis.addvar('output',var['name'])
            for mf in var['mf']:
                fis.input[-1].addmf(mf['name'],mf['params'])
        
        for rule in self.fis_dict['rules']:
            fis.addrule(rule)
        
        return fis
