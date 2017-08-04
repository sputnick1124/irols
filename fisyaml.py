#!/usr/bin/env python

from yapflm import FIS
import yaml


def _varyaml_dict(var):
    var_dict = {}
    var_dict['name'] = var.name
    var_dict['mf'] = [_mfyaml_dict(mf) for mf in var.mf]
    return var_dict

def _mfyaml_dict(mf):
    mf_dict = {}
    mf_dict['name'] = mf.name
    mf_dict['params'] = mf.params
    return mf_dict

def _ruleyaml_dict(rule):
    antecedent = rule.antecedent
    consequent = rule.consequent
    weight = rule.weight
    connection = rule.connection
    return antecedent + consequent + [weight] + [connection]

def fis_to_dict(fis):
    fis_dict = {}
    fis_dict['name'] = fis.name
    fis_dict['type'] = fis.type
    fis_dict['andMethod'] = fis.andMethod
    fis_dict['orMethod'] = fis.orMethod
    fis_dict['aggMethod'] = fis.aggMethod
    fis_dict['impMethod'] = fis.impMethod
    fis_dict['defuzzMethod'] = fis.defuzzMethod
    fis_dict['input'] = [_varyaml_dict(var) for var in fis.input]
    fis_dict['output'] = [_varyaml_dict(var) for var in fis.output]
    fis_dict['rules'] = [_ruleyaml_dict(rule) for rule in fis.rule]
    return fis_dict
    
    
def fis_from_dict(fis_dict):
    fis = FIS(name=fis_dict['name'],
              fistype=fis_dict['type'],
              andMethod=fis_dict['andMethod'],
              orMethod=fis_dict['orMethod'],
              impMethod=fis_dict['impMethod'],
              aggMethod=fis_dict['aggMethod'],
              defuzzMethod=fis_dict['defuzzMethod'])
              
    for var in fis_dict['input']:
        fis.addvar('input',var['name'])
        for mf in var['mf']:
            fis.input[-1].addmf(mf['name'],mf['params'])

    for var in fis_dict['output']:
        fis.addvar('output',var['name'])
        for mf in var['mf']:
            fis.output[-1].addmf(mf['name'],mf['params'])
    
    for rule in fis_dict['rules']:
        fis.addrule(rule)
    
    return fis

def fis_to_yaml(fis,filename):
    with open(filename,'w') as yamlout:
        fis_dict = fis_to_dict(fis)
        yamlout.write(yaml.dump(fis_dict))

def fis_from_yaml(filename):
    with open(filename,'r') as yamlin:
        raw_text = yamlin.readlines()
    fis_dict = yaml.load(''.join(raw_text))
    return fis_from_dict(fis_dict)
