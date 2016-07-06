# -*- coding: utf-8 -*-
"""
Created on Thu Jun  9 13:52:10 2016

@author: nick
"""
import sys, os
sys.path.append('..')
from yapflm import FIS
from fisparse import FISParser
import numpy as np

def fn1(x):
    return list(map(lambda x:x**0.45,x))

def fn2(x):
    return [i if -0.8<i<0.8 else -0.8 if i<=-0.8 else 0.8 for i in x]
    
def fn3(x):
    return list(map(lambda x:x*x,x))


fns = [fn1,fn2,fn3]
#myfis = FIS('myfis')
#myfis.addvar('input','x',[-0.5,1.5])
#myfis.addvar('output','y',[-0.5,1.5])
#myfis.input[0].addmf('s','trimf',[-0.1895 ,  -0.0194   , 0.8362])
#myfis.input[0].addmf('m','trimf',[0.8054  ,  1.0101,    1.3791])
#myfis.input[0].addmf('b','trimf',[0.0048  ,  0.5913   , 1.0248])
#myfis.output[0].addmf('sm','trimf',[-0.1831  , -0.0562   , 0.5103])
#myfis.output[0].addmf('l','trimf',[0.3741   , 1.1991  ,  1.4066])
#rules = [[0,0,1,1],[1,1,1,1],[2,1,1,1]]
#myfis.addrule(rules)

fis_list = [f for f in os.listdir(os.getcwd()) if 'opt.fis' in f]

fis_index = 1
fisparser = FISParser(fis_list[fis_index])
fis = fisparser.fis
#myfisparser = FISParser('fuzzy_crt_opt.fis')
#myfis = myfisparser.fis
#print(fis)

xs = [np.linspace(0,1,1000),
      np.linspace(-1,1,1000),
      np.linspace(-10,10,1000)]
#dx = 0.001
#x = [dx*i for i in range(1000)]
#ya = list(map(lambda x: x**(0.45), x))
#yf = [myfis.evalfis(xx) for xx in x]
x = xs[fis_index]
ya = fns[fis_index](x)
yf = [fis.evalfis(xx) for xx in x]

labels = ['x^0.45',
          '       {-0.8 if x<=-0.8\nx = {x\n       {0.8 if x>=0.8',
          'x^2']
## Plot results
import matplotlib.pyplot as plt
plt.plot(x,ya,'b',x,yf,'g--')
plt.legend([labels[fis_index],"Fuzzy Approx"],loc='best')
plt.show()
