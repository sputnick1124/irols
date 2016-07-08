# -*- coding: utf-8 -*-
"""
Created on Tue Jul  5 10:23:04 2016

@author: nick
"""

import sys
sys.path.append('..')
from ga import GFS, GA
from matplotlib import pyplot as plt
import numpy as np
from scipy.integrate import ode
from multiprocessing import Pool
#import timeit

def springmass(t,x,fis):
    if hasattr(fis,'evalfis'):
        u = fis.evalfis(x)
    else:
        u = fis
    dxdt1 = x[1]
    dxdt2 = -2*0.5*u*x[1] - 0.5*0.5*x[0]
    
    return np.vstack((dxdt1,dxdt2))


def sim(t0,tf,fis):
    t,x1,x2 = [],[],[]
    model = ode(springmass).set_integrator('dopri5')
    model._integrator.iwork[2] = -1 #this suppresses the underlying fortran warning
    tf = 100
    model.set_initial_value(np.vstack([0.2,0]),0).set_f_params(fis)
    dt = 1
    while model.t < tf:
        model.integrate(model.t + dt)
        t.append(model.t)
        x1.append(model.y[0][0])
        x2.append(model.y[1][0])
    return t,x1,x2

def sim1(fis):
    t,x1,x2 = [],[],[]
    model = ode(springmass).set_integrator('dopri5')
    model._integrator.iwork[2] = -1 #this suppresses the underlying fortran warning
    tf = 100
    dt = 0.5
    model.set_initial_value(np.vstack([0.2,0]),0).set_f_params(fis)
    while model.t < tf:
        model.integrate(model.t+dt)
        t.append(model.t)
        x1.append(model.y[0][0])
        x2.append(model.y[1][0])
    x1 = np.array(x1)
    x2 = np.array(x2)
    Ts1_i = np.where(np.abs(x1)>0.003)[0]
    Ts2_i = np.where(np.abs(x1)>0.003)[0]
    if Ts1_i.size:
        t1 = t[Ts1_i[-1]]
    else:
        t1 = t[-1]
    if Ts2_i.size:
        t2 = t[Ts2_i[-1]]
    else:
        t2 = t[-1]
    return max([t1,t2])

class fitness_fn(object):
    def __init__(self,fn,a=0,b=1):
        self.x = np.linspace(a,b,50)
        self.ya = list(map(fn, self.x))
    def __call__(self,fis):
        yf = map(fis.evalfis,self.x)
        mean_yf = sum(yf)/len(yf)
        SS1,SS2 = zip(*(((f-mean_yf)**2,(f-a)**2) for f,a in zip(yf,self.ya)))
#        SSres = sum((f-a)**2 for f,a in zip(yf,self.ya))
        SStot = sum(SS1)
        SSres = sum(SS2)
        return abs(SSres/SStot) if SStot else 1000
        
class model_fitness(object):
    def __init__(self):
        self.model = ode(springmass).set_integrator('dopri5',rtol = 1e-7,nsteps = 1)
        self.model._integrator.iwork[2] = -1 #this suppresses the underlying fortran warning
        self.tf = 100
    def __call__(self,fis):
        t,x1,x2 = [],[],[]
        model = self.model
        model.set_initial_value(np.vstack([0.2,0]),0).set_f_params(fis)
        while model.t < self.tf:
            model.integrate(self.tf,step = True)
            t.append(model.t)
            x1.append(model.y[0][0])
            x2.append(model.y[1][0])
        x1 = np.array(x1)
        x2 = np.array(x2)
        Ts1_i = np.where(np.abs(x1)>0.003)[0]
        Ts2_i = np.where(np.abs(x1)>0.003)[0]
        if Ts1_i.size:
            t1 = t[Ts1_i[-1]]
        else:
            t1 = t[-1]
        if Ts2_i.size:
            t2 = t[Ts2_i[-1]]
        else:
            t2 = t[-1]
        return max([t1,t2]) 

#def fitness(fis):
#    x = [dx*i for i in range(100)]
#    ya = list(map(lambda x: x**(0.45), x))
#    yf = map(fis.evalfis,x)
#    mean_yf = sum(yf)/len(yf)
#    SStot = sum((f-mean_yf)**2 for f in yf)
#    SSres = sum((f-a)**2 for f,a in zip(yf,ya))
#    return abs(SSres/SStot) if SStot else 1000

fn0 = lambda x: x**(0.45)
fn1 = lambda x: x if -0.8<x<0.8 else -0.8 if x<=-0.8 else 0.8
fn2 = lambda x: x*x
fitness1 = fitness_fn(fn0,0,1)

ic = [0.2,0]

#myfis = GFS(init=[5,(1<<4)+(1<<0),3,0],inRange=[-13,13],outRange=[-5,120])
#myfis._points = 1001
myfis = GFS(init=[3,0,2,0],inRange=[-0.5,1.5],outRange=[-0.5,1.5])
myfis.keep_rules = False

#sim1(myfis.randomize())

myga = GA()
myga.addSystem(myfis)
#p = Pool(4)
#results = p.map_async(sim1,myga.populations[0])
#f = results.get()
#p.close()
#p.join()

myga.addFitness(fitness1)
best = myga.evalGA()


x = np.linspace(0,1,1000)
#ya = fn1(x)
ya = map(fn0,x)
yf = map(best.evalfis,x)
#yf = [myfis.evalfis(xx) for xx in x]



plt.plot(x,ya,'b',x,yf,'g--')
plt.legend(['fn',"Fuzzy Approx"],loc='best')
plt.show()