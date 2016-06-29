# Nick fuzzy Tip
from __future__ import print_function
import sys,timeit
sys.path.append('..')
from yapflm import FIS
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D

tipfis = FIS('tipfis')
tipfis.addvar('input','quality',[0,10])
tipfis.addvar('input','service',[0,10])
tipfis.addvar('output','tip',[0,26])
tipfis.input[0].addmf('poor','trimf',[0,0,5])
tipfis.input[0].addmf('average','trimf',[0,5,10])
tipfis.input[0].addmf('good','trimf',[5,10,10])
tipfis.input[1].addmf('poor','trimf',[0,0,5])
tipfis.input[1].addmf('average','trimf',[0,5,10])
tipfis.input[1].addmf('good','trimf',[5,10,10])
tipfis.output[0].addmf('low','trimf',[0,0,13])
tipfis.output[0].addmf('medium','trimf',[0,13,25])
tipfis.output[0].addmf('high','trimf',[13,25,25])

rules = [[0,0,0,1,1],
         [None,1,1,1,0],
         [2,2,2,1,1]]

tipfis.addrule(rules)
print(tipfis)

qual = 6.5
serv = 9.8
out = 'quality is {0} and service is {1}, tip is {2:0.4g}%'.format(qual,serv,tipfis.evalfis([qual,serv]))


print(out)

t0 = timeit.default_timer()
timetrials = 10000
for timetrial in range(timetrials):
    output = tipfis.evalfis([serv,qual])
t1 = timeit.default_timer()
print("Took ", (t1-t0), " seconds to complete ", timetrials, " calls")
print("Averaged to be ", (t1-t0)/timetrials, " seconds per call")
