# MATLAB import fuzzy Tip
import sys
sys.path.append('..')
from yapflm import FIS
from fisparse import FISParser

tipfisparser = FISParser('tipper.fis')
tipfis = tipfisparser.fis

print(tipfis)

qual = 1
serv = 9
out = 'quality is {0} and service is {1}, tip is {2:0.4g}%'.format(qual,serv,tipfis.evalfis([qual,serv]))

print(out)
