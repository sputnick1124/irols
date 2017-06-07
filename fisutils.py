#!/usr/bin/env python

import matplotlib.pyplot as plt

def plot_fis(fis):
    for i in fis.input:
        plt.figure()
        ax = plt.axes()
        names = []
        for m in i.mf:
            ax.plot(m.params,[0] + [1]*(len(m.params)>>1) + [0])
            names.append(m.name)
        plt.legend(names)
    
    for o in fis.output:
        plt.figure()
        ax = plt.axes()
        names = []
        for m in o.mf:
            ax.plot(m.params,[0] + [1]*(len(m.params)>>1) + [0])
            names.append(m.name)
        plt.legend(names)
