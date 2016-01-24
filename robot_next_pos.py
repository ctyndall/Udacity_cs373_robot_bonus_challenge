# -*- coding: utf-8 -*-
"""
Created on Sun Sep 27 15:29:14 2015

@author: Me
"""
from math import *
import numpy as np
from scipy.optimize import leastsq
import matplotlib.pyplot as plt

def residuals(p, y, x):
    A, B,C, D = p
    err = y - (A * np.sin(B * x + C) + D)
    return err

def peval(x, p):
    return p[0] * np.sin(p[1] * x + p[2]) + p[3]
    
def estimate_next_pos(measurement, OTHER = None, debug = False):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""   
    MINIMUM = 100
    if not OTHER: 
        x = measurement[0] #first value of measurements initialized
        y = measurement[1]
        coeffs = [0.0, 0.0, 0.0, 0.0]
        OTHER = {'xs':[x], 'ys':[y], 'xcoeffs':[coeffs], 'ycoeffs':[coeffs], 'measuring':True} #initialize
        xy_estimate = (x,y)
        return xy_estimate, OTHER
    
    #add last measurement
    OTHER['xs'].append(measurement[0])
    OTHER['ys'].append(measurement[1])
    
    #skip fitting until a minimum of data points have been collected
    if len(OTHER['xs']) < MINIMUM:
        xy_estimate = measurement
        return xy_estimate, OTHER
        
    #initialize variables
    if all(OTHER['xcoeffs'][-1]) == 0.0 or all(OTHER['ycoeffs'][-1]) == 0.0:
        OTHER['measuring'] = False
        #xcoeffs initialize
        xs = np.array(OTHER['xs'])
        n = np.arange(len(xs)) 
        A = (max(xs) - min(xs)) / 2 #guess amplitude
        D = sum(xs) / len(xs) #average x's to get offset, may be very innacurrate to start      
        xs_fft = np.fft.fft(xs - D)  #remove offset and perform fft
        B_loc = np.argmax(abs(xs_fft[:len(xs_fft)/2]))
        B = 2*pi * B_loc / len(xs_fft)#.5#turning
        C = np.arcsin((xs[0] - D) / (max(xs) - D))#np.angle(xs_fft[B_loc])#pi / 6  #np.arcos(A / xs[0])  #TODO update this from default 
        OTHER['xcoeffs'].append([A,B,C,D])
        
        #ycoeffs initalize   
        ys = np.array(OTHER['ys'])
        n = np.arange(len(ys))
        A = (max(ys) - min(ys)) / 2 #guess amplitude
        D = sum(ys) / len(ys) #average x's to get offset, may be very innacurrate to start
        ys_fft = np.fft.fft(ys - D)
        B_loc = np.argmax(abs(ys_fft[:len(ys_fft)/2]))
        B = 2*pi * B_loc / len(ys_fft) # .15#turning
        C = np.arcsin((ys[0] - D) / (max(ys) - D)) #np.angle(ys_fft[B_loc])#0.0 #pi / 6  #np.arcos(A / ys[0])  #TODO update this from default
        OTHER['ycoeffs'].append([A,B,C,D])
        
    #fit x's  
    xs = np.array(OTHER['xs'])
    n = np.arange(len(xs)) #index at 0
    p0 = OTHER['xcoeffs'][-1]
    plsq_x = leastsq(residuals, p0, args=(xs, n))
    xcoeffs = plsq_x[0]
    OTHER['xcoeffs'].append(xcoeffs)
    x_est = peval(len(xs), plsq_x[0]) #note len(xs) will estimate the next position since the sin equation is based on 0 index
    #print p0, plsq_x[0]        
    
    #fit y's
    ys = np.array(OTHER['ys'])
    n = np.arange(len(ys))
    p0 = OTHER['ycoeffs'][-1]    
    plsq_y = leastsq(residuals, p0, args=(ys, n))
    ycoeffs = plsq_y[0]
    OTHER['ycoeffs'].append(ycoeffs)
    y_est = peval(len(ys), plsq_y[0])
    #print plsq_y[0]
    
    if debug:
        print xcoeffs, ycoeffs
        plt.plot(n, xs,label = 'x-pos')
        plt.plot(n, peval(n,plsq_x[0]))
        plt.plot(n, ys, label = 'y-pos')
        plt.plot(n, peval(n,plsq_y[0]))
        plt.legend()
        plt.show() 
        print ""
    
    xy_estimate = x_est, y_est
    return xy_estimate, OTHER