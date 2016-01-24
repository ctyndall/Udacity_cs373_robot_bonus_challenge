# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY 
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3 and 4. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random
import copy
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import leastsq
import robot_next_pos

debug = False

'''Estimate Robot next position from part2 assignment'''
def residuals(p, y, x):
    A, B,C, D = p
    err = y - (A * np.sin(B * x + C) + D)
    return err

def peval(x, p):
    return p[0] * np.sin(p[1] * x + p[2]) + p[3]
    
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""   
    return robot_next_pos.estimate_next_pos(measurement, OTHER, debug)
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves. 

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    # This function will be called after each time the target moves.

    #initialize hunter to stay straight 
    turning = 0.
    
    xy_next_estimate, OTHER = estimate_next_pos(target_measurement, OTHER) 
    distance_to_next = distance_between(hunter_position, xy_next_estimate)    
    d = distance_to_next
    #if far away or still measuring sensors, just go toward bot next pos
    if distance_to_next > 5 * max_distance or OTHER['measuring'] == True:
        new_heading = get_heading(hunter_position, xy_next_estimate)
        turning = angle_trunc(new_heading - hunter_heading)
    
    #extrapolate enough moves ahead that hunter can reach location in time
    else:
        d = distance_to_next
        m = len(OTHER['xs'])
        n = 1
        timeout = 0
        planned = False                
        while not planned and 'xs' in OTHER:
            timeout += 1
            #aim for additional spots ahead of robot           
            if d < n * max_distance:
                planned = True    
                break
            xy_next_estimate = (peval(m+n,OTHER['xcoeffs'][-1]),peval(m+n,OTHER['ycoeffs'][-1]))
            d = distance_between(hunter_position, xy_next_estimate) 
            n += 1
            #if leas
            if timeout > 10:
                #print 'timeout'
                OTHER['xcoeffs'].append([0.0,0.0,0.0,0.0])
                OTHER['ycoeffs'].append([0.0,0.0,0.0,0.0])
                return turning, d, OTHER
        
        new_heading = get_heading(hunter_position, xy_next_estimate)
        turning = angle_trunc(new_heading - hunter_heading)
        
    distance = d   
    
    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.97 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)
        
        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return (caught, ctr)

def demo_grading_turtle(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
       # if ctr > 100: #delay animation
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    #turtle.close()
    return (caught, ctr)

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

####Run Tests########
'''Change True/False below to run tests with different visualizations'''
noise_factor =  2.0
target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = noise_factor*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)
hunter = robot(-10.0, -10.0, 0.0)

#Run once and visualize chase
if False:
    res = demo_grading_turtle(hunter, target, next_move)

#Run once and visualize the least squares optimization of x,y position estimates
if False:
    debug = True
    res = demo_grading(hunter, target, next_move)

#Do several runs to get a passing rate and average steps to catch.  Failed runs results in 1000 steps.
if True:
    tests = []
    steps = []
    for i in range(100):
#        target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
#        measurement_noise = noise_factor*target.distance
#        target.set_noise(0.0, 0.0, measurement_noise)
#        hunter = robot(-10.0, -10.0, 0.0)
        res = demo_grading(hunter, target, next_move)
        tests.append(res[0])
        steps.append(res[1])
        
    print "Percentage Pass: ", 100*sum(tests)/float(len(tests))
    print "Average Steps: ", sum(steps)/float(len(steps))