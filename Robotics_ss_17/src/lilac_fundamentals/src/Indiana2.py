#!/usr/bin/python
import rospy
from time import sleep

from create_fundamentals.srv import *

# define services from driver
node = rospy.init_node('indiana')
print 'Waiting for create driver ...'

play_song = rospy.ServiceProxy('play_song', PlaySong)
print 'Create driver is ready!'


# define some note lengths
# change the top MEASURE (4/4 time) to get faster/slower speeds
MEASURE = 160 * 0.6 ;
HALF = MEASURE/2
Q = MEASURE/4
E = MEASURE/8
Ed = MEASURE*3/16
S = MEASURE/16
 
MEASURE_TIME = MEASURE/64.
 
	
print("play songs...")
play_song(1)
sleep(MEASURE_TIME*1.625)
 
play_song(2)
sleep(MEASURE_TIME*1.75)
 
play_song(3)
sleep(MEASURE_TIME*1.625)
 
play_song(4)
sleep(MEASURE_TIME*2.25)
 
play_song(5)
sleep(MEASURE_TIME*1.75)
 
play_song(6)
sleep(MEASURE_TIME*1.5)
 
