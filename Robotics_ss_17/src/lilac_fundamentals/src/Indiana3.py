#!/usr/bin/python
import rospy
from time import sleep

from create_fundamentals.srv import *
from lilac_fundamentals.srv import *

def playIndiana(req):
	play_song = rospy.ServiceProxy('play_song', PlaySong)	
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
 
	
 
	return True

def indianaServer():
	# define services from driver
	node = rospy.init_node('Indiana')
	store_song = rospy.ServiceProxy('store_song', StoreSong)
	# define silence
	r = 30
 
	# map note names in the lilypad notation to irobot commands
	c4 = 60
	cis4 = des4 = 61
	d4 = 62
	dis4 = ees4 = 63
	e4 = 64
	f4 = 65
	fis4 = ges4 = 66
	g4 = 67
	gis4 = aes4 = 68
	a4 = 69
	ais4 = bes4 = 70
	b4 = 71
	c5 = 72
	cis5 = des5 = 73
	d5 = 74
	dis5 = ees5 = 75
	e5 = 76
	f5 = 77
	fis5 = ges5 = 78
	g5 = 79
	gis5 = aes5 = 80
	a5 = 81
	ais5 = bes5 = 82
	b5 = 83
	c6 = 84
	cis6 = des6 = 85
	d6 = 86
	dis6 = ees6 = 87
	e6 = 88
	f6 = 89
	fis6 = ges6 = 90
 
	# define some note lengths
	# change the top MEASURE (4/4 time) to get faster/slower speeds
	MEASURE = 160 * 0.6 ;
	HALF = MEASURE/2
	Q = MEASURE/4
	E = MEASURE/8
	Ed = MEASURE*3/16
	S = MEASURE/16
 
	MEASURE_TIME = MEASURE/64.

	print("send songs...")
	# first upload the songs to the irobot...
	store_song(0, [r,Q])

	store_song(1, [64,Q,r,E,55,E,r,E,60,MEASURE])
	store_song(2, [64-12,E,f5-12,MEASURE,r,Q,g5-12,Q,r,E])
	store_song(3, [a5-12,E,b5-12,E,r,E,f6-12,MEASURE,r,Q])
	store_song(4, [a5-12,Q,r,E,b5-12,E,60-12,HALF,d6-12,HALF,e6-12,HALF,64,Q])
	store_song(5, [r,E,f5-12,E,g5-12,E,r,E,60,MEASURE,r,Q])
	store_song(6, [d6-12,Q,r,E,64-12,E,f5-12,MEASURE])
	
	
	print 'Create driver is ready!'
	s = rospy.Service('play_indiana', PlayIndiana, playIndiana)
	rospy.spin()



 

 
if __name__ == "__main__":
     indianaServer()
