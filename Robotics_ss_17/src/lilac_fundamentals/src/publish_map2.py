#!/usr/bin/python
import os
import rospy
from time import sleep
from lilac_fundamentals.msg import *
import rospkg

T = Cell.TOP
B = Cell.BOTTOM
L = Cell.LEFT
R = Cell.RIGHT

rospack = rospkg.RosPack()
filename = os.path.join(rospack.get_path("lilac_fundamentals"), "map.txt")
with open(filename, 'r') as f:
    exec "map_layout = {}".format(f.read())

print "publishing map: {}".format(map_layout)

node = rospy.init_node('example_map_publisher')
pub = rospy.Publisher('map', Grid, queue_size=1)

grid = Grid()
for row_layout in map_layout:
    row = Row()
    for cell_layout in row_layout:
        cell = Cell()
        cell.walls = cell_layout
        row.cells.append(cell)
    grid.rows.append(row)

while not rospy.is_shutdown():
    pub.publish(grid)
