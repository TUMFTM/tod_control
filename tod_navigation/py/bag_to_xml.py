# run with python2.7 only (system-wide installed, used by ros -> rosbag is installed)

# Copyright Feiler 2020

"""This file extracts a xml-route out off a rosbag."""

import rosbag
import argparse

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", type=str, \
        default='/home/USER/Documents/2020-11-24_all-sensors/all_sensors.bag',\
        help='Provide the path to the bagfile')
    parser.add_argument("-o", type=str, \
        default='/home/USER/Documents/2020-11-24_all-sensors/all_sensors.xml',\
        help='Provide the path to the xml-file. Must end with *.xml')
    parser.add_argument("-t", type=str, \
        default='/Vehicle/VehicleBridge/odometry',\
        help='Provide the odometry topic')
    parser.add_argument("-nth", type=int, \
        default=10,\
        help='Only every nth element is considered -> reduces number of route \
            points')
    args = parser.parse_args()
    pathToRosBag = args.i
    pathToStoreFile = args.o
    topic = args.t
    everyNthPointIsConsidered = args.nth

    bag = rosbag.Bag(pathToRosBag)
    positionList = []
    count = 0
    for topic, msg, t in bag.read_messages(topics=topic):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if (count%everyNthPointIsConsidered) == 0:
            positionList.append([x,y])
            count = 0
        count += 1

    with open(pathToStoreFile, "w") as fileToStore:
        fileToStore.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        fileToStore.write("<Route>\n")
        for pair in positionList:
            fileToStore.write("    <Point x='" + str(pair[0]) + \
                "' y='" + str(pair[1]) + "' />\n")
        fileToStore.write("</Route>\n")