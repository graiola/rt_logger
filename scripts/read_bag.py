#!/usr/bin/python

import rosbag
import scipy.io
import numpy
import sys
import getopt


def key_check(k, d):
    if k in d.keys():
        return d
    else:
        d.setdefault(k)
        d[k] = []
        return d


def parse(bag_file, topic_name):
    bag = rosbag.Bag(bag_file)

    time = []
    values = {}

    for topic, msg, t in bag.read_messages('/' + topic_name):
        time.append(msg.time.data.secs + msg.time.data.nsecs * 10e-9)
        for i_data in range(0, len(msg.array)):
            key = msg.array[i_data].name.data
            value = numpy.array(msg.array[i_data].array.data)
            rows = msg.array[i_data].array.layout.dim[0].size
            cols = msg.array[i_data].array.layout.dim[1].size
            value.reshape(rows, cols)
            values = key_check(key, values)
            values[key].append(value)

    mat_file_name = topic_name + '_time.mat'
    print "File saved: ", mat_file_name
    scipy.io.savemat(mat_file_name, mdict={'time': time})

    for key in values.keys():
        mat_file_name = topic_name + '_' + key + '_values.mat'
        scipy.io.savemat(mat_file_name, mdict={key: values[key]})
        print "File saved: ", mat_file_name

    bag.close()


def main(argv):
    bag_file = ''
    topic_name = ''
    usage = 'test.py -b <bag_file> -t <topic_name>'
    try:
        opts, args = getopt.getopt(argv, "hb:t:", ["help", "bag=", "topic="])
    except getopt.GetoptError as err:
        print str(err)
        print usage
        sys.exit(2)
    if not opts:
        print usage
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print usage
            sys.exit()
        elif opt in ("-b", "--bag"):
            bag_file = arg
        elif opt in ("-t", "--topic"):
            topic_name = arg

    print 'Loading bag file: ', bag_file
    print 'Parsing topic: ', topic_name

    parse(bag_file, topic_name)


if __name__ == "__main__":
    main(sys.argv[1:])