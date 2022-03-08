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


def parse_specific_topic(bag, topic_name):

    time = []
    values = {}

    #topic_name = topic_name.replace("/", "")  # always remove the escape symbol
    for topic, msg, t_rx in bag.read_messages(topic_name):
        t_tx = 1.0 * msg.time.data.secs + 0.1 * msg.time.data.nsecs * 10e-9
        time.append(t_tx)
        for i_data in range(0, len(msg.array)):
            key = msg.array[i_data].name.data
            value = numpy.array(msg.array[i_data].array.data)
            rows = msg.array[i_data].array.layout.dim[0].size
            cols = msg.array[i_data].array.layout.dim[1].size
            value = value.reshape(rows, cols)
            values = key_check(key, values)
            values[key].append(value)

    data_name =  topic_name.replace("/", "",1).replace("/", "_") + '_time'
    mat_file_name = data_name + '.mat'
    print "File saved: ", mat_file_name
    scipy.io.savemat(mat_file_name, mdict={data_name: time})

    for key in values.keys():
        data_name = topic_name.replace("/", "",1).replace("/", "_") + '_' + key
        mat_file_name = data_name + '.mat'
        scipy.io.savemat(mat_file_name, mdict={data_name: values[key]})
        print "File saved: ", mat_file_name


def parse(bag_file, topic_name=''):

    bag = rosbag.Bag(bag_file)

    supported_data_type = 'rt_logger/LoggerNumericArray'

    # Get all the topic names and their data type
    topics = bag.get_type_and_topic_info()[1].keys()
    types = bag.get_type_and_topic_info()[1].values()
    for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
        types.append(bag.get_type_and_topic_info()[1].values()[i][0])

    if topic_name == '':  # Parse all the topics
        i_topic = 0
        for topic in topics:
            if types[i_topic].msg_type == supported_data_type:
                parse_specific_topic(bag, topic)
            i_topic += 1
    else:  # Parse a specific topic
        idx = topics.index(topic_name)
        if idx and types[idx].msg_type == supported_data_type:
            parse_specific_topic(bag, topic_name)
        else:
            print 'The topic does not exist or its data type is not correct. Supported data type is', supported_data_type

    bag.close()


def main(argv):
    bag_file = ''
    topic_name = ''
    usage = 'read_bag.py -b <bag_file.bag> [-t <topic_name>]'
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
        if opt in ("-h", "--help"):
            print usage
            sys.exit()
        elif opt in ("-b", "--bag"):
            bag_file = arg
        elif opt in ("-t", "--topic"):
            topic_name = arg

    print 'Loading bag file: ', bag_file
    if topic_name:
        print 'Parsing topic: ', topic_name
        parse(bag_file, topic_name)
    else:
        print 'Parsing all available topics'
        parse(bag_file)


if __name__ == "__main__":
    main(sys.argv[1:])
