import argparse

from .reader import BagReader


def main(argv=None):
    parser = argparse.ArgumentParser(description="Inspect a ROS1 bag without ROS")
    parser.add_argument("bag")
    args = parser.parse_args(argv)
    for topic in BagReader(args.bag).topics():
        print("{}\t{}\t{}".format(topic.name, topic.msgtype, topic.message_count))
    return 0
