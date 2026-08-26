import argparse

from .factory import open_dataset


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Inspect a ROS1 bag, ROS2 bag, or directory dataset without ROS")
    parser.add_argument("dataset")
    args = parser.parse_args(argv)
    for topic in open_dataset(args.dataset).topics():
        print("{}\t{}\t{}".format(topic.name, topic.msgtype, topic.message_count))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
