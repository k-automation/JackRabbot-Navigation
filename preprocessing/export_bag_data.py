import argparse
import rosbag
from cv_bridge import CvBridge
import cv2
from os.path import splitext, basename, join, exists
from os import makedirs
from copy import copy

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('bag_files', metavar='bags', type=str, nargs='+',
                   help='file names for bags')
parser.add_argument('--out', dest='out_path', default='sibot/left/image_rect', help='output path')

args = parser.parse_args()

# get bags from arguments
bagfiles = args.bag_files
image_topics = ['sibot/left/image_rect', 'sibot/right/image_rect']
base_image_path = args.out_path
out_filename = 'out.txt'
bin_count = 20
out_image_size = (512, 272)

bridge = CvBridge()


def classify_joy(right_lr_axis, bins):
    bin_size = float(.8) / float(bins)
    if right_lr_axis == 0.0:
        return 0
    elif right_lr_axis < 0:
        return 1
    else:
        return 2
    #for i in range(bins):
    #    if right_lr_axis < ((i * bin_size) - .4):
    #        return i
    #return bins - 1


for bagfile in bagfiles:
    base_bag_name = splitext(basename(bagfile))[0]
    topic_to_out_file = {}
    topic_to_image_dir = {}
    for topic in image_topics:
        dir_ext = "-".join(topic.split('/')[1:])
        dir_name = "{0}-{1}".format(base_bag_name, dir_ext)
        image_dir = join(base_image_path, dir_name)
        if not exists(image_dir):
            makedirs(image_dir)
        topic_to_image_dir[topic] = image_dir
        topic_to_out_file[topic] = open(join(image_dir, out_filename), 'w')
    bag = rosbag.Bag(bagfile)
    last_joy_msg = None
    last_joy_time = None
    i = 0
    prev_t = None
    for topic, msg, t in bag.read_messages():
        if topic == 'joy':
            last_joy_msg = copy(msg)
            last_joy_time = copy(t)
        elif topic in topic_to_image_dir:
            image_name = "frame{:03}.jpg".format(i)
            image_path = join(image_dir, image_name)
            cv_image = bridge.imgmsg_to_cv2(msg)
            cv_image = cv2.resize(cv_image, out_image_size)
            correct_class = classify_joy(last_joy_msg.axes[3], bin_count)
            if cv2.imwrite(join(topic_to_image_dir[topic], image_name), cv_image):
                line = "{0} {1}\n".format(image_name, correct_class)
                topic_to_out_file[topic].write(line)
                i += 1
    for topic in topic_to_out_file:
        topic_to_out_file[topic].close()
 
