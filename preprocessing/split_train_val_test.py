import argparse
import cv2
from os.path import splitext, basename, join, exists, isdir
from os import makedirs
from copy import copy
import glob
from random import shuffle
from collections import deque

parser = argparse.ArgumentParser(description='Generate train, val, test sets.')
parser.add_argument('--root', type=str, default='', help='root folder of images')
parser.add_argument('--dest', dest='out', default='', help='output path')
parser.add_argument('--topic', dest='topic', default='sibot/right/image_rect', help='topic for training and testing')
parser.add_argument('--seq', dest='seq_len', default=1, type=int, help='length of image sequence')

args = parser.parse_args()

root_folder = args.root
if not args.out:
  out_folder = root_folder
else:
  out_folder = args.out

topic_dir = "_".join(args.topic.split('/')[1:])

image_files = glob.glob(join(root_folder, '*.jpeg'))
image_files.sort()
if not isdir(out_folder):
  makedirs(out_folder)

label_files = glob.glob(join(root_folder, '*/{0}/out.txt'.format(topic_dir)))
N = len(label_files)
perm = range(N)
shuffle(perm)

train_file = open(join(out_folder, 'train.txt'), 'w')
val_file = open(join(out_folder, 'val.txt'), 'w')
test_file = open(join(out_folder, 'test.txt'), 'w')

sequence = deque()

for i in range(int(round(0.7*N))):
  for line in open(label_files[perm[i]], 'r'):
    filename, label = line.split()
    sequence.appendleft(filename)
    if len(sequence) < args.seq_len:
        continue
    if len(sequence) > args.seq_len:
        sequence.pop()
    line_out = "{0} {1}\n".format(" ".join(sequence), label)
    train_file.write(line_out)

sequence = deque()

for i in range(int(round(0.7*N)), int(round(0.9*N))):
  for line in open(label_files[perm[i]], 'r'):
    filename, label = line.split()
    sequence.appendleft(filename)
    if len(sequence) < args.seq_len:
        continue
    if len(sequence) > args.seq_len:
        sequence.pop()
    line_out = "{0} {1}\n".format(" ".join(sequence), label)
    val_file.write(line_out)

for i in range(int(round(0.9*N)), N):
  for line in open(label_files[perm[i]], 'r'):
    filename, label = line.split()
    sequence.appendleft(filename)
    if len(sequence) < args.seq_len:
        continue
    if len(sequence) > args.seq_len:
        sequence.pop()
    line_out = "{0} {1}\n".format(" ".join(sequence), label)
    test_file.write( line )

train_file.close()
val_file.close()
test_file.close()
