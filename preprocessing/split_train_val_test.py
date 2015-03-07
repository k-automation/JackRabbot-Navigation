import argparse
import cv2
from os.path import splitext, basename, join, exists
from os import makedirs
from copy import copy

parser = argparse.ArgumentParser(description='Generate train, val, test sets.')
parser.add_argument('root_folder', metavar='images', type=str, nargs='1',
                    help='root folder of images')
parse.add_argument('--dest', dest='out', default='', help='output path')

args = parser.parse_args()

root_folder = args.root_folder
if not args.out:
  out_folder = root_folder
else:
  out_folder = args.out

image_files = glob.glob(join(root_folder, '*.jpeg'))
image_files.sort()
if not isdir(out_folder):
  makedirs(out_folder)

label_files = glob.glob(join(root_folder, '*/out.txt'))
N = len(label_files)
perm = randperm(N)

train_file = open(join(out_folder, 'train.txt'), 'w')
val_file = open(join(out_folder, 'val.txt'), 'w')
test_file = open(join(out_folder, 'test.txt'), 'w')

for i in range(round(0.7*N)):
  for line in open(label_files[perm[i]], 'r'):
    train_file.write( line )
for i in range(round(0.7*N), round(0.9*N)):
  for line in open(label_files[perm[i]], 'r'):
    val_file.write( line )
for i in range(round(0.9*N), N):
  for line in open(label_files[perm[i]], 'r'):
    test_file.write( line )

train_file.close()
val_file.close()
test_file.close()
