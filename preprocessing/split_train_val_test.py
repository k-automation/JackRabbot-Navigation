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
  out_folder = join(root_folder, 'optflow/')
else:
  out_folder = args.out

image_files = glob.glob(join(root_folder, '*.jpeg'))
image_files.sort()
if not isdir(out_folder):
  makedirs(out_folder)

