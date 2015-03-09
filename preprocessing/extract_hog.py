import cv2
import argparse
from os import listdir, makedirs
from os.path import isdir, isfile, join, basename
import glob
import numpy as np

parser = argparse.ArgumentParser(description='Convert images to hog features.')
parser.add_argument('root_folder', metavar='images', type=str, nargs='+',
                    help='root folder of images')
parser.add_argument('--dest', dest='out', default='', help='output path')

args = parser.parse_args()

root_folder = args.root_folder[0]

if not args.out:
  out_folder = join(root_folder, 'hog/')
else:
  out_folder = args.out

image_files = glob.glob(join(root_folder, '*.jpg'))

if not isdir(out_folder):
  makedirs(out_folder)

hog = HOGDescriptor((64,64), (16,16), (8,8), (8,8), 9)
for image_path in image_files:
  hog_path = join(out_folder, basename(image_path))
  im = cv2.imread(image_path)
  h = hog.compute(im)
  h = h.reshape(im.shape)
  cv2.imshow('hog', h)
  cv2.imwrite(hog_path, h)