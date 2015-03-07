import cv2
import argparse
from os import listdir, makedirs
from os.path import isdir, isfile, join, basename
import glob

parser = argparse.ArgumentParser(description='Convert images to hog features.')
parser.add_argument('root_folder', metavar='images', type=str, nargs='1',
                    help='root folder of images')
parse.add_argument('--dest', dest='out', default='', help='output path')

args = parser.parse_args()

root_folder = args.root_folder
if not args.out:
  out_folder = join(root_folder, 'hog/')
else:
  out_folder = args.out

image_files = glob.glob(join(root_folder, '*.jpeg'))
if not isdir(out_folder):
  makedirs(out_folder)

hog = cv2.HOGDescriptor()
for image_path in image_files:
  hog_path = join(out_folder, basename(image_path))
  im = cv2.imread(image_path)
  h = hog.compute(im)
  cv2.imwrite(hog_path, h)