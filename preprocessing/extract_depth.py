import cv2
import argparse
from os import listdir, makedirs
from os.path import isdir, isfile, join, basename
import glob

parser = argparse.ArgumentParser(description='Compute Depth map from stereo images.')
parser.add_argument('root_folder', metavar='images', type=str, nargs='1',
                    help='root folder of images')
parser.add_argument('--left', dest='left_path', default='left/', help='Left camera subpath')
parser.add_argument('--right', dest='right_path', default='right/', help='Right camera subpath')

args = parser.parse_args()

root_folder = args.root_folder
out_folder = join(root_folder, 'depth/')

left_files = glob.glob(join(root_folder, args.left_path, '*.jpeg'))

if not isdir(out_folder):
  makedirs(out_folder)

stereo = cv2.createStereoBM(numDisparities=16, blockSize=15)
for left_path in left_files:
  right_path = join(root_folder, args.right_path, basename(left_path))
  depth_path = join(out_folder, basename(left_path))
  
  imgL = cv2.imread(left_path, CV_LOAD_IMAGE_GRAYSCALE)
  imgR = cv2.imread(right_path, CV_LOAD_IMAGE_GRAYSCALE)
  depth_map = stereo.compute(imgL, imgR)
  
  cv2.imwrite(depth_path, depth_map)