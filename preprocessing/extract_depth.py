import cv2
import argparse
from os import listdir, makedirs, rename
from os.path import isdir, isfile, join, basename
import glob

parser = argparse.ArgumentParser(description='Compute Depth map from stereo images.')
parser.add_argument('root_folder', metavar='images', type=str, nargs='+',
                    help='root folder of images')
parser.add_argument('--left', dest='left_path', default='left_image_rect/', help='Left camera subpath')
parser.add_argument('--right', dest='right_path', default='right_image_rect/', help='Right camera subpath')

args = parser.parse_args()

root_folder = args.root_folder[0]
out_folder = join(root_folder, 'depth/')

left_files = glob.glob(join(root_folder, args.left_path, '*.jpg'))
left_files.sort()
#[rename(f,join(root_folder, args.left_path,"frame{:03}.jpg".format(i))) for f,i in zip(left_files, range(len(left_files)))]
#right_files = glob.glob(join(root_folder, args.left_path, '*.jpg'))
#right_files.sort()
#[rename(f,join(root_folder, args.right_path,"frame{:03}.jpg".format(i))) for f,i in zip(right_files, range(len(right_files)))]
#print left_files
if not isdir(out_folder):
  makedirs(out_folder)

stereo = cv2.StereoBM(1, 16, 15)
for left_path in left_files:
  right_path = join(root_folder, args.right_path, basename(left_path))
  depth_path = join(out_folder, basename(left_path))
  print right_path
  imgL = cv2.imread(left_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
  imgR = cv2.imread(right_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
  print imgL.shape
  print imgR.shape
  depth_map = stereo.compute(imgL, imgR)
  
  cv2.imwrite(depth_path, depth_map)