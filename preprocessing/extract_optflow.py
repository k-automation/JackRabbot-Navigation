import cv2
import argparse
from os import listdir, makedirs
from os.path import isdir, isfile, join, basename
import glob

parser = argparse.ArgumentParser(description='Convert images to optical flow.')
parser.add_argument('root_folder', metavar='images', type=str, nargs='1',
                    help='root folder of images')
parse.add_argument('--dest', dest='out', default='', help='output path')

args = parser.parse_args()

root_folder = args.root_folder
if args.out:
  out_folder = join(root_folder, 'optflow/')
else:
  out_folder = args.out

image_files = glob.glob(join(root_folder, '*.jpeg'))
image_files.sort()
if not isdir(out_folder):
  makedirs(out_folder)

prev = cv2.imread(imagefiles[0], CV_LOAD_IMAGE_GRAYSCALE)
for image_path in image_files[1:]:
  out_path = join(out_folder, basename(image_path))
  
  next = cv2.imread(image_path, CV_LOAD_IMAGE_GRAYSCALE)
  flow = cv2.calcOpticalFlowFarneback(prev, next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
  prev = next
  
  cv2.imwrite(out_path, flow)

  mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
  hsv[...,0] = ang*180/np.pi/2
  hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
  rgb = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    
  cv2.imshow('flow',rgb)
