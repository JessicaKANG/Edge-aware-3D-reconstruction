#!/usr/bin/python
#! -*- encoding: utf-8 -*-

# this script is to start the OpenEDGE pipeline

# Usage:
#  $ python OpenEDGE_pipeline.py OPENEDGE_BIN_DIR ./examples_dir 
#  i.e:
#  $ python OpenEDGE_pipeline.py /home/user/OpenEDGE/build/src ./final_examples
#
# 

import commands
import os
import subprocess
import sys

def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)

if len(sys.argv) < 3:
  print ("/!\ Invalid input")
  print ("Usage %s OPENEDGE_BIN_DIR ./GT_DATASET" % sys.argv[0])
  sys.exit(1)

OPENEDGE_BIN = sys.argv[1]
if not (os.path.exists(OPENEDGE_BIN)):
  print("/!\ Please use a valid OPENEDGE_BIN directory.")
  sys.exit(1)

input_dir = sys.argv[2]

# Run for each dataset of the input dir perform
#  1. convert .png RGB images to .ppm gray images
#  2. extract edges using Edison 
#  3. convert .ppm edges to .png edges
#  4. generate dense sfm_data using EdgeGraph
#  5. run g2o process to get optimized sfm_data
##  

for directory in os.listdir(input_dir):

  print directory

  print ("1. convert .png RGB images to .ppm gray images")
  command = OPENEDGE_BIN + "/Image_Converter"
  command = command + " " + input_dir + "/" + directory + "/images/"
  command = command + " " + input_dir + "/" + directory + "/gray_imgs/"
  command = command + " .ppm"
  proc = subprocess.Popen((str(command)), shell=True)
  proc.wait()

  print ("2. extract edges using Edison")
  #command = "cd " + input_dir + "/" + directory + "/gray_imgs/"
  #proc = subprocess.Popen((str(command)), shell=True)
  #proc.wait()

  print ("3. convert .ppm edges to .png edges")
  command = OPENEDGE_BIN + "/Image_Converter"
  command = command + " " + input_dir + "/" + directory + "/edges/"
  command = command + " " + input_dir + "/" + directory + "/edges/"
  command = command + " .png"
  proc = subprocess.Popen((str(command)), shell=True)
  proc.wait()

  print ("4. generate dense sfm_data using EdgeGraph")
  EdgeGraph3D_dir = "/home/jessica/jessica/EdgeGraph3D-fixed/build/EdgeGraph3D"
  command = EdgeGraph3D_dir + " " + input_dir + "/" + directory + "/images/"
  command = command + " " + input_dir + "/" + directory + "/edges/"
  command = command + " " + input_dir + "/" + directory + "/working_folder/"
  command = command + " " + input_dir + "_openMVG_output_HIGH/SfM_output/" + directory + "/SfM_Global/sfm_data.json"
  command = command + " " + input_dir + "/" + directory + "/EdgeGraph3D_output.json"
  proc = subprocess.Popen((str(command)), shell=True)
  proc.wait()

  print ("5. run g2o process to get optimized sfm_data")
  m = 1
  for i in range(0, 5):
	  command = OPENEDGE_BIN + "/OpenEDGE_main_G2O_BA_process"
	  command = command + " -i " + input_dir + "/" + directory + "/EdgeGraph3D_output.json"
	  command = command + " -o " + input_dir + "/" + directory + "/"
	  command = command + " -m " + str(m)
	  proc = subprocess.Popen((str(command)), shell=True)
	  proc.wait()
	  m = m * 10;
  m = .1
  for i in range(0, 4):
	  command = OPENEDGE_BIN + "/OpenEDGE_main_G2O_BA_process"
	  command = command + " -i " + input_dir + "/" + directory + "/EdgeGraph3D_output.json"
	  command = command + " -o " + input_dir + "/" + directory + "/"
	  command = command + " -m " + str(m)
	  proc = subprocess.Popen((str(command)), shell=True)
	  proc.wait()
	  m = m * .1;

sys.exit(1)

