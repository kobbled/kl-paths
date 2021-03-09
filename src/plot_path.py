# This script will plot draw objects from a fanuc controller
# Must input the robot's ip, and the name of the draw object
# The program must have been previously run and populated.

# ..todo:: implement with karel sockets for handshake with fanuc
# controller.

#ftplib ref: http://zetcode.com/python/ftp/
import os
import sys
import time
import socket
import ftplib
from ftplib import FTP
import re
from random import randint
import argparse

import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

#ftp parameters
ROBOT_IP = '127.0.0.1'
ROBOT_PORT = 80
USERNAME = ""
PASSWORD = ""

SAVE_DIRECTORY = 'plots'

PATH_MAP = 'PATH_PLAN'
PATH_MAP_SUFFIX = 'V'
PATH_MAP_TYPE = 'INTEGER'
PATH_DATA = 'LINES'
PATH_DATA_SUFFIX = 'V'
PATH_DATA_TYPE = 'VECTOR'

folder_files = ''

ppath = []

def random_color_gen():
  """Generates a random RGB color
  
  :return: 3 elements in the form [R, G, B]
  :rtype: list
  """
  r = randint(0, 255)
  g = randint(0, 255)
  b = randint(0, 255)
  return (r, g, b)

def parsePath(args):
  parsefile = folder_files +'/' + SAVE_DIRECTORY + '/' + args.rbt_fl + '.VA'

  pattern_map = rf"Field: {PATH_MAP}\.NODEDATA\[(\d+)\]\.{PATH_MAP_SUFFIX} Access: RW: {PATH_MAP_TYPE} =\s*(\d+)"
  patternx = r"X:\s*(-?\d{0,3}\.\d{1,3})"
  patterny = r"Y:\s*(-?\d{0,3}\.\d{1,3})"

  with open(parsefile,'r') as f:
    lines = f.readlines()

    #get path index order
    path_order = []
    for i in range(len(lines)):
      m1 = re.search(pattern_map, lines[i])
      if m1:
        path_order.append(int(m1.group(2)))

  # load full file into memory
  textfile = open(parsefile, 'r')
  filetext = textfile.read()
  textfile.close()
  
  lines = None
  # get path coordinates
  for i in range(len(path_order)):
    pattern_data = rf"Field: {PATH_DATA}\.NODEDATA\[{path_order[i]}\]\.{PATH_DATA_SUFFIX} Access: RW: {PATH_DATA_TYPE} =\s*(.*)"
    m2 = re.search(pattern_data, filetext)
    vec = m2.group(1)
    # get x coordinate
    mvec = re.search(patternx, vec)
    new_x = 0.0
    if mvec:
      new_x = float(mvec.group(1))
    # get y coordinate
    mvec = re.search(patterny, vec)
    new_y = 0.0
    if mvec:
      new_y = float(mvec.group(1))
    ppath.append([path_order[i], [new_x, new_y]])

def print_cont(list_obj):
  for i in range(len(list_obj)):
    print("{}: {:.3f}, {:.3f}".format(list_obj[i][0], list_obj[i][1][0], list_obj[i][1][1]))

def plot():
  fig, ax = plt.subplots()

  #ploygons
  Path = mpath.Path
  
  #path
  if len(ppath) > 0:
    idx, verts = zip(*ppath)
    xs, ys = zip(*verts)
    ax.plot(xs, ys, 'o--', lw=2, color='red', ms=10)

  ax.grid()
  ax.axis('equal')
  plt.show()


class RobotFTP(object):
    
  def __init__(self, filename, ip, port = 80, username = "", password = ""):

    if (ip == ""):
        input("Robot FTP IP not defined, edit the top of the script to fix")
        sys.exit()

    if (username == ""):
        username = "anonymous"

    print("Trying to connect to: " + ip)
    self.ftp = FTP(ip) #compose of FTP class
    self.ftp.login()
    print("Connected to robot")

    self.ip = ip
    self.port = port
    self.username = username

    #store full list of programs to be ran
    self.program = filename + '.VA'

    self.savefile()

  def savefile(self):
    filename = self.program.upper()
    try:
      save_dir = folder_files +'/' + SAVE_DIRECTORY
      if not os.path.exists(save_dir):
        os.makedirs(save_dir)
      
      with open(save_dir + '/'+ filename, 'wb+') as fp:
        self.ftp.cwd("md:")
        self.ftp.retrbinary('RETR ' + self.program, fp.write)
    except ftplib.all_errors as e:
      print('FTP error:', e)

def main():
  description=("Visualization tool for interpretting paths on the"
               "FANUC controller")

  parser = argparse.ArgumentParser(prog='plot_drawing', description=description,
                                  formatter_class=argparse.MetavarTypeHelpFormatter)

  parser.add_argument('rbt_fl', type=str, nargs='?',
        help="Name of karel file")

  args = parser.parse_args()

  folder_files = os.path.dirname(os.path.realpath(__file__))
  folder_files = os.path.abspath(os.path.join(folder_files, os.pardir))
  print('parent fldr: ', folder_files)
  # start an ftp instance
  robot = RobotFTP(args.rbt_fl, ROBOT_IP, ROBOT_PORT, username = USERNAME, password = PASSWORD)

  parsePath(args)
  print('path')
  print_cont(ppath)
  plot()

if __name__ == "__main__":
  main()
