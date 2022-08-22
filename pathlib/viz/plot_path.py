# This script will plot draw objects from a fanuc controller
# Must input the robot's ip, and the name of the draw object
# The program must have been previously run and populated.

# ..todo:: implement with karel sockets for handshake with fanuc
# controller.

#ftplib ref: http://zetcode.com/python/ftp/
from decimal import ROUND_05UP
import os
import sys
import time
import socket
import ftplib
from ftplib import FTP
import re
from random import randint
import argparse

from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
from math import pi

import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

import collections

#ftp parameters
ROBOT_IP = '127.0.0.1'
ROBOT_PORT = 80
USERNAME = ""
PASSWORD = ""

SAVE_DIRECTORY = 'plots'

CONTOUR_VARNAME = 'CONTOURS'
CONTOUR_VEC_SUFFIX = 'V'
CONTOUR_VEC_TYPE = 'VECTOR'
CONTOUR_CODE_SUFFIX = 'CODE'
CONTOUR_CODE_TYPE = 'SHORT'

LINES_VARNAME = 'LINES'
LINE_START_SUFFIX = 'R0'
LINE_END_SUFFIX = 'R1'
LINE_TYPE = 'VECTOR'
LINES_TYPENAME = 'T_SEG2D_POLY'
LINES_TYPENAME2 = 'T_VEC_PATH'

PATH_MAP = 'PATH_PLAN'
PATH_MAP_SUFFIX = 'V'
PATH_MAP_TYPE = 'INTEGER'
PATH_DATA = 'LINES'
PATH_DATA_SUFFIX = 'V'
MOTION_DATA_TYPE = 'VECTOR'

PATH_VARNAME = 'PPATH'
TOOLPATH_VARNAME = 'PTH'
TOOLPATH_SUFFIX = 'V'
TOOLPATH_TYPE = 'TYP'
TOOLPATH_CODE = 'CODE'
TOOLPATH_GROUP = '1'

# ** robodk parameters **
PART_NAME = 'path'
PROG_NAME = 'AutoProgram'

#pattern1 = r"Field: {LINES_VARNAME}\.NODEDATA\[(\d{1,5})\]\.{LINE_START_SUFFIX} Access: RW: VECTOR =\s*(.*)"
#pattern2 = r"Field: {LINES_VARNAME}\.NODEDATA\[(\d{1,5})\]\.{LINE_END_SUFFIX} Access: RW: VECTOR =\s*(.*)"

folder_files = ''

raster_lines = []
polygons = []
t_polygon = collections.namedtuple('t_polygon',
  'coords '
  'code '
  'polygon '
  'tangent'
)

# for plotting on drawing
rpath = []
# for robodk path
tpath = []

t_path = collections.namedtuple('t_path',
  'pose '
  'type '
  'code '
  'tangent'
)
path_plan = []

#enum toolpath type
PTH_TOOLING = 100
PTH_LINKING = 105

def random_color_gen():
  """Generates a random RGB color
  
  :return: 3 elements in the form [R, G, B]
  :rtype: list
  """
  r = randint(0, 255)
  g = randint(0, 255)
  b = randint(0, 255)
  return (r, g, b)
  

def parseContour(member_name, args, out_list):

  parsefile = folder_files +'/' + SAVE_DIRECTORY + '/' + args.rbt_fl + '.VA'

  pattern_code = rf"Field: {member_name}\.NODEDATA\[(\d+)\]\.{CONTOUR_CODE_SUFFIX} Access: RW: {CONTOUR_CODE_TYPE} =\s*(.*)"
  pattern_vector = rf"Field: {member_name}\.NODEDATA\[(\d+)\]\.{CONTOUR_VEC_SUFFIX} Access: RW: {CONTOUR_VEC_TYPE} =\s"
  pattern_polygon = rf"Field: {member_name}\.NODEDATA\[(\d+)\]\.POLYGON Access: RW: SHORT =\s*(.*)"
  pattern_tangent = rf"Field: {member_name}\.NODEDATA\[(\d+)\]\.TANGENT Access: RW: VECTOR =\s"
  patternx = r"X:\s*(-?\d{0,3}\.\d{1,3})"
  patterny = r"Y:\s*(-?\d{0,3}\.\d{1,3})"

  coords = ('', '')
  code = ''
  polygon = ''
  tangent = ('', '')

  with open(parsefile,'r') as f:

    lines = f.readlines()
    for i in range(len(lines)):

      m2 = re.search(pattern_code, lines[i])
      m1 = re.search(pattern_vector, lines[i])
      m3 = re.search(pattern_tangent, lines[i])
      m4 = re.search(pattern_polygon, lines[i])

      nid = None
      if m1:
        # get index
        nid = int(m1.group(1)) - 1
        # get x coordinate
        mvec = re.search(patternx, lines[i+1])
        new_x = 0.0
        if mvec:
          new_x = float(mvec.group(1))
        # get x=y coordinate
        mvec = re.search(patterny, lines[i+1])
        new_y = 0.0
        if mvec:
          new_y = float(mvec.group(1))
        # append to list
        coords = (new_x, new_y)
      
      if m2:
        # get index
        nid = int(m2.group(1)) - 1
        code = int(m2.group(2))
      
      if m3:
        # get index
        nid = int(m3.group(1)) - 1
        # get x coordinate
        mvec = re.search(patternx, lines[i+1])
        new_x = 0.0
        if mvec:
          new_x = float(mvec.group(1))
        # get x=y coordinate
        mvec = re.search(patterny, lines[i+1])
        new_y = 0.0
        if mvec:
          new_y = float(mvec.group(1))
        # append to list
        tangent = (new_x, new_y)
      
      if m4:
        # get index
        nid = int(m4.group(1)) - 1
        polygon = int(m4.group(2))

      poly = t_polygon(
          coords = coords,
          code = code,
          polygon = polygon,
          tangent = tangent
        )
      if nid is not None:
        if len(out_list) > nid:
          out_list[nid] = poly
        else:
          out_list.insert(nid, poly )
        
        nid = None


def parsePath(member_name, args, out_list):

  parsefile = folder_files +'/' + SAVE_DIRECTORY + '/' + args.rbt_fl + '.VA'

  pattern_code = rf"Field: {member_name}\.NODEDATA\[(\d+)\]\.{TOOLPATH_CODE} Access: RW: SHORT =\s*(.*)"
  pattern_vector = rf"Field: {member_name}\.NODEDATA\[(\d+)\]\.{TOOLPATH_SUFFIX} Access: RW: XYZWPR =\s"
  pattern_type = rf"Field: {member_name}\.NODEDATA\[(\d+)\]\.{TOOLPATH_TYPE} Access: RW: SHORT =\s*(.*)"
  pattern_tangent = rf"Field: {member_name}\.NODEDATA\[(\d+)\]\.TANGENT Access: RW: VECTOR =\s"
  
  patterconfig = r"Group\:\s{TOOLPATH_GROUP}\s*Config\:\s[A-Z]\s[A-Z]\s[A-Z],\s\d,\s\d,\s\d\s*"
  patternxyz = r"X:\s*(-?\d{0,3}\.\d{1,3})\s*Y:\s*(-?\d{0,3}\.\d{1,3})\s*Z:\s*(-?\d{0,3}\.\d{1,3})"
  patternwpr = r"W:\s*(-?\d{0,3}\.\d{1,3})\s*P:\s*(-?\d{0,3}\.\d{1,3})\s*R:\s*(-?\d{0,3}\.\d{1,3})"
  patternx = r"X:\s*(-?\d{0,3}\.\d{1,3})"
  patterny = r"Y:\s*(-?\d{0,3}\.\d{1,3})"
  patternz = r"Z:\s*(-?\d{0,3}\.\d{1,3})"

  pose = ''
  code = ''
  typ = ''
  tangent = ('', '')

  with open(parsefile,'r') as f:

    lines = f.readlines()
    for i in range(len(lines)):

      m2 = re.search(pattern_code, lines[i])
      m1 = re.search(pattern_vector, lines[i])
      m3 = re.search(pattern_tangent, lines[i])
      m4 = re.search(pattern_type, lines[i])

      nid = None
      if m1:
        # get index
        nid = int(m1.group(1)) - 1
        # get x coordinate
        mconfig = re.search(patterconfig, lines[i+1])
        mxyz = re.search(patternxyz, lines[i+2])
        mwpr = re.search(patternwpr, lines[i+3])

        pose = ([float(mxyz.group(1)), float(mxyz.group(2)), float(mxyz.group(3)), float(mwpr.group(1)), float(mwpr.group(2)), float(mwpr.group(3))] )
      
      if m2:
        # get index
        nid = int(m2.group(1)) - 1
        code = int(m2.group(2))
      
      if m3:
        # get index
        nid = int(m3.group(1)) - 1
        # get x coordinate
        mvec = re.search(patternx, lines[i+1])
        new_x = 0.0
        if mvec:
          new_x = float(mvec.group(1))
        # get x=y coordinate
        mvec = re.search(patterny, lines[i+1])
        new_y = 0.0
        if mvec:
          new_y = float(mvec.group(1))
        # get z coordinate
        mvec = re.search(patternz, lines[i+1])
        new_z = 0.0
        if mvec:
          new_z = float(mvec.group(1))
        # append to list
        tangent = (new_x, new_y, new_z)
      
      if m4:
        # get index
        nid = int(m4.group(1)) - 1
        typ = int(m4.group(2))

      nPath = t_path(
          pose = pose,
          code = code,
          type = typ,
          tangent = tangent
        )
      if nid is not None:
        if len(out_list) > nid:
          out_list[nid] = nPath
        else:
          out_list.insert(nid, nPath )
        
        nid = None

def line2toolpath(in_list, out_list):
  for i in range(len(in_list)):

    nPath = t_path(
            pose = in_list[i].coords,
            code = in_list[i].code,
            type = 1,
            tangent = in_list[i].tangent
          )
    
    out_list.append(nPath)

def parsePlan(args, lst):
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
  
  # get path coordinates
  for i in range(len(path_order)):
    path_plan.append([path_order[i], lst[path_order[i]-1].pose])
  
def parsePath3D(args):
  parsefile = folder_files +'/' + SAVE_DIRECTORY + '/' + args.rbt_fl + '.VA'

  pattern_map = rf"Field: {TOOLPATH_VARNAME}\.NODEDATA\[(\d+)\]\.{TOOLPATH_SUFFIX} Access: RW: XYZWPR =\s*Group\:\s{TOOLPATH_GROUP}\s*Config\:\s[A-Z]\s[A-Z]\s[A-Z],\s\d,\s\d,\s\d\s*(.*)\s*(.*)\s*Field: {TOOLPATH_VARNAME}\.NODEDATA\[(\d+)\]\.SPEED Access: RW: REAL = -?[\d.]+e[+-]?\d+?\s*Field: PTH.NODEDATA\[\d+\].{TOOLPATH_CODE} Access: RW: SHORT = (\d+)\s*Field: PTH.NODEDATA\[\d+\].{TOOLPATH_TYPE} Access: RW: SHORT = (\d+)"
  patternxyz = r"X:\s*(-?\d{0,3}\.\d{1,3})\s*Y:\s*(-?\d{0,3}\.\d{1,3})\s*Z:\s*(-?\d{0,3}\.\d{1,3})"
  patternwpr = r"W:\s*(-?\d{0,3}\.\d{1,3})\s*P:\s*(-?\d{0,3}\.\d{1,3})\s*R:\s*(-?\d{0,3}\.\d{1,3})"

  p = []

  with open(parsefile,'r') as f:
    lines = f.read()
    m1 = re.finditer(pattern_map, lines)
    if m1:
      for x in m1:
        nid = int(x.group(1)) - 1
        m2 = re.search(patternxyz, x.group(2))
        m3 = re.search(patternwpr, x.group(3))
        p.append([float(m2.group(1)), float(m2.group(2)), float(m2.group(3)), float(m3.group(1)), float(m3.group(2)), float(m3.group(3))] )
        #codes
        tpath_codes.append((int(x.group(4)), int(x.group(5))))

    return(np.array(p))


def print_cont(list_obj):
  for i in range(len(list_obj)):
    print("{}: {:.3f}, {:.3f} : {:.3f}, {:.3f}".format(list_obj[i].code, list_obj[i].coords[0], list_obj[i].coords[1], list_obj[i].tangent[0], list_obj[i].tangent[1]))

def print_plan(list_obj):
  for i in range(len(list_obj)):
      if len(list_obj[i][1]) > 3:
        print("{}: [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}]".format(list_obj[i][0], list_obj[i][1][0], list_obj[i][1][1], list_obj[i][1][2], list_obj[i][1][3], list_obj[i][1][4], list_obj[i][1][5]))
      else:
        print("{}: [{:.1f}, {:.1f}]".format(list_obj[i][0], list_obj[i][1][0], list_obj[i][1][1]))

def print_path(list_obj):
  for i in range(len(list_obj)):
    print("{}: [{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}] : {:.3f}, {:.3f}, {:.3f}".format(i+1,list_obj[i].pose[0],list_obj[i].pose[1],list_obj[i].pose[2],list_obj[i].pose[3],list_obj[i].pose[4],list_obj[i].pose[5], list_obj[i].tangent[0], list_obj[i].tangent[1], list_obj[i].tangent[2]))

def slice(obj, name):
  list = []
  for o in obj:
    list.append(getattr(o, name))
  
  return list

def plot():
  fig, ax = plt.subplots()

  #ploygons
  Path = mpath.Path
  if len(polygons) > 0:
    codes = slice(polygons, 'code')
    verts = slice(polygons, 'coords')
    tang = slice(polygons, 'tangent')
    start_idx = 0 ; end_idx = len(polygons)

  for i in range(len(polygons)):
    if codes[i] == Path.MOVETO:
      start_idx = i
    if (codes[i] == Path.CLOSEPOLY) or (codes[i] == Path.STOP):
      end_idx = i

      color = random_color_gen()
      face_color = list(map(lambda i: i*1.0/255, color))
      color2 = (color[0]*2%255, color[1]*2%255, color[2]*2%255)
      edge_color = list(map(lambda i: i*1.0/255, color2))

      path = mpath.Path(verts[start_idx:end_idx], codes[start_idx:end_idx])
      patch = mpatches.PathPatch(path, facecolor=face_color, edgecolor=edge_color, alpha=0.5)
      ax.add_patch(patch)

    # add tangent vectors
    ax.arrow(verts[i][0], verts[i][1], tang[i][0]*5, tang[i][1]*5, head_width=3, head_length=5, fc='r', ec='r')
  
  #lines
  if len(raster_lines) > 0:
    verts = slice(raster_lines, 'coords')
    codes = slice(raster_lines, 'code')
    poly = slice(raster_lines, 'polygon')
    tang = slice(raster_lines, 'tangent')

    r0 = []
    r1 = []

    for i in range(len(raster_lines)):
      if codes[i] == mpath.Path.MOVETO:
        a = list(verts[i])
        a.extend(list(tang[i]))
        r0.append(a)
      if (codes[i] == Path.CLOSEPOLY) or (codes[i] == Path.STOP):
        a = list(verts[i])
        a.extend(list(tang[i]))
        r1.append(a)

    r0 = np.array(r0)
    r1 = np.array(r1)
  
    x = list(zip(r0[:,0], r1[:,0]))
    y = list(zip(r0[:,1], r1[:,1]))

  for i in range(len(r0)):
    color = random_color_gen()
    color = list(map(lambda x: x*1.0/255, color))
    #color = 'yellow'
    draw = plt.Line2D(x[i], y[i], color=color)
    ax.add_line(draw)

    # add tangent vectors
    ax.arrow((r0[i][0]+r1[i][0])/2, (r0[i][1]+r1[i][1])/2, r0[i][2]*5, r0[i][3]*5, head_width=3, head_length=5, fc='g', ec='g')

  #path
  if len(path_plan) > 0:
    idx, verts = zip(*path_plan)
    if len(verts[0]) > 2:
      #position array
      xs, ys, _, _, _, _ = zip(*verts) 
    else:
      #vec2d array
      xs, ys = zip(*verts)
    ax.plot(xs, ys, 'o--', lw=2, color='red', ms=5)
  
  ax.grid()
  ax.axis('equal')
  plt.show()

def op_code(code):
  if code == mpath.Path.MOVETO:
    return(mpath.Path.STOP)
  if code == mpath.Path.STOP:
    return(mpath.Path.MOVETO)
  
  return(code)

def plot3D(lst):
  RDK = Robolink()

  RDK.Render(False)

  # Remove previous objects
  obj_delete = RDK.Item(PART_NAME, ITEM_TYPE_OBJECT)
  if obj_delete.Valid():
      obj_delete.Delete()

  # remove all targets
  obj_delete = RDK.Item(PROG_NAME, ITEM_TYPE_PROGRAM)
  while obj_delete.Valid():
    obj_delete.Delete()

  # Get the main/only robot in the station
  robot = RDK.Item('', ITEM_TYPE_ROBOT)
  if not robot.Valid():
      raise Exception("Robot not valid or not available")

  frame = robot.getLink(ITEM_TYPE_FRAME)
  tool = robot.getLink(ITEM_TYPE_TOOL)
  
  prog = RDK.AddProgram(PROG_NAME)
  prog.ShowInstructions(True)
  prog.setPoseFrame(frame)
  prog.setPoseTool(tool)

  if len(lst) > 0:
    #draw lines
    made_object = False
    crve = []
    code = -1
    for i in range(len(lst)):
      #reset curve
      if (lst[i].type == PTH_LINKING):
        crve = []
        code = -1
      
      if (lst[i].type == PTH_TOOLING):
        if ((lst[i].code == mpath.Path.MOVETO) or (lst[i].code == mpath.Path.STOP)) and (code == -1):
          code = lst[i].code

        if (lst[i].code == code):
          crve.append(lst[i].pose)
        elif (lst[i].code == op_code(code)):
          crve.append(lst[i].pose)
          if made_object:
            RDK.AddCurve(crve, objct, True)
          else:
            objct = RDK.AddCurve(crve)
            objct.setParent(frame)
            objct.setName(PART_NAME)
            made_object = True
        else:
          crve.append(lst[i].pose)


    #draw points

    for i in range(len(lst)):
      target = RDK.AddTarget('T%i' % (i), prog)
      target.setAsCartesianTarget()
      target.setPose(xyzrpw_2_pose(lst[i].pose))
      prog.MoveL(target)
    
    prog.ShowTargets(False)
    RDK.Render(True)



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
               "FANUC controller. Depends on Robodk API.")

  parser = argparse.ArgumentParser(prog='plot_drawing', description=description,
                                  formatter_class=argparse.MetavarTypeHelpFormatter)

  parser.add_argument('rbt_fl', type=str, nargs='?',
        help="Name of karel file")
  parser.add_argument('-r', '--robodk', action='store_true', dest='use_robodk',
        help='Plot path in robodk')
  parser.add_argument('-p', '--pathplan', action='store_true', dest='onpathplan',
        help='visualize only a pathplanning module')

  args = parser.parse_args()

  folder_files = os.path.dirname(os.path.realpath(__file__))
  folder_files = os.path.abspath(os.path.join(folder_files, os.pardir))
  print('parent fldr: ', folder_files)
  # start an ftp instance
  robot = RobotFTP(args.rbt_fl, ROBOT_IP, ROBOT_PORT, username = USERNAME, password = PASSWORD)

  # get lines
  parseContour(LINES_VARNAME, args, raster_lines)

  # get contours
  parseContour(CONTOUR_VARNAME, args, polygons)

  # get path
  if args.onpathplan:
    # for pathplan test
    line2toolpath(raster_lines, rpath)
  else:
    parsePath(PATH_VARNAME, args, rpath)
  
  # path plan
  parsePlan(args, rpath)

  if args.use_robodk:
    # for actual toolpath
    parsePath(TOOLPATH_VARNAME, args, tpath)

  print('polygons')
  print_cont(polygons)
  print('lines')
  print_cont(raster_lines)
  print('path')
  print_plan(path_plan)

  if (not args.onpathplan) and (args.use_robodk):
    print('toolpath')
    print_path(tpath)

  if args.use_robodk:
    #plot robodk path
    plot3D(tpath)

  #plot drawing
  plot()

  #keep window from closing
  plt.show()



if __name__ == "__main__":
  main()
