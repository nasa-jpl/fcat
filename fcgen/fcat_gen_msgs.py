#!/usr/bin/env python3
import sys
import yaml
import re

def snake2camel(str):
  return ''.join(x.capitalize() or '_' for x in str.split('_'))

def fctype2msgtype(fctype):
  if fctype == "double":
      return "float64"
  elif fctype == "uint8_t":
      return "uint8"
  elif fctype == "int8_t":
      return "int8"
  elif fctype == "uint16_t":
      return "uint16"
  elif fctype == "int16_t":
      return "int16"
  elif fctype == "uint32_t":
      return "uint32"
  elif fctype == "int32_t":
      return "int32"
  elif fctype == "bool":
      return "bool"
  else:
    return "fcat_gen_msgs_error"

def isCsCommand(s):
  return ( ("csp" in s) or ("csv" in s) or ("cst" in s))

def getHeaderStr():
    return "string name\n"

class FcatMsgGenerator:
  def __init__(self, yaml_file):
    print("Opening input YAML file: %s" % yaml_file)
    self.data = yaml.load(open(yaml_file, 'r'), Loader=yaml.Loader)

  def gen_msgs(self):
    print("Generating msgs...")
    for state in self.data['states']:
        filename = "msg/" + snake2camel(state['name']) + "State.msg"
        f = open(filename, 'w')
        f.write(getHeaderStr())
        for field in state['fields']:
            type_str = fctype2msgtype(field['type']) 
            f.write(type_str + " " + field['name'] + "\n")
        f.close()

    for cmd in self.data['commands']:
      if isCsCommand(cmd['name']):
        filename = "msg/" + snake2camel(cmd['name']) + "Cmd.msg"
        f = open(filename, 'w')
        f.write(getHeaderStr())
        for field in cmd['fields']:
            type_str = fctype2msgtype(field['type']) 
            f.write(type_str + " " + field['name'] + "\n")
        f.close()

    print("Successfully created msgs")
  
  def gen_srvs(self):
    print("Generating srvs...")
    for cmd in self.data['commands']:
        if isCsCommand(cmd['name']):
            continue # these need to be msgs, not services
        filename = "srv/" + snake2camel(cmd['name']) + "Cmd.srv"
        f = open(filename, 'w')
        f.write(getHeaderStr())
        for field in cmd['fields']:
            type_str = fctype2msgtype(field['type']) 
            f.write(type_str + " " + field['name'] + "\n")
        f.write("---\n")
        f.write("bool error\n") ## use a simple bool return status
        f.close()
    print("Successfully created srvs")

if __name__ == '__main__':
  fmg = FcatMsgGenerator(sys.argv[1])
  fmg.gen_msgs()
  fmg.gen_srvs()

