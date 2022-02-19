# Example of reading mouse data directly

from sys import argv,stderr,exit,stdout
from struct import unpack

if len(argv)<2:
  stderr.write("""
    Useage: python %s <device>
      where <device> is typically /dev/input/mice or /dev/input/mouse0
  """)
  exit(-1)


with open(argv[1],"rb") as f:
  while True:
    x,y,b = unpack("bbb",f.read(3))
    stdout.write("\r %3g,%3g %2x" % (x,y,b))
    stdout.flush()
