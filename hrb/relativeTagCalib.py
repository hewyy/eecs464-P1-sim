from socket import socket, AF_INET, SOCK_DGRAM, error as SocketError
from numpy import asarray
from json import loads as json_loads, dumps as json_dumps
from sys import stdout

if __name__ != "__main__":
  raise RuntimeError("Run this as a script")

try:
  s.close()
except Exception:
  pass

s = socket(AF_INET, SOCK_DGRAM )
s.bind(("",0xB00))
s.setblocking(0)

lst = []
msg = None
rh={}
i=0 	
longest = 0
allow = set([13,14,15])
while longest<100: #number of samples
  try:
    # read data as fast as possible
    m = s.recv(1<<16)
    if m and len(m)>2:
      msg = m
    else:
      continue
  except SocketError, se:
    # until we've run out; last message remains in m
    continue
  # make sure we got something
  if not msg:
    continue
  #Assembly pose dictionary with relative inverse by tag IDs
  rone = array([[0,0,0,1]])
  dat = json_loads(msg)
  if len(dat)<2:
    continue
  # If there are duplicates of any ID --> skip this frame
  if len(set([d['i'] for d in dat])) < len(dat):
    print " -- duplicates found"
    continue
  h={}
  for d in dat:
    nm = d['i']
    if not nm in allow:
      continue
    a=r_[d['x'],rone]
    r=a[:3,:3].T
    t=a[:3,3]
    ainv = r_[c_[r,dot(r,-t)],rone]
    h[nm] = (a,ainv)
    print nm,
  #Assemble dictionary of relative pairs.
  lst = h.keys()
  lst.sort()
  for j in lst:
    for k in lst[lst.index(j)+1:]:
      if (j,k) not in rh:
        L=list()
        rh[j,k] = L
      else:
        L = rh[j,k]
      L.append(dot(h[j][1],h[k][0]))
      longest = max(len(L),longest)
  print " -- ",longest
  stdout.flush()

print '-'*40,"RESULTS"
res = {}
for k,v in rh.iteritems():
  m = array(v)
  # Use median to estimate rotation matrix elements
  R0 = median(m[:,:3,:3].reshape(m.shape[0],9),0).reshape(3,3)
  # Polar decomposition
  U,s,V = svd(R0)
  R = dot(U,V)
  # Use median to estimate translation
  t = median(m[:,:3,3],0)
  cal = r_[c_[R,t],rone]
  res["%d_%d" % k] = [ list(x) for x in cal ]
  print k
  print cal.round(3)

print "Calibration data is |%s|" % json_dumps(res)


