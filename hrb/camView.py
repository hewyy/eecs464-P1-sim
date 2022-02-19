from joy import JoyApp, progress
from joy.plans import AnimatorPlan
from joy.decl import KEYDOWN 
from cv2 import VideoCapture

if __name__ != "__main__":
  raise RuntimeError("Run this as a script")



def _animation(fig):
  c = VideoCapture(0)
  ax = fig.add_subplot(1,1,1)
  while True:
    ok,img = c.read()
    if ok:
      ax.cla()
      ax.imshow(img[::-1,...])
    yield

class App(JoyApp):
  def onStart(self):
      AnimatorPlan(self,_animation).start()

  def onEvent(self,evt):
      if evt.type == KEYDOWN:
        return JoyApp.onEvent(self,evt)
        
app = App(cfg={'windowSize':(1080,740)})
app.run()
  
