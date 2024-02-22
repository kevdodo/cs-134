
GOTO_REC_T = 5
REC_T = 5
GOTO_PRI_T = 5
GRAB_T = 5
class State():  # Class names should be CamelCase

    def __init__(self):  # Fixed typo here
      self.T = 0
      self.t = 0
      self.prSt = 'GOTO_REC'
      self.nxSt = 'GOTO_REC'
      self.priority = None
      self.position0 = 0#TODO actual pos
      self.orientation0 = 0#TODO actual
      self.pd = self.position0
      self.vd = 0
      self.rd = self.orientation0
      self.wd = 0
      self.reconPos = 0#TODO actual
      self.reconOr = 0

    def setT(self):
      match self.prSt:
          case 'GOTO_REC':
              self.T = GOTO_REC_T
          case 'REC':
              self.T = REC_T
          case 'GOTO_PRI':
              self.T = GOTO_PRI_T
          case 'GRAB':
              self.T = GRAB_T

    def updateTime(self):
        if self.prSt != self.nxSt:
            self.t = 0
        elif self.t < self.T:
            self.t += 1#TODO add timing.

    def updateState(self):
        self.prSt = self.nxSt

    def updateNextState(self):
        if (self.t < self.T):
            self.nxSt = self.prSt
        else:
          match self.prSt:
            case 'GOTO_REC':
                self.nxSt = 'REC'
            case 'REC':
                if self.priority == None:
                  self.nxSt = 'REC'
                else:
                  self.nxSt = 'GOTO_PRI'
            case 'GOTO_PRI':
                self.nxSt = 'GRAB'
            case 'GRAB':
                self.nxSt = 'GOTO_REC'
        self.position0 = self.pd
    
    def performState(self):
       match self.prSt:
            case 'GOTO_REC':
              self.gotoRec()
            case 'REC':
              self.recon()
            case 'GOTO_PRI':
              self.gotoPri()
            case 'GRAB':
              self.grab()

  
    def gotoRec(self):
       pd, vd = spline5(self.t, self.T, self.position0, self.reconPos)
       rd, wr = spline5(self.t, self.T, self.orientation0, self.reconOr)
       return pd, vd, rd, wr
      
    # must be cyclic
    def recon(self):
       pd, vd, rd, wr = fkin(sin(qs))
       return pd, vd, rd, wr
    
    def gotoPri(self):
       pass
    
    def grab(self):
       pass