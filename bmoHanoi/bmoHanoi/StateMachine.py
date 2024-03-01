GOTO_REC_T = 7
REC_T = 1
GOTO_PRI_T = 5
GRAB_T = 3
READY_T = 10
HONE_T = 5
GO_UP_T = 4
HOLD_T = 3
PRI_HALF_T = 5
HONE_HALF_T = 5
MOVE_DOWN_T = 3

TIME_DICT = {'GOTO_REC' : GOTO_REC_T, 
                          'REC' : REC_T, 
                          'GOTO_PRI' : GOTO_PRI_T,
                          'GOTO_READY': READY_T, 
                          'HONE' : HONE_T,
                          'GO_UP': GO_UP_T,
                          'HOLD': HOLD_T,
                          'GOTO_PRI_HALF': PRI_HALF_T,
                          'HONE_HALF' :HONE_HALF_T,
                          'MOVE_DOWN' : MOVE_DOWN_T,
                          'REC_HALF': REC_T}

class StateMachine():
    def __init__(self, sec, nano):
        self.prSt = 'GOTO_REC'
        self.nxSt = 'GOTO_REC'
        self.priorityDonut = None
        self.T = GOTO_REC_T
        self.t = 0.0
        self.dt = 0.0
        self.start_time = sec + nano*10**(-9)
        self.initJointPos = None
        self.grab = False
        
    def setT(self):
        self.T = TIME_DICT[self.prSt]
    
    def get_state(self):
        return self.prSt
    
    def updateTime(self, sec, nano):
        if self.t < self.T:
            now = sec + nano*10**(-9)
            self.dt = (now - self.start_time) - self.t
            self.t = now - self.start_time

    def updateState(self):
        self.prSt = self.nxSt

    def updateNextState(self, sec, nano, priorityDonut, peg):
        if (self.t < self.T):
            self.nxSt = self.prSt
        else:
          match self.prSt:
            case 'GOTO_REC':
                self.nxSt = 'REC'
            case 'REC':
                # self.get_logger().info(f"priority {(self.priorityDonut)}")
                if peg:
                  self.nxSt = "GOTO_PEG"
                elif priorityDonut is not None:
                  self.nxSt = 'HONE_HALF'
                else:
                  self.nxSt = 'REC'
            case 'HONE_HALF': 
                self.nxSt = 'MOVE_DOWN'
            case 'MOVE_DOWN':
                self.nxSt = 'HONE'
            case 'HONE': 
                self.nxSt = 'REC_HALF'
            case 'REC_HALF':
                self.nxSt = 'GOTO_PRI'
            case 'GOTO_PRI':
                self.nxSt = 'HOLD'
            case 'HOLD':
                self.nxSt = 'GO_UP'
            case 'GO_UP':
                self.nxSt = 'GOTO_REC'
            case 'GOTO_PEG':
                self.nxSt = 'GO_DOWN'
            case 'GO_DOWN':
                self.nxSt = 'HOLD_OFF'
            case 'HOLD_OFF':
                self.nxST = 'GOTO_REC'

                  
          self.t = 0.0
          self.dt = 0.0
          self.start_time = sec + nano*10**(-9)
          return True
        
    def get_curr_state(self):
        return self.prSt 