GOTO_REC_T = 4
REC_T = 1
GOTO_PRI_T = 3
GRAB_T = 2
GAME_STATE_T = 5
HONE_T = 3
GO_UP_T = 4
HOLD_T = 3
HONE_HALF_T = 2
PRI_HALF_T = 3
HONE_HALF_T = 3
MOVE_DOWN_T = 3
BAD_GRAB_T = 3
HONE_BLACK_T = 3


GOTO_PEG_T = 6

TIME_DICT = {'GOTO_REC' : GOTO_REC_T, 
                          'REC' : REC_T, 
                          'GOTO_PRI' : GOTO_PRI_T,
                          'GOTO_GAME_STATE': GAME_STATE_T, 
                          'HONE' : HONE_T,
                          'GO_UP': GO_UP_T,
                          'HOLD': HOLD_T,
                          'GOTO_PRI_HALF': PRI_HALF_T,
                          'HONE_HALF' :HONE_HALF_T,
                          'MOVE_DOWN' : MOVE_DOWN_T,
                          'REC_HALF': REC_T,
                          'GOTO_REC_PEG': GOTO_REC_T,
                          'REC_PEG': REC_T,
                          'GOTO_PEG': GOTO_PEG_T,
                          'GO_DOWN': GO_UP_T,
                          'HOLD_OFF': HOLD_T,
                          'SPECIAL_REC': REC_T,
                          'BAD_GRAB': BAD_GRAB_T,
                          'HONE_BLACK': HONE_BLACK_T}

class StateMachine():
    def __init__(self, sec, nano):
        self.prSt = 'GOTO_REC'
        self.nxSt = 'GOTO_REC'

        # self.prSt = 'GOTO_GAME_STATE'

        # self.nxSt = 'GOTO_GAME_STATE'
        self.priorityDonut = None
        self.T = GOTO_REC_T
        self.t = 0.0
        self.dt = 0.0
        self.start_time = sec + nano*10**(-9)
        self.initJointPos = None
        self.grab = False
        self.brodie = True

        self.restart = False
        
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

    def updateNextState(self, sec, nano, priorityDonut, place, successful_grab, peg):
        if (self.t < self.T):
            self.nxSt = self.prSt
            if self.restart and not self.brodie:
                # reset the whole thing
                self.prSt = 'GOTO_REC'
                self.nxSt = 'GOTO_REC'
                self.t = 0.0
                self.dt = 0.0
                self.start_time = sec + nano*10**(-9)
                self.brodie = True
                return True
        else:
          match self.prSt:
            case 'GOTO_REC':
                self.nxSt = 'REC'
            # case 'GOTO_GAME_STATE':
                self.nxSt = 'REC'          
            case 'REC':
                if (priorityDonut is not None and not place) or (place and peg):
                #   if (peg in ['black1', 'black2', 'black3']) and place:
                #       self.nxSt = ''
                #   else:
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
                if place:
                    self.nxSt = 'GOTO_PEG'
                else:
                    self.nxSt = 'GOTO_PRI'
            case 'GOTO_PRI':
                self.nxSt = 'HOLD'
            case 'HOLD':
                if (successful_grab):
                    self.nxSt = 'GO_UP'
                else:
                    self.nxSt = 'BAD_GRAB'
            case 'GO_UP':
                self.nxSt = 'GOTO_REC'
            case 'BAD_GRAB':
                  self.nxSt = 'REC_HALF'
            case 'GOTO_PEG':
                self.nxSt = 'GO_DOWN'
            case 'GO_DOWN':
                self.nxSt = 'GOTO_REC'
            case 'SPECIAL_REC':
                self.nxSt = 'GOTO_REC'
            case 'HONE_BLACK':
                self.nxSt = 'HONE'
                  
          self.t = 0.0
          self.dt = 0.0
          self.start_time = sec + nano*10**(-9)
          return True
        
    def get_curr_state(self):
        return self.prSt 
