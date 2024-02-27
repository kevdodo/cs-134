import copy 

def spline(t, T, p0, pf, v0, vf):
    # Compute the parameters.
    a = p0
    b = v0
    c =   3*(pf-p0)/T**2 - vf/T    - 2*v0/T
    d = - 2*(pf-p0)/T**3 + vf/T**2 +   v0/T**2
    # Compute the current (p,v).
    p = a + b * t +   c * t**2 +   d * t**3
    v =     b     + 2*c * t    + 3*d * t**2
    return (p,v)

class Spline():
    def __init__(self, p0, pf, v0, vf, T, space) -> None:
        self.p0 = p0 
        self.pf = pf 
        self.v0 = v0 
        self.vf = vf 
        self.T = T 

        assert(space in ['Tip', 'Joint'])
        self.space = space 

    def getSpace(self):
        return self.space 
    
    def evaluate(self, t):
        '''
        Compute the p, v of the given spline.
        
        Inputs:
        t - time in seconds since the start of the current spline 

        Outputs:
        p, v - the position and the velocity 
        '''

        # TODO: Spline interpolation only works for position, for orientation you need to write your own interpolation code here! 
        '''
        Note: 

        If using orientation, you will have to divide this function into 2 parts
        IF the space is joint, then just interpolate joint wise
        If the space is task, then interpolate position regularly, but interpolating the 
        orientation should be done separately, and returned. 
        '''

        p, v = spline(t, self.T, self.p0, self.pf, self.v0, self.vf)          
        return p, v  
    
    def completed(self, t):
        '''
        Returns true if the spline is completed, false otherwise 
        '''
        return t > self.T

class SegmentQueue():
    '''
    Attributes: 

    self.queue: The queue which holds the queue of splines that are enqueued
    self.q: The joint space configuration of the robot currently 
    self.t: The current time 
    self.t0: The time the curret spline was initiated. This is useless if there 
    is no current spline 

    '''
    def __init__(self) -> None:
        self.queue = []
        self.t0 = 0
        self.t = 0 


    def enqueue(self, segment):
        '''
        Function to enqueue a segment
        segment: A "Spline" object
        '''
        if len(self.queue) == 0:
            # If empty queue, set current segment's start time as the current time
            self.t0 = self.t 
            
        self.queue.append(copy.deepcopy(segment))

    def clear(self):
        '''
        Clears the queue, stopping all motion. 
        '''
        self.queue = []

    def update(self, t):
        '''
        Update function. Every single tick, this function must be run, so that 
        the segment queue can keep track of the current time and q.

        t: The current time
        
        '''

        self.t = t 

        if len(self.queue) == 0:
            # If nothing in queue, just update the time
            return 
        
        if self.queue[0].completed(self.t - self.t0):
            # If the current segment is completed, discard it
            self.queue.pop(0)

            if len(self.queue) == 0:
                return 
            else:
                # If there is a new segment, set the start time of that segment to the current time
                self.t0 = t 

    def evaluate(self):
        '''
        Returns the current p, v (in the given space)
        '''
        if len(self.queue) == 0: 
            raise Exception("No Segment to Run")

        else: 
            return self.queue[0].evaluate(self.t - self.t0)


    def getCurrentSpace(self):
        '''
        Returns the space of the current spline, if there is any spline.
        Returns 'No Spline' if no spline
        '''

        if len(self.queue) == 0: 
            return 'No Spline'

        else: 
            return self.queue[0].getSpace()
