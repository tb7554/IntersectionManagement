from __future__ import division
import os, sys
from scipy.stats.mstats_basic import tmax

os.environ["SUMO_HOME"] = "/sumo" # Home directory, stops SUMO using slow web lookups for XML files
os.environ["SUMO_BINARY"] = "/usr/local/bin/sumo" # binary for SUMO simulations
 
def getOpenPort():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("",0))
    s.listen(1)
    port = s.getsockname()[1]
    s.close()
    return port

class minMaxGreenTimeController:
    
    def __init__(self, Tmin, Tmax):
        """ Controller that uses basic Tmin and Tmax to adjust green time """
        self._Tmin = Tmin
        self._Tmax = Tmax
        self._initialGreenTime = (Tmax+Tmin)/2
        
    def getInitialGreenTime(self):
        """ Calculates the intial green time that all lights start with """
        return self._initialGreenTime
        
    def getNewGreenTime(self, A, B, timer):
        """ Takes the target number of vehicles to remove from the queue, compares withe the actual number. Returns updated green time."""
        if B > A:
            Gt = (timer + self._Tmin)/2
            print("Decreasing green time")
        elif B < A:
            Gt = (timer + self._Tmax)/2
            print("Increasing Green Time")
        else:
            Gt = timer
            print("No change in green time")  
        return Gt

class LmaxQueueController:
    
    def __init__(self):
        """ Controller that maximises the total number of vehicles released at each phase, regardless of fairness """
        self._L = None
    
    def setL(self, L_array):
        """ Turn the L array into a matrix, for faster operations"""
        self._L = np.matrix(L_array)
        
    def LdotX(self, x):
        """ Returns the dot product of matrix L and the vector x"""
        return np.dot(self._L, np.matrix(x).transpose())
    
    def bestQueueSet(self, x):
        """Picks the best queue to release, based on total number of vehicles in non-conflicting queues"""
        Qindex = np.argmax(self.LdotX(x))
        return Qindex

class intersectionController:
     
    def __init__(self, numQueues, L, greenTimeController, queueController):
        """ Class which controls the lights at each intersection. This class keps track of properties such as
        the time elapsed since the last phase. The algorithm for determining green times and queues will be defined 
        elsewhere and called by this function, in order to make it easy to switch algorithms """
        # Static properties of the intersection
        self.id = '0/0'
        self._numQueues = numQueues # The number of queues at this intersection (i.e. the number of combinations of in queue and out queue)
        self._L = L
        self._Lstrings = self.setLstrings(L)
    
        # Algorithms used for picking queues and calculating green time
        self._timerControl = greenTimeController
        self._queueControl = queueController
        
        # Current values for dynamic properties
        self._currentQindex = 0
        self._currentPhaseString = self._Lstrings[0] # Initialise light settings as all red (will change in the first step of the simulation
        
        # Values to track important variables and state changes
        self._greenTimer = greenTimeController.getInitialGreenTime() # Elapsed time since last phase
        self._amberTimer = 0
        self._state = False # True means green state, False means amber state
        self._queueGreenTimes = [greenTimeController.getInitialGreenTime()]*numQueues
        self._nextGreenString = self._Lstrings[0]
        self._Xs = [0,0,0,1,1,1,0,0,0,1,1,1]
        self._As = [2,2,2,3,3,3,2,2,2,3,3,3]
        self._Bs = [3,3,3,2,2,2,3,3,3,2,2,2]
    
    def __repr__(self):
        """String representation to see what is happening in the class"""
        return ("""
Current Q Index: %.2f
Current Phase String: %s
Curent Green Timer : %.1f
Current Amber Timer: %.1f
Current State : %s
Current Green Times : %s
Next Green String : %s
Queues : %s""" % (float(self._currentQindex), str(self._currentPhaseString), self._greenTimer, self._amberTimer, self._state, str(self._queueGreenTimes), self._nextGreenString, self._Xs))
    
    def setLstrings(self, L):
        """Turns the conflict matrix L into strings that reflect possible traffic light states"""
        strings = []
        for set in L:
            new_string = []
            for light in set:
                if light > 0:
                    new_string.append('r')
                else:
                    new_string.append('G')
            strings.append("".join(new_string))
        return strings
    
    def setAmberPhase(self, old_phase_string, new_phase_string, amberPhaseLength = 3):
        """ Sets the intermediate phase between traffic lights. Returns the phase duration and traffic light string. """
        amberPhase = []
        
        old_phase = list(old_phase_string)
        new_phase = list(new_phase_string)
        
        if old_phase == new_phase:
            amberPhaseLength = 0
            amberPhase = new_phase
        else:
            for ii in range(0, len(old_phase)):
                if old_phase[ii] == 'r' and new_phase[ii] == 'r':
                    amberPhase.append('r')
                elif (old_phase[ii] == 'g' or old_phase[ii] == 'G') and (new_phase[ii] == 'r'):
                    amberPhase.append('y')
                elif old_phase[ii] == 'r' and (new_phase[ii] == 'g' or new_phase[ii] == 'G'):
                    amberPhase.append('y')
                else:
                    print("Something wrong in amber phase logic.")
                    
        amberPhaseString = "".join(amberPhase)
             
        self._amberTimer = amberPhaseLength
        self._currentPhaseSettings = amberPhaseString
    
    def updateGreenTime(self):
        """Updates the green time for the current queue (which will be used next time the queue receives a green light) using the timer algorithm"""
        Gt_current = self._queueGreenTimes[self._currentQindex]
        A_current = self._As[self._currentQindex]
        B_current = self._Bs[self._currentQindex]
        
        Gt_new = self._timerControl.getNewGreenTime(A_current, B_current, Gt_current)
        
        print(Gt_current, A_current, B_current, Gt_new)
        
        elements_to_update = self._L[self._currentQindex][:]
        
        for ii in range(0,len(elements_to_update)):
            if elements_to_update[ii] == 1 : self._queueGreenTimes[ii] = Gt_new # Apply Gt_new to all queues
              
    def updateQs(self):
        """Updates the length of the queues using traci"""
        for ii in range(0,len(self._Xs)):
            if self._Xs[ii] == 0:
                self._Xs[ii] = 1
            else:
                self._Xs[ii] = 0
                
    def updateA(self, fractionReduction):
        """Updates the target number of vehicles to remove from each queue"""
    
    def updateB(self):
        """Updates the actual number of vehicles removed from the queue"""
            
    def chooseQueues(self):
        return self._queueControl.bestQueueSet(self._Xs)
    
    def getQstring(self, Qindex):
        return self._Lstrings[Qindex]
        
    def update(self, stepsize):
        if not(self._state) and self._amberTimer <= 0:
            traci.trafficlights.setRedYellowGreenState("0/0", self._nextGreenString)
            self._state = True
            print("Amber Phase Over. Switching to %s" % self._nextGreenString)
        elif not(self._state) and self._amberTimer > 0:
            self._amberTimer -= stepsize
        elif self._state and self._greenTimer <= 0:
                self.updateQs()
                self.updateGreenTime()
                newQindex = self.chooseQueues()
                self._greenTimer = self._queueGreenTimes[newQindex]
                self._nextGreenString = self.getQstring(newQindex)
                self.setAmberPhase(self._currentPhaseString, self._nextGreenString, amberPhaseLength=3)
                traci.trafficlights.setRedYellowGreenState("0/0", self._currentPhaseSettings)
                self._currentQindex = newQindex
                self._state = False
                print("Green Phase Over. Switching to %s" % self._currentPhaseSettings)
                print(self._queueGreenTimes)
        elif self._state and self._greenTimer > 0:
            self._greenTimer -= stepsize
        else:
            print("Something wrong in update phase logic")
        
class inersectionControllerContainer:
    
    def __init__(self):
        self._ICs = {}
        
    def addIC(self, IC):
        None

if __name__ == "__main__":
    
    import socket # to execute the getOpenPort() function
    import traci # SUMO API
    import subprocess
    import numpy as np
    from sumolib import net
    
    # Input arguments
    netFile_filepath = "netFiles/grid.net.xml" #sys.argv[1]
    routeFile_filepath = "netFiles/grid.rou.xml" #sys.argv[2]
    stepLength = 0.1
    tripInfoOutput_filepath = "tripsoutput.xml"
    traciPort = getOpenPort()
    
    sln = net.readNet(netFile_filepath)
    
    TLS  = sln._tlss
    
    cons = TLS[1]._connections
    
    for con in cons:
        print(con[0].getID(), con[1].getID(), con[2])
        
    
    
    """
    # if guiOn: sumoBinary += "-gui" Need an options parser to add this, currently just setting gui to default
    sumoCommand = ("%s -n %s -r %s --step-length %.2f --tripinfo-output %s --remote-port %d --no-step-log" % \
                   (os.environ["SUMO_BINARY"], netFile_filepath, routeFile_filepath, stepLength, tripInfoOutput_filepath, traciPort))
    sumoProcess = subprocess.Popen(sumoCommand, shell=True, stdout=sys.stdout, stderr=sys.stderr)
    print("Launched process: %s" % sumoCommand)
    
    traci.init(traciPort) # Open up traci on a free port
    
    # initialise the step
    step = 0
    
    Tmin = 20
    Tmax = 100
    L_flat = [[1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0], [1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0], [1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0], [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1], [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1], [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1], [1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0], [1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0], [1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0], [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1], [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1], [0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1]]
    timer = minMaxGreenTimeController(Tmin, Tmax)
    queueControl = LmaxQueueController()
    queueControl.setL(L_flat)
    
    intersectionController_1 = intersectionController(12, L_flat, timer, queueControl)
    
    # run the simulation
    while step == 0 or traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        #timeNow = traci.simulation.getCurrentTime()
        
        intersectionController_1.update(stepLength)
              
        step += stepLength
    """