# -*- coding: utf-8 -*-
from __future__ import division, print_function
import os, sys
import matplotlib.pyplot as plt

os.environ["SUMO_HOME"] = "/sumo" # Home directory, stops SUMO using slow web lookups for XML files
os.environ["SUMO_BINARY"] = "/usr/local/bin/sumo-gui" # binary for SUMO simulations
 
def getOpenPort():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("",0))
    s.listen(1)
    port = s.getsockname()[1]
    s.close()
    return port

class minMaxGreenTimeController:
    
    def __init__(self, Tmin, Tmax, x_star):
        """ Controller that uses basic Tmin and Tmax to adjust green time """
        self._Tmin = Tmin
        self._Tmax = Tmax
        self._x_star = x_star
        self._initialGreenTime = (Tmax+Tmin)/2
        
    def getInitialGreenTime(self):
        """ Calculates the intial green time that all lights start with """
        return self._initialGreenTime
        
    def getNewGreenTime(self, X, B, timer):
        """ Takes the target number of vehicles to remove from the queue, compares withe the actual number. Returns updated green time."""
        A=x*self._x_star
        if B > A:
            Gt = (timer + self._Tmin)/2
            #print("Decreasing green time")
        elif B < A:
            Gt = (timer + self._Tmax)/2
            #print("Increasing Green Time")
        else:
            Gt = timer
            #print("No change in green time")  
        return Gt

class LmaxQueueController:
    
    def __init__(self):
        """ Controller that maximises the total number of vehicles released at each phase, regardless of fairness """
    
    def modifyLwithC(self, L, C):
        """ Modify the L matrix to exclude lanes that have nowhere to send traffic (i.e. target lanes are full) """
        
    
    def LdotX(self, ):
        """ Returns the dot product of matrix L and the vector x"""
        return np.dot(self._L, np.matrix(X).transpose())
    
    def bestQueueSet(self, IC):
        """Picks the best queue to release, based on total number of vehicles in non-conflicting queues"""
        L = IC._L
        LdotX = np.dot(L, IC._Xs)
        
        return int(round(np.random.rand()*10))

class intersectionController:
     
    def __init__(self, ICid, numQueues, incLanes, incLanes2indexDict, outLanes, outLanes2indexDict, L, L_strings, greenTimeController, queueController):
        """ Class which controls the lights at each intersection. This class keps track of properties such as
        the time elapsed since the last phase. The algorithm for determining green times and queues will be defined 
        elsewhere and called by this function, in order to make it easy to switch algorithms """
        # Static properties of the intersection
        self._id = ICid
        self._numQueues = numQueues # The number of queues at this intersection (i.e. the number of combinations of in queue and out queue)
        self._incLanes = incLanes
        self._incLanes2index = incLanes2indexDict
        self._outLanes = outLanes
        self._outLanes2index = outLanes2indexDict
        
        self._indexDependencyMatrix = []
        
        self._L = L
        self._Lstrings = L_strings
    
        # Algorithms used for picking queues and calculating green time
        self._timerControl = greenTimeController
        self._queueControl = queueController
        
        # Current values for dynamic properties
        self._currentQindex = 0
        self._currentPhaseString = "".join(self._Lstrings[0]) # Initialise light settings as all red (will change in the first step of the simulation
        
        # Values to track important variables and state changes
        self._greenTimer = greenTimeController.getInitialGreenTime() # Elapsed time since last phase
        self._amberTimer = 0
        self._state = False # True means green state, False means amber state
        self._queueGreenTimes = [greenTimeController.getInitialGreenTime()]*numQueues
        self._nextGreenString = "".join(self._Lstrings[0])
        
        self._Xs = [0]*numQueues # The queue size for each link
        self._Cs = [0]*numQueues # The capacity of the links each queue wishes to join
        
        self._Bs = [0]*numQueues
        self._vehList_k0 = {}
        self._vehList_kGt = {}
        
        for lane in self.getIncLanes():
            self._vehList_k0.update({lane:[]})
            self._vehList_kGt.update({lane:[]})
    
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
Queues : %s
A : %s
B : %s""" % (float(self._currentQindex), str(self._currentPhaseString), self._greenTimer, self._amberTimer, self._state, str(self._queueGreenTimes), self._nextGreenString, self._Xs, self._As, self._Bs))
    
    # Set property commands
    def setAmberPhase(self, old_phase_string, new_phase_string, amberPhaseLength = 3):
        """ Sets the intermediate phase between green times. Returns the phase duration and traffic light string. """
        amberPhase = []
        
        old_phase = list(old_phase_string)
        new_phase = list(new_phase_string)
        
        if old_phase == new_phase:
            amberPhaseLength = 1
            amberPhase = new_phase
        else:
            for ii in range(0, len(old_phase)):
                if old_phase[ii] == 'r' and new_phase[ii] == 'r':
                    amberPhase.append('r')
                elif old_phase[ii] == 'r' and (new_phase[ii] == 'g' or new_phase[ii] == 'G'):
                    amberPhase.append('r')
                elif (old_phase[ii] == 'g' or old_phase[ii] == 'G') and (new_phase[ii] == 'r'):
                    amberPhase.append('y')
                elif old_phase[ii] == 'G' and new_phase[ii] == 'G':
                    amberPhase.append('G')
                else:
                    print("Something wrong in amber phase logic. Old: %s, New: %s" % (old_phase[ii], new_phase[ii]))
                    
        amberPhaseString = "".join(amberPhase)
             
        self._amberTimer = amberPhaseLength
        self._currentPhaseString = amberPhaseString

    def setXval(self,index,value):
        self._Xs[index] = value
        
    def setBval(self, index, value):
        self._Bs[index] = value
    
    def setVehList_k0_val(self, lane, vehList):
        try:
            self._vehList_k0[lane] = vehList
        except ValueError:
            self._vehList_k0.update({lane:vehList})
    
    def setVehList_kGt_val(self, lane, vehList):   
        try:
            self._vehList_kGt[lane] = vehList
        except ValueError:
            self._vehList_kGt.update({lane:vehList})     
    
    def updateGreenTime(self):
        """Updates the green time for the current queue (which will be used next time the queue receives a green light) using the timer algorithm"""
        Gt_current = self._queueGreenTimes[self._currentQindex]
        X_current = self._Xs[self._currentQindex]
        B_current = self._Bs[self._currentQindex]
        
        Gt_new = self._timerControl.getNewGreenTime(X_current, B_current, Gt_current)
        
        elements_to_update = self._L[self._currentQindex][:]
        
        for ii in range(0,len(elements_to_update)):
            if elements_to_update[ii] == 1 : self._queueGreenTimes[ii] = Gt_new # Apply Gt_new to all queues
        
        #print(self._id, self._queueGreenTimes)
              
    def updateQueues(self):
        """Updates the length of the queues using traci"""
        # The length of each queue is just the number of vehicles in it.
        # Get the list of all lanes incoming into the junction
        # For every lane, measure the number of vehicles in the queue
        for lane in self.getIncLanes():
            Qlength = traci.lane.getLastStepVehicleNumber(lane)
            # For every linkIndex assigned to this lane, update link index as follows 'vehicles_in_lane / num_links'
            num_indexes_assigned_to_lane = len(self.getIndexesFromLane(lane))
            value = Qlength/num_indexes_assigned_to_lane
            # Input into matrix X
            for index in self.getIndexesFromLane(lane):
                self.setXval(index,value)
                
    def updateCapacities(self):
        """Updates self._Cs with the capacity of the outgoing lanes"""
        print("Out Lanes", self.getOutLanes())
        for lane in self.getOutLanes():
            print(traci.lane.getLastStepOccupancy(lane))
            
    def updateA(self, fractionReduction):
        """Updates the target number of vehicles to remove from each queue"""
        # Set A by mapping the array of queues lengths (X) to a function that multiplies it by the fraction reduction
        self._As = (map(lambda x : x*fractionReduction, self.getXs()))
    
    def updateB(self):
        """Updates the actual number of vehicles removed from the queue"""
        # Identify only individual vehicles removed from the queue, that were there at the start
        # Compare the vehicles at the start to the vehicles at the end
        
        for lane in self.getIncLanes():
            # Get the list of IDs
            vehIDs = traci.lane.getLastStepVehicleIDs(lane)
            # Store them in the dictionary
            self.setVehList_kGt_val(lane, vehIDs)
            
            indexes = self.getIndexesFromLane(lane)
            num_indexes_assigned_to_lane = len(indexes)
            
            startCount = self.getVehList_kGt_forLane(lane)
            endCount = self.getVehList_k0_forLane(lane)
            
            vehCount = 0
            
            for element in startCount:
                if element not in endCount : vehCount += 1
                
            value = vehCount/num_indexes_assigned_to_lane
            
            for index in indexes:
                self.setBval(index, value)
                
            self.setVehList_k0_val(lane, vehIDs)
        
    def update(self, stepsize):
        if not(self._state) and self._amberTimer <= 0:
            traci.trafficlights.setRedYellowGreenState(self._id, self._nextGreenString)
            self._currentPhaseString = self._nextGreenString
            self._state = True
            print("%s: Amber Phase Over. Switching to %s" % (self._id, self._nextGreenString))
        elif not(self._state) and self._amberTimer > 0:
            self._amberTimer -= stepsize
        elif self._state and self._greenTimer <= 0:
                self.updateQueues()
                self.updateCapacities()
                self.updateB()
                self.updateGreenTime()
                newQindex = self.chooseQueues()
                self._greenTimer = self._queueGreenTimes[newQindex]
                self._nextGreenString = self.getQstring(newQindex)
                self.setAmberPhase(self._currentPhaseString, self._nextGreenString, amberPhaseLength=5)
                traci.trafficlights.setRedYellowGreenState(self._id, self._currentPhaseString)
                self._currentQindex = newQindex
                self._state = False
                print("%s: Green Phase Over. Switching to %s" % (self._id, self._currentPhaseString))
        elif self._state and self._greenTimer > 0:
            self._greenTimer -= stepsize
        else:
            print("Something wrong in update phase logic")
            
    def chooseQueues(self):
        return self._queueControl.bestQueueSet(self)
    
    def getQstring(self, Qindex):
        string_array = self._Lstrings[Qindex]
        return "".join(string_array)
    
    def getXs(self):
        return self._Xs
    
    def getIncLanes(self):
        return self._incLanes
    
    def getIncLanes2indexDict(self):
        return self._incLanes2index
    
    def getOutLanes(self):
        return self._outLanes
    
    def getOutLanes2indexDict(self):
        return self._outLanes2index
    
    def getIndexesFromLane(self, lane):
        return self._incLanes2index[lane]
    
    def getVehList_k0(self):
        return self._vehList_k0
    
    def getVehList_k0_forLane(self, lane):
        return self._vehList_k0[lane]
        
    def getVehList_kGt(self):
        return self._vehList_kGt
    
    def getVehList_kGt_forLane(self, lane):
        return self._vehList_kGt[lane]
        
class intersectionControllerContainer:
    
    def __init__(self, TLJuncsContainer, timerControl, queueControl):
        self._TLJuncs = TLJuncsContainer._TLjunctions
        self._ICs = self.addIntersectionControllers(timerControl, queueControl)
        
    def addIntersectionControllers(self, timerControl, queueControl):
        ICs = {}
        for junction in self._TLJuncs:
            ICid = junction
            numQueues = self._TLJuncs[junction].getNumQueues()
            L, L_strings = self._TLJuncs[junction].findCompatibleFlows_variablePriorityModel()
            incLanes = self._TLJuncs[junction].getIncLanes()
            outLanes = self._TLJuncs[junction].getOutLanes()
            incLanes2indexDict = self._TLJuncs[junction].getIncLane2IndexesDict()
            outLanes2indexDict = self._TLJuncs[junction].getOutLane2IndexesDict()
            IC = intersectionController(ICid, numQueues, incLanes, incLanes2indexDict, outLanes, outLanes2indexDict, L, L_strings, timerControl, queueControl)
            ICs.update({ICid:IC})
        return ICs
    
    def updateICqueues(self, stepsize):
        for IC in self._ICs:
            self._ICs[IC].update(stepsize)

def async_plot(fig_1,x,y):
    pass

if __name__ == "__main__":
    
    import socket # to execute the getOpenPort() function
    import traci # SUMO API
    import subprocess
    import numpy as np
    from sumolib import net
    import generateL
    from multiprocessing import cpu_count, Pool
    import time
    
    # Input arguments
    netFile_filepath = "netFiles/grid.net.xml" #sys.argv[1]
    routeFile_filepath = "netFiles/grid.rou.xml" #sys.argv[2]
    stepLength = 0.1
    tripInfoOutput_filepath = "tripsoutput.xml"
    traciPort = getOpenPort()
    
    # if guiOn: sumoBinary += "-gui" Need an options parser to add this, currently just setting gui to default
    sumoCommand = ("%s -n %s -r %s --step-length %.2f --tripinfo-output %s --remote-port %d --no-step-log" % \
                   (os.environ["SUMO_BINARY"], netFile_filepath, routeFile_filepath, stepLength, tripInfoOutput_filepath, traciPort))
    sumoProcess = subprocess.Popen(sumoCommand, shell=True, stdout=sys.stdout, stderr=sys.stderr)
    print("Launched process: %s" % sumoCommand)
    
    # Open up traci on a free port
    traci.init(traciPort)
    
    # initialise the step
    step = 0
    target_frac = 1 
    Tmin = 30
    Tmax = 90
    timer = minMaxGreenTimeController(Tmin, Tmax, target_frac)
    
    queueControl = LmaxQueueController()

    TLnet = generateL.getTLobjects(netFile_filepath)
    ICcontainer = intersectionControllerContainer(TLnet, timer, queueControl)

    y = [[0] for x in range(4)]
    print(y)
    z = [[0] for x in range(4)]
    print(z)
    linkIndex = [0,3,6,9]
    
    # run the simulation
    while step == 0 or traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        ICcontainer.updateICqueues(stepLength)
        
        step += stepLength
    
