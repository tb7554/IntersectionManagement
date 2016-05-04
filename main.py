# -*- coding: utf-8 -*-
from __future__ import division, print_function
import os, sys, subprocess
import matplotlib.pyplot as plt
import plotting as tbplot
import tools
import traci  # SUMO API
import generateL as genL
import controllers as ctrl
import numpy as np

os.environ["SUMO_HOME"] = "/sumo" # Home directory, stops SUMO using slow web lookups for XML files
os.environ["SUMO_BINARY"] = "/usr/local/bin/sumo" # binary for SUMO simulations

class intersectionController:
     
    def __init__(self, ICid, numQueues, incLanes, incLanes2indexDict, outLanes, outLanes2indexDict, L, L_strings, x_star, greenTimeController, queueController):
        """ Class which controls the lights at each intersection. This class keps track of properties such as
        the time elapsed since the last phase. The algorithm for determining green times and queues will be defined 
        elsewhere and called by this function, in order to make it easy to switch algorithms """
        # Static properties of the intersection
        self._id = ICid
        self._numQueues = numQueues # The number of queues at this intersection (i.e. the number of combinations of in queue and out queue)
        self._incLanes = incLanes
        self._incLanes2index = incLanes2indexDict
        self._index2incLane = tools.reverseDict(incLanes2indexDict)
        self._outLanes = outLanes
        self._outLanes2index = outLanes2indexDict
        self._index2outLane = tools.reverseDict(outLanes2indexDict)
        
        self._indexDependencyMatrix = []
        
        self._L = L
        self._Lstrings = L_strings
    
        # Algorithms used for picking queues and calculating green time
        self._timerControl = greenTimeController
        self._queueControl = queueController
        
        # Current values for dynamic properties
        self._currentQindex = 0
        self._currentOpenQueues = self._L[self._currentQindex,0:]
        self._currentPhaseString = "".join(self._Lstrings[0]) # Initialise light settings as all red (will change in the first step of the simulation
        
        # Values to track important variables and state changes
        self._greenTimer = greenTimeController.getInitialGreenTime() # Elapsed time since last phase
        self._amberTimer = 0
        self._state = False # True means green state, False means amber state
        self._queueGreenTimes = [greenTimeController.getInitialGreenTime()]*numQueues
        self._nextGreenString = "".join(self._Lstrings[0])
        
        self._Xs = [0]*numQueues # The queue size for each link
        self._Cs = [999]*numQueues # The capacity of the links each queue wishes to join
        
        self._x_star = x_star
        self._As = [0]*numQueues
        self._Acompare = 0
        self._Bcompare = 0
        
        self._vehList_k0 = {}
        self._vehList_kGt = {}
        self._Gt_changeStep = {}
        self._Gt_historical = {}
        
        for lane in self.getIncLanes():
            self._vehList_k0.update({lane:[]})
            self._Gt_changeStep.update({lane:[]})
            self._Gt_historical.update({lane:[]})
    
    def __repr__(self):
        """String representation to see what is happening in the class"""
        return ("""
Current Q Index: %.2f
Current open queues: %s
Current Phase String: %s
Curent Green Timer : %.1f
Current Amber Timer: %.1f
Current State : %s
Current Green Times : %s
Next Green String : %s
Queues : %s
C : %s
Acompare: %f
Bcompare: %f
A : %s""" % (float(self._currentQindex), self._currentOpenQueues, str(self._currentPhaseString), self._greenTimer, self._amberTimer, self._state, str(self._queueGreenTimes), self._nextGreenString, self._Xs, self._Cs, self._Acompare, self._Bcompare, self._As))
    
    # Set property commands
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

    def updateGtRecords(self, index, timeStep, Gt):
        lanes = self._index2incLane[index]
        for lane in lanes:
            self._Gt_changeStep[lane].append(timeStep)
            self._Gt_historical[lane].append(Gt)
    
    def setCongestedLanes2Red(self):
        for ii in range(self._numQueues):
            if self._Cs[ii] < 1:
                self._nextGreenString = self.setQueue2Red(ii, self._nextGreenString)
        
    def setQueue2Red(self, queue, TLstring):
        TLstring = list(TLstring)
        TLstring[queue] = 'r'
        return "".join(TLstring)
        
    def setQueue2Green(self, queue, TLstring):
        TLstring = list(TLstring)
        TLstring[queue] = 'g'
        return "".join(TLstring)

    #Main logic for updating the queues and the green time and sending it to SUMO

    def updateQueues(self):
        """Updates the length of the queues using traci"""
        # The length of each queue is just the number of vehicles in it.
        # Get the list of all lanes incoming into the junction
        # For every lane, measure the number of vehicles in the queue
        for lane in self.getIncLanes():
            Qlength = traci.lane.getLastStepVehicleNumber(lane)
            # For every linkIndex assigned to this lane, update link index as follows 'vehicles_in_lane / num_links'
            num_indexes_assigned_to_lane = len(self.getIndexesFromIncLane(lane))
            value = Qlength/num_indexes_assigned_to_lane
            # Input into matrix X
            for index in self.getIndexesFromIncLane(lane):
                self.setXval(index,value)
                
    def updateCapacities(self):
        """Updates self._Cs with the capacity of the outgoing lanes"""
        for lane in self.getOutLanes2indexDict():
            vehLength = traci.lane.getLastStepLength(lane)
            if vehLength:
                gap = (2*vehLength)/3
                laneLength = traci.lane.getLength(lane)
                vehCount = traci.lane.getLastStepVehicleNumber(lane)
                spaces_total = int(laneLength/(vehLength + gap))
            else:
                laneLength = traci.lane.getLength(lane)
                vehCount = traci.lane.getLastStepVehicleNumber(lane)
                spaces_total = int(laneLength/(5 + (2*5)/3))
            for index in self.getIndexesFromOutLane(lane):
                self._Cs[index] = spaces_total - vehCount

    def updateBcompare(self):
        """Updates the actual number of vehicles removed from the queue"""
        # Identify only individual vehicles removed from the queue, that were there at the start
        # Compare the vehicles at the start to the vehicles at the end

        # Initialise a counter

        for lane in self.getIncLanes():
            # Get the indexes assigned to the current lane
            indexes = self.getIndexesFromIncLane(lane)
            # Get the number of indexes
            num_indexes_assigned_to_lane = len(indexes)

            # Get the final count (current vehicles in the lane)
            endCount = traci.lane.getLastStepVehicleIDs(lane)
            # Get the number of vehicles at the start of the green time
            startCount = self._vehList_k0[lane]

            # if a vehicle there at the start is not longer there, then increase the count by 1
            for veh in startCount:
                if veh not in endCount : self._Bcompare += 1

            # Update the list of vehicles at the intersection to be compared next time.
            self._vehList_k0[lane] = endCount

    def resetB(self):
        self._Bcompare = 0

    def updateGreenTime(self, step):
        """Updates the green time for the current queue
        (which will be used next time the queue receives a green light) using the timer algorithm"""
        Gt_current = self._queueGreenTimes[self._currentQindex]
        Gt_new = self._timerControl.getNewGreenTime(self._Acompare, self._Bcompare, Gt_current)

        elements_to_update = self._L[self._currentQindex][0:]

        for ii in range(0, len(elements_to_update)):
            if elements_to_update[ii] == 1 : self._queueGreenTimes[ii] = Gt_new  # Apply Gt_new to all queues

        self.updateGtRecords(self._currentQindex, step, Gt_new)

    def updateA(self):
        """Updates the target number of vehicles to remove from each queue"""
        # Set A by mapping the array of queues lengths (X) to a function that multiplies it by the fraction reduction
        self._As = (map(lambda x : x*self._x_star, self.getXs()))
        self._Acompare = np.sum(np.multiply(self._As, self._currentOpenQueues))


    def chooseQueues2Release(self):
        self._currentQindex = self._queueControl.bestQueueSet(self)
        self._currentOpenQueues = self._L[self._currentQindex]

    def setGreenTimer(self):
        self._greenTimer = self._queueGreenTimes[self._currentQindex]

    def setGreenString(self):
        self._nextGreenString = "".join(self._Lstrings[self._currentQindex])

    def setAmberPhase(self, amberPhaseLength = 5):
        """ Sets the intermediate phase between green times. Returns the phase duration and traffic light string. """
        amberPhase = []

        old_phase = list(self._currentPhaseString)
        new_phase = list(self._nextGreenString)

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
                elif (old_phase[ii] == 'g') and (new_phase[ii] == 'g'):
                    amberPhase.append('g')
                elif old_phase[ii] == 'G' and new_phase[ii] == 'G':
                    amberPhase.append('G')
                else:
                    print("Something wrong in amber phase logic. Old: %s, New: %s" % (old_phase[ii], new_phase[ii]))

        amberPhaseString = "".join(amberPhase)

        self._amberTimer = amberPhaseLength
        self._currentPhaseString = amberPhaseString

    def sendTLsettings2SUMO(self):
        traci.trafficlights.setRedYellowGreenState(self._id, self._currentPhaseString)

    # Main update function
    def update(self, stepsize, step):
        # If the traffic light is in an amber phase and amber timer has reached zero. Go into the green phase.
        if not(self._state) and self._amberTimer <= 0:
            traci.trafficlights.setRedYellowGreenState(self._id, self._nextGreenString)
            self._currentPhaseString = self._nextGreenString
            self._state = True

        # Else if in the amber phase but the amber timer has not reached zero, decrement the amber timer
        elif not(self._state) and self._amberTimer > 0:
            self._amberTimer -= stepsize
        # Else if the traffic light is in the green phase and the green timer has reached zero. Update all variables
        # and calculate the new green time and phase. Then switch into the amber phase.
        elif self._state and self._greenTimer <= 0:
                # Update the queue lengths at each link
                self.updateQueues()
                # Update the capacities of each exit lane
                self.updateCapacities()
                # Update the number of vehicles which were cleared during the last green phase
                self.updateBcompare()
                # Update the green time for the links used in the last phase
                self.updateGreenTime(step)
                # Update the time step when the phase was changed
                #self._updateGtRecords_greenTime()
                #self.updateGtRecords_changeStep(step)

                # Update the queues to be set to green in the next phase
                self.chooseQueues2Release()
                # Update the target number of vehicles to be removed during the next phase
                self.updateA()

                # Update the green timer according to the queues to be unlocked
                self.setGreenTimer()

                # Update the green string according to the queue
                self.setGreenString()

                # Set queues for which the outgoing lane is congested to red (discontinued due to poor performance)
                #self.setCongestedLanes2Red()   # Turned off the lane closing behaviour as it caused long queues at green lights

                # Set the amber phase according to the next green phase
                self.setAmberPhase(amberPhaseLength=5)

                # Transmit the settings to SUMO
                self.sendTLsettings2SUMO()

                # Set the state of the intersection to false, indicating the start of the amber phase
                self.resetB()
                self._state = False

        # Else if the traffic light is in a green phase and the green timer is not finished, decrement the green timer
        elif self._state and self._greenTimer > 0:
            self._greenTimer -= stepsize
            self.updateBcompare()
        # Catch all to check for logical errors
        else:
            print("Something wrong in update phase logic")

    def debug(self):
        print(self.__repr__)

    # Get functions
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
    
    def getIndexesFromIncLane(self, lane):
        return self._incLanes2index[lane]

    def getIndexesFromOutLane(self, lane):
        return self._outLanes2index[lane]
    
    def getVehList_k0(self):
        return self._vehList_k0
    
    def getVehList_k0_forLane(self, lane):
        return self._vehList_k0[lane]
        
    def getVehList_kGt(self):
        return self._vehList_kGt
    
    def getVehList_kGt_forLane(self, lane):
        return self._vehList_kGt[lane]
        
class intersectionControllerContainer:
    
    def __init__(self, TLJuncsContainer, x_star, timerControl, queueControl):
        self._TLJuncs = TLJuncsContainer._TLjunctions
        self._ICs = self.addIntersectionControllers(x_star, timerControl, queueControl)
        
    def addIntersectionControllers(self, x_star, timerControl, queueControl):
        ICs = {}
        for junction in self._TLJuncs:
            ICid = junction
            numQueues = self._TLJuncs[junction].getNumQueues()
            L, L_strings = self._TLJuncs[junction].findCompatibleFlows_variablePriorityModel()
            incLanes = self._TLJuncs[junction].getIncLanes()
            outLanes = self._TLJuncs[junction].getOutLanes()
            incLanes2indexDict = self._TLJuncs[junction].getIncLane2IndexesDict()
            outLanes2indexDict = self._TLJuncs[junction].getOutLane2IndexesDict()
            IC = intersectionController(ICid, numQueues, incLanes, incLanes2indexDict, outLanes, outLanes2indexDict, L, L_strings, x_star, timerControl, queueControl)
            ICs.update({ICid:IC})
        return ICs

    def updateICqueues(self, stepsize, step):
        for IC in self._ICs:
            self._ICs[IC].update(stepsize, step)

def async_plot(fig_1,x,y):
    pass

if __name__ == "__main__":
    
    if "-gui" in sys.argv:
        os.environ["SUMO_BINARY"] = "/usr/local/bin/sumo-gui" 
    
    # Input arguments
    netFile_filepath = "netFiles/grid.net.xml" #sys.argv[1]
    routeFile_filepath = "netFiles/grid.rou.xml" #sys.argv[2]
    stepLength = 0.1
    tripInfoOutput_filepath = "tripsoutput.xml"

    traciPort = tools.getOpenPort()

    target_frac = 1
    Tmin = 5
    Tmax = 120
    timer = ctrl.minMaxGreenTimeController(Tmin, Tmax, target_frac)
    # timer = ctrl.PGreenTimeController(0.1, 40, target_frac)
    print(timer)

    queueControl = ctrl.CongestionAwareLmaxQueueController()

    TLnet = genL.getTLobjects(netFile_filepath)
    ICcontainer = intersectionControllerContainer(TLnet, target_frac, timer, queueControl)

    # if guiOn: sumoBinary += "-gui" Need an options parser to add this, currently just setting gui to default
    sumoCommand = ("%s -n %s -r %s --step-length %.2f --tripinfo-output %s --remote-port %d --no-step-log --time-to-teleport -1" % \
                   (os.environ["SUMO_BINARY"], netFile_filepath, routeFile_filepath, stepLength, tripInfoOutput_filepath, traciPort))
    sumoProcess = subprocess.Popen(sumoCommand, shell=True, stdout=sys.stdout, stderr=sys.stderr)
    print("Launched process: %s" % sumoCommand)
    
    # Open up traci on a free port
    traci.init(traciPort)
    
    # initialise the step
    step = 0

    # run the simulation
    while step == 0 or traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        ICcontainer.updateICqueues(stepLength, step)

        #ICcontainer._ICs['0/0'].debug()

        step += stepLength
    
    traci.close()
    sys.stdout.flush()
    
    sumoProcess.wait()
    
    print(tbplot.meanWaitSteps(tripInfoOutput_filepath, stepLength))
    print(tbplot.meanDepartDelay(tripInfoOutput_filepath))


    for junction in ICcontainer._ICs:
        plt.figure(junction)
        for lane in ICcontainer._ICs[junction]._Gt_changeStep:
            if ICcontainer._ICs[junction]._Gt_changeStep[lane]:
                plt.plot(ICcontainer._ICs[junction]._Gt_changeStep[lane], ICcontainer._ICs[junction]._Gt_historical[lane], hold=True)

    plt.show()