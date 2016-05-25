import numpy as np
import tools
from collections import defaultdict
import traci
import random

class intersectionController:
    def __init__(self, tls_id, num_queues, link_direction, incLanes, incLanes2indexDict, outLanes, outLanes2indexDict, phase_matrix_by_link_index, phase_matrix_by_lane, phase_strings,
                 x_star, greenTimeController, queueController):
        """ Class which controls the lights at each intersection. This class keps track of properties such as
        the time elapsed since the last phase. The algorithm for determining green times and queues will be defined
        elsewhere and called by this function, in order to make it easy to switch algorithms """
        # Static properties of the intersection
        self._id = tls_id
        self._num_queues = num_queues  # The number of queues at this intersection (i.e. the number of combinations of in queue and out queue)
        self._dir = link_direction # The turning direction of each queue

        self._incLanes = incLanes
        self._incLanes2index = incLanes2indexDict
        self._index2incLane = tools.reverseDict(incLanes2indexDict)

        self._outLanes = outLanes
        self._outLanes2index = outLanes2indexDict
        self._index2outLane = tools.reverseDict(outLanes2indexDict)

        self._phase_matrix_by_link_index = phase_matrix_by_link_index # Possible queue combinations for different phases
        self._phase_matrix_by_lane = phase_matrix_by_lane
        self._phase_strings = phase_strings # Strings representing the light settings of each phase

        # Current values for dynamic properties
        self._current_queue_index = 0
        self._current_open_queues = self._L[self._current_queue_index, 0:]
        self._current_open_indexes = np.nonzero(self._current_open_queues)[0]
        self._current_open_lanes = []
        self._current_phase_string = "".join(self._phase_strings[0])  # Initialise light settings as all red (will change in the first step of the simulation

        # Values to track important variables and state changes
        self._green_timer = greenTimeController.getInitialGreenTime()  # Elapsed time since last phase
        self._amber_timer = 0
        self._state = False  # True means green state, False means amber state
        self._queue_green_times = [greenTimeController.getInitialGreenTime()] * num_queues
        self._next_green_string = "".join(random.choice(self._phase_strings)) # Choose the initial green string at random

        self._queue_lengths_by_link_index = [0] * num_queues  #  The queue length for each index
        self._capacities_by_link_index = [999] * num_queues  # The capacity of the links each queue wishes to join

        # Algorithms used for picking queues and calculating green time
        self._timerControl = greenTimeController
        self._queueControl = queueController

        self._proportion_of_vehicles_to_remove = x_star # Proportion of vehicles to remove

        self._number_of_vehicles_to_remove_by_link_index = [0] * num_queues # Number of vehilces

        self._vehicles_to_remove_this_time_step_value_for_green_time_calculation = 0
        self._vehicles_removed_value_for_green_time_calculation = 0

        self._number_of_vehicles_to_remove_by_lane = defaultdict()

        self._vehicles_removed_from_lane_per_time_step = defaultdict(list)
        self._vehicles_entering_lane_per_time_step = defaultdict(list)

        self._vehs_at_start_of_step = defaultdict(list)
        self._vehs_at_end_of_step = defaultdict(list)

        # Model based controller values
        self._mu = defaultdict(list)
        self._lambda = defaultdict(list)

        # self._mu_G_minute = mus[0] # mean car exit rate
        # self._lambda_G_minute = lams[0] # mean car entry rate
        # self._mu_G_hour = mus[1]  # mean car exit rate
        # self._lambda_G_hour = lams[1]  # mean car entry rate
        # self._mu_G_year = mus[2]
        # self._lambda_G_year = lams[2]

        # Output
        self._OUTPUT_green_time_change_step = defaultdict(list)
        self._OUTPUT_green_time_setting = defaultdict(list)

        for lane in self.getIncLanes():
            self._mu.update({lane: 0})
            self._lambda.update({lane: 0})
            self._vehicles_removed_from_lane_per_time_step.update({lane: []})
            self._vehicles_entering_lane_per_time_step.update({lane: []})
            self._OUTPUT_green_time_change_step.update({lane: []})
            self._OUTPUT_green_time_setting.update({lane: []})

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
A : %s""" % (float(self._current_queue_index), self._current_open_queues, str(self._current_phase_string), self._green_timer,
             self._amber_timer, self._state, str(self._queue_green_times), self._next_green_string, self._queue_lengths_by_link_index, self._capacities_by_link_index,
             self._vehicles_to_remove_this_time_step_value_for_green_time_calculation, self._vehicles_removed_value_for_green_time_calculation, self._number_of_vehicles_to_remove_by_link_index))

    # Set property commands
    def set_queue_length_by_link_index(self, index, value):
        self._queue_lengths_by_link_index[index] = value

    def set_vehs_in_lane_at_start_of_step(self, lane, vehList):
        self._vehs_at_start_of_step[lane] = vehList

    def set_vehs_in_lane_at_end_of_step(self, lane, vehList):
        self._vehs_at_end_of_step[lane] = vehList

    def update_green_time_records_by_link_index(self, index, timeStep, Gt):
        lane = self._index2incLane[index]
        self._OUTPUT_green_time_change_step[lane].append(timeStep)
        self._OUTPUT_green_time_setting[lane].append(Gt)

    def setCongestedLanes2Red(self):
        for ii in range(self._num_queues):
            if self._capacities_by_link_index[ii] < 1:
                self._next_green_string = self.setQueue2Red(ii, self._next_green_string)

    def setQueue2Red(self, queue, TLstring):
        TLstring = list(TLstring)
        TLstring[queue] = 'r'
        return "".join(TLstring)

    def setQueue2Green(self, queue, TLstring):
        TLstring = list(TLstring)
        TLstring[queue] = 'g'
        return "".join(TLstring)

    # Main logic for updating the queues and the green time and sending it to SUMO

    def updateQueues(self):
        """Updates the length of the queues using traci"""
        # The length of each queue is just the number of vehicles in it.
        # Get the list of all lanes incoming into the junction
        # For every lane, measure the number of vehicles in the queue
        for lane in self.getIncLanes():
            queue_length = traci.lane.getLastStepVehicleNumber(lane)
            # For every linkIndex assigned to this lane, update link index as follows 'vehicles_in_lane / num_links'
            num_indexes_assigned_to_lane = len(self.getIndexesFromIncLane(lane))
            value = queue_length / num_indexes_assigned_to_lane
            # Input into matrix X
            for index in self.getIndexesFromIncLane(lane):
                self.set_queue_length_by_link_index(index, value)

    def updateCapacities(self):
        """Updates self._Cs with the capacity of the outgoing lanes"""
        for lane in self.getOutLanes2indexDict():
            vehLength = traci.lane.getLastStepLength(lane)
            if vehLength:
                gap = (2 * vehLength) / 3
                laneLength = traci.lane.getLength(lane)
                vehCount = traci.lane.getLastStepVehicleNumber(lane)
                spaces_total = int(laneLength / (vehLength + gap))
            else:
                laneLength = traci.lane.getLength(lane)
                vehCount = traci.lane.getLastStepVehicleNumber(lane)
                spaces_total = int(laneLength / (5 + (2 * 5) / 3))
            for index in self.getIndexesFromOutLane(lane):
                self._capacities_by_link_index[index] = spaces_total - vehCount

    def updateBcompare(self):
        global stepWindow4MuAndLambda
        """Updates the actual number of vehicles removed from the queue"""
        # Identify only individual vehicles removed from the queue, that were there at the start
        # Compare the vehicles at the start to the vehicles at the end

        # Initialise a counter

        for lane in self._current_open_lanes:
            # Get the final count (current vehicles in the lane)
            endCount = traci.lane.getLastStepVehicleIDs(lane)
            # Get the number of vehicles at the start of the green time
            startCount = self._vehList_k0[lane]

            # if a vehicle there at the start is not longer there, then increase the count by 1
            BperStep = 0
            lambda_per_step = 0
            for veh in startCount:
                if veh not in endCount:
                    self._vehicles_removed_value_for_green_time_calculation += 1
                    BperStep += 1

            for veh in endCount:
                if veh not in startCount:
                    lambda_per_step += 1

            self._vehicles_removed_from_lane_per_time_step[lane].append(BperStep)
            if len(self._vehicles_removed_from_lane_per_time_step[lane]) > stepWindow4MuAndLambda:
                alpha_mu = self._vehicles_removed_from_lane_per_time_step[lane].pop(0)
                omega_mu = BperStep
                self._mu[lane] = self._mu[lane] + ((omega_mu - alpha_mu) / stepWindow4MuAndLambda)
            else:
                self._mu[lane] = ((len(self._vehicles_removed_from_lane_per_time_step[lane]) - 1) / len(self._vehicles_removed_from_lane_per_time_step[lane])) * self._mu[lane] + (
                                                                                                                  1 / len(
                                                                                                                      self._vehicles_removed_from_lane_per_time_step[
                                                                                                                          lane])) * BperStep

            self._vehicles_entering_lane_per_time_step[lane].append(lambda_per_step)
            if len(self._vehicles_entering_lane_per_time_step) > stepWindow4MuAndLambda:
                alpha_lam = self._vehicles_entering_lane_per_time_step[lane].pop(0)
                omega_lam = lambda_per_step
                self._lambda[lane] = self._lambda[lane] + ((omega_lam - alpha_lam) / stepWindow4MuAndLambda)
            else:
                self._lambda[lane] = ((len(self._vehicles_entering_lane_per_time_step[lane]) - 1) / len(self._vehicles_entering_lane_per_time_step[lane])) * \
                                     self._lambda[lane] + 1 / len(self._vehicles_entering_lane_per_time_step[lane]) * lambda_per_step

            # Update the list of vehicles at the intersection to be compared next time.
            self._vehList_k0[lane] = endCount

    def resetB(self):
        self._vehicles_removed_value_for_green_time_calculation = 0

    def updateGreenTime(self, step):
        """Updates the green time for the current queue
        (which will be used next time the queue receives a green light) using the timer algorithm"""
        Gt_new = self._timerControl.getNewGreenTime(self)

        elements_to_update = self._L[self._current_queue_index][0:]

        for ii in range(0, len(elements_to_update)):
            if elements_to_update[ii] == 1: self._queue_green_times[ii] = Gt_new  # Apply Gt_new to all queues

        self.update_green_time_records_by_link_index(self._current_queue_index, step, Gt_new)

    def updateA(self):
        """Updates the target number of vehicles to remove from each queue"""
        # Set A by mapping the array of queues lengths (X) to a function that multiplies it by the fraction reduction
        self._number_of_vehicles_to_remove_by_link_index = (map(lambda x: x * self._proportion_of_vehicles_to_remove, self.getXs()))
        self._vehicles_to_remove_this_time_step_value_for_green_time_calculation = np.sum(np.multiply(self._number_of_vehicles_to_remove_by_link_index, self._current_open_queues))

    def chooseQueues2Release(self):
        self._current_queue_index = self._queueControl.bestQueueSet(self)
        self._current_open_queues = self._L[self._current_queue_index]
        self._current_open_indexes = np.nonzero(self._current_open_queues)[0]
        self._current_open_lanes = []
        for index in self._current_open_indexes:
            lane = self.getIncLaneFromIndex(index)
            if lane not in self._current_open_lanes: self._current_open_lanes.append(lane)

    def setGreenTimer(self):
        self._green_timer = self._queue_green_times[self._current_queue_index]

    def setGreenString(self):
        self._next_green_string = "".join(self._Lstrings[self._current_queue_index])

    def setAmberPhase(self, amberPhaseLength=5):
        """ Sets the intermediate phase between green times. Returns the phase duration and traffic light string. """
        amberPhase = []

        old_phase = list(self._current_phase_string)
        new_phase = list(self._next_green_string)

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

        self._amber_timer = amberPhaseLength
        self._current_phase_string = amberPhaseString

    def sendTLsettings2SUMO(self):
        traci.trafficlights.setRedYellowGreenState(self._id, self._current_phase_string)

    # Main update function
    def update(self, stepsize, step):
        # If the traffic light is in an amber phase and amber timer has reached zero. Go into the green phase.
        if not (self._state) and self._amber_timer <= 0:
            traci.trafficlights.setRedYellowGreenState(self._id, self._next_green_string)
            self._current_phase_string = self._next_green_string
            self._state = True

        # Else if in the amber phase but the amber timer has not reached zero, decrement the amber timer
        elif not (self._state) and self._amber_timer > 0:
            self._amber_timer -= stepsize
        # Else if the traffic light is in the green phase and the green timer has reached zero. Update all variables
        # and calculate the new green time and phase. Then switch into the amber phase.
        elif self._state and self._green_timer <= 0:
            # ORDER IS IMPORTANT IN THIS SECTION. DO NOT REORDER WITHOUT FULL UNDERSTANDING OF THE CHANGES TO OBJECT PROPERTIES.
            # Update the queue lengths at each link
            self.updateQueues()
            # Update the capacities of each exit lane
            self.updateCapacities()
            # Update the number of vehicles which were cleared during the last green phase
            self.updateBcompare()
            # Update the green time for the links used in the last phase
            self.updateGreenTime(step)
            # Update the time step when the phase was changed
            # self._updateGtRecords_greenTime()
            # self.updateGtRecords_changeStep(step)

            # Update the queues to be set to green in the next phase
            self.chooseQueues2Release()
            # Update the target number of vehicles to be removed during the next phase
            self.updateA()

            # Update the green timer according to the queues to be unlocked
            self.setGreenTimer()
            # Update the green string according to the queue
            self.setGreenString()

            # Set queues for which the outgoing lane is congested to red (discontinued due to poor performance)
            # self.setCongestedLanes2Red()   # Turned off the lane closing behaviour as it caused long queues at green lights

            # Set the amber phase according to the next green phase
            self.setAmberPhase(amberPhaseLength=5)

            # Transmit the settings to SUMO
            self.sendTLsettings2SUMO()

            # Set the state of the intersection to false, indicating the start of the amber phase
            self.resetB()
            self._state = False

        # Else if the traffic light is in a green phase and the green timer is not finished, decrement the green timer
        elif self._state and self._green_timer > 0:
            self._green_timer -= stepsize
            self.updateBcompare()
        # Catch all to check for logical errors
        else:
            print("Something wrong in update phase logic")

    def debug(self):
        pass
        # print(self._currentOpenLanes)
        # print(self._timerControl.getNewGreenTime(self))

    # Get functions
    def getXs(self):
        return self._queue_lengths_by_link_index

    def getAcompare(self):
        return self._vehicles_to_remove_this_time_step_value_for_green_time_calculation

    def getBcompare(self):
        return self._vehicles_removed_value_for_green_time_calculation

    def getGt_current(self):
        return self._queue_green_times[self._current_queue_index]

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

    def getIncLaneFromIndex(self, index):
        return self._index2incLane[index]

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

    def getLambda(self, lane):
        return self._lambda[lane]

    def getMu(self, lane):
        return self._mu[lane]

    def getCurrentOpenLanes(self):
        return self._current_open_lanes


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
            IC = intersectionController(ICid, numQueues, incLanes, incLanes2indexDict, outLanes, outLanes2indexDict, L,
                                        L_strings, x_star, timerControl, queueControl)
            ICs.update({ICid: IC})
        return ICs

    def updateICqueues(self, stepsize, step):
        for IC in self._ICs:
            self._ICs[IC].update(stepsize, step)