# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np
import random

class MinMaxGreenTimeController:
    
    def __init__(self, Tmin, Tmax):
        """ Controller that uses basic Tmin and Tmax to adjust green time """
        self._Tmin = Tmin
        self._Tmax = Tmax
        self._initialGreenTime = (Tmax+Tmin)/2
        
    def __repr__(self):
        return "Min/Max Time Controller: Controller that uses basic Tmin and Tmax to adjust green time "
        
    def get_initial_green_time(self):
        """ Calculates the intial green time that all lights start with """
        return self._initialGreenTime
        
    def get_new_green_time(self, IC):
        """ Takes the target number of vehicles to remove from the queue, compares withe the actual number. Returns updated green time."""

        target_number_cars_cleared = IC.get_a_compare()
        actual_number_cars_cleared = IC.get_b_compare()
        Gt_old = IC.get_current_green_time()

        # Increase green time if too few cars cleared. Decrease green time if too many cars cleared.
        if actual_number_cars_cleared > target_number_cars_cleared:
            Gt_new = (Gt_old + self._Tmin)/2
        elif actual_number_cars_cleared < target_number_cars_cleared:
            Gt_new = (Gt_old + self._Tmax)/2
        else:
            Gt_new = Gt_old

        return Gt_new
    
class PGreenTimeController:
    
    def __init__(self, K, G0):
        """ Controller that uses an estimated error in the green to make adjustments """
        self._K = K
        self._initialGreenTime = G0
        
    def __repr__(self):
        return "Proportional Green Time Controller: Controller that uses an estimated error in the green to make adjustments"
        
    def get_initial_green_time(self):
        """ Calculates the intial green time that all lights start with """
        return self._initialGreenTime
        
    def get_new_green_time(self, IC):
        """ Takes the target number of vehicles to remove from the queue, compares withe the actual number. Returns updated green time."""

        target_number_cars_cleared = IC.get_a_compare()
        actual_number_cars_cleared = IC.get_b_compare()
        Gt_old = IC.get_current_green_time()

        if target_number_cars_cleared:
            error = ((target_number_cars_cleared-actual_number_cars_cleared)/target_number_cars_cleared)*Gt_old
        else:
            error = 0

        return Gt_old + self._K*error

class ModelBasedGreenTimeController:

    def __init__(self, Tmin, Tmax):
        self._Tmin = Tmin
        self._Tmax = Tmax
        self._initialGreenTime = (Tmax+Tmin)/2

    def __repr__(self):
        return "Uses the model of traffic in flow to calculate green time. Staturates if the values exceed certain limits"

    def get_initial_green_time(self):
        return self._initialGreenTime

    def get_new_green_time(self, IC):

        open_lanes = IC.get_current_open_lanes()
        target_number_cars_cleared = IC.get_a_compare()

        max_green_time = self._Tmin
        mu = 0
        lam = 0

        for lane in open_lanes:
            mu += IC.get_mu(lane)
            lam += IC.get_lambda(lane)

        if target_number_cars_cleared > 0:
            try:
                modelBased_Gt = target_number_cars_cleared/(mu-lam)
            except ZeroDivisionError:
                max_green_time = self._Tmax
        else:
            modelBased_Gt = self._Tmin

        if modelBased_Gt > self._Tmax : modelBased_Gt = self._Tmax

        if max_green_time < modelBased_Gt : max_green_time = modelBased_Gt

        return max_green_time

class LmaxQueueController:
    
    def __init__(self):
        """ Controller that maximises the total number of vehicles released at each phase, regardless of fairness """
    
    def best_queue_set(self, intersection_controller):
        """Picks the best queue to release, based on total number of vehicles in non-conflicting queues"""
        phases = intersection_controller.get_phase_matrix_by_link_index()
        queues = intersection_controller.get_queues()

        phases_queues_dot_product = np.dot(phases, queues)
        best_choices = np.nonzero(phases_queues_dot_product == np.amax(phases_queues_dot_product))[0]

        return random.choice(best_choices)

class CongestionAwareLmaxQueueController:
    
    def __init__(self):
        pass
    
    def discount_congested_queues(self, phases, capacities):
        """Sets phase entry to 0 for queues which have nowhere to go"""
        for row, phase in enumerate(phases):
            for col, entry in enumerate(phase):
                if capacities[col] < 1 :
                    phases[row][col] = 0
        return phases
                 
    def best_queue_set(self, intersection_controller):
        """Picks the best queue to release, based on total number of vehicles in non-conflicting queues"""
        phase_benefit = []
        phases = intersection_controller.get_phase_matrix_by_link_index()

        queues = intersection_controller.get_queues()
        capacities = intersection_controller.get_capacities()

        phases_discounted = self.discount_congested_queues(phases, capacities)
        discounted_phases__queues_dot_product = np.dot(phases_discounted, queues)

        best_choices = np.nonzero(discounted_phases__queues_dot_product == np.amax(discounted_phases__queues_dot_product))[0]

        return random.choice(best_choices)

class CongestionDemandOptimisingQueueController:

    def __repr__(self):
        return """Uses a matrix of the outgoing lanes for each queue, and knowledge of congestion, in order to further
        optimise the queues to unlock"""

    def get_L_matrix_from_phase(self, phase):
        L = []
        for row_index, row_status in enumerate(phase):
            L.append([])
            for other_index, col_status in enumerate(phase):
                if row_status and col_status:
                    L[row_index].append(1)
                else:
                    L[row_index].append(0)

        return L

    def bounded_demand(self, x_tilda, capacity_vec):
        return [min(val) for val in zip(x_tilda,capacity_vec)]

    def demand_per_queue(self, combined_out_flows_and_L_matrix, x_bounded):

        return [(queue_bounded/np.sum(combined_out_flows_and_L_matrix[ii][:])) if np.sum(combined_out_flows_and_L_matrix[ii][:]) != 0 else 0 for ii, queue_bounded in enumerate(x_bounded)]

    def best_queue_set(self, intersection_controller):

        phase_benefit = []
        phases = intersection_controller.get_phase_matrix_by_link_index()
        receiving_lanes_index = [[1 if ii in intersection_controller.get_indicies_of_outgoing_lane(intersection_controller.get_outgoing_lane_from_index(jj))
                                else 0
                                for ii in range(intersection_controller.get_num_queues())]
                                for jj in range(intersection_controller.get_num_queues())]

        queues = intersection_controller.get_queues()
        capacities = intersection_controller.get_capacities()

        for phase in phases:
            L = self.get_L_matrix_from_phase(phase)
            combined_out_flows_and_L_matrix = np.multiply(receiving_lanes_index, L)
            x_tilda = np.dot(combined_out_flows_and_L_matrix, queues)

            x_bounded = self.bounded_demand(x_tilda, capacities)
            x_bounded_per_queue = self.demand_per_queue(combined_out_flows_and_L_matrix, x_bounded)

            total_demand_bounded_by_capacity = np.dot(phase, x_bounded_per_queue)

            phase_benefit.append(total_demand_bounded_by_capacity)

        best_choices = np.nonzero(phase_benefit == np.amax(phase_benefit))[0]

        return random.choice(best_choices)


