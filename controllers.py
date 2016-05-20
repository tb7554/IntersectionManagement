# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np
import random

class minMaxGreenTimeController:
    
    def __init__(self, Tmin, Tmax):
        """ Controller that uses basic Tmin and Tmax to adjust green time """
        self._Tmin = Tmin
        self._Tmax = Tmax
        self._initialGreenTime = (Tmax+Tmin)/2
        
    def __repr__(self):
        return "Min/Max Time Controller: Controller that uses basic Tmin and Tmax to adjust green time "
        
    def getInitialGreenTime(self):
        """ Calculates the intial green time that all lights start with """
        return self._initialGreenTime
        
    def getNewGreenTime(self, IC):
        """ Takes the target number of vehicles to remove from the queue, compares withe the actual number. Returns updated green time."""

        target_number_cars_cleared = IC.getAcompare()
        actual_number_cars_cleared = IC.getBcompare()
        Gt_old = IC.getGt_current()

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
        
    def getInitialGreenTime(self):
        """ Calculates the intial green time that all lights start with """
        return self._initialGreenTime
        
    def getNewGreenTime(self, IC):
        """ Takes the target number of vehicles to remove from the queue, compares withe the actual number. Returns updated green time."""

        target_number_cars_cleared = IC.getAcompare()
        actual_number_cars_cleared = IC.getBcompare()
        Gt_old = IC.getGt_current()

        if target_number_cars_cleared:
            error = ((target_number_cars_cleared-actual_number_cars_cleared)/target_number_cars_cleared)*Gt_old
        else:
            error = 0

        return Gt_old + self._K*error

class modelBasedController:

    def __init__(self, Tmin, Tmax):
        self._Tmin = Tmin
        self._Tmax = Tmax
        self._initialGreenTime = (Tmax+Tmin)/2

    def __repr__(self):
        return "Uses the model of traffic in flow to calculate green time. Staturates if the values exceed certain limits"

    def getInitialGreenTime(self):
        return self._initialGreenTime

    def getNewGreenTime(self, IC):

        openLanes = IC.getCurrentOpenLanes()
        target_number_cars_cleared = IC.getAcompare()

        maxGreenTime = self._Tmin
        mu = 0
        lam = 0

        for lane in openLanes:
            mu += IC.getMu(lane)
            lam += IC.getLambda(lane)

        if target_number_cars_cleared > 0:
            try:
                modelBased_Gt = target_number_cars_cleared/(mu-lam)
            except ZeroDivisionError:
                maxGreenTime = self._Tmax
        else:
            modelBased_Gt = self._Tmin

        if modelBased_Gt > self._Tmax : modelBased_Gt = self._Tmax

        if maxGreenTime < modelBased_Gt : maxGreenTime = modelBased_Gt

        if "1/1to1/0_0" in openLanes:
            print(openLanes, target_number_cars_cleared, mu, lam, modelBased_Gt, maxGreenTime)

        return maxGreenTime


class LmaxQueueController:
    
    def __init__(self):
        """ Controller that maximises the total number of vehicles released at each phase, regardless of fairness """
    
    def bestQueueSet(self, IC):
        """Picks the best queue to release, based on total number of vehicles in non-conflicting queues"""
        LdotX = np.dot(IC._L, IC._Xs)
        Qmax = np.argmax(LdotX)
        return Qmax

class CongestionAwareLmaxQueueController:
    
    def __init__(self):
        pass
    
    def discountCongestedQueues(self, L, Cs):
        """Sets L to 0 for queues which have nowhere to go"""
        for ii in range(len(Cs)):
            for jj in range(len(Cs)):
                if Cs[ii] < 1 :
                    L[ii,jj] = 0
                    L[jj,ii] = 0
        return L
                 
    def bestQueueSet(self, IC):
        """Picks the best queue to release, based on total number of vehicles in non-conflicting queues"""
        L = IC._L.copy()
        Cs = IC._Cs
        Xs = IC._Xs
        Lc = self.discountCongestedQueues(L, Cs)
        LdotX = np.dot(Lc, Xs)
        Qmax = np.argmax(LdotX)
        return Qmax

class CongestionDemandOptimisingQueueController:

    def __repr__(self):
        return """Uses a matrix of the outgoing lanes for each queue, and knowledge of congestion, in order to further
        optimise the queues to unlock"""

    def calculate_O(self, L_tilda, L):
        return np.multiply(L_tilda, L)

    def calculate_x_tilda(self, O, x):
        return np.dot(O, x)

    def bounded_demand(self, x_tilda, capacity_vec):
        return [max(val) for val in zip(x_tilda,capacity_vec)]

    def demandPerQueue(self, O, x_bounded):
        return [(x_bounded[ii]/np.sum(O[ii][:])) for ii in x_bounded]

    def bestQueueSet(self, IC):

        L = IC._L
        L_tilda = IC._L_tilda
        x = IC._Xs
        capacity_vec = IC._Cs

        O = self.calculate_O(L_tilda, L)
        x_tilda = self.calculate_x_tilda(O, x)
        x_bounded = self.bounded_demand(x_tilda, capacity_vec)
        x_bounded_per_queue = self.demandPerQueue(O, x_bounded)

        total_demand_bounded_by_capacity = np.dot(L, x_bounded_per_queue)

        best_choices = np.nonzero(total_demand_bounded_by_capacity == np.amax(total_demand_bounded_by_capacity))[0]
        best_choice = random.choice(best_choices)

        return best_choice


