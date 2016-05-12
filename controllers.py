# -*- coding: utf-8 -*-
from __future__ import print_function, division
import numpy as np

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
        for lane in openLanes:
            mu = IC.getMu(lane)
            lam = IC.getLambda(lane)

            try:
                modelBased_Gt = target_number_cars_cleared/(mu-lam)
            except ZeroDivisionError:
                maxGreenTime = self._Tmax

            if modelBased_Gt > self._Tmax : modelBased_Gt = self._Tmax

            if maxGreenTime < modelBased_Gt : maxGreenTime = modelBased_Gt

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