class minMaxGreenTimeController:
    
    def __init__(self, Tmin, Tmax, x_star):
        """ Controller that uses basic Tmin and Tmax to adjust green time """
        self._Tmin = Tmin
        self._Tmax = Tmax
        self._x_star = x_star
        self._initialGreenTime = (Tmax+Tmin)/2
        
    def __repr__(self):
        return "Min/Max Time Controller: Controller that uses basic Tmin and Tmax to adjust green time "
        
    def getInitialGreenTime(self):
        """ Calculates the intial green time that all lights start with """
        return self._initialGreenTime
        
    def getNewGreenTime(self, a, b, timer):
        """ Takes the target number of vehicles to remove from the queue, compares withe the actual number. Returns updated green time."""
        
        if b > a:
            Gt = (timer + self._Tmin)/2
            #print("Decreasing green time")
        elif b < a:
            Gt = (timer + self._Tmax)/2
            #print("Increasing Green Time")
        else:
            Gt = timer
            #print("No change in green time")  
        return Gt
    
class PGreenTimeController:
    
    def __init__(self, K, G0, x_star):
        """ Controller that uses an estimated error in the green to make adjustments """
        self._K = Tmin
        self._x_star = x_star
        self._initialGreenTime = G0
        
    def __repr__(self):
        return "Proportional Green Time Controller: Controller that uses an estimated error in the green to make adjustments"
        
    def getInitialGreenTime(self):
        """ Calculates the intial green time that all lights start with """
        return self._initialGreenTime
        
    def getNewGreenTime(self, X_prev, B, timer):
        """ Takes the target number of vehicles to remove from the queue, compares withe the actual number. Returns updated green time."""
        A = X_prev*self._x_star
        if A:
            error = ((A-B)/A)*timer
        else:
            error = 0
        return timer + self._K*error

class LmaxQueueController:
    
    def __init__(self):
        """ Controller that maximises the total number of vehicles released at each phase, regardless of fairness """
    
    def bestQueueSet(self, IC):
        """Picks the best queue to release, based on total number of vehicles in non-conflicting queues"""
        L = IC._L
        LdotX = np.dot(L, IC._Xs)
        Qmax = np.argmax(LdotX)
        return Qmax

class CongestionAwareLmaxQueueController:
    
    def __init__(self):
        pass
    
    def discountCongestedQueues(self, L, Cs):
        """Sets L to 0 for queues which have nowhere to go"""
        for ii in range(len(Cs)):
            for jj in range(len(Cs)):
                if Cs[ii] == 0 : L[ii,jj] = 0
                
        return L
                 
    def bestQueueSet(self, IC):
        """Picks the best queue to release, based on total number of vehicles in non-conflicting queues"""
        L = self.discountCongestedQueues(IC._L, IC._Cs)
        LdotX = np.dot(L, IC._Xs)
        Qmax = np.argmax(LdotX)
        return Qmax