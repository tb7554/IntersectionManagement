# -*- coding: utf-8 -*-
from __future__ import print_function
import numpy as np
import xml.etree.cElementTree as ET

class TLSlogic:
    '''Stores Traffic Light logic from the net file'''
    def __init__(self, TLSid, TLStype, programID, offset, phaseStrings, phaseDurations):
        self._id = TLSid
        self._type = TLStype
        self._programID = programID
        self._offset = offset
        self._phaseStrings = phaseStrings
        self._phaseDurations = phaseDurations
        
        self._TLconnections = {}
        
    def __str__(self):
        
        string = "TLS Logic\n"
        for attr in self.__dict__:
            string += attr.lstrip('_')
            string += ' : '
            string += str(self.__dict__[attr])
            string += '\n'
        return string
    
    def showRelationships(self):
        for conn in self._TLconnections:
            conObject = self._TLconnections[conn]
            print(conObject._linkIndex, conObject._dir)
            
    def addConnection(self, ETconn):
        frm = ETconn.attrib['from']
        to = ETconn.attrib['to']
        fromLane = ETconn.attrib['fromLane']
        toLane = ETconn.attrib['toLane']
        via = ETconn.attrib['via']
        tl = ETconn.attrib['tl']
        linkIndex = int(ETconn.attrib['linkIndex'])
        direction = ETconn.attrib['dir']
        state = ETconn.attrib['state']
        self._TLconnections.update({frm + "_" + fromLane : TLconnection(frm, to, fromLane, toLane, via, tl, linkIndex, direction, state)})
        
    def setFrom2indexes(self):
        for con in self._TLconnections:
            self._fromLane2indexes.update()

class TLconnection:
    'Stores Traffic light controlled junction connections from the net file'
    def __init__(self, frm, to, fromLane, toLane, via, tl, linkIndex, direction, state):
        self._from = frm
        self._to = to
        self._fromLane = fromLane
        self._toLane = toLane
        self._via = via
        self._tl = tl
        self._linkIndex = linkIndex
        self._dir = direction
        self._state = state
        self._fromLane2indexes = {}
        
    def __str__(self):
        string = "Connection - "
        for attr in self.__dict__:
            string += attr.lstrip('_')
            string += ' : '
            string += str(self.__dict__[attr])
            string += ', '
        return string
    
    def getOutLane(self):
        return self._toLane

class TLjunction:
    '''Stores junction logic from the net file'''
    def __init__(self, juncID, incLanes, intLanes, requests_responseMatrix, requests_foesMatrix, requests_cont):
        self._id = juncID
        self._incLanes = incLanes
        self._outLanes = []
        self._intLanes = intLanes
        
        self._numRequests = len(intLanes)
        
        self._requests_responseMatrix = requests_responseMatrix
        self._requests_foesMatrix = requests_foesMatrix
        self._requests_cont = requests_cont
        
        self._TLconnections = {}
        self._edge2lanes = {}
        self._incLane2indexes = {}
        self._outLane2indexes = {}
        
        self._directions = []
        
    def __str__(self):
        
        string = "Junction Logic\n"
        for attr in self.__dict__:
            string += attr.lstrip('_')
            string += ' : '
            string += str(self.__dict__[attr])
            string += '\n'
        return string
    
    def addConnection(self, ETconn):
        frm = ETconn.attrib['from']
        to = ETconn.attrib['to']
        fromLane = ETconn.attrib['fromLane']
        toLane = ETconn.attrib['toLane']
        via = ETconn.attrib['via']
        tl = ETconn.attrib['tl']
        linkIndex = int(ETconn.attrib['linkIndex'])
        direction = ETconn.attrib['dir']
        state = ETconn.attrib['state']
        self._TLconnections.update({linkIndex : TLconnection(frm, to, fromLane, toLane, via, tl, linkIndex, direction, state)})
    
    def setEdge2Lanes(self):
        for conn in self._TLconnections:
            fromEdge = self._TLconnections[conn]._from
            fromLane = fromEdge + '_' + self._TLconnections[conn]._fromLane
            
            if fromEdge not in self._edge2lanes.keys():
                self._edge2lanes.update({fromEdge : fromLane})
            elif fromLane not in self._edge2lanes[fromEdge]:
                self._edge2lanes[fromEdge].append(fromLane)
    
    def setIncLanes2Indexes(self):
        for conn in self._TLconnections:
            fromEdge = self._TLconnections[conn]._from
            fromLane = fromEdge + '_' + self._TLconnections[conn]._fromLane
            index = self._TLconnections[conn]._linkIndex
            if fromLane not in self._incLane2indexes.keys():
                self._incLane2indexes.update({fromLane:[index]})
            elif index not in self._incLane2indexes[fromLane]:
                self._incLane2indexes[fromLane].append(index)
            else:
                print("Something wrong in setIncLanes2Indexes Logic.")
    
    def setOutLanes2Indexes(self):
        for conn in self._TLconnections:
            toEdge = self._TLconnections[conn]._to
            toLane = toEdge + '_' + self._TLconnections[conn]._toLane
            index = self._TLconnections[conn]._linkIndex
            if toLane not in self._outLane2indexes.keys():
                self._outLane2indexes.update({toLane:[index]})
            elif index not in self._outLane2indexes[toLane]:
                self._outLane2indexes[toLane].append(index)
            else:
                print("Something wrong in setOutLanes2Indexes Logic.")
    
    def setIndex2dirs(self):
        for ii in range(0, self._numRequests):
            self._directions.append(self._TLconnections[ii]._dir)
    
    def setOutLanes(self):
        outLanes = []
        for connection in self.getConnections():
            outLane = self.getConnections()[connection].getOutLane()
            if outLane not in outLanes : outLanes.append(outLane)
    
    def customMaxValue(self, priorityArray, itemsForComparison):
        '''Lets you assign a custom order of priority to the items in array. Returns the highest value.'''
        index = len(priorityArray)
        for item in itemsForComparison:
            try:
                new_index = priorityArray.index(item)
                if new_index < index : index = new_index
            except ValueError:
                pass
        return priorityArray[index]
         
    def findCompatibleFlows_lanePriorityModel(self):
        
        # Pull the relevant bits of data from the object.
        n = self._numRequests
        light_settings_matrix = np.zeros([n,n]).astype(str)
        L = np.zeros([n,n])
        foes = np.matrix(self._requests_foesMatrix)
        
        # Using the principle that two flows crossing makes them incompatible, we set the matrix L, which gives flow compatability, and matrix light_matrix, which gives
        # the light setting 
        for ii in range(0,n):
            for jj in range(0,n):
                if foes[ii,jj]:
                        light_settings_matrix[ii,jj] = 'r'
                else:
                    light_settings_matrix[ii,jj] = 'G'
                    L[ii,jj] = 1
        
        if __name__ == "__main__":
            print(light_settings_matrix) 
            print('\n')
            print(L.astype(int)) 
            print('\n')
        
        # We now need to add the relationship between dependent flows. Flows are dependent if, for example, they originate from the same lane. A flow with value 1 which is dependent on a flow
        # with value 0, must also be set to zero. The resulting matrix is called L_squashed.
        lane2index = self._lane2indexes
        L_squashed = np.zeros([n,n])
        
        # For every lane (which will be the lane with priority)
        for lane_1 in lane2index:
            # compared with every other lane (which will be dependent on lane_1)
            for lane_2 in lane2index:
                indexes_1 = lane2index[lane_1] # Get the indexes from that lane
                indexes_2 = lane2index[lane_2] # Get the indexes from that lane
                # We are going to make a sub-matrix from L, based on the two lanes being compared (and including all their indexes)
                L_sub = L[min(indexes_1):max(indexes_1)+1,min(indexes_2):max(indexes_2)+1]
                # If every value in this sub matrix is not equal to 1, then the whole submatrix must be set to zero. We simply use
                # the logical numpy call .all() to determine this. If false, leave every value in L_squashed as 0 for this matrix
                # if true, set every value to 1.
                if L_sub.all() : L_squashed[min(indexes_1):max(indexes_1)+1,min(indexes_2):max(indexes_2)+1] = 1        
        
        if __name__ == "__main__":
            print(L_squashed.astype(int))
            print('\n')
        
        # Using the light settings defined above, and the knowledge of independent flows from L_squashed, we create an adjusted
        # light_settings_matrix called 'lights_squashed', which takes into account dependency between flows. We also take into account
        # the priority of differenct light settings (e.g. if a traffic light for flow 'a' is green for flow 'b', but red for
        # flow 'c', and 'b' is dependent on 'c' (i.e. 'b' and 'c' are on the same lane), then 'a' must be set to red. Similarly
        # if 'a' has priority over 'b' but must give priority to 'c', then 'a' must be set to give priority.
         
        lights_squashed = np.empty([n,n]).astype(str)
        priorityLights = []
        for ii in range(0,n):
            lightChoices = []
            for jj in range(0,n):
                if L_squashed[ii][jj] == 1 :
                    lightChoices.append(light_settings_matrix[jj][ii])
            priorityLights.append(self.customMaxValue(['r', 'g', 'G'], lightChoices))
        
        for ii in range(0,n):
            for jj in range(0,n):
                if L_squashed[ii][jj] == 1 : 
                    lights_squashed[ii][jj] = priorityLights[jj]
                else:
                    lights_squashed[ii][jj] = 'r'
        
        if __name__ == "__main__":          
            print(lights_squashed)
        
        return  L_squashed, lights_squashed
    
    def findCompatibleFlows_variablePriorityModel(self):
        
        # Pull the relevant bits of data from the object.
        n = self._numRequests
        light_settings_matrix = np.zeros([n,n]).astype(str)
        L = np.zeros([n,n])
        foes = np.matrix(self._requests_foesMatrix)
        response = np.matrix(self._requests_responseMatrix)
        
        # Using the principle that two flows crossing makes them incompatible, we set the matrix L, which gives flow compatability, and matrix light_matrix, which gives
        # the light setting 
        for ii in range(0,n):
            for jj in range(0,n):
                ii_dir = self._directions[ii]
                jj_dir = self._directions[jj]
                # ii has foe jj, and jj has priority at unsignalled junctions
                if foes[ii,jj] and (response[ii,jj] or response[jj,ii]):
                    # if ii is turning right, it has priority over left turns. It will not conflict with other right turns, and conflicting straight flows will be red.
                    if ii_dir == 'r' and jj_dir == 'l':
                        light_settings_matrix[ii,jj] = 'g'
                        L[ii,jj] = 1
                    elif ii_dir =='r' and jj_dir == 's':
                        light_settings_matrix[ii,jj] = 'r'
                        L[ii,jj] = 0
                    # if ii is going straight, it has priority. If jj is going straight, it must have a red light, if jj is turning left it may have a green light but give way.
                    # If jj is turning right it will have a red light, or it will give way.
                    elif ii_dir == 's' and (jj_dir == 's' or jj_dir == 'r'):
                        light_settings_matrix[ii,jj] = 'r'
                        L[ii,jj] = 0
                    elif ii_dir == 's' and jj_dir == 'l':
                        light_settings_matrix[ii,jj] = 'g'
                        L[ii,jj] = 1
                    # if ii is turning left it will give way to straight flows and right turning flows. If jj is left turning flow it will be red.
                    elif ii_dir == 'l' and (jj_dir == 's' or jj_dir == 'r'):
                        light_settings_matrix[ii,jj] = 'G'
                        L[ii,jj] = 1
                    elif ii_dir == 'l' and jj_dir =='l':
                        light_settings_matrix[ii,jj] = 'r'
                        L[ii,jj] = 0
                elif not(foes[ii,jj]):
                    light_settings_matrix[ii,jj] = 'G'
                    L[ii,jj] = 1
                else:
                    print("Forgotten case when %d, %d" % (ii,jj))
                    
        
        if __name__ == "__main__":
            print(L.astype(int)) 
            print(light_settings_matrix) 
        
        # We now need to add the relationship between dependent flows. Flows are dependent if, for example, they originate from the same lane. A flow with value 1 which is dependent on a flow
        # with value 0, must also be set to zero. The resulting matrix is called L_squashed.
        incLane2index = self._incLane2indexes
        L_squashed = np.zeros([n,n])
        
        # For every lane (which will be the lane with priority)
        for lane_1 in incLane2index:
            # compared with every other lane (which will be dependent on lane_1)
            for lane_2 in incLane2index:
                indexes_1 = incLane2index[lane_1] # Get the indexes from that lane
                indexes_2 = incLane2index[lane_2] # Get the indexes from that lane
                # We are going to make a sub-matrix from L, based on the two lanes being compared (and including all their indexes)
                L_sub = L[min(indexes_1):max(indexes_1)+1,min(indexes_2):max(indexes_2)+1]
                # If every value in this sub matrix is not equal to 1, then the whole submatrix must be set to zero. We simply use
                # the logical numpy call .all() to determine this. If false, leave every value in L_squashed as 0 for this matrix
                # if true, set every value to 1.
                if L_sub.all() : L_squashed[min(indexes_1):max(indexes_1)+1,min(indexes_2):max(indexes_2)+1] = 1        
        
        if __name__ == "__main__":
            print(L_squashed.astype(int))
            print('\n')
        
        # Using the light settings defined above, and the knowledge of independent flows from L_squashed, we create an adjusted
        # light_settings_matrix called 'lights_squashed', which takes into account dependency between flows. We also take into account
        # the priority of differenct light settings (e.g. if a traffic light for flow 'a' is green for flow 'b', but red for
        # flow 'c', and 'b' is dependent on 'c' (i.e. 'b' and 'c' are on the same lane), then 'a' must be set to red. Similarly
        # if 'a' has priority over 'b' but must give priority to 'c', then 'a' must be set to give priority.
         
        lights_squashed = np.empty([n,n]).astype(str)
        priorityLights = []
        for ii in range(0,n):
            lightChoices = []
            for jj in range(0,n):
                if L_squashed[ii][jj] == 1 :
                    lightChoices.append(light_settings_matrix[jj][ii])
            priorityLights.append(self.customMaxValue(['r', 'g', 'G'], lightChoices))
        
        for ii in range(0,n):
            for jj in range(0,n):
                if L_squashed[ii][jj] == 1 : 
                    lights_squashed[ii][jj] = priorityLights[jj]
                else:
                    lights_squashed[ii][jj] = 'r'
        
        if __name__ == "__main__":          
            print(lights_squashed)
        
        return  L_squashed, lights_squashed
        
    # A few get functions to tidy up the object and stop people accessing properties directly
    
    def getConnections(self):
        return self._TLconnections
    
    def getIncLanes(self):
        return self._incLanes
        
    def getIntLanes(self):
        return self._intLanes
    
    def getOutLanes(self):
        return self._outLanes
        
    def getNumQueues(self):
        return self._numRequests
    
    def getEdge2LanesDict(self):
        return self._edge2lanes
    
    def getLanesFromEdge(self, edgeID):
        return self._edge2lanes[edgeID]
    
    def getIncLane2IndexesDict(self):
        return self._incLane2indexes
    
    def getOutLane2IndexesDict(self):
        return self._outLane2indexes
    
    def getIndexesFromLane(self, laneID):        
        return self._lane2index[laneID]  

class TLobjects:
    'Container for all the traffic light objects definied above'
    def __init__(self, ET_TLjuncs, ETconns):
        self._TLjunctions = {}
        self.addTLJunctions(ET_TLjuncs, ETconns)
        self.addRelationalDicts()
        
    def __repr__(self):
        
        for TLSlogic in self._TLSlogics:
            print(self._TLSlogics[TLSlogic])
        for TLjunction in self._TLjunctions:
            print(self._TLjunctions[TLjunction])
        
    def addTLJunctions(self, ET_TLjuncs, ETconns):
        '''Takes the extracted traffic light controlled junctions from Element Tree and parses them into objects in this container'''
        for junction in ET_TLjuncs:
            juncID = junction.attrib['id']
            incLanes = junction.attrib['incLanes'].split()
            intLanes = junction.attrib['intLanes'].split()
            requests_response = []
            requests_foes = []
            requests_cont = []
            for request in junction:
                # the response and foe bit strings are read right to left in the xml file, here we reverse it for clarity, so that all bitstrings are read left to right
                response = list(request.attrib['response'])
                response.reverse()
                requests_response.append(map(int, response))
                
                foes = list(request.attrib['foes'])
                foes.reverse()
                requests_foes.append(map(int, foes))
                
                requests_cont.append(request.attrib['cont'])
            
            requests_responseMatrix = requests_response
            requests_foesMatrix = requests_foes
            self._TLjunctions.update({juncID : TLjunction(juncID, incLanes, intLanes, requests_responseMatrix, requests_foesMatrix, requests_cont)})    

        for conn in ETconns:
            juncTarget = conn.attrib['tl']
            self._TLjunctions[juncTarget].addConnection(conn)
        
    def addRelationalDicts(self):
        for juncTarget in self._TLjunctions:    
            self._TLjunctions[juncTarget].setEdge2Lanes()
            self._TLjunctions[juncTarget].setIncLanes2Indexes()
            self._TLjunctions[juncTarget].setOutLanes()
            self._TLjunctions[juncTarget].setOutLanes2Indexes()
            self._TLjunctions[juncTarget].setIndex2dirs()
            self._TLjunctions[juncTarget].findCompatibleFlows_variablePriorityModel()
            self._TLjunctions[juncTarget].setOutLanes()

def getTLobjects(netFile_filepath):
    netFileParsed = ET.parse(netFile_filepath)
    netRoot = netFileParsed.getroot() 
    
    TLlogic = []
    TLjunctions = []
    TLconnections = []
    
    for item in netRoot:
        
        if item.tag == "tlLogic" : TLlogic.append(item)
        if item.tag == "junction" and item.attrib["type"] == 'traffic_light': TLjunctions.append(item)
        if item.tag == "connection" and 'tl' in item.keys(): TLconnections.append(item)    
    
    TLnet = TLobjects(TLjunctions, TLconnections)
    
    return TLnet
    

if __name__ == "__main__":
    netFile_filepath = "netFiles/grid.net.xml"
    netFileParsed = ET.parse(netFile_filepath)
    netRoot = netFileParsed.getroot() 
    
    TLlogic = []
    TLjunctions = []
    TLconnections = []

    for item in netRoot:
        
        if item.tag == "tlLogic" : TLlogic.append(item)
        if item.tag == "junction" and item.attrib["type"] == 'traffic_light': TLjunctions.append(item)
        if item.tag == "connection" and 'tl' in item.keys(): TLconnections.append(item)
    
    net = TLobjects(TLjunctions, TLconnections)
    junctions = net._TLjunctions
