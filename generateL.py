import numpy as np
import xml.etree.cElementTree as ET
import operator
import functools

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
        self._intLanes = intLanes
        self._numRequests = len(intLanes)
        
        self._requests_responseMatrix = requests_responseMatrix
        self._requests_foesMatrix = requests_foesMatrix
        self._requests_cont = requests_cont
        
        self._TLconnections = {}
        self._edge2lanes = {}
        self._lane2indexes = {}
        self._outLanes = []
        
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
    
    def setLanes2Indexes(self):
        for conn in self._TLconnections:
            fromEdge = self._TLconnections[conn]._from
            fromLane = fromEdge + '_' + self._TLconnections[conn]._fromLane
            index = self._TLconnections[conn]._linkIndex
            if fromLane not in self._lane2indexes.keys():
                self._lane2indexes.update({fromLane:[index]})
            elif index not in self._lane2indexes[fromLane]:
                self._lane2indexes[fromLane].append(index)
            else:
                print("Something wrong in setLanes2Indexes Logic.")
         
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
         
    def findCompatibleFlows(self):
        n = self._numRequests
        light_settings_matrix = np.zeros([n,n]).astype(str)
        L = np.zeros([n,n])
        foes = self._requests_foesMatrix
        dirs = self._directions
        
        for ii in range(0,n):
            for jj in range(0,n):
                if foes[ii][jj]:
                    d_row = dirs[ii]
                    d_col = dirs[jj]
                    if d_row == "s" and d_col == "s":
                        light_settings_matrix[ii][jj] = 'r'
                        L[ii][jj] = 0
                    elif d_row == "s" and d_col == "r":
                        light_settings_matrix[ii][jj] = 'r'
                        L[ii][jj] = 0
                    elif d_row == "s" and d_col == "l":
                        light_settings_matrix[ii][jj] = 'r'
                        L[ii][jj] = 0
                    elif d_row == "r" and d_col == "s":
                        light_settings_matrix[ii][jj] = 'r'
                        L[ii][jj] = 0
                    elif d_row == 'r' and d_col == 'l':
                        light_settings_matrix[ii][jj] = 'r'
                        L[ii][jj] = 0
                    elif d_row == "l" and d_col == "s":
                        light_settings_matrix[ii][jj] = 'r'
                        L[ii][jj] = 0
                    elif d_row == 'l' and d_col == 'l':
                        light_settings_matrix[ii][jj] = 'r'
                        L[ii][jj] = 0
                    elif d_row == 'l' and d_col =='r':
                        light_settings_matrix[ii][jj] = 'r'
                        L[ii][jj] = 0
                    else:
                        print("Forgot case when d_row %s and d_col %s" % (d_row, d_col))
                else:
                    light_settings_matrix[ii][jj] = 'G'
                    L[ii][jj] = 1
        
        
        
        print(light_settings_matrix) 
        print('\n')
        print(L.astype(int)) 
        print('\n')
        
        
        lane2index = self._lane2indexes
        L_squashed = np.zeros([n,n])
        for lane in lane2index:
            indexes = lane2index[lane]
            for ii in range(0,n):
                lane_queue_values = []
                for index in indexes:
                    lane_queue_values.append(L[index][ii])
                logical_and_result = functools.reduce(operator.mul,lane_queue_values,1)
                for index in indexes:
                    L_squashed[index][ii] = logical_and_result
        
        """Need a way to zero lanes which cannot be opened due to interdependence with other lights"""
                
                    
        print(L_squashed.astype(int))
        print('\n')
        
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
                    
        print(lights_squashed)
        
       # print("".join(lights_squashed[0]))
        #print("".join(lights_squashed[3]))
        
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
    
    def getLane2IndexesDict(self):
        return self._lane2indexes
    
    def getIndexesFromLane(self, laneID):        
        return self._lane2index[laneID]  

    
    
"""     
        print(lights_squashed)
        
        foes = self._requests_foesMatrix
        priors = self._requests_responseMatrix
        lane2index = self._lane2indexes
        
        print(np.matrix(foes))
        print("\n")
        print(np.matrix(priors))
        print("\n")
        
        n = self._numRequests
        
        L_priority = np.zeros([n, n])
        L_conflict = np.zeros([n, n])
        L = np.zeros([n,n])
        L_squashed = np.zeros([n,n])
        
        for lane in lane2index:
            indexes = lane2index[lane]
            for index in indexes:
                L_row = []
                for entry in foes[index][:]:
                    if entry == 0:
                        L_row.append(int(1))
                    elif entry == 1:
                        L_row.append(int(0))
                    else:
                        print("Unexpected integer greater than 0 in find compatible flows")        
                
                L[index] = L_row
                L_priority[index] = np.subtract(L_row, priors[index][:])
                L_conflict[index] = np.add(L_row, priors[index][:])
            
            
            for ii in range(0,n):
                lane_queue_values = []
                for index in indexes:
                    lane_queue_values.append(L[index][ii])
                print(lane_queue_values)
                logical_and_result = functools.reduce(operator.mul,lane_queue_values,1)
                print(logical_and_result)
                for index in indexes:
                    L_squashed[index][ii] = logical_and_result
        
        print(L.astype(int))
        
        print('\n')
        
        print(L_squashed.astype(int))
        print('\n')
        print(np.matrix(priors))
        
        Lp = np.multiply(L, L_priority)
        
        L_flat = []
        for ii in range(0,n):
            L_flat.append(list(L[ii]))"""    
                    
                

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
            self._TLjunctions[juncTarget].setLanes2Indexes()
            self._TLjunctions[juncTarget].setIndex2dirs()
            self._TLjunctions[juncTarget].findCompatibleFlows()
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
    
    """
    for junc in junctions:
        print(junctions[junc]._id)
        print(junctions[junc]._requests_foesMatrix)
        print("\n")
        print(junctions[junc]._requests_responseMatrix)
        print("\n")
    """
    
    junctions['0/0'].findCompatibleFlows()

    
"""
a = ["rrrGGgrrrGGg", "rrryygrrryyg", "rrrrrGrrrrrG", "rrrrryrrrrry", "GGgrrrGGgrrr", "yygrrryygrrr", "rrGrrrrrGrrr", "rryrrrrryrrr"]

numLanes = 4

numConnections = len(a[0])

b = []

for lane in range(0,numConnections):
    
    d = [0] * (numConnections)
    
    for item in a:
        
        c = list(item)
        
        for letter in range(0,numConnections):

            if (c[lane] == "G" or c[lane] == "g") and (c[letter] == "G" or c[letter] == "g") :
                d[letter] = 1
                
    b.append(d)
        
        
print(b)

e = np.matrix(b)

print(e)


strings = []
for set in b:
    new_string = []
    for light in set:
        if light > 0:
            new_string.append('r')
        else:
            new_string.append('G')
    strings.append("".join(new_string))
    
print(strings)
"""
