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

class TLjunction:
    '''Stores junction logic from the net file'''
    def __init__(self, juncID, incLanes, intLanes, requests_responseMatrix, requests_foesMatrix, requests_cont):
        self._id = juncID
        self._incLanes = incLanes
        self._edge2lanes = {}
        self._lanes2indexs = {} 
        self._intLanes = intLanes
        self._numReqests = len(intLanes)
        
        self._requests_responseMatrix = requests_responseMatrix
        self._requests_foesMatrix = requests_foesMatrix
        self._requests_cont = requests_cont
        
    def __str__(self):
        
        string = "Junction Logic\n"
        for attr in self.__dict__:
            string += attr.lstrip('_')
            string += ' : '
            string += str(self.__dict__[attr])
            string += '\n'
        return string
    
    
    
class TLobjects:
    'Container for all the traffic light objects definied above'
    def __init__(self, ET_TLSlogics, ETconns, ET_TLjuncs):
        self._TLSlogics = {}
        self._TLjunctions = {}
        
        self.addTLSlogics(ET_TLSlogics, ETconns)
        self.addTLJunctions(ET_TLjuncs)
        
        
    def __repr__(self):
        
        for TLSlogic in self._TLSlogics:
            print(self._TLSlogics[TLSlogic])
        for TLjunction in self._TLjunctions:
            print(self._TLjunctions[TLjunction])
        
    def addTLSlogics(self, ET_TLSlogics, ETconns):
        '''Takes the extracted TLS and Connection objects from Element Tree (which are stored in an array) and parses them to turn them into objects'''
        for logic in ET_TLSlogics:
            TLSid = logic.attrib['id']
            TLStype = logic.attrib['type']
            programID = logic.attrib['programID']
            offset = logic.attrib['offset']
            phaseStrings = []
            phaseDurations = []
            for phase in logic:
                phaseStrings.append(phase.attrib['state'])
                phaseDurations.append(phase.attrib['duration'])
            self._TLSlogics.update({TLSid : TLSlogic(TLSid, TLStype, programID, offset, phaseStrings, phaseDurations)})
        
        for conn in ETconns:
            TLStarget = conn.attrib['tl']
            self._TLSlogics[TLStarget].addConnection(conn)
        
    def addTLJunctions(self, ET_TLjuncs):
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
            
            requests_responseMatrix = np.matrix(requests_response)
            requests_foesMatrix = np.matrix(requests_foes)
            self._TLjunctions.update({juncID : TLjunction(juncID, incLanes, intLanes, requests_responseMatrix, requests_foesMatrix, requests_cont)})    


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

net = TLobjects(TLlogic, TLconnections, TLjunctions)

TLS = net._TLSlogics
junctions = net._TLjunctions

for junc in junctions:
    print(junctions[junc]._id)
    print(junctions[junc]._requests_foesMatrix)
    print("\n")
    print(junctions[junc]._requests_responseMatrix)
    print("\n")


    
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
