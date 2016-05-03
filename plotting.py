# -*- coding: utf-8 -*-
import xml.etree.cElementTree as ET
import numpy as np
import pandas as pd

def parseXML2object(filepath):
    parsedFile = ET.parse(filepath)
    root = parsedFile.getroot()
    return root

def combineChildrenAttributeValuesInADicionary(root, *args):
    dict = {}
    for arg in args:
        dict.update({arg:[]})
    for child in root:
        for arg in args:
            dict[arg].append(child.attrib[arg])
    return dict

def meanWaitSteps(results_Filepath, step_size):
    root = parseXML2object(results_Filepath)
    dict = combineChildrenAttributeValuesInADicionary(root, "waitSteps")
    waitSteps = map(int, dict["waitSteps"])
    waitTime = map(lambda x : x*step_size, waitSteps)
    return np.mean(waitTime)
    
def meanDepartDelay(results_Filepath):
    root = parseXML2object(results_Filepath)
    dict = combineChildrenAttributeValuesInADicionary(root, "departDelay")
    departDelay = map(float, dict["departDelay"])
    return np.mean(departDelay)

if __name__ == "__main__":
    
    step_size = 0.1
    
    results_Filepath = "tripsoutput.xml"
    root = parseXML2object(results_Filepath)
    
    dict = combineChildrenAttributeValuesInADicionary(root, "id", "waitSteps", "duration", "departDelay")
    
    waitSteps = map(int, dict["waitSteps"])
    waitTime = map(lambda x : x*step_size, waitSteps)
    
    print(np.mean(waitTime))
    
    