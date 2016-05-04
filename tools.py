# -*- coding: utf-8 -*-
import socket

def getOpenPort():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("",0))
    s.listen(1)
    port = s.getsockname()[1]
    s.close()
    return port

def reverseDict(dictionary):
    reversedDict = {}
    for entry in dictionary:
        for item in dictionary[entry]:
            try:
                reversedDict[item].append(entry)
            except KeyError:
                reversedDict.update({item:[entry]})

    return reversedDict