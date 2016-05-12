# -*- coding: utf-8 -*-
from __future__ import division
import socket
from matplotlib import pyplot as plt

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
    for entry in reversedDict:
        if len(reversedDict[entry]) == 1 : reversedDict[entry] = reversedDict[entry][0]

    return reversedDict

class PIDcontroller():

    def __init__(self, Kp, Ki, Kd):
        self._Kp=Kp
        self._Ki=Ki
        self._Kd=Kd

        self._cumError = 0
        self._prevError = 0

    def update(self, target_value, measured_value):
        error = target_value - measured_value
        self._cumError += error
        change_in_error = error - self._prevError

        return measured_value + self._Kp*error + self._Ki*self._cumError + self._Kd*change_in_error


#
# controller = PIDcontroller(0.1, 0.00001, 0.1)
#
# target = 0.5
# measured_value = 0
#
# a = []
#
# for ii in range(0,10):
#     measured_value = controller.update(target, measured_value)
#     a.append(measured_value)
#     print(measured_value)
#
# plt.plot(range(len(a)), a)
# plt.show()
#
# x = [1,2,3,4,5]
# y = 3
#
#
# for ii in x:
#     if ii > y:
#         print(ii,y,y/ii)
#     elif ii < y:
#         print(ii,y,ii/y)
#     else:
#         print(ii,y,1)
