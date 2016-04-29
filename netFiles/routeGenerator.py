#!/usr/bin/env python
"""
@file    routeGenerator.py
@author  Simon Box
@date    31/01/2013

Code to generate a routes file for the "simpleT" SUMO model.

"""

import random

routes = open("grid.rou.xml", "w")
print >> routes, """<routes>
<vType id="typeCar" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="25" guiShape="passenger"/>
<vType id="typeBus" accel="0.8" decel="4.5" sigma="0.5" length="17" minGap="3" maxSpeed="25" guiShape="bus"/>

<route id="bottom0totop0" edges="bottom0to0/0 0/0to0/1 0/1totop0" />
<route id="bottom0totop1" edges="bottom0to0/0 0/0to1/0 1/0to1/1 1/1totop1" />
<route id="bottom0toright1" edges="bottom0to0/0 0/0to1/0 1/0to1/1 1/1toright1" />
<route id="left0toright1" edges="left0to0/0 0/0to1/0 1/0to1/1 1/1toright1" />
<route id="top0toright0" edges="top0to0/1 0/1to1/1 1/1to1/0 1/0toright0" />
<route id="top1toleft1" edges="top1to1/1 1/1to0/1 0/1toleft1" />
"""

N = 9000
peS = 1./30
peW = 1./10
pwS = 1./30
pwE = 1./10
psE = 1./50
psW = 1./50

lastVeh = 0
vehNr = 0
for i in range(N):
	if random.uniform(0,1) < peS:
	    print >> routes, '    <vehicle id="%i" type="typeCar" route="bottom0totop0" depart="%i" />' % (vehNr, i)
	    vehNr += 1
	    lastVeh = i
	if random.uniform(0,1) < peW:
	    print >> routes, '    <vehicle id="%i" type="typeCar" route="bottom0totop1" depart="%i" />' % (vehNr, i)
	    vehNr += 1
	    lastVeh = i
	if random.uniform(0,1) < pwS:
	    print >> routes, '    <vehicle id="%i" type="typeCar" route="bottom0toright1" depart="%i" />' % (vehNr, i)
	    vehNr += 1
	    lastVeh = i
	if random.uniform(0,1) < pwE:
	    print >> routes, '    <vehicle id="%i" type="typeCar" route="left0toright1" depart="%i" />' % (vehNr, i)
	    vehNr += 1
	    lastVeh = i
	if random.uniform(0,1) < psE:
	    print >> routes, '    <vehicle id="%i" type="typeCar" route="top0toright0" depart="%i" />' % (vehNr, i)
	    vehNr += 1
	    lastVeh = i
	if random.uniform(0,1) < psW:
	    print >> routes, '    <vehicle id="%i" type="typeCar" route="top1toleft1" depart="%i" />' % (vehNr, i)
	    vehNr += 1
	    lastVeh = i

print >> routes, "</routes>"
routes.close()

