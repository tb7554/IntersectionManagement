# -*- coding: utf-8 -*-
from __future__ import division, print_function
import os, sys, subprocess
import matplotlib.pyplot as plt
import plotting as tbplot
import tools
import traci  # SUMO API
import generateL as genL
import controllers as ctrl
from intersection_controller import IntersectionController, IntersectionControllerContainer
from vehicle_routing_intersection import RouteController

os.environ["SUMO_HOME"] = "/sumo" # Home directory, stops SUMO using slow web lookups for XML files
os.environ["SUMO_BINARY"] = "/usr/local/bin/sumo-gui" # binary for SUMO simulations

if __name__ == "__main__":
    
    if "-gui" in sys.argv:
        os.environ["SUMO_BINARY"] = "/usr/local/bin/sumo-gui" 
    
    # Input arguments
    netFile_filepath = "netFiles/acosta/acosta_buslanes_fixed.net.xml" #sys.argv[1]
    routeFile_filepath = "netFiles/acosta/acosta.rou.xml" #sys.argv[2]
    additional_file = "netFiles/acosta/acosta_vtypes.add.xml"
    step_length = 0.1
    tripInfoOutput_filepath = "tripsoutput.xml"

    traciPort = tools.getOpenPort()

    target_frac = 0.5
    Tmin = 10
    Tmax = 60

    timer = ctrl.ModelBasedGreenTimeController(Tmin, Tmax)
    queue_control = ctrl.LmaxQueueController()

    intersection_controller_container = IntersectionControllerContainer()
    intersection_controller_container.add_intersection_controllers_from_net_file(netFile_filepath, target_frac, timer, queue_control)

    # if guiOn: sumoBinary += "-gui" Need an options parser to add this, currently just setting gui to default
    sumoCommand = ("%s -n %s -r %s -a %s --step-length %.2f --tripinfo-output %s --remote-port %d --no-step-log --time-to-teleport -1 --netstate-dump acosta_dump.xml" % \
                   (os.environ["SUMO_BINARY"], netFile_filepath, routeFile_filepath, additional_file, step_length, tripInfoOutput_filepath, traciPort))
    sumoProcess = subprocess.Popen(sumoCommand, shell=True, stdout=sys.stdout, stderr=sys.stderr)
    print("Launched process: %s" % sumoCommand)
    
    # Open up traci on a free port
    traci.init(traciPort)
    
    # initialise the step
    step = 0

    # run the simulation
    while step < 0 or traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        intersection_controller_container.update_intersection_controllers(step, step_length)

        #intersection_controller_container.print_details('235')

        step += step_length

    traci.close()
    sys.stdout.flush()
    
    sumoProcess.wait()