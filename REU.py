#!/usr/bin/env python
#
# Copyright (c) 2018 Michele Segata <segata@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import os
import sys
import random
import time

import math
#import matplotlib.pyplot as plt

from utils import add_platooning_vehicle, communicate, get_distance, \
    start_sumo, running, add_vehicle

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from plexe import Plexe, DRIVER, ACC, CACC, RPM, GEAR, RADAR_REL_SPEED, SPEED, RADAR_DISTANCE

# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5

# cruising speed
velocity = 30

ACC_HEADWAY=1.5
HEADWAY_DISTANCE=ACC_HEADWAY * velocity

# inter-vehicle distance

#DISTANCE = ACC_HEADWAY * velocity + 2

# sinusoid frequency
FREQ = 0.2
# sinusoid amplitude
AMP = 10 / 3.6

#speed_initial=120 / 36
#headaway=0.3
#headaway_distance=headaway*speed_initial

# maneuver actors
LEADER = "v.0"
CANDIDATE = "v.1"
RANDOMCAR = "v.0.0"
N_VEHICLES = 4
RUNTIME=3000
#step=range(101, 200)


def add_vehicles(plexe, n, real_engine=False):
    """
    Adds a platoon of n vehicles to the simulation, plus an additional one
    farther away that wants to join the platoon
    :param plexe: API instance
    :param n: number of vehicles of the platoon
    :param real_engine: set to true to use the realistic engine model,
    false to use a first order lag model
    """
    # add a platoon of n vehicles
    #add_vehicle(plexe, RANDOMCAR, 3 * (DISTANCE + LENGTH), 0, 30)

    for i in range(n):
        vid = "v.%d" % i
        add_platooning_vehicle(plexe, vid, (n - i + 1) * (DISTANCE + LENGTH)
                               , 0, velocity, DISTANCE, real_engine=False, vtype="vtypeauto")
        plexe.set_fixed_lane(vid, 0, safe=False)
        traci.vehicle.setSpeedMode(vid, 0)
        plexe.use_controller_acceleration(vid, False)
        if i == 0:
            plexe.set_active_controller(vid, ACC)
            plexe.set_acc_headway_time(vid, ACC_HEADWAY)
        else:
            plexe.set_active_controller(vid, ACC)
            plexe.set_acc_headway_time(vid, ACC_HEADWAY)
        if i > 0:
            plexe.enable_auto_feed(vid, True, LEADER, "v.%d" % (i-1))
            plexe.set_cc_desired_speed(vid, 50)
    

def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    gui = True
    start_sumo("cfg/freeway.sumo.cfg", False, gui=gui)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    data=open("data.txt", "w+")
    #Info=open("distance.txt","w+");
    #vel_c=open("vel_c.txt", "w+");
    while running(demo_mode, step, RUNTIME):

        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == RUNTIME:
            start_sumo("cfg/freeway.sumo.cfg", True, gui=gui)
            step = 0
            random.seed(1)

        traci.simulationStep()

        
        if step == 0:
            # create vehicles and track the joiner
            #add_vehicle(plexe, RANDOMCAR, 4.5 * (DISTANCE + LENGTH), 0, 25)
            add_vehicles(plexe, N_VEHICLES, real_engine=False)
            if gui:
                traci.gui.trackVehicle("View #0", LEADER)
                traci.gui.setZoom("View #0", 20000)
        if step >= 1:
            #if step  == 500:
                #time = step / 100.0
                #speed = SPEED + AMP * math.sin(2 * math.pi * FREQ * time)
            plexe.set_acc_headway_time(CANDIDATE, 1.4)

            #Radar=plexe.get_vehicle_data(LEADER)
            #velocity_v=Radar[SPEED]
            #if velocity_v != velocity:
              #  plexe.set_acc_headway_time(CANDIDATE, (36/velocity_v))

     
        #if step % 1000 == 0:
         #   t =time.time()
          #  random.seed(int(t))
           # new_headway = random.randint(101, 200)
            #    #plexe.set_cc_desired_speed(LEADER, speed)
            #plexe.set_acc_headway_time(CANDIDATE, new_headway / 100.00)
            #print(new_headway)

        if real_engine and setter is not None:
            # if we are running with the dashboard, update its values
            tracked_id = traci.gui.getTrackedVehicle("View #0")
            if tracked_id != "":
                ed = plexe.get_engine_data(tracked_id)
                vd = plexe.get_vehicle_data(tracked_id)
                setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

        
        calculated=plexe.get_acc_acceleration(CANDIDATE)
        Radar_info=plexe.get_radar_data(CANDIDATE)
        Info_C=plexe.get_vehicle_data(CANDIDATE)
        velocity_C=Info_C[SPEED]
  
        #RELATIVE=Radar_info[RADAR_REL_SPEED]
        print(calculated, Radar_info[RADAR_DISTANCE], velocity_C, file=data)
        #print(Radar_info[RADAR_DISTANCE], file=Info)
        #print(velocity_C, file=vel_c)
        #print(new_headway)
        step += 1

    traci.close()

    data.close()
    #Info.close()
    #vel_c.close()

if __name__ == "__main__":
    main(False, False)
