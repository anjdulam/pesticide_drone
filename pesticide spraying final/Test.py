from __future__ import print_function
import time
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Command
from dronekit import *
from math import radians, sin, cos, sqrt, atan2
from distance import *
hold = False
pitch = False
ret=False
spray=False
curr_dist = 0



def object_movement():
    dist = measure_distance()
    if int(dist) - curr_dist < 0:
        return True
    return False
    
def detect_object():
    dist = measure_distance()
    if dist >= 5:
        return False
    global curr_dist
    curr_dist = int(dist)
    return True
    
def checkabort(abort_flag=True):
    global ret
    ret=abort_flag
    
def checksprinkler(flag):
    global spray
    spray=flag
    
def setsprinkler(vehicle, spray):
    if spray == False:
        print("Stopped the Spray")
        cmd =vehicle.message_factory.command_long_encode( 1, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 900, 0, 0, 0, 0, 0)
        vehicle.send_mavlink(cmd)
    else:
        print("Started the spray")
        cmd =vehicle.message_factory.command_long_encode( 1, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 2000, 0, 0, 0, 0, 0)
        vehicle.send_mavlink(cmd)



 
def fetch_container_val(initval):
    global spray
    if initval <= 0:
        spray=False
        print("Tank has run out of pesticide")
    elif 0 < initval < 0.5:
        print("Tank is running out of pesticide")
        print("Current Tank volume: ", initval)
    else:
        print("Current Tank volume: ", initval)
    preex = initval - 0.060
    return preex


def abortmission(vehicle):
    global ret
    ret =False
    print("EMERGENCY RTL TRIGGERED")
    vehicle.mode=VehicleMode("RTL")
    print("Returning To Launch")
    print("Disarming motors22")
    vehicle.armed = False
    print("Close vehicle object")
    vehicle.close()

def buttonhold(x):
    global hold
    hold = True

def buttonpitch(y):
    global pitch
    pitch = True

def distance_to_current_waypoint(vehicle):
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def arm_and_takeoff(vehicle,altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(altitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def pos_hold(vehicle):
    global pitch
    print("Drone Holded")
    vehicle.mode = VehicleMode("GUIDED")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0,
        0, 0)
    k = 1
    vehicle.send_mavlink(msg)
    while k <= 5:  
        k += 1
        if(pitch or object_movement()):
            pitch = False
            pitch_back(vehicle)
            break
        time.sleep(1)

def pitch_back(vehicle):
    global pitch
    print("Pitching back")
    vehicle.mode = VehicleMode("GUIDED")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        -1, 0, 0, # x, y, z velocity in m/s
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    time.sleep(1)
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    l = 1
    while l<=5:
        if(pitch or object_movement()):
            print("Pitching back")
            msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            -1, 0, 0, # x, y, z velocity in m/s
            0, 0, 0,
                0, 0)
            pitch = False
            vehicle.send_mavlink(msg)
            time.sleep(1)
            msg = vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b0000111111000111,
                0, 0, 0,
                0, 0, 0, # x, y, z velocity in m/s
                0, 0, 0,
                0, 0)
            vehicle.send_mavlink(msg)
            l = -1
        l += 1
        time.sleep(1)
    vehicle.mode = VehicleMode("RTL") 

def grid_mission(vehicle,data,groundspeed,altitude):

    cmds = vehicle.commands

    print(" Clearing any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command(
         0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 0, altitude))
    
    for i in range(len(data)):
        cmds.add(Command( 
            0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
              data[i]['lat'], data[i]['lng'], altitude))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 
                     0, 0, 0, 0, 0, data[len(data)-1]['lat'], data[len(data)-1]['lng'], altitude))
    cmds.upload()
    print("Commands uploaded")

def fly(data,altitude,groundspeed,airspeed,cont): 
    #connection_string = 'COM4' 
    connection_string = "udp:127.0.0.1:14550" 
    print('Connecting to vehicle on: %s' % connection_string) 
    vehicle = connect(connection_string, wait_ready=True,timeout=60,baud=57600) 
    #vehicle = connect(connection_string, wait_ready=True) 
    flag = 0 
    try: 
        global hold 
        global pitch 
        global ret
        global spray
        grid_mission(vehicle,data,groundspeed,altitude) 
        arm_and_takeoff(vehicle,altitude) 
        print("Set default/target airspeed to {}".format(airspeed)) 
        vehicle.airspeed = airspeed 
        print("MISSION IN STARTING SOON..") 
        time.sleep(3) 
        vehicle.commands.next=0 
        vehicle.mode = VehicleMode("AUTO") 
        while True: 
 
            if(ret==True): 
                abortmission(vehicle) 
                break 
 
            if(hold or detect_object()): 
                hold = False 
                pos_hold(vehicle) 
                break 
                                                        
            nextwaypoint=vehicle.commands.next 
            if(nextwaypoint==2):
                setsprinkler(vehicle,True)
                spray=True
            if(spray): 
                val=fetch_container_val(cont) 
                print(val) 
                cont=val 
            print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint(vehicle))) 
            if nextwaypoint==len(data): #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit. 
                setsprinkler(vehicle,False)
                print("MISSION ACCOMPLISHED!!") 
                break 
             
            time.sleep(1) 
    except KeyboardInterrupt: 
        print("User interrupted the program") 
    finally: 
        print("Going back home") 
        vehicle.parameters['RTL_ALT'] = 0
        vehicle.mode = VehicleMode("RTL") 
        print("Disarming motors") 
        vehicle.armed = False 
        vehicle.close()
