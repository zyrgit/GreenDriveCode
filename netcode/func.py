#!/usr/bin/env python
import math

def get_bearing_given_lat_lng(lat1, lon1, lat2, lon2):# heading
    startLat = math.radians(lat1) 
    startLong = math.radians(lon1)
    endLat = math.radians(lat2)
    endLong = math.radians(lon2)
    dLong = endLong - startLong
    dPhi = math.log(math.tan(endLat/2.0+math.pi/4.0)/math.tan(startLat/2.0+math.pi/4.0))
    if abs(dLong) > math.pi:
        if dLong>0.0:
            dLong=-(2.0*math.pi-dLong)
        else:
            dLong=(2.0*math.pi+dLong)
    bearing = (math.degrees(math.atan2(dLong, dPhi)) + 360.0) % 360.0
    return bearing


def get_sumo_vars(traci):
	# 64 speed, 132 dist, 101 fuel, 80 road id, 122 wait time, 86 lane pos, 66 pos(x,y)
    VarSpeed = traci.constants.VAR_SPEED
    VarDist = traci.constants.VAR_DISTANCE 
    VarFuel = traci.constants.VAR_FUELCONSUMPTION 
    VarEdgeId= traci.constants.VAR_ROAD_ID
    VarWaitTime=traci.constants.VAR_WAITING_TIME
    VarLaneId = traci.constants.VAR_LANE_ID
    VarLanePos= traci.constants.VAR_LANEPOSITION
    VarPosXY = traci.constants.VAR_POSITION 
    var2str={
     traci.constants.VAR_SPEED: "spd",
     traci.constants.VAR_DISTANCE : "dist",
     traci.constants.VAR_FUELCONSUMPTION : "fuel",
     traci.constants.VAR_ROAD_ID: "road",
     traci.constants.VAR_WAITING_TIME: "wait",
     traci.constants.VAR_LANE_ID: "lane",
     traci.constants.VAR_LANEPOSITION: "lpos",
     traci.constants.VAR_POSITION : "xy",
    }
    varlistroute=[VarSpeed, VarEdgeId, VarDist, VarFuel, VarWaitTime, VarLanePos,VarLaneId,VarPosXY]
    return var2str, varlistroute


def get_latlng_given_xy(xy, net_obj): 
	''' map/scale from sumo XY pos to real world GPS latlng.
	- xy: list [x=3595.04, y=1115.68]
	- net_obj: from sumolib.net.readNet(OutNetfile)
	See: sumolib.net.convertLonLat2XY and sumolib.net.convertXY2LonLat, traci.simulation.convert2D
	URL: http://sumo.sourceforge.net/userdoc/Tools/Sumolib.html
	'''
	x,y=xy
	lon, lat = net_obj.convertXY2LonLat(x, y)
	return [lat,lon]


def all_same_heading(hdlist, allow_180=False, threshold=15.):
    hd = hdlist[0]
    for i in range(1,len(hdlist)):
        if not same_angle(hd,hdlist[i], allow_180, threshold):
            return False
    return True

def same_angle(a1,a2, allow_180=False, threshold=15.):
    if abs(a1-a2)<threshold or abs(a1-a2+360)<threshold or abs(a1-a2-360)<threshold:
        return True
    if allow_180:
        if same_angle(a1-180,a2,False,threshold) or same_angle(a1+180,a2,False,threshold):
            return True
    return False

