#!/usr/bin/env python
from common import * # SUMO_Tools_dir
from configure.assemble import *
from configure.params import region2serverIP,Mytools_Rel_Dir
import requests
import datetime
from common import constants
from a4loadServer import query_sumo_edge
from sumo_data import SampleData,WeightDistribution,YearData
from common.util import get_weekday_index_given_ymdhms,get_weekday_index_given_mdyhms12,get_hour_index_given_str,in_bbox_latlng,get_heading_given_dirStr,get_minute_given_str,get_moved_xy
from common.constants import MetersPerMile
import json

addpath= mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)
from geo import get_bearing_latlng2,get_dist_meters_latlng2,min_angle_diff,dist_point_to_line_of_2pts
addpath= mypydir+"../zyrcode/"
if addpath not in sys.path: sys.path.append(addpath)
from myosrm import gen_map_html_from_path_list


def select_trips_on_year_mon_wkday(TaxiYearMonTripDumpFile, y,m,d):
	print("[ select_trips_on_year_mon_wkday ] reading "+TaxiYearMonTripDumpFile)
	data = pickle.load(open(TaxiYearMonTripDumpFile,"rb"))
	ret = []
	for dic in data[y][m]:
		if dic['weekday']==d:
			ret.append(dic)
	print("[select_trips]",y,m,d,"#%d"%len(ret))
	return ret

def map_xy_to_sumo_edge(x,y,netObj):
	n, deltas = 0, [5., 10. , 20., 40., 80., 160.,]
	edgeID , tried = None, []
	for n in range(len(deltas)): 
		candi = netObj.getNeighboringEdges(x, y, r =deltas[n])
		if len(candi)==0: 
			continue
		mapped = 0
		tried, minD = [],1e6
		for edge, d in candi:
			eid = edge.getID()
			tried.append(eid)
			if d<minD:
				minD = d
				edgeID = str(eid)
				mapped=1
		if mapped==1:
			break
	return edgeID , tried


def map_trip_to_sumo_net(data, netObj, selected_hour_list, start_sec, TaxiTripXmlFile, trip_prefix, vtype, TaxiTripToStatsFile, maxDepartureTime): # use a1*.py to extract data. 
	print('[ map_trip_to_sumo_net ] in#',len(data))
	dcnt=0
	timed = []
	timeOffset = 3600* min(selected_hour_list)
	minTime , maxTime = 1e9, 0
	for dic in data:
		dcnt+=1
		if dic['hour'] not in selected_hour_list:
			continue
		eid , tried = map_xy_to_sumo_edge(dic['x1'],dic['y1'], netObj)
		if eid is None:
			print(dic)
			for e in tried:
				res = query_sumo_edge(e,Server_IP_Port)
				print("1 Not "+e,res)
			print("1 NOT Mapped by rtree !! Consider increase dist-delta r? ")
			continue
		startEdge = eid
		eid , tried = map_xy_to_sumo_edge(dic['x2'],dic['y2'], netObj)
		if eid is None:
			print(dic)
			for e in tried:
				res = query_sumo_edge(e,Server_IP_Port)
				print("2 Not "+e,res)
			print("2 NOT Mapped by rtree !! Consider increase dist-delta r? ")
			continue
		endEdge = eid
		dic['eid1']=startEdge
		dic['eid2']=endEdge
		hour = dic['hour']
		minute = dic['minute']
		departSec  = hour * 3600 + minute*60 - timeOffset
		if departSec >maxTime: maxTime=departSec 
		if departSec <minTime: minTime=departSec 
		timed.append([departSec , dic])
		if dcnt%100==1: 
			print(dcnt,len(data))
			print(dic)
	timed.sort()
	tspan = maxTime-minTime
	minTime = start_sec
	maxTime = tspan + minTime

	deltaT = float(maxTime-minTime)/max(1,len(timed))
	trip2stats = {}
	with open(TaxiTripXmlFile,"w") as f:
		f.write('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n')
		t = minTime
		ind = 0
		for _,dic in timed:
			tripID = trip_prefix+str(ind)
			wstr = '	<trip id="%s" depart="%.2f" from="%s" to="%s" type="%s" departLane="best" departSpeed="max" departPos="base" arrivalPos="max"/>\n'%( tripID , t , dic['eid1'], dic['eid2'] , vtype)
			f.write(wstr)
			t+= deltaT
			ind+=1
			trip2stats[tripID] = dic
		f.write('</routes>')
	print("Done "+TaxiTripXmlFile)
	print("pickle dump "+TaxiTripToStatsFile)
	pickle.dump(trip2stats, open(TaxiTripToStatsFile, "wb"))


def gen_routes_given_trips(Netfile, TaxiTripXmlFile, TaxiRouteEdgesFile):
	cmd="duarouter --net-file %s --route-files %s --additional-files %s -o %s --seed 1 --ignore-errors --remove-loops --vtype-output .dummy.xml"%(Netfile,TaxiTripXmlFile,VTypeDefXmlFile,TaxiRouteEdgesFile)
	print("\n"+cmd)
	subprocess.call(cmd,shell=True)


Header = ["Trip ID","Taxi ID","Trip Start Timestamp","Trip End Timestamp","Trip Seconds","Trip Miles","Pickup Census Tract","Dropoff Census Tract","Pickup Community Area","Dropoff Community Area","Fare","Tips","Tolls","Extras","Trip Total","Payment Type","Company","Pickup Centroid Latitude","Pickup Centroid Longitude","Pickup Centroid Location","Dropoff Centroid Latitude","Dropoff Centroid Longitude","Dropoff Centroid  Location","Community Areas"]
def hindex(col):
	return Header.index(col)


def get_pass_by_trips(AddTlsNetfile, TaxiStorageFile):
	netObj = sumolib.net.readNet(AddTlsNetfile)
	bbox = netObj.getBBoxXY()
	minX, minY = bbox[0]
	maxX, maxY = bbox[1]
	minLon, minLat = netObj.convertXY2LonLat(minX, minY)
	maxLon, maxLat = netObj.convertXY2LonLat(maxX, maxY)
	bufD = 150.*0.00001 # boundary shrink
	minLon, minLat, maxLon, maxLat = minLon+bufD, minLat+bufD, maxLon-bufD, maxLat-bufD
	print("osm bbox", minLat,minLon ,  maxLat,maxLon)
	data = {}
	cnt , thresh, badline = 0, 1, 0
	afn = mypydir + mapFolder +os.sep+"taxi" +os.sep + 'taxi-pass-append.txt'
	def _write(wstr):
		with open(afn,'a') as f:
			f.write(wstr)

	with open(TaxiStorageFile,"r") as f:
		for l in f:
			st = l.split('"')
			assert len(st)<=3, l
			if len(st)>1:
				l = st[0]+st[2] # get rid of ',' in between ""
			st = l.split(",")
			try:
				slat = float(st[hindex("Pickup Centroid Latitude")])
				slng = float(st[hindex("Pickup Centroid Longitude")])
				elat = float(st[hindex("Dropoff Centroid Latitude")])
				elng = float(st[hindex("Dropoff Centroid Longitude")])
			except:
				badline+=1
				continue
			# init filter by min/max lat/lon
			vertm = dist_point_to_line_of_2pts([minLat,minLon],[slat,slng],[elat,elng])
			vertx = dist_point_to_line_of_2pts([maxLat,maxLon],[slat,slng],[elat,elng])
			# too far away
			if vertm>4000 or vertx>4000:
				continue
			# filter out inside bbox, already generated 
			if in_bbox_latlng(slat,slng, minLat,minLon,maxLat,maxLon) and in_bbox_latlng(elat,elng, minLat,minLon,maxLat,maxLon):
				continue
			# get route as list of latlngs
			try:
				res = requests.post(config.GreenRoute_IP_Port+'/html/route', data = {'startLat':slat,'startLng':slng,'endLat':elat,'endLng':elng} ).json()
			except:
				print("Err requests.post "+config.GreenRoute_IP_Port+'/html/route')
				continue
			latlnglst = []
			for latlngStr in str(res['gpstrace']).split("~|"):
				tmp = latlngStr.split(",")
				latlnglst.append( [float(tmp[0]),float(tmp[1])] ) 
			# find 1st pt to be in bbox:
			i = 0
			while i<len(latlnglst):
				lat, lng = latlnglst[i]
				if in_bbox_latlng(lat,lng, minLat,minLon,maxLat,maxLon):
					break
				i+=1
			starti = i
			if starti==len(latlnglst):
				continue
			while i<len(latlnglst):
				lat, lng = latlnglst[i]
				if not in_bbox_latlng(lat,lng, minLat,minLon,maxLat,maxLon):
					break
				i+=1
			endi = i-1
			airDist = get_dist_meters_latlng2(latlnglst[starti],latlnglst[endi])
			if airDist < 1200:
				continue # air-dist too small
			slat,slng = latlnglst[starti]
			elat,elng = latlnglst[endi]

			if 1: # Note: duration/seconds/dist/miles info is invalid
				try:
					seconds = int(st[hindex("Trip Seconds")])
				except: seconds = 0.
				airDist = get_dist_meters_latlng2([slat,slng], [ elat,elng])
				try:
					miles = float(st[hindex("Trip Miles")])
				except: miles = 0.
				if airDist<10 or miles<0.01 or seconds<1:
					continue

				speed = miles * MetersPerMile / seconds
				hd =  get_bearing_latlng2([slat,slng], [ elat,elng])
				date = st[hindex("Trip Start Timestamp")].strip()
				weekday = get_weekday_index_given_mdyhms12(date)# 09/18/2013 07:45:00 AM
				hour = get_hour_index_given_str(date)
				minute = get_minute_given_str(date)
				month, day , year = date.split(' ')[0].split('/')
				month, day , year = int(month), int(day), int(year )

				if year != 2017:  continue
				if month != 4: continue
				if year not in data: data[year] = {}
				if month not in data[year]: data[year][month] = []

				x1,y1 = netObj.convertLonLat2XY(slng,slat)
				x2,y2 = netObj.convertLonLat2XY(elng,elat)
				dic = { 'lat1':slat, 'lng1':slng, 'lat2':elat, 'lng2':elng, 'weekday':weekday, 'speed': speed, 'x1':x1,'y1':y1 , 'x2':x2,'y2':y2 , 'hd':hd, 'year':year,'month':month, "day":day ,"hour":hour,"minute":minute}
				data[year][month].append(dic)
				_write(json.dumps(dic)+'\n')
				cnt+=1
				if cnt>=thresh:
					thresh*=2
					print(cnt)
					print(dic)
	print("cnt",cnt,"badline",badline)
	if cnt>0: 
		fn = mypydir + mapFolder +os.sep+"taxi" +os.sep + 'taxi-pass.txt'
		print("pickle dump to "+fn)
		pickle.dump(data, open(fn,"wb"))


if __name__ == "__main__":
	random.seed(1)

	netObj = sumolib.net.readNet(AddTlsNetfile)
	addr  = R['addr']
	selected_hour_list = [10]
	trip_prefix = 'c'
	vtype ='vtc1' # taxi, cyan color
	start_sec = 200 # taxi enter when slightly later.

	if 'map' in sys.argv: 
		data = select_trips_on_year_mon_wkday(TaxiYearMonTripDumpFile, 2017,4,2)  
		map_trip_to_sumo_net(data, netObj, selected_hour_list, start_sec, TaxiTripXmlFile, trip_prefix, vtype ,TaxiTripToStatsFile, 3600 )

	if 'route' in sys.argv:
		gen_routes_given_trips(AddTlsNetfile, TaxiTripXmlFile, TaxiRouteEdgesFile)

	if 'passby' in sys.argv:
		get_pass_by_trips(AddTlsNetfile, TaxiStorageFile)
		# again, use a1*.py to process the dumped file. 

