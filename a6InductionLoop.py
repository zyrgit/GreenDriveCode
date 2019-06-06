#!/usr/bin/env python
from common import * # SUMO_Tools_dir
from configure.assemble import *
from configure.params import Mytools_Rel_Dir
import requests
import numpy as np
import datetime
from copy import deepcopy
from common.util import get_moved_xy
from common import constants
from a4loadServer import query_sumo_edge
from sumo_data import SampleData,WeightDistribution,YearData
from common.util import get_weekday_index_given_ymdhms,get_weekday_index_given_mdyhms12,get_hour_index_given_str,in_bbox_latlng,get_heading_given_dirStr,get_weekday_index_given_dateStr

addpath= mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)
from geo import get_bearing_latlng2,get_dist_meters_latlng2,min_angle_diff


hour2traffic = { # hourly traffic
	0:0.99,  1:0.7,  2:0.5,  3:0.6,  4:1.1,  5:2.5,  6:5.2,  7:5.8,  8:6.0,  9:5.5,  10:4.9,  11:5.1,  12:5.0,  13:5.2,  14:5.65,  15:7.2,  16:7.7,  17:7.5,  18:6.1,  19:4.6,  20:3.5,  21:2.9,  22:2.1,  23:1.5
	}
HourlyTrafficWeights = WeightDistribution(24,name='HourlyTrafficWeights')
for i,v in hour2traffic.items():
	HourlyTrafficWeights.set_weight_at_index(i,v)
HourlyTrafficWeights.normalize()
periods=[[20,21,22,23,0,1,2,3,4,5], [6,7,8,9],[10,11,12,13,14],[15,16,17,18,19]]



def import_a6_func_gen_loop_xml(loopOutSuffix, print_str=False):
	if print_str: print("pickle load "+LoopCntDataDumpFile)
	if not os.path.exists(SumoLoopOutDir): os.makedirs(SumoLoopOutDir)
	loopdata = pickle.load(open(LoopCntDataDumpFile,"rb"))
	gen_induction_det_xml(loopdata, LoopXmlFile, loopOutSuffix, print_str=print_str)
	if print_str: print("pickle load "+SpeedDataDumpFile)
	speeddata = pickle.load(open(SpeedDataDumpFile,"rb"))
	add_speed_loops_to_det_xml(speeddata, LoopXmlFile , loopOutSuffix, print_str=print_str)



'''Loop Counts,  1279 rows,  
2006-2007
https://data.cityofchicago.org/Transportation/Average-Daily-Traffic-Counts/pfsx-4n4m
'''
def parseTrafficCountsFile(TrafficCountsFile, netObj):
	header = ["ID",	"Traffic Volume Count Location  Address",	"Street",	"Date of Count",	"Total Passing Vehicle Volume",	"Vehicle Volume By Each Direction of Traffic",	"Latitude",	"Longitude",	"Location"] # no dup location  
	bbox = netObj.getBBoxXY()
	minX, minY = bbox[0]
	maxX, maxY = bbox[1]
	minLon, minLat = netObj.convertXY2LonLat(minX, minY)
	maxLon, maxLat = netObj.convertXY2LonLat(maxX, maxY)
	bufD = 300.*0.00001
	minLon, minLat, maxLon, maxLat = minLon+bufD, minLat+bufD, maxLon-bufD, maxLat-bufD
	print("osm bbox", minLat,minLon ,  maxLat,maxLon)
	head = None
	data = []
	Data_minLon, Data_minLat, Data_maxLon, Data_maxLat = None,None,None,None
	all_locs , all_wkdays = {}, {}
	with open(TrafficCountsFile,"r") as f:
		for l in f:
			if head is None: 
				head=l
				continue
			st = l.split(",")
			lat = float( st[6] )
			lon = float( st[7] )
			if Data_minLat is None:
				Data_minLon, Data_minLat, Data_maxLon, Data_maxLat = lon, lat, lon, lat
			if Data_minLon> lon: Data_minLon = lon
			if Data_minLat> lat: Data_minLat = lat
			if Data_maxLon< lon: Data_maxLon = lon
			if Data_maxLat< lat: Data_maxLat = lat

			if   in_bbox_latlng(lat,lon, minLat,minLon,maxLat,maxLon):
				x,y = netObj.convertLonLat2XY(lon,lat)
				dateStr = st[3] # 03/19/2006
				weekday = get_weekday_index_given_dateStr(dateStr)
				all_wkdays[weekday] = all_wkdays.get(weekday,0)+1
				mon ,day ,year = dateStr.split('/')
				mon ,day ,year = int(mon ),int(day ),int(year) 
				cnts = int(st[4]) 
				name = st[2].strip()
				dic = { 'lat':lat, 'lng':lon, 'weekday':weekday, 'cnts': cnts, 'x':x,'y':y ,"name":name, 'day':day, 'month':mon, 'year':year}
				dirs = st[5].split('/',1) #East Bound: 14900 / West Bound: 14000
				added  = 0
				try: # two oppos directions, needs to split up if two-way road
					dirstr,cnt = dirs[0].split(":") # add direction
					hd = get_heading_given_dirStr(dirstr)
					if hd is not None:
						dic1 = deepcopy(dic)
						dic1['hd'],dic1['cnt'] = hd, int(cnt)
						data.append(dic1)
						added  = 1
				except: pass
				try:
					dirstr,cnt = dirs[1].split(":")
					hd = get_heading_given_dirStr(dirstr)
					if hd is not None:
						dic['hd'],dic['cnt'] = hd, int(cnt)
						data.append(dic)
						added  = 1
				except: pass
				if added==0: 
					print(l,"Not added ??")
					print("\n\n\nNot added ??")
					sys.exit(1)

	pprint.pprint(data[0:5])
	print("[parseTrafficCountsFile]", len(data)) 
	return data


def mapLoopToSumoNet(data,netObj, pickleFile=None):  
	dcnt=0
	for dic in data:
		dcnt+=1
		print(dcnt,len(data))
		n, deltas = 0, [2., 6., 10. ,25. , 50, 90]
		sucess= 0
		while sucess==0: # allow retry as inc search radius r:
			candi=[]
			while len(candi)==0 and n< len(deltas):
				candi = netObj.getNeighboringEdges(dic['x'],dic['y'], r=deltas[n])
				n+=1
			if len(candi)==0: 
				print(dic)
				print("NOT Mapped by rtree !! Consider increase dist-delta r? ")
				break
			mapped = 0
			tried, minD = [],1e6
			for edge, d in candi:
				eid = edge.getID()
				res = query_sumo_edge(eid, Server_IP_Port)
				res["eid"]=eid
				res["rd"]=d
				tried.append(res)
				ehead = res['heading']
				if min_angle_diff(dic['hd'],ehead)>45:
					continue
				if d<minD:
					minD = d
					nlane = edge.getLaneNumber()
					dic['eid']= str(eid)
					dic['nlane']=nlane
					mapped=1
			if mapped==0:
				print('')
				print(dic)
				print("heading not match with these:?")
				print(tried)
			if mapped==1: 
				sucess= 1
		if sucess==0:
			raw_input("Fail. Check a2.py GUI, proceed? ...")
	if pickleFile:
		print('pickle.dump(data) to '+pickleFile)
		pickle.dump(data,open(pickleFile,"wb"))
	return data


def gen_induction_det_xml(data, LoopXmlFileName, loopOutSuffix='', print_str=True):
	if print_str: print("\ngen "+LoopXmlFileName)
	of = open(LoopXmlFileName,"w")
	NoOutputLoopXmlFile = LoopXmlFileName.rstrip(".xml")+".noout.xml"
	if print_str: print("\ngen "+NoOutputLoopXmlFile)
	of2 = open(NoOutputLoopXmlFile,"w")
	if not os.path.exists(SumoLoopOutDir): os.makedirs(SumoLoopOutDir)
	DetOutputFile= "out%s.txt"%loopOutSuffix  
	if print_str: print("loop out write to: "+DetOutputFile)
	repositionEdges = set()
	if os.path.exists(LoopFeedbackMovePositionFile):
		if print_str: print('reading '+LoopFeedbackMovePositionFile)
		with open(LoopFeedbackMovePositionFile,'r') as f:
			for l in f:
				repositionEdges.add(l.strip())
	of.write('<additional>\n')
	of2.write('<additional>\n')
	pos = -2.5
	freq = 300
	for dic in data:
		if 'eid' not in dic: 
			print(dic, '   not mapped.  #data', len(data))
			continue
		eid = dic['eid']
		if eid in repositionEdges: lpos = -50
		else: lpos = pos
		for i in range(dic['nlane']):
			laneID =  eid +'_%d'%i
			wstr = '    <inductionLoop id="'+laneID+'" lane="'+laneID+'" pos="%.1f" file="%s" freq="%d" friendlyPos="true"/>\n'%(lpos, DetOutputFile, freq)
			of.write(wstr)
			wstr = '    <inductionLoop id="'+laneID+'" lane="'+laneID+'" pos="%.1f" file="/dev/null" freq="%d" friendlyPos="true"/>\n'%(lpos, freq)
			of2.write(wstr)
	of.write('</additional>\n')
	of2.write('</additional>\n')
	of.close()
	of2.close()



''' Real time road seg speed 
--  50M rows: 11GB+ about 1 year data downloaded .  
https://data.cityofchicago.org/Transportation/Chicago-Traffic-Tracker-Historical-Congestion-Esti/sxs8-h27x
'''
def parseSpeedCsvFile(SpeedSampleDatasetFile, netObj, pickleFile=None):
	header = ["TIME","SEGMENT_ID","SPEED","STREET","DIRECTION","FROM_STREET","TO_STREET","LENGTH","STREET_HEADING","COMMENTS","BUS_COUNT","MESSAGE_COUNT","HOUR","DAY_OF_WEEK","MONTH","RECORD_ID","START_LATITUDE","START_LONGITUDE","END_LATITUDE","END_LONGITUDE","START_LOCATION","END_LOCATION","Community Areas","Zip Codes","Wards"] 
	bbox = netObj.getBBoxXY()
	minX, minY = bbox[0]
	maxX, maxY = bbox[1]
	minLon, minLat = netObj.convertXY2LonLat(minX, minY)
	maxLon, maxLat = netObj.convertXY2LonLat(maxX, maxY)
	bufD = 300.*0.00001 # boundary shrink
	moveD = 10.*0.00001 # move det location 
	LinesPerDay = 10000
	minLon, minLat, maxLon, maxLat = minLon+bufD, minLat+bufD, maxLon-bufD, maxLat-bufD
	print("osm bbox", minLat,minLon ,  maxLat,maxLon)
	head = None
	data = []
	key2data={}  
	cnt , thresh , msg = 0,16,''
	Data_minLon, Data_minLat, Data_maxLon, Data_maxLat = None,None,None,None
	all_locs , loc_cnts = {} , {}
	with open(SpeedSampleDatasetFile,"r") as f:
		for l in f:
			if head is None: 
				head=l
				continue
			cnt+=1
			if cnt>= thresh: 
				print("l# %d, #data %d"%(cnt,len(data)))
				thresh*=2
				if cnt > 2000000 : # about enough?
					if len(data)>0 and data[-1]['data'].has_enough_data() and data[0]['data'].has_enough_data():
						msg = "Good amount of data, each weekday good"
						break
				if len(data)>5 and cnt > 4000000: # enough ?
					msg = "Large amount of data, stop"
					break
			st = l.split(",")
			if len(st)!=len(header): 
				print("bad line... "+l)
				continue
			elat = float( st[header.index("END_LATITUDE")] ) # look at end latlng
			elon = float( st[header.index("END_LONGITUDE")] )
			slat = float( st[header.index("START_LATITUDE")] ) # start loc gps
			slon = float( st[header.index("START_LONGITUDE")] )
			sname = st[header.index("STREET")].strip()
			keyStr= "%s,%.5f,%.5f"%(sname,elat,elon)
			loc_cnts[keyStr] = loc_cnts.get(keyStr,0)+1

			if Data_minLat is None:
				Data_minLon, Data_minLat, Data_maxLon, Data_maxLat = min(slon,elon), min(slat,elat), max(slon,elon), max(slat,elat)
			if Data_minLon> min(slon,elon): Data_minLon = min(slon,elon)
			if Data_minLat> min(slat,elat): Data_minLat = min(slat,elat)
			if Data_maxLon< max(slon,elon): Data_maxLon = max(slon,elon)
			if Data_maxLat< max(slat,elat): Data_maxLat = max(slat,elat)

			speed = float(st[header.index("SPEED")])* constants.Mph2MetersPerSec
			if speed>0 and in_bbox_latlng(elat,elon, minLat,minLon,maxLat,maxLon):
				hd =  get_bearing_latlng2([slat,slon], [ elat,elon])
				direction = st[header.index("DIRECTION")].strip()
				heading = get_heading_given_dirStr(direction)
				if  min_angle_diff(hd,heading)<50 : 
					dx, dy  = slon-elon, slat - elat
					dlon = dx / (dx**2+dy**2)**0.5 * moveD
					dlat = dy / (dx**2+dy**2)**0.5 * moveD # move towards start a little 
					lon,lat = elon+dlon , elat+dlat
					x,y = netObj.convertLonLat2XY(lon,lat)
					name = st[header.index("STREET")].strip()

					keyStr= "%s,%.5f,%.5f"%(name,lat,lon)
					dateStr = st[header.index("TIME")] # 04/07/2019 02:40:36 PM
					weekday = get_weekday_index_given_mdyhms12(dateStr)
					hour = get_hour_index_given_str(dateStr)
					mon, day, year = dateStr.split(" ",1)[0].split('/')
					mon, day, year = int(mon), int(day), int(year) 
					if keyStr not in key2data:
						sp = YearData(keyStr, 30)
						sp.ingest(speed, hour, weekday, day, mon, year)
						dic = { 'lat':lat, 'lng':lon, 'data': sp, 'x':x,'y':y ,"name":name, 'hd':hd, "dir":direction  } 
						key2data[keyStr] = dic
						data.append(dic)
					else:
						dic = key2data[keyStr]
						dic['data'].ingest(speed, hour, weekday, day, mon, year)
					#  about 10,000 records per day 
				else:
					print(l)
					print("heading not matched...",hd,heading)
					sys.exit(1)

	pprint.pprint(data[0:5])
	print("[parseSpeedCsvFile] len#",len(data)) # about 18 locations. (name,lat,lon)
	print(msg)
	print('#loc_cnts', len(loc_cnts))
	return data


def mapSpeedSamplesToSumoEdges(data, netObj, pickleFile=None): 
	# add edge info inplace, and dump again and overwrite.
	dcnt=0
	for dic in data:
		dcnt+=1
		print(dcnt,len(data))
		deltas = [2., 6., 10. , 20., 40., 80., 120]
		mapped , tried = 0,[]
		for n  in range(len(deltas)): # allow retry as inc search radius r:
			candi = netObj.getNeighboringEdges(dic['x'], dic['y'], r=deltas[n])
			if len(candi)==0: 
				continue
			tried, minD = [],1e6
			for edge, d in candi:
				eid = edge.getID()
				res = query_sumo_edge(eid, Server_IP_Port)
				res["eid"]=eid
				res["rd"]=d
				tried.append(res)
				ehead = res['heading']
				if min_angle_diff(dic['hd'],ehead)>45:
					continue
				if d<minD:
					minD = d
					nlane = edge.getLaneNumber()
					dic['eid']= str(eid)
					dic['nlane']=nlane
					mapped=1
			if mapped ==1: break
		if mapped==0:
			print(dic)
			print("? ? heading not match with any of:")
			pprint.pprint(tried)
			raw_input("Failed. Check with a2.py, proceed? ...")
	if pickleFile:
		print('pickle.dump(data) to '+pickleFile)
		pickle.dump(data,open(pickleFile,"wb"))
	return data


def add_speed_loops_to_det_xml(data, LoopXmlFileName , loopOutSuffix='', print_str=True):
	ids = set()
	if os.path.exists(LoopXmlFileName):
		if print_str: print("reading "+LoopXmlFileName)
		with open(LoopXmlFileName,"r") as f:
			for l in f:
				if l.lstrip().startswith('<inductionLoop id='):
					lid = l.split(' id="',1)[1].split('" ',1)[0] # lane id = det id
					ids.add(lid)
	NoOutputLoopXmlFile = LoopXmlFileName.rstrip(".xml")+".noout.xml"
	if print_str: 
		print("Got original #",len(ids),"add data#",len(data))
		print("\ngen "+LoopXmlFileName)
		print("\ngen "+NoOutputLoopXmlFile)
	of = open(LoopXmlFileName,"w")
	of2 = open(NoOutputLoopXmlFile,"w")
	repositionEdges = set()
	if os.path.exists(LoopFeedbackMovePositionFile):
		if print_str: print('reading '+LoopFeedbackMovePositionFile)
		with open(LoopFeedbackMovePositionFile,'r') as f:
			for l in f:
				repositionEdges.add(l.strip())

	DetOutputFile = "out%s.txt"%loopOutSuffix  
	if print_str: print("loop out write to: "+DetOutputFile)
	of.write('<additional>\n')
	of2.write('<additional>\n')
	pos = -2.5
	freq = 300
	for dic in data: # add speed sample location loops:
		if 'eid' not in dic:
			print(dic,'not mapped.')
			continue
		eid = dic['eid']
		if eid in repositionEdges: lpos = -50
		else: lpos = pos
		for i in range(dic['nlane']):
			laneID = dic['eid']+'_%d'%i
			wstr = '    <inductionLoop id="'+laneID+'" lane="'+laneID+'" pos="%.1f" file="%s" freq="%d" friendlyPos="true"/>\n'%(lpos, DetOutputFile, freq)
			of.write(wstr)
			wstr = '    <inductionLoop id="'+laneID+'" lane="'+laneID+'" pos="%.1f" file="/dev/null" freq="%d" friendlyPos="true"/>\n'%(lpos, freq)
			of2.write(wstr)
			if laneID in ids: ids.remove(laneID)
	of.write("\n")
	of2.write("\n")
	for laneID in ids: # add loop counter data locations:
		eid = laneID.rsplit('_',1)[0]
		if eid in repositionEdges: lpos = -50
		else: lpos = pos
		wstr = '    <inductionLoop id="'+laneID+'" lane="'+laneID+'" pos="%.1f" file="%s" freq="%d" friendlyPos="true"/>\n'%(lpos, DetOutputFile, freq)
		of.write(wstr)
		wstr = '    <inductionLoop id="'+laneID+'" lane="'+laneID+'" pos="%.1f" file="/dev/null" freq="%d" friendlyPos="true"/>\n'%(lpos, freq)
		of2.write(wstr)
	of.write('</additional>\n')
	of2.write('</additional>\n')
	of.close()
	of2.close()



def gen_flow_count_speed_csv(loopMatchedData, FlowFileName, HourlyTrafficWeights, selected_hour_ind, year, end_time_sec, stable_time_sec = 600, detect_num = 3):
	# its results is for flowrouter 
	HourlyTrafficWeights.normalize()
	ratio = HourlyTrafficWeights.get_percentage_at(selected_hour_ind)
	start_minute = stable_time_sec/60
	end_minute = end_time_sec/60
	delta_t = (end_minute- start_minute )/(detect_num -1)
 	print("@hour#%d"%selected_hour_ind, "ratio%.2f"%ratio, "Det start minute %d"%start_minute, "end %d"%end_minute, "delta_t",delta_t )
 	print("gen  "+FlowFileName)
	of = open(FlowFileName,"w")
	of.write('Detector;Time;qPKW;vPKW\n')
	for loopdic, speeddic in loopMatchedData:
		wkday = loopdic['weekday']
		month = loopdic['month']
		speed = speeddic['data'].get_mean_at_hour_wkd_mon_year(selected_hour_ind, wkday, month, year)
		speed = int(speed * 3.6) # in km/h
		count =  loopdic['cnt'] * ratio
		cntPerLane = int( count / loopdic['nlane'] )
		for i in range(loopdic['nlane']):
			for t in range(detect_num ) :
				dtime = start_minute + t*delta_t
				laneID = loopdic['eid']+'_%d'%i
				wstr = laneID+";%d;%d;%d\n"%(dtime,cntPerLane,speed)
				of.write(wstr)
				print(wstr)
	of.close()


def gen_routes_flowrouter(netFile, detectorFile, flowFile, RouteOutFromFLow):
	cmd = SUMO_Tools_dir+"/detector/flowrouter.py -v -n %s -d %s -f %s -o %s -b 0 -i 10"%(netFile,detectorFile,flowFile,RouteOutFromFLow)
	print("\n"+cmd)
	help = SUMO_Tools_dir+"/detector/flowrouter.py -help"
	subprocess.call(help, shell=True)
	subprocess.call(cmd, shell=True)


def replace_ori_rou_with_flowrouter_routes(InRouteFile, RouteOutFromFLow, vtype=None):
	rids = []
	with open(RouteOutFromFLow,'r') as f:
		for l in f:
			if l.lstrip().startswith('<route id='):
				rid = l.split(' id="',1)[1].split('" ')[0]
				rids.append(rid)
	print(len(rid),"route ids e.g.",rids[0])
	# replace original rou.xml with these rids.
	NewRouteFile = InRouteFile.rstrip(".xml")+".repfw.xml"
	print("[replace_ori_rou_with_flowrouter_routes] gen "+NewRouteFile)
	of = open(NewRouteFile,"w")
	replacedVid2Rid = {}
	do_replace = 1
	with open(InRouteFile,"r") as f: # assume ori-rou format: <veh ..> \n <edges ..> \n <veh/> \n
		f.seek(0,2)
		endBytes = f.tell()
		f.seek(0,0)
		while True:
			l = f.readline()
			if l=='' and f.tell()==endBytes: break
			if do_replace:
				ll = l.strip()
				if ll.startswith('<vehicle id='):
					rid = rids.pop(0)
					try:
						vid = ll.split(' id="',1)[1].split('" ',1)[0]
					except:
						print(l,ll)
						sys.exit(1)
					replacedVid2Rid[vid]= rid
					print("Replacing",vid,rid)
					if vtype is not None:
						s1, s2 = ll.split(' type="',1) # replace vtype as well.
						_, s2 = s2.split('" ',1)
						ll = s1+ ' type="' + vtype+ '" '+ s2
					newl = ll.rstrip(">")+' route="'+rid+'">\n'
					of.write(newl)
					l = f.readline() # skip edges def.
					l = f.readline() # end of vehicle/> tag
					of.write(l)
					if rids==[]: do_replace=0
				else:
					of.write(l)
			else:
				of.write(l)
	of.close()
	return replacedVid2Rid



if __name__ == "__main__":
	random.seed(1)
	if not os.path.exists(Trace_Dir): os.makedirs(Trace_Dir)
	if not os.path.exists(LoopSpeedDataDir): os.makedirs(LoopSpeedDataDir)
	if not os.path.exists(SumoLoopOutDir): os.makedirs(SumoLoopOutDir)

	netObj = sumolib.net.readNet(AddTlsNetfile)
	print("Be sure to have server.py running! ")

	if "speed" in sys.argv:# gen speed sample data:
		speeddata = parseSpeedCsvFile(SpeedSampleDatasetFile, netObj, SpeedDataDumpFile)
		# speeddata = pickle.load(open(SpeedDataDumpFile,"rb"))
		mapSpeedSamplesToSumoEdges(speeddata, netObj, SpeedDataDumpFile)

	if "loop" in sys.argv: # gen induction loop data
		loopdata = parseTrafficCountsFile(TrafficCountsFile, netObj)
		mapLoopToSumoNet(loopdata,netObj, LoopCntDataDumpFile)
	
	if 'xml' in sys.argv: # write loop def xml, given loop count locations 
		loopOutSuffix =  R['lsuf'] 
		print("pickle load "+LoopCntDataDumpFile)
		loopdata = pickle.load(open(LoopCntDataDumpFile,"rb"))
		gen_induction_det_xml(loopdata, LoopXmlFile, loopOutSuffix)
		# add loops at speed locations.
		print("pickle load "+SpeedDataDumpFile)
		speeddata = pickle.load(open(SpeedDataDumpFile,"rb"))
		add_speed_loops_to_det_xml(speeddata, LoopXmlFile , loopOutSuffix)


	

	