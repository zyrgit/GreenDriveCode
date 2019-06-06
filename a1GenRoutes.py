#!/usr/bin/env python
from common import * # SUMO_Tools_dir
from configure.assemble import *
from configure.params import Mytools_Rel_Dir,region2serverIP
import traci
from common.constants import MetersPerMile
from common.util import get_weekday_index_given_ymdhms,get_weekday_index_given_mdyhms12,get_hour_index_given_str,in_bbox_latlng,get_heading_given_dirStr,get_minute_given_str
from configure.runcl0327 import addr2RouteFilters 

addpath = mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)
from geo import get_bearing_latlng2,get_dist_meters_latlng2,min_angle_diff

print(R) # defined in configure/*


''' ----------- gen random traffic, this is the base of all traffic --------- '''

i2files={ # index  0:vtype, 1:tripFile, 2:prefix,
 0:[config.tripAttribute%"vta1", mypydir +mapFolder+os.sep+"%s.trip0.xml"%config.osmfile, "a"],
 1:[config.tripAttribute%"vta1", mypydir +mapFolder+os.sep+"%s.trip1.xml"%config.osmfile, "a"],
 2:[config.tripAttribute%"vta1", mypydir +mapFolder+os.sep+"%s.trip2.xml"%config.osmfile, "a"],
 3:[config.tripAttribute%"vta1", mypydir +mapFolder+os.sep+"%s.trip3.xml"%config.osmfile, "a"],
}
tripsIndex=[0,1,2,3] # trips to include.
tripIndUseMyEdgeWeights = [0,2] # force using my edge weights, if duaiterate fails to avoid congestion.

if     'trips' in sys.argv:
	for ind in tripsIndex:
		print("gen  "+i2files[ind][1])
		cmd= SUMO_Tools_dir+"/randomTrips.py -b %d -e %d -n %s -p %f -o %s --prefix %s --trip-attributes %s --additional-files %s --min-distance %d --fringe-factor %d --seed %d --length "%(config.simulationStartTime, config.simulationEndTime, AddTlsNetfile, config.tripDepartPeriod, i2files[ind][1], i2files[ind][2], i2files[ind][0], VTypeDefXmlFile, config.tripMinDistance, config.fringeFactor, ind)
		print("\n"+cmd)
		subprocess.call(cmd,shell=True)
	

''' ---------- gen base traffic/routes using   duarouter ---------------'''

if    'duarouter' in sys.argv:
	for ind in tripsIndex:
		in_trip_files = i2files[ind][1]
		print("input: "+in_trip_files )
		addr_rou_xml = DuaOutRoutesFile.rstrip('.xml')+ '%d.xml'%ind
		print("gen "+addr_rou_xml) # base: addr.rou*.xml

		if os.path.exists(EdgeWeightFile) and ind in tripIndUseMyEdgeWeights:  
			cmd="duarouter --net-file %s --route-files %s --additional-files %s -o %s --seed %d --ignore-errors --remove-loops --weight-files %s --weight-attribute traveltime --vtype-output .dummy.xml "%(AddTlsNetfile,in_trip_files ,VTypeDefXmlFile,addr_rou_xml, ind, EdgeWeightFile)
		else: # do not use edge weight, and will use dua iter
			cmd="duarouter --net-file %s --route-files %s --additional-files %s -o %s --seed %d --ignore-errors --remove-loops --vtype-output .dummy.xml "%(AddTlsNetfile, in_trip_files ,VTypeDefXmlFile, addr_rou_xml, ind)
		print("\n"+cmd)
		time.sleep(1)
		subprocess.call(cmd,shell=True)



''' ---------- optimize routes using   duaIterate ---------------'''

if    'iter' in sys.argv:

	DUA_PY_CMD_FILE = SUMO_Tools_dir+ os.sep+ "assign"+os.sep+ "duaIterate.py"
	if 'usb' in sys.argv:
		USB_Store_dua_dir = '/Volumes/usb/%s/dua/'%config.osmfile 
	else: USB_Store_dua_dir = None
	if My_Platform == "mac":
		niterCmd = ' -l 10'
	else: 
		niterCmd = ' --max-convergence-deviation 0.05' # on cluster, more iters.
	
	for ind in tripsIndex:
		addr_rou_xml = DuaOutRoutesFile.rstrip('.xml')+ '%d.xml'%ind  
		edgeWeightCmd='' # no longer use edge weights, let iter do it. 
		if os.path.exists(EdgeWeightFile) and ind in tripIndUseMyEdgeWeights: 
			edgeWeightCmd = ' duarouter--weight-files %s duarouter--weight-attribute traveltime '%EdgeWeightFile

		if USB_Store_dua_dir is None: # run it limited iters on mac  
			dua_out_dir = DUA_OUT_DIR_Prefix + str(ind)
			if not os.path.exists(dua_out_dir): 
				os.makedirs(dua_out_dir)
			cmd = "python "+ DUA_PY_CMD_FILE +" -n %s"%AddTlsNetfile +" -t %s"%i2files[ind][1] +" %s -b %d -e %d --aggregation 500 "%(niterCmd, config.simulationStartTime, config.simulationEndTime) +" --additional %s"%( VTypeDefXmlFile )+" --disable-summary --disable-tripinfos --weight-memory --pessimism 2 " +" duarouter--additional-files %s"%( VTypeDefXmlFile ) +" duarouter--vtype-output %s/dummy.xml"%dua_out_dir + " duarouter--ignore-errors duarouter--remove-loops duarouter--seed %d "%ind + edgeWeightCmd
		else: # run longer on external storage disk 
			dua_out_dir = USB_Store_dua_dir.rstrip('/')+ str(ind)
			if not os.path.exists(dua_out_dir): 
				os.makedirs(dua_out_dir)
			cmd = "python "+ DUA_PY_CMD_FILE +" -n %s"%AddTlsNetfile +" -t %s"%i2files[ind][1] +" %s -b %d -e %d --aggregation 500 "%(niterCmd, config.simulationStartTime, config.simulationEndTime) +" --additional %s"%( VTypeDefXmlFile )+" --disable-summary --disable-tripinfos --weight-memory --pessimism 2 " +" duarouter--additional-files %s"%( VTypeDefXmlFile ) +" duarouter--vtype-output .dummy.xml" + " duarouter--ignore-errors duarouter--remove-loops duarouter--seed %d "%ind + edgeWeightCmd
		print("\n"+cmd+"\n")
		subprocess.Popen(cmd, shell=True, cwd= dua_out_dir)
	print("\nMove  dua*/*.rou.xml  to main dir yourself ! !")



# rm ./addr/dua*/*._0xx.rou.xml 
if      'clean' in sys.argv: 
	cmd = 'rm '+ mypydir +mapFolder+os.sep +"*.rou*.xml"
	print(cmd)
	subprocess.call(cmd,shell=True)
	for ind in tripsIndex:
		cmd = 'rm -rf '+ DUA_OUT_DIR_Prefix + str(ind)
		print(cmd)
		subprocess.call(cmd,shell=True)
	sys.exit(0)

# copy dua*/*._0xx.rou.xml to ./  # defined in configure/addr/base.py
if      'cp' in sys.argv: 
	for ind in tripsIndex:
		dua_route_file = config.routePoolFile.rstrip('.xml')+'%d.xml'%ind
		cmd = 'rsync -av '+DUA_OUT_DIR_Prefix + str(ind)+os.sep+ config.routePoolFile +" "+mypydir +mapFolder+os.sep+ dua_route_file
		print(cmd)
		subprocess.call(cmd,shell=True)
	# then re-index ids in all rou files.
	files = []
	for ind in tripsIndex:
		dua_route_file = mypydir +mapFolder+os.sep+ config.routePoolFile.rstrip('.xml')+'%d.xml'%ind
		files.append(dua_route_file)
	cnt = 0
	for fn in files:
		newfn = fn+".tmp.xml"
		of = open(newfn,'w')
		with open(fn, 'r') as f:
			for l in f:
				ll= l.lstrip()
				if ll.startswith('<vehicle id='):
					vid, rest = ll.split(' id="',1)[1].split('"',1)
					pref = vid[0]
					newl = '    <vehicle id="'+ pref+ str(cnt)+'"'+rest
					cnt+=1
					of.write(newl)
				else:
					of.write(l)
		of.close()
		os.rename(newfn,fn)

	sys.exit(0)



''' -------- split duaIterate routes into pieces, RUN after duaIterate !!!!! ----------'''

route_edges_str_list = []

def proc_route_lines(lines, ret_attr =[]):
	if len(ret_attr )==0:
		l = lines[0]
		assert l.startswith('<vehicle'), lines
		vid = l.split('<vehicle id="',1)[1].split('"',1)[0]
		vtype = l.split(' type="',1)[1].split('"',1)[0]
		depart, attr = l.split(' depart="',1)[1].split('" ',1)
		ret_attr.append(attr.rstrip('>'))
	l = lines[1]
	assert l.startswith('<route'), lines
	estr = l.lstrip('<route edges="').rstrip('"/>')
	return estr.split(' '), estr

if not os.path.exists(SplitRoutesDir): os.makedirs(SplitRoutesDir)
base_route_file = mypydir +mapFolder+os.sep + config.routePoolFile # /*._0xx.rou.xml
split_num = 40 # split for density control
split_vtypes = ['vta1','vtb1']
split_vid_prefs = ['a','b']
simuStartT = config.simulationStartTime
simuEndT = config.simulationEndTime


if      'splitduaroutes' in sys.argv: # split up dua routes, to allow diff density/penetration
	ret_attr =[]
	in_rou_files = []
	for ind in tripsIndex:
		dua_route_file = base_route_file.rstrip('.xml')+'%d.xml'%ind # /*._0xx.rou.xml
		in_rou_files.append(dua_route_file)
	for fn in addr2RouteFilters[R['addr']].keys():
		fpath = mypydir +mapFolder+os.sep +fn
		if fpath not in in_rou_files: 
			in_rou_files.append(fpath)
	print('in_rou_files',in_rou_files)

	random.seed(1)
	for fn in in_rou_files:
		portion, e2ratio, eset = 1, {}, set()
		rouFileName = fn.rsplit(os.sep,1)[1]
		if rouFileName in addr2RouteFilters[R['addr']]:
			dic = addr2RouteFilters[R['addr']][rouFileName]
			portion = dic['portion']
			e2ratio = dic['edgeaccept']
			eset = set(e2ratio.keys())
		print('reading  '+fn)
		print(portion,e2ratio)
		with open(fn, 'r') as f:
			f.seek(0,2)
			endBytes = f.tell()
			f.seek(0,0)
			while True:
				l = f.readline()
				if l.lstrip().startswith('<routes '): break
			while True:
				l = f.readline()
				if l=='' and f.tell()==endBytes: break
				ll = l.strip()
				lines = []
				if ll.startswith('<vehicle id='):
					lines.append(ll)
					lines.append(f.readline().strip()) #<route edges
					f.readline() # skip #</vehicle>
					elist , estr = proc_route_lines(lines, ret_attr)
					prob = portion
					for avoidedge in set(elist) & eset:
						prob *= e2ratio[avoidedge]
						break
					rand = random.random()
					if rand<prob:
						route_edges_str_list.append(estr)

	random.shuffle(route_edges_str_list)
	print(route_edges_str_list[0:5])
	print("# routes read", len(route_edges_str_list)) # rid not matter, will re-index
	print('writing routes def into  '+RoutesEdgesDefXmlFile)
	cnt = 0
	of = open(RoutesEdgesDefXmlFile, 'w') # merge into one!
	of.write('<routes>\n')
	for es in route_edges_str_list:
		of.write('    <route id="%d" edges="%s"/>\n'%(cnt,es))
		cnt+=1
	of.write('</routes>\n')
	max_route_id = cnt-1
	print('max_route_id',max_route_id)

	print('writing split routes',split_num,split_vtypes)
	tripAttribute = ret_attr[0]
	print('tripAttribute:',tripAttribute)
	num_routes_per_split = len(route_edges_str_list)/split_num

	vidBase = 0
	for i in range(split_num):
		delta_t = (simuEndT - simuStartT) /num_routes_per_split 
		print('delta_t',delta_t)

		for t in range(len(split_vtypes)):	
			vtype = split_vtypes[t]
			vpref = split_vid_prefs[t]
			fn = SplitRoutesDir+ "%d-%s.xml"%(i,vpref )
			print("  writing "+fn)
			of = open(fn,'w')
			of.write('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n')

			departTime = simuStartT
			minID = i* num_routes_per_split
			maxNum = num_routes_per_split 
			r = 0 # consistent rid across vtypes.
			vind = 0 # repeat vid across vtypes. 
			while departTime < simuEndT:
				vid  = "%s%d"%(vpref , vidBase+ vind)
				vind +=1
				vstr = '    <vehicle id="%s" type="%s" depart="%.2f" %s route="%d">\n'%(vid,vtype, departTime, tripAttribute , r+minID)
				departTime += delta_t
				r = (r+1)% maxNum
				of.write(vstr)
				of.write("    </vehicle>\n")
			of.write('</routes>\n')
			of.close()
		vidBase+= vind


''' -------- read taxi data for all IL Chi regions ----------'''

if 'procrawtaxi' in sys.argv: # takes time, do not rerun.
	of = open(TaxiStorageFile,"w") # on usb, 15GB
	with open(TaxiRawDownloadFile,"r") as f:
		print(f.readline())
		for l in f:
			if not l.endswith(",,,,,,,\n"):
				of.write(l)
	of.close()


''' -------- extract target area from taxi data --------- '''

header = ["Trip ID","Taxi ID","Trip Start Timestamp","Trip End Timestamp","Trip Seconds","Trip Miles","Pickup Census Tract","Dropoff Census Tract","Pickup Community Area","Dropoff Community Area","Fare","Tips","Tolls","Extras","Trip Total","Payment Type","Company","Pickup Centroid Latitude","Pickup Centroid Longitude","Pickup Centroid Location","Dropoff Centroid Latitude","Dropoff Centroid Longitude","Dropoff Centroid  Location","Community Areas"]
def ind(col):
	return header.index(col)

minYear  = 2016 

if    'extracttaxi' in sys.argv:# load from TaxiStorageFile
	if not os.path.exists(TaxiDataDir): os.makedirs(TaxiDataDir)
	print('[ extracttaxi ] to '+TaxiYearMonTripDumpFile)
	netObj = sumolib.net.readNet(AddTlsNetfile)
	bbox = netObj.getBBoxXY()
	minX, minY = bbox[0]
	maxX, maxY = bbox[1]
	minLon, minLat = netObj.convertXY2LonLat(minX, minY)
	maxLon, maxLat = netObj.convertXY2LonLat(maxX, maxY)
	bufD = 150.*0.00001 # boundary shrink
	minLon, minLat, maxLon, maxLat = minLon+bufD, minLat+bufD, maxLon-bufD, maxLat-bufD
	meanLat = (minLat+maxLat)/2
	meanLon = (minLon+maxLon)/2
	print("osm bbox", minLat,minLon ,  maxLat,maxLon)
	data = {}
	cnt , thresh, badline = 0, 16, 0
	Data_minLon, Data_minLat, Data_maxLon, Data_maxLat = None,None,None,None
	nearby_loc_latlngs, all_loc_pairs = [], {}
	with open(TaxiStorageFile,"r") as f:
		print(f.readline())
		for l in f:
			# line may contain "*,*" as one column
			st = l.split('"')
			assert len(st)<=3, l
			if len(st)>1:
				l = st[0]+st[2] # get rid of ',' in between ""
			st = l.split(",")
			if len(st)!= len(header): 
				badline+=1
				print("\nbad line "+l)
			if 0: print(st)
			try:
				slat = float(st[ind("Pickup Centroid Latitude")])
				slng = float(st[ind("Pickup Centroid Longitude")])
				elat = float(st[ind("Dropoff Centroid Latitude")])
				elng = float(st[ind("Dropoff Centroid Longitude")])
			except:
				badline+=1
				continue

			if Data_minLat is None:
				Data_minLon, Data_minLat, Data_maxLon, Data_maxLat = min(slng,elng), min(slat,elat), max(slng,elng), max(slat,elat)
			if Data_minLon> min(slng,elng): Data_minLon = min(slng,elng)
			if Data_minLat> min(slat,elat): Data_minLat = min(slat,elat)
			if Data_maxLon< max(slng,elng): Data_maxLon = max(slng,elng)
			if Data_maxLat< max(slat,elat): Data_maxLat = max(slat,elat)

			if in_bbox_latlng(slat,slng, minLat,minLon,maxLat,maxLon) and in_bbox_latlng(elat,elng, minLat,minLon,maxLat,maxLon):
				try:
					seconds = int(st[ind("Trip Seconds")])
				except: seconds = 0.
				airDist = get_dist_meters_latlng2([slat,slng], [ elat,elng])
				try:
					miles = float(st[ind("Trip Miles")])
				except: miles = 0.
				if airDist<10 or miles<0.01 or seconds<1:
					continue

				speed = miles * MetersPerMile / seconds
				hd =  get_bearing_latlng2([slat,slng], [ elat,elng])
				date = st[ind("Trip Start Timestamp")].strip()
				weekday = get_weekday_index_given_mdyhms12(date)# 09/18/2013 07:45:00 AM
				hour = get_hour_index_given_str(date)
				minute = get_minute_given_str(date)
				month, day , year = date.split(' ')[0].split('/')
				month, day , year = int(month), int(day), int(year )

				if year < minYear:
					continue
				if year not in data: data[year] = {}
				if month not in data[year]: data[year][month] = []

				x1,y1 = netObj.convertLonLat2XY(slng,slat)
				x2,y2 = netObj.convertLonLat2XY(elng,elat)
				dic = { 'lat1':slat, 'lng1':slng, 'lat2':elat, 'lng2':elng, 'weekday':weekday, 'speed': speed, 'x1':x1,'y1':y1 , 'x2':x2,'y2':y2 , 'hd':hd, "miles":miles, "seconds":seconds,'year':year,'month':month, "day":day ,"hour":hour,"minute":minute}
				data[year][month].append(dic)
				cnt+=1
				if cnt>=thresh:
					thresh*=2
					print(cnt, badline)
					print(dic)

	print(data.keys()) 
	print("cnt",cnt,"badline",badline)
	if cnt>0: 
		print("pickle dump to "+TaxiYearMonTripDumpFile)
		pickle.dump(data, open(TaxiYearMonTripDumpFile,"wb"))





