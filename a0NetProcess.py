#!/usr/bin/env python
from common import *
from configure.assemble import *
from configure.params import Mytools_Rel_Dir,region2serverIP
from netcode.xml import *
from netcode.func import *
from a4loadServer import query_sumo_junction

addpath= mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)
from logger import SimpleAppendLogger,ErrorLogger


''' Steps:
1. init netconvert to .net.xml TAG_netconvert1
2. get ori list of tls/stops from osm TAG_gettls
3. extract potential tls/stop nodes from .net.xml TAG_extract
4. re-netconvert adding tls/stops TAG_gen_tls_net
5. overwrite <allow> tag to allow passenger for all bus lanes. TAG_lane_allow
6. randomize tls offset
7. change speed limit.
'''

Redo_list = [1,2,3,4,5,6,7] # re-run 
def rerun(i):
	if isinstance(i,int): return i in Redo_list
	for x in i: 
		if x in Redo_list: return True
	return False

QUOTE = config.OSM_QUOTE
sumo_road_type_dir = SUMO_HOME+os.sep+"data"+os.sep+"typemap"+os.sep

base_type_file = "osmNetconvert.typ.xml" 
type_files=[ base_type_file ]
type_files=",".join(list(map(lambda s: sumo_road_type_dir +s, type_files)))

NetconvertCmd = 'netconvert --osm-files %s --type-files %s --output-file %s --geometry.remove --roundabouts.guess --ramps.guess --junctions.join --tls.guess-signals --tls.discard-simple --tls.join --remove-edges.isolated --keep-edges.by-vclass passenger --remove-edges.by-type highway.track,highway.services,highway.unsurfaced --no-turnarounds.except-deadend '


if not os.path.exists(OriOutNetfile) or rerun(1): # TAG_netconvert1
	Netconvert=NetconvertCmd%(DownloadedOsmFile,type_files,OriOutNetfile)
	print(Netconvert)
	subprocess.call(Netconvert,shell=True)


if not os.path.exists(NoTlsNodefile) or rerun([1,3]):
	netObj = sumolib.net.readNet(OriOutNetfile)


''' -------- get ori list of tls/stops from osm file --------'''
''' OSM:
	<node id="2454764998" lat="40.112617" lon="-88.2500906" version="1">
		<tag k="stop" v="minor"/> --OR 
		<tag k="highway" v="stop"/> --OR /Both
		<tag k="highway" v="traffic_signals"/> --OR
	</node>
'''
osm_tls_ori = set()
osm_stop_priority_ori = set() # stop signs are treated as priority for now.
isstop, istls=0,0
curnid=None
if not os.path.exists(AddTlsNetfile) or rerun([2,3]): #TAG_gettls
	with open(DownloadedOsmFile,"r") as f:
		for l in f:
			ll=l.lstrip()
			if ll.startswith("<node id="):
				curnid=ll.split("id=",1)[1].split(" ",1)[0].strip(QUOTE)
			elif curnid:
				if ll.startswith('<tag k="stop"') or ll.startswith('<tag k="highway" v="stop"'):
					isstop=1
				elif ll.startswith('<tag k="highway" v="traffic_signals"'):
					istls=1
				elif ll.startswith("</node>"):
					if isstop: osm_stop_priority_ori.add(curnid)
					if istls: osm_tls_ori.add(curnid)
					isstop, istls=0,0
					curnid=None
	print("Ori OSM has #tls",len(osm_tls_ori))
	print("Ori OSM has #stops",len(osm_stop_priority_ori))


''' --------- output nodes with priority-type, could have tls ---------'''
# in-edge-num, in-lane-num, node-id, lat, lng, x,y
osm_tls_exist = set()
osm_stop_priority_exist = set() # still there after netconver.
allNodeIdSet = set()
any_junc_id_for_test = None

if not os.path.exists(NoTlsNodefile) or rerun(3): # TAG_extract
	print("gen "+NoTlsNodefile)
	ofd = open(NoTlsNodefile,"w")
	with open(OriOutNetfile,"r") as f:
		for l in f:
			if l.startswith("    <junction id"):
				''' type of node? '''
				st1 = l.split("type=",1)
				stype = st1[1].split(" ",1)[0]
				sid = st1[0].split("=")[1].strip().strip('"')
				allNodeIdSet.add(sid)
				if any_junc_id_for_test is None: any_junc_id_for_test=sid
				if sid in osm_tls_ori:
					osm_tls_exist.add(sid)
				elif sid in osm_stop_priority_ori:
					osm_stop_priority_exist.add(sid)
				if stype =='"priority"':
					st2 = l.split("incLanes=",1)
					''' see if 4 incoming edges ? '''
					slane = st2[1].split("intLanes=",1)[0].strip().strip('"')
					st3 = slane.split(" ")
					lset=set()
					for edge in st3:
						edge = edge.split("#")[0]
						if edge not in lset: lset.add(edge)
					if len(lset) in [3,4]:
						xy= st2[0].split("x=")[1].split("y=")
						xi = xy[0].strip().strip('"')
						yi = xy[1].strip().strip('"')
						latlng=get_latlng_given_xy([float(xi),float(yi)] , netObj)
						ofd.write("%d,%d,%s,%.6f,%.6f,%s,%s\n"%(len(lset),len(st3),sid,latlng[0],latlng[1],xi,yi))
	ofd.close()
	print("Tls still in net:",len(osm_tls_exist))
	print("Stops still in net:",len(osm_stop_priority_exist))

tls_id_potential=set()
stop_allway_id_potential=set() # allway_stop
stop_priority_id_potential=set()# priority_stop

if os.path.exists(NoTlsNodefile) and rerun(3):
	print("read "+NoTlsNodefile)
	with open(NoTlsNodefile,"r") as f:
		for l in f:
			st = l.split(",")
			if st[0] == "4":
				if int(st[1])>=6 and st[2].isdigit(): # large, non-clustered
					tls_id_potential.add(st[2])
				elif int(st[1])==5 and st[2].isdigit(): # larger, priority
					stop_priority_id_potential.add(st[2])
				elif int(st[1])==4 and st[2].isdigit(): # small, 4-way
					stop_allway_id_potential.add(st[2])
			elif st[0] == "3":
				if int(st[1])>=5 and st[2].isdigit(): # larger, pri
					stop_priority_id_potential.add(st[2])
	print("Potential #tls",len(tls_id_potential))
	print("Potential #allway_stop",len(stop_allway_id_potential))
	print("Potential #priority_stop",len(stop_priority_id_potential))


''' ------------ remove tls/stop from blacklists ---------------'''
blacklist_to_be_plain = set()
if os.path.exists(TlsIdsBlacklistFile) and rerun(3):
	with open(TlsIdsBlacklistFile,"r") as f:
		for l in f:
			l=l.strip()
			if l and l.isdigit(): # 'joinedS*' cannot be node id. Node ID != Tls ID...
				blacklist_to_be_plain.add(l)
	print("Black listed tls #", len(blacklist_to_be_plain))

# this junction file is generated by sumo b0.py with data server.py running
if os.path.exists(JunctionBlacklistFile) and rerun(3):
	with open(JunctionBlacklistFile,"r") as f:
		for l in f:
			l=l.strip()
			if l and l in allNodeIdSet: blacklist_to_be_plain.add(l)
	print("Black listed J #", len(blacklist_to_be_plain))# no tls for these

for tid in blacklist_to_be_plain:
	if tid in tls_id_potential:
		tls_id_potential.remove(tid)
		print(tid+"  removed from tls_id_potential due to blacklist.")
	if tid in osm_tls_exist:
		osm_tls_exist.remove(tid)
		print(tid+"  removed from osm_tls_exist due to blacklist.")


''' ------------ get expected num of tls/stop ---------------'''
if rerun([3,5]):
	bbox = get_xml_conv_ori_bbox(OriOutNetfile)
	area = abs(bbox["ce"]-bbox["cw"])* abs(bbox["cn"]-bbox["cs"])
	print("area km^2",area/1000/1000)
	e_tls = config.Tls_Density * area
	e_allway = config.Stop_All_Density * area
	e_priority = config.Stop_Pri_Density * area
	print("Expected #tls",e_tls)
	print("Expected #allway_stop",e_allway)
	print("Expected #priority_stop",e_priority)


''' ------------ exclude stop signs at x,y ---------------'''
Server_Working=1
for junc in [any_junc_id_for_test]: 
	if query_sumo_junction(junc, IP_Port=Server_IP_Port) is None:
		Server_Working=0
		print("Server_Working=0")
		raw_input("Server not working, unable to change node property! (if you're running for the first time then it's alright) Enter...")
	break

stop_blacklist = list()
if os.path.exists(StopsignBlacklistFile) and rerun(3):
	with open(StopsignBlacklistFile,"r") as f:
		for l in f:
			x,y =l.split()
			x,y =float(x),float(y)
			stop_blacklist.append([x,y])
	print("Locations to avoid stop signs:",stop_blacklist)
XYDistThresh = 100. # meters range.


''' ---------------- add tls/stop to patch file: ---------'''
''' Patch node type:
https://sumo.dlr.de/wiki/Tutorials/ScenarioGuide#Example:_Patching_the_type_of_a_node
'''
if not os.path.exists(PatchNetworkFile) or rerun(3):
	random.seed(1)
	prob = max(0., float(e_tls-len(osm_tls_exist))/max(1,len(tls_id_potential)) )
	print("adding tls with prob. %.3f from pool %d"%(prob,len(tls_id_potential)))
	tls_id_add = osm_tls_exist.copy()
	if prob>0:
		for x in tls_id_potential:
			if random.random()<prob: tls_id_add.add(x)

	prob = max(0., float(e_allway)/max(1,len(stop_allway_id_potential) ))
	print("adding all-stop with prob. %.3f from pool %d"%(prob,len(stop_allway_id_potential)))
	stop_allway_add =set()
	if prob>0:
		for junc in stop_allway_id_potential:
			if random.random()<prob: 
				blocked = 0
				if Server_Working:
					try:
						res = query_sumo_junction(junc, IP_Port=Server_IP_Port)
					except:
						print("\nserver.py Not running? Try fix that first.")
					x,y  = res["x"],res["y"] 
					for bx,by in stop_blacklist:
						if (bx-x)**2 + (by-y)**2 < XYDistThresh**2:
							blocked=1
							break
				if blocked==0:
					stop_allway_add.add(junc)
				else: 
					print(junc+"  avoid stop sign at %.1f,%.1f"%(x,y ))
					blacklist_to_be_plain.add(junc)

	prob = max(0., float(e_priority-len(osm_stop_priority_exist))/max(1,len(stop_priority_id_potential)) )
	print("adding pri-stop with prob. %.3f from pool %d"%(prob,len(stop_priority_id_potential)))
	stop_priority_add = osm_stop_priority_exist.copy()
	if prob>0:
		for junc in stop_priority_id_potential:
			if random.random()<prob: 
				blocked = 0
				if Server_Working:
					res = query_sumo_junction(junc, IP_Port=Server_IP_Port)
					x,y  = res["x"],res["y"] 
					for bx,by in stop_blacklist:
						if (bx-x)**2 + (by-y)**2 < XYDistThresh**2:
							blocked=1
							break
				if blocked==0:
					stop_priority_add.add(junc)
				else: 
					print(junc+"  avoid stop sign at %.1f,%.1f"%(x,y ))
					blacklist_to_be_plain.add(junc)

	print("Total #tls",len(tls_id_add))
	print("Total #allway_stop",len(stop_allway_add))
	print("Total #priority_stop",len(stop_priority_add))


	print("gen  "+PatchNetworkFile)
	with open(PatchNetworkFile,"w") as f:
		f.write("<nodes>\n")
		for nid in tls_id_add:
			f.write('    <node id="%s" type="traffic_light"/>\n'%nid)
		for nid in stop_allway_add:
			f.write('    <node id="%s" type="allway_stop"/>\n'%nid)
		for nid in stop_priority_add:
			f.write('    <node id="%s" type="priority_stop"/>\n'%nid)
		# patch to remove tls for blacklist
		for nid in blacklist_to_be_plain:
			if nid in allNodeIdSet:
				f.write('    <node id="%s" type="priority"/>\n'%nid)
		f.write("</nodes>\n")


''' --------- call netconvert with patch file ---------'''
# netconvert --sumo-net-file your.net.xml --node-files patch.nod.xml -o yourpatched.net.xml
if not os.path.exists(AddTlsNetfile) or rerun(4): # TAG_gen_tls_net
	print("gen  "+AddTlsNetfile)
	Netconvert= "netconvert --sumo-net-file %s --node-files %s -o %s"%(OriOutNetfile,PatchNetworkFile,AddTlsNetfile)
	print(Netconvert)
	tmp = subprocess.Popen(Netconvert.split(), stdout=subprocess.PIPE)
	pout = tmp.communicate()[0]
	print(pout)


''' ----------- change bus lane to allow passenger car -----------'''

if os.path.exists(AddTlsNetfile) and rerun(5): # TAG_lane_allow
	print("TAG_lane_allow  "+AddTlsNetfile)
	tmpfn = AddTlsNetfile+".lane"
	of = open(tmpfn,"w")
	splitTag1=" allow="+QUOTE # also has tag disallow= 
	splitTag2=QUOTE+" "
	splitTag3=" disallow="+QUOTE 
	target = "passenger"
	cnt=0
	with open(AddTlsNetfile,"r") as f:
		for l in f:
			ll = l.lstrip()
			if ll.startswith("<lane id="):
				st = ll.split(splitTag1,1)
				if len(st)>1: # contains <allow=>
					part1, rest = st
					allowstr, part2 = rest.split(splitTag2,1)
					if target not in allowstr:
						allowstr+= " "+ target
						cnt+=1
						wstr = "        "+part1 + splitTag1 + allowstr + splitTag2 + part2
						of.write(wstr)
					else:
						of.write(l)
				else:
					of.write(l)
				# check if banned passenger cars
				st = ll.split(splitTag3,1)
				if len(st)>1:
					assert target not in st[1].split(splitTag2,1)[0], l
			else:
				of.write(l)
	of.close()
	os.remove(AddTlsNetfile)
	os.rename(tmpfn,AddTlsNetfile)
	print("Added %d bus lanes to allow passenger cars"%cnt)


''' ----------- change tls offset -----------'''

if os.path.exists(AddTlsNetfile) and rerun(6): 
	print("change tls offset  "+AddTlsNetfile)
	tmpfn = AddTlsNetfile+".tlsoffset"
	of = open(tmpfn,"w")
	with open(AddTlsNetfile,"r") as f:
		for l in f:
			ll = l.lstrip()
			if ll.startswith('<tlLogic id='):
				part1,part2 = ll.split(' offset="',1)
				offset  = random.randint(0,10)
				of.write(part1+' offset="%d'%offset + '">\n')
			else:
				of.write(l)
				
	of.close()
	os.remove(AddTlsNetfile)
	os.rename(tmpfn,AddTlsNetfile)


