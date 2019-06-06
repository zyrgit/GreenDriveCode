#!/usr/bin/env python
from common import * # SUMO_Tools_dir
from configure.assemble import *
from configure.params import region2serverIP,Mytools_Rel_Dir
import requests
import matplotlib.pyplot as plt
from common.util import get_moved_xy
from a4loadServer import query_sumo_edge,query_sumo_junction
import traci

addpath = mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)
from geo import get_bearing_latlng2,get_dist_meters_latlng2
from CacheManager import CacheManager

addpath= mypydir+"../zyrcode/" # https://github.com/zyrgit/GreenRouteCode
if addpath not in sys.path: sys.path.append(addpath)
from mygmaps import GoogleMaps
from myosrm import gen_map_html_from_path_list
from common.osrm import get_fuel_given_latlng_list

addpath= mypydir+"../zyrcode/code/"
if addpath not in sys.path: sys.path.append(addpath)
from constants import * # addr2ip

''' Mapping green routes and google's to sumo net.
https://sumo.dlr.de/wiki/Tools/Routes#tracemapper.py
'''

gmaps = GoogleMaps() # need your API key!
_Cache_Dir = "cache/"
mm_nid2latlng= CacheManager(overwrite_prefix=True) # need redis


def gen_edges_tracemapper(traceFile, netFile, routeFile): 
	cmd = SUMO_Tools_dir+"/route/tracemapper.py -v --geo --fill-gaps --delta 10 -n %s -t %s -o %s"%(netFile,traceFile,routeFile)
	print("\n"+cmd)
	subprocess.call(cmd, shell=True)


def fix_edges_routecheck(netFile, routeFile): 
	cmd = SUMO_Tools_dir+"/route/routecheck.py -v --fix --inplace --net %s %s"%(netFile,routeFile)
	print("\n"+cmd)
	subprocess.call(cmd, shell=True)


def interpolate_points(latlng1,latlng2, num=2):# use GPS on edge instead of at intersections.
	res= []
	for i in range(1,1+num):
		lat = latlng2[0]*float(i)/(1+num)+latlng1[0]*float(1+num- i)/(1+num)
		lng = latlng2[1]*float(i)/(1+num)+latlng1[1]*float(1+num- i)/(1+num)
		res.append([lat,lng])
	return res

def random_from_list(lst):
	return lst[random.randint(0,len(lst)-1)]



def get_google_latlngs_given_2latlng(latlng1,latlng2, overwrite=False, show=False, RouteStatsDir=None, index=None, html_show = None, print_res=False):
	key = "g%.5f,%.5f,%.5f,%.5f"%(latlng1[0],latlng1[1],latlng2[0],latlng2[1]) 
	if isinstance(index, int): index = str(index)
	cache_file = _Cache_Dir+key
	if not overwrite and os.path.exists(cache_file):
		with open(cache_file,"r") as f:
			print("read from cache "+cache_file)
			return f.readline()
	ret = gmaps.get_route_given_2latlng(latlng1,latlng2) # [[latlng, dist, duration ],] sparse steps
	# convert google's sparse latlng into dense latlng using osrm server.
	latlngs=[]
	for stt in ret:
		latlngs.append([ stt[0][0],stt[0][1] ])
	nodeslist , addr = [] , config.addr
	get_fuel_given_latlng_list(latlngs, addr, addr2ip[addr], URL_Match, mm_nid2latlng, loose_end_dist_allow=20, nodeslist=nodeslist, print_res=print_res)
	if mm_nid2latlng.get_id()!="osm/cache-%s-nodeid-to-lat-lng.txt"%addr:
		mm_nid2latlng.use_cache(meta_file_name="osm/cache-%s-nodeid-to-lat-lng.txt"%addr, ignore_invalid_mem=True)
	# convert node ids into latlng list
	ret= []
	for nid in nodeslist:
		latlng = mm_nid2latlng.get(nid)
		if latlng is None: print("mm_nid2latlng get None",nid)
		ret.append( [latlng, 0, 0] ) # same format [[latlng, dist, duration ],]

	gps , distances, gmapTrace, sumDist = [],[],"",0.
	lastDx, lastDy = None,None
	for i in range(len(ret)-1): 
		latlng, dist, duration = ret[i]
		latlng2, dist2, duration2 = ret[i+1]
		hd = get_bearing_latlng2(latlng,latlng2)
		sumDist+= get_dist_meters_latlng2(latlng,latlng2)
		lng,lat = get_moved_xy(latlng[1],latlng[0],(hd+90)%360, 0.0000) # 0.0 means not moving
		if lastDx is None: 
			lastDx, lastDy = (lng - latlng[1]),(lat - latlng[0])
		else:
			Dx, Dy = (lng - latlng[1]) , (lat - latlng[0])
			lng,lat = (Dx +lastDx)/2.+latlng[1] , (Dy +lastDy)/2.+latlng[0]
			lastDx, lastDy = Dx, Dy
		gps.append([lat,lng])
		distances.append(dist)
		gmapTrace+= "new google.maps.LatLng(%.5f,%.5f),"%(lat,lng)
	latlng, dist, duration = ret[-1]
	lng,lat = get_moved_xy(latlng[1],latlng[0],(hd+90)%360, 0.0000)
	Dx, Dy = (lng - latlng[1]) , (lat - latlng[0])
	lng,lat = (Dx +lastDx)/2.+latlng[1] , (Dy +lastDy)/2.+latlng[0]
	gps.append([lat,lng])
	distances.append(dist)
	strlist , gmapTrace , path = [],'',[]
	xs , ys = [],[]
	for i in range(len(gps)-1):
		latlng1,latlng2 = gps[i],gps[i+1]
		inter = interpolate_points(latlng1,latlng2, max(2, int(distances[i+1]/30) ) )
		for latlng in inter:
			strlist.append( "%.5f,%.5f"%(latlng[1],latlng[0]) )
			xs.append(latlng[1])
			ys.append(latlng[0])
			gmapTrace+= "new google.maps.LatLng(%.5f,%.5f),"%(latlng[0],latlng[1])
			path.append(latlng)
	strlist.append( "%.5f,%.5f"%(latlng2[1],latlng2[0]) )
	gmapTrace+= "new google.maps.LatLng(%.5f,%.5f),"%(latlng2[0],latlng2[1])
	path.append(latlng2)
	if RouteStatsDir:
		gen_map_html_from_path_list([path], RouteStatsDir+"%s-gg.html"%index , '', disturb=False, right_col_disp_list=[html_show if html_show else "", "Dist %.1f"%sumDist] )
	xs.append(latlng2[1])
	ys.append(latlng2[0])
	if show: 
		plt.plot(xs,ys,'-ko')
		plt.show()
	wstr = " ".join(strlist)
	if overwrite or not os.path.exists(cache_file):
		with open(cache_file,"w") as f:
			f.write(wstr)
	return wstr


def get_green_latlngs_given_2latlng(latlng1,latlng2, overwrite=False, show=False, RouteStatsDir=None, index=None, html_show = None):
	key = "f%.5f,%.5f,%.5f,%.5f"%(latlng1[0],latlng1[1],latlng2[0],latlng2[1]) 
	if isinstance(index, int): index = str(index)
	fn = _Cache_Dir+key
	if not overwrite and os.path.exists(fn):
		with open(fn,"r") as f:
			return f.readline()
	res = requests.post(config.GreenRoute_IP_Port+'/html/route', data = {'startLat':latlng1[0],'startLng':latlng1[1],'endLat':latlng2[0],'endLng':latlng2[1]} ).json()
	ret=[]
	for latlngStr in str(res['gpstrace']).split("~|"):
		st = latlngStr.split(",")
		ret.append( [float(st[0]),float(st[1])] ) 
	gps , gmapTrace, sumDist, lastDx, lastDy = [],'',0., None,None
	for i in range(len(ret)-1): 
		latlng = ret[i]
		latlng2 = ret[i+1]
		sumDist+= get_dist_meters_latlng2(latlng,latlng2)
		gps.append(latlng)
		gmapTrace+= "new google.maps.LatLng(%.5f,%.5f),"%tuple(latlng)
	latlng = ret[-1]
	gps.append(latlng)
	strlist,gmapTrace,path = [],'',[]
	xs , ys = [],[]
	for i in range(len(gps)-1):
		latlng1,latlng2 = gps[i],gps[i+1]
		inter = interpolate_points(latlng1,latlng2, 2)
		for latlng in inter:
			strlist.append( "%.5f,%.5f"%(latlng[1],latlng[0]) )
			xs.append(latlng[1])
			ys.append(latlng[0])
			gmapTrace+= "new google.maps.LatLng(%.5f,%.5f),"%(latlng[0],latlng[1])
			path.append(latlng)
	strlist.append( "%.5f,%.5f"%(latlng2[1],latlng2[0]) )
	gmapTrace+= "new google.maps.LatLng(%.5f,%.5f),"%(latlng2[0],latlng2[1])
	path.append(latlng2)
	if RouteStatsDir:
		gen_map_html_from_path_list([path], RouteStatsDir+"%s-gf.html"%index , '', disturb=False, right_col_disp_list=[html_show if html_show else "", "Dist %.1f"%sumDist] )
	xs.append(latlng2[1])
	ys.append(latlng2[0])
	if show: 
		plt.plot(xs,ys,'-ko')
		plt.show()
	wstr = " ".join(strlist)
	if overwrite or not os.path.exists(fn):
		with open(fn,"w") as f:
			f.write(wstr)
	return wstr


def query_fuel_consumption(latlng1,latlng2):
	startLat,startLng = latlng1
	endLat,endLng = latlng2
	res = requests.post(config.GreenRoute_IP_Port+'/html/fuel', data = {'startLat':startLat,'startLng':startLng,'endLat':endLat,'endLng':endLng,}).json()
	ggas=res["google-fuel"] if "google-fuel" in res else 0.
	fgas=res["green-fuel"] if "green-fuel" in res else 0.
	return {"ggas":ggas, "fgas":fgas}


def sample_OD_edges_from_routes(routeFile, num, accept_rate):
	od = set()
	print("[ sample_OD_edges_from_routes ] "+routeFile)
	with open(routeFile,"r") as f:
		for l in f:
			if not l.lstrip().startswith("<route edges="):
				continue
			if random.random()<accept_rate:
				elist = l.split(' edges="',1)[1].split('"')[0].split(' ')
				od.add( (elist[0],elist[-1]) )
				if len(od)>=num:
					break
	return od


def sample_OD_latlngs_from_routes(routeFile, num=20, accept_rate=1, minDist=1000, maxDist=4000):
	# - accept_rate: prob accepting OD from routes in routeFile= .rou.xml
	# - minDist maxDist: air dist between OD. 
	print("[ sample_OD_latlngs_from_routes ] "+routeFile)
	print(num, accept_rate, minDist, maxDist)
	res = []
	while len(res)<num:
		ode = sample_OD_edges_from_routes(routeFile, num, accept_rate)
		for e1,e2 in ode:
			ret = query_sumo_edge(e1, Server_IP_Port)
			j1,j2 = ret['to'],ret['from']
			ret1 = query_sumo_junction(j1, Server_IP_Port)
			ret2 = query_sumo_junction(j2, Server_IP_Port)
			latlng1 = [ (ret1['lat']+ret2['lat'])/2 , (ret1['lng']+ret2['lng'])/2 ]# mid of edge 
			ret = query_sumo_edge(e2, Server_IP_Port)
			j1,j2 = ret['to'],ret['from']
			ret1 = query_sumo_junction(j1, Server_IP_Port)
			ret2 = query_sumo_junction(j2, Server_IP_Port)
			latlng2 = [ (ret1['lat']+ret2['lat'])/2 , (ret1['lng']+ret2['lng'])/2 ]
			dist = get_dist_meters_latlng2(latlng1 , latlng2)
			if dist>= minDist and dist<= maxDist:
				print("sampled frE %s to %s"%(e1,e2)+ " air-dist %.1f "%dist)
				res.append( [latlng1 , latlng2] )
				if len(res)==num: break
	return res


def fix_dist_mismatch_after_fix(TracemapperOutEdgesFile,commonIndSet):
	rprefix = ["gg","gf"]
	rpref2ind2edges = {}
	tmpRouteFile = TracemapperOutEdgesFile.rstrip(".xml")+".fix.xml"
	of = open(tmpRouteFile,"w")
	of.write('<?xml version="1.0"?>\n<routes>\n')
	with open(TracemapperOutEdgesFile,"r") as f:
		f.seek(0,2)
		endBytes = f.tell()
		f.seek(0,0)
		while True:
			l = f.readline().strip()
			if l=='' and f.tell()==endBytes: break
			if l.startswith('<route id="'):
				l = l.split('<!--',1)[0] # contains commented line parts.
				rid = l.split(' id="',1)[1].split('" ')[0]
				pref = rid[0:2]
				ind = int(rid[2:])
				if ind in commonIndSet:
					if pref not in rpref2ind2edges: rpref2ind2edges[pref]={}
					rpref2ind2edges[pref][ind] = l.split(' edges="',1)[1].rstrip('"/>').split(' ')# overwrite duplicated route id with later ones.
	for ind in sorted(rpref2ind2edges[pref].keys()):
		ge = rpref2ind2edges[rprefix[0]][ind]
		fe = rpref2ind2edges[rprefix[1]][ind]
		for i in range(0,3):# fix begin
			if ge[0]==fe[i]:
				rpref2ind2edges[rprefix[1]][ind] = fe[i:]
				break
		ge = rpref2ind2edges[rprefix[0]][ind]
		fe = rpref2ind2edges[rprefix[1]][ind]
		for i in range(len(fe)-1,len(fe)-3,-1): # fix end
			if ge[-1] ==fe[i]:
				rpref2ind2edges[rprefix[1]][ind] = fe[0:i+1]
				break
		of.write('    <route id="'+rprefix[1]+str(ind)+'" edges="'+' '.join(rpref2ind2edges[rprefix[1]][ind])+'"/>\n')
		of.write('    <route id="'+rprefix[0]+str(ind)+'" edges="'+' '.join(rpref2ind2edges[rprefix[0]][ind])+'"/>\n')
	of.write("</routes>\n")
	of.close()



def get_gf_common_valid_route_inds_after_fix(TracemapperOutEdgesFile):
	print("[ get_gf_common_valid_route_inds_after_fix ] %s"%TracemapperOutEdgesFile)
	gset=set()
	fset=set()
	r2set={"gf":fset, "gg":gset}
	with open(TracemapperOutEdgesFile,"r") as f:
		for l in f:
			l=l.strip()
			if l.startswith("<route id="):
				rid = l.split(' id="',1)[1].split('" ',1)[0]
				r2set[rid[0:2]].add(int(rid[2:])) # will dedup.
	common = fset & gset
	print("common indices: ",common)
	return common


def replace_ori_rou_with_gf_common_routes(RouteFile, split_index, vprefs_to_replace, commonIndSet, replace_prob, RouteStatsDir, RouteFileHasEdgesWithin):
	# - RouteFileHasEdgesWithin: if rou.xml has '<edges' inside each vehicle route.
	replacedVid2Rid = {}
	for prefix  in  vprefs_to_replace: # replace /0-a.xml and 0-b.xml ...
		if split_index is not None:
			ori_route_file = RouteFile % (split_index , prefix)
		else:
			ori_route_file = RouteFile % (prefix)
		NewRouteFile = ori_route_file.rstrip(".xml")+".repcp.xml"
		random.seed(2) # -a -b should have same replacement
		rIndList , rcnt = list(commonIndSet) , 0 # round robin selection
		rprefix = ["gf","gg",""] # greenroute, google's, duarouter's. 
		rtypes = range(len(rprefix))  
		print("[ replace_ori_rou_with_gf_common_routes ] gen "+ NewRouteFile)
		of = open(NewRouteFile,"w")
		ori_num , rep_num = 0,0

		with open(ori_route_file,"r") as f: # assume ori format: <veh ..> \n <edges ..> \n <veh/> \n
			f.seek(0,2)
			endBytes = f.tell()
			f.seek(0,0)
			while True:
				l = f.readline()
				if l=='' and f.tell()==endBytes: break
				ll = l.strip()
				if ll.startswith('<vehicle id='):
					do_replace = random.random() < replace_prob
					if do_replace:
						ori_num += len(rtypes)
						rind = rIndList[ rcnt%len(rIndList) ]
						rcnt+=1
						for rtype in rtypes:
							rid = rprefix[rtype] +str(rind)
							try:
								vid = ll.split(' id="',1)[1].split('" ',1)[0]
							except:
								if ll == '</routes>':
									break
								else:
									print(l,ll,"bad line?")
									sys.exit(1)
							replacedVid2Rid[vid]= rid

							print("Replacing",vid,rid)
							rep_num +=1
							if RouteFileHasEdgesWithin: 
								newl = ll.rstrip(">")+' route="'+rid+'">\n'
							else:
								newl = ll.rsplit(' route=',1)[0]+' route="'+rid+'">\n'
							of.write(newl)
							if RouteFileHasEdgesWithin: l = f.readline() # skip edges def.
							l = f.readline() # end of vehicle/> tag
							of.write(l)
							l = f.readline() # read new vehicle id= line. or end </route> tag...
							if l=='' and f.tell()==endBytes: break
							ll = l.strip()
						of.write(l)
					else:
						ori_num += 1
						of.write(l)
				else:
					of.write(l)
		of.close()
	print("ori_num",ori_num,'rep_num',rep_num,"common#",len(commonIndSet))
	if RouteStatsDir:
		print('replaced Vid#',len(replacedVid2Rid))
		pickle.dump(replacedVid2Rid, open(RouteStatsDir+"0replacedVid2Rid-"+str(split_index),"wb"))
	return replacedVid2Rid



def gen_trace_file(outTraceFile, RouteStatsDir, index, latlng1,latlng2, overwrite=False, show=True):
	# - overwrite: ignore trace cache in ./cache/g*, thus gen new html file.
	if not os.path.exists(RouteStatsDir): os.makedirs(RouteStatsDir)
	if isinstance(index, int): index = str(index)
	theirFuel = query_fuel_consumption(latlng1,latlng2)
	gg = get_google_latlngs_given_2latlng(latlng1,latlng2,overwrite,show,RouteStatsDir,index,html_show="gas=%.1f"%theirFuel['ggas'], print_res=False)
	gf = get_green_latlngs_given_2latlng(latlng1,latlng2,overwrite,show,RouteStatsDir,index,html_show="gas=%.1f"%theirFuel['fgas'])
	with open(outTraceFile,"a") as f:
		f.write("gg%s"%index+":%s\n"%gg)
		f.write("gf%s"%index+":%s\n"%gf)
	fn = RouteStatsDir+"%s-gg.txt"%index
	with open(fn,"w") as f:
		f.write("gas=%.1f"%theirFuel['ggas'])
	fn = RouteStatsDir+"%s-gf.txt"%index
	with open(fn,"w") as f:
		f.write("gas=%.1f"%theirFuel['fgas'])


def gen_compare_edges_from_routes(routeFile,GpsTraceFile,RouteStatsDir, num, accept_rate, minDist, maxDist, overwrite_cache=False):
	# - accept_rate: prob accepting od from routes scanning in .rou.xml
	# get previous max index:
	flist = glob.glob(RouteStatsDir+"*")
	prevMax , nums = None , []
	for fn in flist:
		name = fn.rsplit(os.sep,1)[1].split("-",1)[0] # find previos max index to use 
		if name[0].isdigit():
			nums.append(int(name))
	if len(nums)>0:
		prevMax = max(nums)
	else:
		print(flist)
		prevMax = int(raw_input("\nNo Previous comparison routes, Enter start index: "))
	print("[ gen_compare_edges_from_routes ] checking "+RouteStatsDir)
	print("Previously comparison route index upto", prevMax)
	od = sample_OD_latlngs_from_routes(routeFile, num=num, accept_rate=accept_rate,minDist=minDist,maxDist=maxDist)
	print("[ gen_compare_edges_from_routes ] input OD #%d"%len(od))
	cnt=0
	for latlng1,latlng2 in od:
		cnt+=1
		gen_trace_file(GpsTraceFile, RouteStatsDir, cnt+prevMax, latlng1,latlng2, overwrite=overwrite_cache, show=False)




def sample_OD_edges_from_indexed_routes(RoutesEdgesDefXmlFile, num, accept_rate):
	od = {}
	print("[ sample_OD_edges_from_indexed_routes ] "+RoutesEdgesDefXmlFile)
	with open(RoutesEdgesDefXmlFile,"r") as f:
		for l in f:
			if not l.lstrip().startswith("<route id="):
				continue
			if random.random()<accept_rate:
				rid = l.split(' id="',1)[1].split('"',1)[0]
				elist = l.split(' edges="',1)[1].split('"')[0].split(' ')
				od[rid]= (elist[0],elist[-1]) 
				if len(od)>=num:
					break
	return od


def sample_OD_latlngs_from_indexed_routes(RoutesEdgesDefXmlFile, num, accept_rate,minDist,maxDist):
	print("[ sample_OD_latlngs_from_routes ] "+RoutesEdgesDefXmlFile)
	print(num, accept_rate, minDist, maxDist)
	res = {}
	while len(res)<num:
		oddic = sample_OD_edges_from_indexed_routes(RoutesEdgesDefXmlFile, num, accept_rate)
		for rid in oddic.keys():
			e1,e2 = oddic[rid]
			ret = query_sumo_edge(e1, Server_IP_Port)
			j1,j2 = ret['to'],ret['from']
			ret1 = query_sumo_junction(j1, Server_IP_Port)
			ret2 = query_sumo_junction(j2, Server_IP_Port)
			latlng1 = [ 0.1*ret1['lat']+ 0.9*ret2['lat'] , 0.1*ret1['lng']+ 0.9*ret2['lng'] ]
			ret = query_sumo_edge(e2, Server_IP_Port)
			j1,j2 = ret['to'],ret['from']
			ret1 = query_sumo_junction(j1, Server_IP_Port)
			ret2 = query_sumo_junction(j2, Server_IP_Port)
			latlng2 = [ 0.9*ret1['lat']+ 0.1*ret2['lat'] , 0.9*ret1['lng']+ 0.1*ret2['lng'] ]
			dist = get_dist_meters_latlng2(latlng1 , latlng2)
			if dist>= minDist and dist<= maxDist:
				print("sampled frE %s to %s"%(e1,e2)+ " air-dist %.1f "%dist)
				res[rid]= [latlng1 , latlng2] 
				if len(res)==num: break
	return res


def gen_trace_from_indexed_routes(RoutesEdgesDefXmlFile, IndGpsTraceFile,IndRouteStatsDir, num,accept_rate,minDist, maxDist, overwrite_cache=False):
	i2odgps = sample_OD_latlngs_from_indexed_routes(RoutesEdgesDefXmlFile, num, accept_rate,minDist,maxDist)
	print("[ gen_trace_from_indexed_routes ] input OD #%d"%len(i2odgps))
	if not os.path.exists(IndRouteStatsDir): os.makedirs(IndRouteStatsDir)
	for rid  in i2odgps.keys():  
		latlng1,latlng2 = i2odgps[rid]
		gen_trace_file(IndGpsTraceFile, IndRouteStatsDir, rid, latlng1,latlng2, overwrite=overwrite_cache, show=False)



if __name__ == "__main__":
	random.seed(2) 
	sample_routes_pool_file = RoutesEdgesDefXmlFile # Use routes/routes.xml  rid->edges.

	# insert google's/green route in  ori route files.
	ind_replaced_with_compared_routes = [0,1] # routes/"x-%s.xml"  -> "x-%s.repcp.xml"
	
	route_file_format_to_replace = SplitRoutesDir + "%d-%s.xml"

	vprefs_to_replace = ['a','b']

	if "genroutes" in sys.argv: # get edge OD pairs and route traces 
		# gen_compare_edges_from_routes(sample_routes_pool_file,GpsTraceFile,RouteStatsDir, num=300, accept_rate=0.1, minDist = 500, maxDist=8000, overwrite_cache=False ) # old version.
		gen_trace_from_indexed_routes(sample_routes_pool_file, IndGpsTraceFile,IndRouteStatsDir, 600,0.1,1500, 8000, overwrite_cache=False)

	if 'tracemapper' in sys.argv:  # gen edges and match to sumo.
		gen_edges_tracemapper(IndGpsTraceFile, AddTlsNetfile, TracemapperOutEdgesFile) 
		fix_edges_routecheck(AddTlsNetfile, TracemapperOutEdgesFile)
		common = get_gf_common_valid_route_inds_after_fix(TracemapperOutEdgesFile)
		fix_dist_mismatch_after_fix(TracemapperOutEdgesFile, common)

	if 'replaceroutes' in sys.argv:  # replace ori-rou with compared rids.
		common = get_gf_common_valid_route_inds_after_fix(TracemapperOutEdgesFile)
		for splitInd in ind_replaced_with_compared_routes:
			replace_ori_rou_with_gf_common_routes(route_file_format_to_replace, splitInd, vprefs_to_replace, common, replace_prob=0.2, RouteStatsDir=IndRouteStatsDir, RouteFileHasEdgesWithin=False) # will replace original 


