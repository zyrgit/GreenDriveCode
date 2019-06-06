#!/usr/bin/env python
import numpy as np
import operator
import math
import datetime, pprint
import requests
try:
	from geopy.distance import great_circle
	from geopy.geocoders import Nominatim
except: print("RUN $HOME/anaconda2/bin/pip install geopy")
try:
	import overpass
	overpass_api = overpass.API()
except: print("\nRUN $HOME/anaconda2/bin/pip install overpass\n")

def get_dist_meters_latlng2(latlon1, latlon2):
	return get_dist_meters_latlng(latlon1[0] , latlon1[1] , latlon2[0] , latlon2[1])
def get_dist_meters_latlng(lat1, lon1, lat2, lon2):
	return great_circle((lat1, lon1), (lat2, lon2)).meters

def query_obj_given_id_list(what,idlist):
	idlist=[str(x) for x in idlist]
	if what.startswith("node"):
		response = overpass_api.Get('node(id:%s);'%(",".join(idlist)))
	elif what.startswith("way"):
		response = overpass_api.Get('way(id:%s);'%(",".join(idlist)))
	elif what.startswith("rel"):
		response = overpass_api.Get('rel(id:%s);'%(",".join(idlist))) # not reliable. missing.
	return response

def crawl_nid_to_latlng(nid, print_str=False, silent=False):
	if not silent: print('[ crawl_nid_to_latlng ] %d'%nid)
	res = query_obj_given_id_list('node',[nid])
	if print_str: pprint.pprint(res)
	try:
		lnglat = res["features"][0]["geometry"]["coordinates"]
	except:
		print("[myosrm] crawl_nid_to_latlng() err: query_obj_given_id_list return",res)
		print(nid," node not found by overpy!!! ")
	return [lnglat[1],lnglat[0]]


# copied from zyrcode/costModule due to broken imports. Used for getting node list.
def get_fuel_given_latlng_list(latlngs,addr, backend, URL_Format, mm_nid2latlng, loose_end_dist_allow=80, nodeslist=[], print_res=False):
	gas=0.
	routeUrl = get_route_url_given_latlng_list(latlngs,addr, 1, backend, URL_Format, print_res=print_res)
	ret = requests.get(routeUrl).json()
	if print_res and 0: pprint.pprint(ret)
	gas+= collect_duration_nodelist_given_json(ret, nodeslist)  #nodeslist: to check result
	if mm_nid2latlng.get_id()!="osm/cache-%s-nodeid-to-lat-lng.txt"%addr:
		mm_nid2latlng.use_cache(meta_file_name="osm/cache-%s-nodeid-to-lat-lng.txt"%addr, ignore_invalid_mem=True)
	''' Check ----------- cases where first latlng is not matched '''
	try:
		firstLatlng = mm_nid2latlng.get(nodeslist[0]) or crawl_nid_to_latlng(nodeslist[0],silent=True) # in case of None
	except:
		print("Node",nodeslist[0],"does not exist in either mm or overpy! Try 2nd node...")
		firstLatlng = mm_nid2latlng.get(nodeslist[1]) or crawl_nid_to_latlng(nodeslist[1],silent=True)
	dist = get_dist_meters_latlng2(latlngs[0],firstLatlng)
	if dist > loose_end_dist_allow:
		if print_res: print("Miss matched 1st, dist",dist)
		routeUrl= get_route_url_given_latlng_list([latlngs[0],firstLatlng],addr, 1, backend, URL_Format, print_res=print_res)
		ret = requests.get(routeUrl).json()
		tmpl=[]
		gas+= collect_duration_nodelist_given_json(ret, tmpl)
		nodeslist=tmpl+nodeslist
	''' Check ----------- cases where last latlng is not matched '''
	try:
		lastLatlng = mm_nid2latlng.get(nodeslist[-1]) or crawl_nid_to_latlng(nodeslist[-1],silent=True)# in case of None
	except:
		print("Node",nodeslist[-1],"does not exist in either mm or overpy! Try -2nd node...")
		lastLatlng = mm_nid2latlng.get(nodeslist[-2]) or crawl_nid_to_latlng(nodeslist[-2],silent=True) 
	dist = get_dist_meters_latlng2(latlngs[-1],lastLatlng)
	if dist> loose_end_dist_allow:
		if print_res: print("Miss matched end, dist",dist)
		routeUrl= get_route_url_given_latlng_list([lastLatlng,latlngs[-1]],addr,1, backend, URL_Format, print_res=print_res)
		ret = requests.get(routeUrl).json()
		tmpl=[]
		gas+= collect_duration_nodelist_given_json(ret, tmpl)
		nodeslist=nodeslist+tmpl
	if print_res :print(nodeslist)
	return gas



LngFirst = 1 # osrm url/ret is lng,lat format 

def get_route_url_given_latlng_list(latlngs,addr, insert_every_num, backend, URL_Format, print_res=False):
	''' Return URL with via waypoints in it, given a list of latlngs. 
	GET http://0.0.0.0:5000/route/v1/driving/-88.22219,40.114936;-88..,40..;-88.235836,40.101568?annotations=true&steps=true
	- insert_every_num: insert mid way point every this gps pts.
	'''
	dist = get_dist_meters_latlng2(latlngs[0],latlngs[-1])
	pcnt=0
	i=0
	locs=""
	while i< len(latlngs):
		if i==len(latlngs)-1:
			locs+=str(latlngs[i][LngFirst])+","+str(latlngs[i][1-LngFirst])
		elif i%insert_every_num==0:
			locs+=str(latlngs[i][LngFirst])+","+str(latlngs[i][1-LngFirst])+";"
			pcnt+=1
		i+=1
	distPerPt= dist/pcnt
	routeUrl = URL_Format.format(Backend=backend,Loc=locs)
	routeUrl+= '&steps=true'
	if print_res: print("[ route Url ] "+routeUrl)
	return routeUrl



def collect_duration_nodelist_given_json(ret, nodeslist=[]):
	''' Gather duration/fuel of either route/match returned, in-place append to nodeslist. '''
	gas=0.
	if 'routes' in ret:
		for tmp in ret['routes']:
			gas += tmp['duration']
			legs= tmp['legs']
			for rt in legs:
				nodes = rt['annotation']['nodes']
				nodeslist.extend(nodes)
	elif 'matchings' in ret:  
		for tmp in ret['matchings']:
			gas += tmp['duration']
			numMatch=len(ret["matchings"])
			for m in range(numMatch):
				matchpoints=ret["matchings"][m]["legs"]
				NumAnnotations=len(matchpoints)
				for i in range(NumAnnotations):
					nlist=matchpoints[i]["annotation"]["nodes"]
					nodeslist.extend(nlist)
	return gas # duration of osrm is gas. 

