#!/usr/bin/env python
from common import *
from configure.assemble import *
from configure.params import region2serverIP, Mytools_Rel_Dir
from netcode.xml import *
from netcode.func import *
from common.osrm import get_dist_meters_latlng2
import pprint
import requests

addpath=mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)

iprint = 1 

''' ---------- public API -------------'''

def query_osm_node(nid, IP_Port):
	try: ret= requests.post(IP_Port+'/data/osm_nodes', data = {'id':nid}).json()
	except: ret=None
	return ret #id: {'lat': 40.129513, 'lon': -88.23884 }

def query_sumo_junction(jid, IP_Port):
	try: ret= requests.post(IP_Port+'/data/sumo_junctions', data = {'id':jid}).json()
	except: ret=None
	return ret # id: {'y': 3772.19, 'incLanes': '-182612159#5_0 -182612159#5_1 138670024#4_0 138670024#4_1 182612159#2_0 182612159#2_1 5341711#0_0', 'lat': 40.14264823240558, 'x': 140.17, 'lng': -88.25791933008874, 'type': 'traffic_light'} 

def query_sumo_edge(eid, IP_Port):
	try: ret= requests.post(IP_Port+'/data/sumo_edges', data = {'id':eid}).json()
	except: ret=None
	return ret # id: {'priority': 4, 'from':'38048823', 'type':'highway.residential', 'to':'38002584', 'heading': 90}

def query_sumo_connection(edgeTuple, IP_Port):
	if isinstance(edgeTuple,list):
		edgeTuple = tuple(edgeTuple)
	try: ret= requests.post(IP_Port+'/data/sumo_connections', data = {'id':edgeTuple }).json()
	except: ret=None
	return ret # tuple: [ {'to': '138670027#6', 'tl': 'cluster_1520525197_38075425', 'via': ':cluster_1520525197_38075425_11_0', 'from': '138670027#5', 'dir': 's', 'linkIndex': 11, 'fromLane': '0', 'toLane': '0'},      {'to': '138670027#6', 'tl': 'cluster_1520525197_38075425', 'via': ':cluster_1520525197_38075425_11_1', 'from': '138670027#5', 'dir': 's', 'linkIndex': 12, 'fromLane': '1', 'toLane': '1'} ]

def query_sumo_out_edges(fromEdge, IP_Port):
	try: ret= requests.post(IP_Port+'/data/sumo_out_edges', data = {'from':fromEdge}).json()
	except: ret=None
	return ret # fromEdge: {'-5342274#3': [{'via': ':1349435035_0_0', 'dir': 's', 'fromLane': '0', 'toLane': '0'}, {'via': ':1349435035_0_1', 'dir': 's', 'fromLane': '0', 'toLane': '1'}]}


''' -------- load osm nodes from osm file -------------'''

def load_osm_nodes(fname):
	if iprint: print("[ load_osm_nodes ] loading "+fname)
	osm_nodes = dict()
	for dic in get_osm_nodes(fname):
		osm_nodes[dic['id']] = {'lat': dic['lat'], 'lng': dic['lon'] }
	if iprint: print("%d nodes loaded."%len(osm_nodes))
	return osm_nodes
	#  {id: {'lat': 40.129513, 'lon': -88.23884 } ..}


''' -------- load sumo * from final net file -------------'''

def load_sumo_juntions(fname, netObj=None):
	if iprint: print("[ load_sumo_juntions ] loading "+fname)
	if netObj is None:
		netObj = sumolib.net.readNet(fname)
	sumo_junctions = dict()
	for node in get_xml_junction(fname):
		lat,lng = get_latlng_given_xy([node['x'],node['y']] , netObj)
		sumo_junctions[node['id']] = {'lat': lat, 'lng':lng, 'x':node['x'], 'y':node['y'], 'type':node['type'], 'incLanes':node['incLanes'], }
	if iprint: print("%d junctions loaded."%len(sumo_junctions))
	return sumo_junctions
	#  {id: {'y': 3772.19, 'incLanes': '-182612159#5_0 -182612159#5_1 138670024#4_0 138670024#4_1 182612159#2_0 182612159#2_1 5341711#0_0', 'lat': 40.14264823240558, 'x': 140.17, 'lng': -88.25791933008874, 'type': 'traffic_light'} ..}


def load_sumo_edges(fname, sumo_junctions=None):
	if iprint: print("[ load_sumo_edges ] loading "+fname)
	sumo_edges=dict()
	for elem in get_xml_edge(fname):
		sumo_edges[elem['id']] = {'type':elem['type'],'from':elem['from'],'to':elem['to'],'priority':elem['priority'], }
		# get edge direction/angle
		if sumo_junctions:
			dic = sumo_edges[elem['id']]
			j1,j2 = dic['from'],dic['to']
			lat1,lng1 = sumo_junctions[j1]['lat'],sumo_junctions[j1]['lng']
			lat2,lng2 = sumo_junctions[j2]['lat'],sumo_junctions[j2]['lng']
			heading = get_bearing_given_lat_lng(lat1,lng1,lat2,lng2)
			airdist = get_dist_meters_latlng2([lat1,lng1],[lat2,lng2])
			dic['heading']=heading
			dic['airdist']=airdist
	if iprint: print("%d edges loaded."%len(sumo_edges)) 
	return sumo_edges
	#  {eid: {'priority': 4, 'from':'38048823', 'type':'highway.residential', 'to':'38002584', 'heading': 90, 'airdist':1, } ..}


#https://sumo.dlr.de/wiki/Networks/SUMO_Road_Networks#Connections
def load_sumo_connections(fname):
	if iprint: print("[ load_sumo_connections ] loading "+fname)
	sumo_connections=dict()
	for elem in get_xml_connection(fname):
		tup = (elem['from'],elem['to'])
		if tup not in sumo_connections: sumo_connections[tup]=[]
		sumo_connections[ tup ].append( {'fromLane':elem['fromLane'],'toLane':elem['toLane'],'from':elem['from'],'to':elem['to'],'dir':elem['dir'],'via':elem['via'], 'tl':elem['tl'],'linkIndex':elem['linkIndex'], }
			)
	if iprint: print("%d e2e connections loaded."%len(sumo_connections)) 
	return sumo_connections
	# {tuple: [ {'to': '138670027#6', 'tl': 'cluster_1520525197_38075425', 'via': ':cluster_1520525197_38075425_11_0', 'from': '138670027#5', 'dir': 's', 'linkIndex': 11, 'fromLane': '0', 'toLane': '0'},      {'to': '138670027#6', 'tl': 'cluster_1520525197_38075425', 'via': ':cluster_1520525197_38075425_11_1', 'from': '138670027#5', 'dir': 's', 'linkIndex': 12, 'fromLane': '1', 'toLane': '1'} ] ..}


''' -------- calculate other data structures -------------'''

def load_sumo_out_edges(sumo_connections):
	if iprint: print("[ load_sumo_out_edges ] using sumo_connections ")
	sumo_out_edges=dict()
	for tup,lst in sumo_connections.items():
		fromEd,toEd = tup
		if fromEd not in sumo_out_edges: sumo_out_edges[fromEd]={}
		dic = sumo_out_edges[fromEd]
		if toEd not in dic: dic[toEd]=[]
		tolist = dic[toEd]
		for linkdic in lst:
			tmp = {'via':linkdic['via'],'dir':linkdic['dir'], 'fromLane':linkdic['fromLane'], 'toLane':linkdic['toLane'], }
			tolist.append(tmp)
	if iprint: print("%d e2out loaded."%len(sumo_out_edges)) 
	return sumo_out_edges
	#  { fromEdge: {'-5342274#3': [{'via': ':1349435035_0_0', 'dir': 's', 'fromLane': '0', 'toLane': '0'}, {'via': ':1349435035_0_1', 'dir': 's', 'fromLane': '0', 'toLane': '1'}]}, .. } 
	# - dir: enum ("s" = straight, "t" = turn, "l" = left, "r" = right, "L" = partially left, R = partially right, "invalid" = no direction)




if __name__ == "__main__":
	if 0:
		load_osm_nodes(DownloadedOsmFile)
		sumo_junctions=load_sumo_juntions(AddTlsNetfile)
		load_sumo_edges(AddTlsNetfile, sumo_junctions)
		load_sumo_connections(AddTlsNetfile)

	SERVER_ADDR="0.0.0.0" # instead, see configure/params.py
	SERVER_PORT = 9875
	Server_IP_Port = "http://"+SERVER_ADDR+":"+str(SERVER_PORT)

	# raw tests 
	if 0: print requests.post(Server_IP_Port+'/data/osm_nodes', data = {'id':"27440217"}).json()
	if 0: print requests.post(Server_IP_Port+'/data/sumo_junctions', data = {'id':"cluster_38064726_38064727"}).json()
	if 0: print requests.post(Server_IP_Port+'/data/sumo_edges', data = {'id':"-142289617#0"}).json()
	if 0: print requests.post(Server_IP_Port+'/data/sumo_connections', data = {'id':('-120327301#2','-5342274#3') }).json()
	if 0: print requests.post(Server_IP_Port+'/data/sumo_out_edges', data = {'from':'-120327301#2'}).json()

	if 0: print query_sumo_connection(("-435381809#1","-116025906#2"))
	if 0: print query_sumo_junction("cluster_740237201_740237202_740237231")


