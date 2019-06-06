#!/usr/bin/env python

from .dtype import Str_to_type
import sumolib


def get_osm_nodes(fname, ): # get node osm file.
	for elem in sumolib.output.parse_fast(fname, 'node', ['id', 'lat', 'lon']):
		yield {'id':Str_to_type['idstr'](elem.id), 'lat':Str_to_type['lat'](elem.lat), 'lon':Str_to_type['lon'](elem.lon), }



def get_location_field_from_net_file(fname):
	for elem in sumolib.output.parse_fast(fname, 'location', ['netOffset','convBoundary', 'origBoundary','projParameter']):
		return {'netOffset':elem.netOffset,'convBoundary':elem.convBoundary,'origBoundary':elem.origBoundary,'projParameter':elem.projParameter,}


def get_xml_conv_ori_bbox(fname): # fname: netconvert generated file.
	for elem in sumolib.output.parse_fast(fname, 'location', ['convBoundary', 'origBoundary']):
		# location(convBoundary='0.00,0.00,3773.95,3908.75', origBoundary='-88.258994,40.109101,-88.214991,40.143998')
		# wsen
		st=elem.convBoundary.split(',')
		cw,cs,ce,cn = list(map(lambda x: float(x),st))
		st=elem.origBoundary.split(',')
		ow,os,oe,on = list(map(lambda x: float(x),st))
		return {"cw":cw,"cs":cs,"ce":ce,"cn":cn,"ow":ow,"os":os,"oe":oe,"on":on}


# parse_fast(): The element must be on its own line and the attributes must appear in the given order.
def get_xml_edge(fname, num=-1): # from net file
	for elem in sumolib.output.parse_fast(fname, 'edge', ['id','from','to','priority','type']):
		yield {'id':Str_to_type['idstr'](elem.id), 'type':Str_to_type['type'](elem.type), 'from':Str_to_type['from'](elem.attr_from),'to':Str_to_type['to'](elem.to),'priority':Str_to_type['priority'](elem.priority),}
		if num>=0:
			num-=1
			if num==0: return


def get_xml_junction(fname, num=-1): # get node from netconvert generated file.
	for elem in sumolib.output.parse_fast(fname, 'junction', ['id', 'type', 'x','y','incLanes']):
		yield {'id':Str_to_type['idstr'](elem.id), 'type':Str_to_type['type'](elem.type), 'x':Str_to_type['x'](elem.x),'y':Str_to_type['y'](elem.y),'incLanes':Str_to_type['incLanes'](elem.incLanes),}
		if num>=0:
			num-=1
			if num==0: return


def get_xml_connection(fname, num=-1): # from net file
	# vary attr, cannot parse_fast(), can handle missing attrs.
	for elem in sumolib.output.parse(fname, 'connection', element_attrs={'connection':['to','tl','via','from','dir','linkIndex','fromLane','toLane']}): 
		tmp = {'fromLane':Str_to_type['fromLane'](elem.fromLane), 'dir':Str_to_type['dir'](elem.dir), 'from':Str_to_type['from'](elem.attr_from),'to':Str_to_type['to'](elem.to),'toLane':Str_to_type['toLane'](elem.toLane),'via':Str_to_type['via'](elem.via),}
		tmp['tl'] = Str_to_type['tl'](elem.tl) if elem.tl else None
		tmp['linkIndex'] = Str_to_type['linkIndex'](elem.linkIndex) if elem.linkIndex else None
		yield tmp
		if num>=0:
			num-=1
			if num==0: return


