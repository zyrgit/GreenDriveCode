#!/usr/bin/env python
from common import *
from a4loadServer import query_sumo_connection,query_sumo_edge,query_sumo_junction
from configure.params import Mytools_Rel_Dir,region2serverIP
from configure.assemble import *

mypydir =os.path.abspath(os.path.dirname(inspect.getfile(inspect.currentframe())))  
addpath= mypydir+ Mytools_Rel_Dir
from mem import Mem

config = None
traci = None

# Redis cache on port 6380
mm_sumo = Mem({"prefix":"s!", "expire": 864000, "use_ips":[region2serverIP[R['addr']]['host']] }) 
mm_sumo.set("test",1)
_mm_is_good = 0
if mm_sumo.get('test')==1: 
	print("Mem good on "+region2serverIP[R['addr']]['host'])
	_mm_is_good = 1
else: raise Exception("Mem fail ... "+region2serverIP[R['addr']]['host'])


def construct_edge(eid, isInternal, turn="", allViaLanes=None, tlsid=None, internalNode=None): 
	if not isInternal:
		edgeDic1 = query_sumo_edge(eid, Server_IP_Port)
		if edgeDic1 is None: print('\n   server.py not working ! ...')
		heading = edgeDic1['heading']
		ndDic = query_sumo_junction(edgeDic1["to"], Server_IP_Port)
		node1 = Node(str(edgeDic1["to"]), x=ndDic["x"],y=ndDic["y"], lat=ndDic["lat"],lng=ndDic["lng"], incLanes=ndDic["incLanes"], ntype=ndDic["type"], tlsid=tlsid)
		edge = Edge(eid, heading , endNode=node1, priority=edgeDic1['priority'] ) # it is an edge.
	else:
		# Internal: https://sumo.dlr.de/wiki/Networks/SUMO_Road_Networks#Internal_Edges
		edge = Edge(eid.rsplit("_",1)[0].rsplit("_",1)[0], turn=turn, endNode=internalNode, internalLaneID=eid, priority=0) # eid=lane id.
		edge.set_allViaLanes(allViaLanes)
	return edge


def construct_segment(routeEdges, e2tls, route_id=None, print_str=False, overwrite_cache=False, prefix=''):
	if not overwrite_cache and route_id is not None and _mm_is_good:
		route = mm_sumo.get(prefix+route_id)
		if route is not None:
			return route
	route = Route()
	sid, i = 0, 0
	seg = Segment(sid)
	while i < len(routeEdges)-1:
		e1,e2 = routeEdges[i],routeEdges[i+1]
		edge = construct_edge(e1, False, tlsid=e2tls[e1].id if e1 in e2tls else None )
		con = query_sumo_connection((e1,e2), Server_IP_Port)
		tid, via, turn = con[0]['tl'], str(con[0]['via']), str(con[0]['dir'])
		internals = set()
		for dic in con:
			internals.add(str(dic['via']))
		viaEdge = construct_edge(via, True, turn=turn, allViaLanes=internals, internalNode= edge.endNode)
		if tid:
			seg.append_and_link_edge(edge)
			route.append_and_link_segment(seg)
			sid+=1
			seg = Segment(sid)
			seg.append_and_link_edge(viaEdge)
		else:
			seg.append_and_link_edge(edge)
			seg.append_and_link_edge(viaEdge)
		if i==len(routeEdges)-2:
			edge2 = construct_edge(e2, False, tlsid=e2tls[e2].id if e2 in e2tls else None )
			seg.append_and_link_edge(edge2)
			route.append_and_link_segment(seg)
		i+=1
	route.construct_connection(print_str=print_str)
	if route_id is not None and _mm_is_good: mm_sumo.set(prefix+ route_id,route)
	return route


class Node: # junction
	def __init__(self, nid, **kwargs):
		self.id = nid
		self.x, self.y = kwargs.get("x",0), kwargs.get("y",0)
		self.lat, self.lng = kwargs.get("lat",0), kwargs.get("lng",0)
		self.incLanes = kwargs.get("incLanes","").split(" ")
		self.type = kwargs.get("ntype","")
		self.inEdges = set([lane.rsplit("_",1)[1] for lane in self.incLanes])
		self.is_tls = self.type=='traffic_light'
		self.is_allway_stop = self.type=='allway_stop'
		self.is_priority_stop = self.type=='priority_stop'
		if self.is_tls: self.tlsid = kwargs.get("tlsid",None)
	def __eq__(self, other):
		if self.id == other.id: return True
		return False
	def __ne__(self, other):
		return not self.__eq__(other)
	def __repr__(self):
		return "%s,%s"%(self.id,self.type)  


class Lane: 
	def __init__(self, lid, index, edge, length, speedLimit):
		self.id = lid
		self.ind = index # 0 is right most. inCross always=0.
		self.edge = edge
		self.len = length # could be wrong when inCross
		self.vlimit = speedLimit
	def __eq__(self, other):
		if self.id == other.id: return True
		return False
	def __ne__(self, other):
		return not self.__eq__(other)
	def __repr__(self):
		return "%s"%(self.id)


class Edge:  
	def __init__(self, eid, heading=-1, turn="", endNode=None, internalLaneID=None, priority=-1):
		self.id = str(eid)
		self.nextEdge,  self.prevEdge = None, None
		self.seg = None # point to parent seg
		self.isInternal = self.id.startswith(":")
		self.heading,  self.turn  = heading,  turn
		self.endNode , self.priority = endNode , priority
		self.internalLaneID = str(internalLaneID) # for internal inCross edge only.
		self.build_lanes()
		self.conns,  self.lid2next,  self.lid2prev  = [],{},{} # for non-internal edges only.
		self.laneIndices={"correct":set(),"best":set()}
		if self.isInternal: 
			self.laneIndices["correct"].add(0) # if inCross, always has 0 as lane index.
			self.laneIndices["best"].add(0)
		self.intents =[Intent("lockLane"),Intent("releaseLane"),Intent("speed"), ] # what to do per edge? 

	def pop_intent(self, name):
		return pop_intent(self.intents, name)
	def has_intent(self, name):
		for it in self.intents: 
			if it.name==name: return True
		return False

	def get_lane_choices(self, ltype):
		return self.laneIndices[ltype] #get lane indices.
	def get_correct_lane_indices(self ):
		return self.get_lane_choices("correct") # a set()
	def get_best_lane_indices(self ):
		return self.get_lane_choices("best") # a set()
	def add_lane_choice(self, ltype, iterable):
		assert len(iterable)>0, self.id+" '%s' got empty "%ltype+str(iterable)
		for i in iterable: ## add lane index to set.
			self.laneIndices[ltype].add(i)
	def get_traffic_status_lanes(self,):
		laneObjs = []
		if not self.isInternal:
			for ind in self.get_best_lane_indices():
				laneObjs.append(self.get_lane(ind ) )
			return laneObjs
		return self.viaLaneId2Obj.values() # in cross

	def add_connections(self, conlist): # edge-edge connections, no inCross edge here.
		for con in conlist:
			toadd =  { "fromLane": int(con["fromLane"]), "toLane": int(con["toLane"]), "via": str(con["via"]), "dir": str(con["dir"]), } # index, index, via laneID, turn str.
			if toadd not in self.conns: 
				self.conns.append(toadd)
				self.turn = toadd["dir"]
	def add_lane_map_next(self, conlist): # edge-edge lane mapping to next lids. Not inCross.
		for con in conlist: 
			i1,i2 = int(con["fromLane"]),int(con["toLane"])
			if i1 not in self.lid2next: self.lid2next[i1]=[]
			self.lid2next[i1].append(i2) 
	def add_lane_map_prev(self, prev_conlist): # edge-edge lane mapping to previous inds.
		for con in prev_conlist: 
			i1,i2 = int(con["fromLane"]),int(con["toLane"])
			if i2 not in self.lid2prev: self.lid2prev[i2]=[]
			self.lid2prev[i2].append(i1)
	def set_allViaLanes(self, allViaLanes): # called later, after build_lane()
		self.allViaLanes = allViaLanes # for internal via edge only, i.e. inCross.
		self.viaLaneId2Obj = {}
		for via in self.allViaLanes:
			if self.internalLaneID == via: self.viaLaneId2Obj[via] = self.lanes[0]
			else:
				vlimit = traci.lane.getMaxSpeed(via) 
				llen = traci.lane.getLength(via)
				self.viaLaneId2Obj[via] = Lane(via,0,self,llen,vlimit)

	def build_lanes(self,): # called in init()
		self.lens, self.lanes = {},{}
		if traci: 
			if self.isInternal:
				self.numLane = 1
				laneID = self.internalLaneID
				vlimit = traci.lane.getMaxSpeed(laneID) 
				llen = traci.lane.getLength(laneID)
				self.lanes[0] = Lane(laneID,0,self,llen,vlimit) 
				self.lens[0] = llen
			else:
				self.numLane = traci.edge.getLaneNumber(self.id)
				for i in range(self.numLane):
					laneID = self.id+"_%d"%i
					vlimit = traci.lane.getMaxSpeed(laneID) * config.ReachSpeedLimitRatio
					llen = traci.lane.getLength(laneID)
					self.lanes[i] = Lane(laneID,i,self,llen,vlimit)
					self.lens[i] = llen
	
	def get_lane_length(self, laneIndex=None, laneID=None):
		return self.get_lane(laneIndex=laneIndex, laneID=laneID).len 
	def get_speed_limit(self, laneIndex=None, laneID=None):
		return self.get_lane(laneIndex=laneIndex, laneID=laneID).vlimit
	def get_travel_time(self,):
		if not hasattr(self, '_est_travel_time'):
			ind = list(self.get_best_lane_indices())[0]
			lane = self.get_lane(laneIndex=ind)
			self._est_travel_time = lane.len/lane.vlimit
		return self._est_travel_time

	def get_lane(self, laneIndex=None, laneID=None):
		if self.isInternal: return self.lanes[0] 
		if laneIndex is None:
			laneIndex = int(laneID.rsplit("_",1)[1])
		return self.lanes[laneIndex]
	def get_lane_number(self,):
		return len(self.lanes)
	
	def is_last_in_route(self,):
		return self.nextEdge is None 
	def is_last_in_seg(self,):
		return self.id == self.seg.edges[-1].id
	def get_num_remain_edges_in_seg(self,): # not including self.
		if not hasattr(self, '_num_remain_edges'):
			self._num_remain_edges, tmpE = 0, self
			while not tmpE.is_last_in_seg():
				self._num_remain_edges+=1
				tmpE = tmpE.nextEdge
		return self._num_remain_edges
	def get_tavel_time_from_next_edge_to_seg_end(self,): # not including self.
		if not hasattr(self, '_time_to_end'):
			self._time_to_end, tmpE = 0. , self
			while not tmpE.is_last_in_seg():
				tmpE = tmpE.nextEdge
				self._time_to_end += tmpE.get_travel_time()
		return self._time_to_end

	def get_next_non_internal_edge(self,):
		if not hasattr(self, 'next_non_internal_edge'):
			self.next_non_internal_edge = self
			while True:
				self.next_non_internal_edge = self.next_non_internal_edge.nextEdge
				if self.next_non_internal_edge is None or not self.next_non_internal_edge.isInternal: 
					break
		return self.next_non_internal_edge
	def get_previous_Non_internal_edge(self,):
		if not hasattr(self, 'prev_non_internal_edge'):
			self.prev_non_internal_edge = self
			while True:
				self.prev_non_internal_edge=self.prev_non_internal_edge.prevEdge
				if self.prev_non_internal_edge is None or not self.prev_non_internal_edge.isInternal: 
					break
		return self.prev_non_internal_edge

	def ends_with_tls(self,): 
		if not hasattr(self, '_ends_with_tls'):
			self._ends_with_tls = self.endNode is not None and self.endNode.is_tls
		return self._ends_with_tls
	def ends_with_any_stopsign(self,): 
		if not hasattr(self, '_ends_with_any_stopsign'):
			self._ends_with_any_stopsign = self.endNode is not None and (self.endNode.is_allway_stop or self.endNode.is_priority_stop)
		return self._ends_with_any_stopsign

	def is_inside_tls(self,): # for internal / incross edge.
		assert self.isInternal
		if not hasattr(self, '_is_inside_tls'):
			self._is_inside_tls = self.endNode is not None and self.endNode.is_tls
		return self._is_inside_tls
	
	def get_best_lane_inds_at_seg_end(self,):
		return self.seg.get_best_lane_inds_at_seg_end()
	def get_correct_lane_inds_at_seg_end(self,):
		return self.seg.get_correct_lane_inds_at_seg_end()

	def __eq__(self, other):
		if self.id == other.id: return True
		return False
	def __ne__(self, other):
		return not self.__eq__(other)
	def __repr__(self):
		return "%s{L%.1f H=%.1f t=%s N=%s}"%(self.id, self.lens[0], self.heading, self.turn, str(self.endNode))  




class Segment: # series of edges between tls-tls is a road seg.
	def __init__(self, index):
		self.ind = index
		self.nextSeg = None
		self.prevSeg = None
		self.edges = []
		self.waitTime, self.minSpeed = Record({"wt":0.}), Record({"v":1e6})
		self.stopCounter =  Record({"state":1, 'cnt':0}) # how many times car stopped on this seg
		self.gas, self.lastTime = Record({"gas":0.}), Record({})
		self.intents =[Intent("TlsMove"),Intent("TlsRequest"),Intent("TlsDeReg"),Intent("leaveGroup"), ] # what to do per seg? 

	def pop_intent(self, name):
		return pop_intent(self.intents, name)
	def has_intent(self, name):
		for it in self.intents: 
			if it.name==name: return True
		return False
	def stopped_at_tls(self,):
		minv = self.minSpeed.data["v"]
		waitt = self.waitTime.data["wt"]
		try:
			remdist = self.minSpeed.data["rem"]
		except: return False
		if minv<config.StopSpeedThresh and waitt>=7 and remdist<config.TlsStopPosMaxDist:
			return True
		return False
	def stopped_count(self,): # stop near tls line
		return self.stopCounter.data['cnt']

	def get_first_non_internal_edge(self,):
		if not hasattr(self, 'first_non_internal_edge'): 
			for e in self.edges:
				if not e.isInternal: 
					self.first_non_internal_edge= e
					break
		return self.first_non_internal_edge
	def get_last_non_internal_edge(self,):
		if not hasattr(self, 'last_non_internal_edge'): 
			for i in range(len(self.edges)-1,-1,-1):
				if not self.edges[i].isInternal: 
					self.last_non_internal_edge= self.edges[i]
					break
		return self.last_non_internal_edge

	def append_and_link_edge(self,edge):
		if len(self.edges)>0: 
			self.edges[-1].nextEdge = edge
			edge.prevEdge = self.edges[-1] 
		self.edges.append(edge)
		edge.seg = self
	
	def is_last_seg(self,):
		return self.nextSeg is None or self.nextSeg.ind == self.ind
	def ends_with_tls(self,):
		if not hasattr(self, '_ends_with_tls'):
			self._ends_with_tls = self.get_last_non_internal_edge().ends_with_tls()
		return self._ends_with_tls

	def get_best_lane_inds_at_seg_end(self,):
		if not hasattr(self, '_best_lane_inds_at_seg_end'):
			self._best_lane_inds_at_seg_end=self.get_last_non_internal_edge().get_best_lane_indices()
		return self._best_lane_inds_at_seg_end
	def get_correct_lane_inds_at_seg_end(self,):
		if not hasattr(self, '_correct_lane_inds_at_seg_end'):
			self._correct_lane_inds_at_seg_end=self.get_last_non_internal_edge().get_correct_lane_indices()
		return self._correct_lane_inds_at_seg_end
	def get_node_id_at_seg_end(self,):
		if not hasattr(self, '_node_id_at_seg_end'):
			self._node_id_at_seg_end=self.get_last_non_internal_edge().endNode.id
		return self._node_id_at_seg_end

	def get_seg_length(self,):
		if not hasattr(self, '_seg_length'):
			self._seg_length , tmpE = 0., self.edges[0]
			while True:
				ind = list(tmpE.get_best_lane_indices())[0]
				self._seg_length += tmpE.get_lane(laneIndex=ind).len
				if tmpE.is_last_in_seg(): break
				tmpE = tmpE.nextEdge
		return self._seg_length
	def get_seg_tavel_time(self,):  
		if not hasattr(self, '_seg_travel_time'):
			self._seg_travel_time , tmpE = 0. , self.edges[0]
			while tmpE is not None:
				self._seg_travel_time += tmpE.get_travel_time()
				if tmpE.is_last_in_seg(): break
				tmpE = tmpE.nextEdge
		return self._seg_travel_time
	def get_seg_avg_speed_limit(self,):
		return self.get_seg_length()/self.get_seg_tavel_time()

	def __repr__(self):
		return "S%d%s"%(self.ind, str(self.edges) )



class Route:
	def __init__(self,):
		self.id = None
		self.car = None
		self.segs =[]
		self.curEdge, self.nowSpeed = None , 0
		self.iprint = False

	def update_0_time(self, timestamp): # 1st called. 
		self.time = timestamp

	def update_1_edge(self, nowLaneId, lanePos, print_str=False): # 2nd called. 
		if print_str or len(nowLaneId)<=1: print("[update_1_edge] %s"%nowLaneId)
		if not nowLaneId.startswith(":"):
			try:
				nowEdgeId,laneInd = nowLaneId.rsplit("_",1)
			except:
				print(self.segs)
				print("\nBad nowLaneId  '%s' \n"%nowLaneId)
				print(self.id, self.car.id)
		else: # internal inCross:
			nowEdgeId,laneInd = nowLaneId.rsplit("_",1)[0].rsplit("_",1)[0], 0
		if self.curEdge is None: # begin of init. 
			self.curEdge = self.segs[0].edges[0]
			if self.iprint: print("start lanePos",lanePos,"e0",nowEdgeId)
		if print_str: print("  searching %s"%nowEdgeId)
		errors = 0
		while errors<=5: # sometimes jump backwards, need to traceback for at most # times. 
			try:
				tmpE=self.curEdge # for debug only.
				while self.curEdge.id != nowEdgeId:
					if print_str: print("    not this edge %s"%self.curEdge.id )
					self.curEdge = self.curEdge.nextEdge
				break
			except:
				errors+=1
				pprint.pprint(self)
				print("\n"+self.car.id+"  could not find matching edge of lane\n   "+nowLaneId)
				if tmpE: 
					print("last edge is ",tmpE, "seg", tmpE.seg)
					self.curEdge = tmpE.prevEdge
				else: break
				print("move backwards to ",self.curEdge, " and try again")
		if errors>5:
			print("Stale cache?  Forgot reset max cache time?")
			sys.exit(0)
		if print_str: print("  found %s"%self.curEdge.id)
		self.remainDist = self.curEdge.get_lane_length(laneIndex=int(laneInd))-lanePos # to next tls stop line. 
		tmpE = self.curEdge
		while not tmpE.is_last_in_seg():
			tmpE = tmpE.nextEdge
			laneInd = list(tmpE.get_best_lane_indices() )[0]
			self.remainDist += tmpE.get_lane_length(laneIndex=laneInd)
		self.curEdge.seg.lastTime.t = self.time # some times skip seg in less than 1s. 
		return self.curEdge

	def update_2_speed(self, nowSpeed):
		self.nowSpeed = nowSpeed
		segminSpeedRec = self.curEdge.seg.minSpeed
		if segminSpeedRec.t is None or segminSpeedRec.data["v"] >= nowSpeed:
			segminSpeedRec.data["v"] = nowSpeed
			segminSpeedRec.data["rem"] = self.remainDist  
			segminSpeedRec.t = self.time
		stopRec = self.curEdge.seg.stopCounter
		if nowSpeed < config.StopSpeedThresh and (self.remainDist< config.TlsStopPosMaxDist or self.curEdge.endNode.is_tls) and self.car.wrongLane == 0:
			if stopRec.data['state'] >0:   
				stopRec.data['state'] = 0
				stopRec.data['cnt']+=1
		else:
			if stopRec.data['state']==0:
				stopRec.data['state']=1

	def update_3_data(self, gas=0. ):
		if self.nowSpeed >=0.0 and self.nowSpeed < config.StopSpeedThresh:
			segwaitTimeRec = self.curEdge.seg.waitTime
			if segwaitTimeRec.t is None: segwaitTimeRec.t = self.time
			segwaitTimeRec.data["wt"] += self.time - segwaitTimeRec.t
			segwaitTimeRec.t = self.time
		seggas = self.curEdge.seg.gas
		seggas.data["gas"] += gas


	def append_and_link_segment(self,seg):
		if len(self.segs)>0: 
			self.segs[-1].nextSeg = seg
			seg.prevSeg = self.segs[-1]
			self.segs[-1].edges[-1].nextEdge = seg.edges[0]
			seg.edges[0].prevEdge = self.segs[-1].edges[-1] 
		self.segs.append(seg)

	def construct_connection(self, print_str=False):
		# after route is built, build connections between edges, called by construct_segment()
		if print_str: print("\n[construct_connection] add_connections:")
		e1 = self.segs[0].get_first_non_internal_edge()
		while True:
			e2 = e1.get_next_non_internal_edge()
			if e2 is not None:
				tup = (e1.id, e2.id)
				lst = query_sumo_connection(tup, Server_IP_Port)
				e1.add_connections(lst)
				e1.add_lane_map_next(lst)
				e2.add_lane_map_prev(lst)
				if print_str: print(e1.id,e2.id,lst)
			else: break
			if e1.is_last_in_route(): break
			e1=e2
		# derive best/correct lane ind according to conns
		if print_str: print("\n[construct_connection] add correct lanes:")
		e1 = self.segs[0].get_first_non_internal_edge()
		while True:
			if e1.is_last_in_route(): 
				e1.add_lane_choice("correct", e1.lanes.keys()) # all lanes good.
				break
			else: 
				indices=set()
				for dic in e1.conns:
					indices.add(int(dic["fromLane"]))
				e1.add_lane_choice("correct", indices)
			if print_str: print(e1.id, e1.get_lane_choices("correct"))
			e1=e1.get_next_non_internal_edge()
		if print_str: print("\n[construct_connection] add best lanes Rev:")
		e2 = self.segs[-1].get_last_non_internal_edge()
		ind2 = e2.get_lane_choices("correct")
		e2.add_lane_choice("best", ind2) # last edge, don't care.
		if print_str: print(e2.id, e2.get_lane_choices("best"))
		while True:
			e1 = e2.get_previous_Non_internal_edge()
			if e1 is not None:
				ind1 = e1.get_lane_choices("correct")
				mapped = set()
				for i2 in ind2: # find best lanes in e1.
					if i2 in e2.lid2prev:
						for i1 in e2.lid2prev[i2]:
							mapped.add(i1)
				if len(mapped)==0: # no direct connection.
					frms = set(e2.lid2prev.keys())
					ind2 = find_best_lane_ind_set(frms, ind2)
					for i2 in ind2: # do it again.
						for i1 in e2.lid2prev[i2]:
							mapped.add(i1)
				ind2 = find_best_lane_ind_set(ind1,mapped) 
				e1.add_lane_choice("best", ind2) 
				if print_str: print(e1.id, e1.get_lane_choices("best"))
			else: break
			e2=e1


	def get_num_stop_signs(self,):
		tmpE= self.segs[0].edges[0]
		cnt = 0
		while tmpE:
			if tmpE.ends_with_any_stopsign():
				cnt+=1
			tmpE = tmpE.nextEdge
		return cnt

	def __repr__(self):
		return "\n".join([ str(s) for s in self.segs ])


class Record:
	def __init__(self, initData, **kwargs):
		self.data = initData # a dict()
		self.t = kwargs.pop("t", None) # last update timestamp.
		for k,v in kwargs.items():
			self.data[k] = v
	def __repr__(self):
		return str(self.t)+str(self.data)

class Intent:
	def __init__(self, name, **kwargs):
		self.name = name # a str
		self.data = dict()
		for k,v in kwargs.items():
			self.data[k] = v
	def __repr__(self):
		return "%s"%self.name


def pop_intent(intents, name):
	i=0 
	while i< len(intents):
		if intents[i].name == name:
			return intents.pop(i)
		i+=1
	return None

def find_best_lane_ind_set(correct, target):
	# return indices in correct that are closest to target elements.
	assert len(correct)>0 and len(target)>0, str(correct)+","+str(target)
	inters = correct & target
	if len(inters)>0: return inters
	minDiff = 1e6
	for i in correct:
		for j in target:
			if abs(i-j)<minDiff:
				minDiff=abs(i-j)
				inters=set([i])
			elif abs(i-j)==minDiff:
				inters.add(i)
	assert len(inters)>0, "?"+str(correct)+","+str(target)
	return inters

