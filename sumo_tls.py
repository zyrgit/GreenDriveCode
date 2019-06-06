#!/usr/bin/env python
from common import *
from common.imp_analysis import *
from configure.assemble import *
import json, pprint
import inspect
from common.util import VoteCluster,merge_votes,get_max_vote
from a4loadServer import query_sumo_edge
from sumo_event import *
from netcode.func import all_same_heading, same_angle
config = None 
from configure.runtime import a3LearnTlsTolerate


''' ----------- Tls learning process --------------'''

class EventTime: # time absolute/relative to stage start?
	def __init__(self, absTime, relative):
		self.abs = absTime
		self.rel = relative
	def __repr__(self):
		return "%.1f/%.1f"%(self.abs , self.rel)

class TlsMove:
	MergeTimeDiff = 3.
	def __init__(self, fromEdge, toEdge, eventTime, etype="",stage_d=None):
		self.fromEdge = fromEdge
		self.toEdge = toEdge
		self.eventTime = eventTime # EventTime(abs,rel)
		self.time = eventTime.abs
		self.etype = etype
		self.d = stage_d
	def set_stage_d(self, stage_d):
		self.d = stage_d
	def __eq__(self, other):
		if abs(self.eventTime.rel - other.eventTime.rel)> TlsMove.MergeTimeDiff:
			return False
		if self.fromEdge == other.fromEdge and self.toEdge == other.toEdge:
			return True
		return False 
	def __ne__(self, other):
		return not self.__eq__(other)
	def __repr__(self):
		return "%s,%s:%s"%(self.fromEdge,self.toEdge,str(self.eventTime))


class MovesOfPhase:
	def __init__(self, absTime, pid=None):
		self.id = pid # phase ID.
		self.absTime = absTime # start abs time.
		self.moves=[]# list of TlsMove(fr,to,et) 
		self.sameCycleNextPhasesDuras =[] #[[timegap,mvph],.. happened next In same cycle.
		self.cycleLength = None
	def set_cycle_len(self,cyc):
		self.cycleLength = cyc

	def add_move(self,tlsMove):
		dup = 0
		for mymove in self.moves:
			if mymove == tlsMove:
				dup=1
				break
		if dup==0: self.moves.append(tlsMove) 

	def merge_move_from_other(self, other):
		for mov in other.moves:
			self.add_move(mov) # will de-dup.

	def add_next_dura_phase(self, duraPhase):# [duration, movphase]
		if duraPhase[1].id != self.id:
			self.sameCycleNextPhasesDuras.append(duraPhase)

	def merge_phases_from_other(self, other):
		for duraPhase in other.sameCycleNextPhasesDuras:
			self.add_next_dura_phase(duraPhase)

	def get_estimated_duration(self, maxMinRatio=10., gapRatio=0.2, print_str=False):
		votelist=[] 
		for duraPhase in self.sameCycleNextPhasesDuras:
			votelist.append( VoteCluster( duraPhase[0], 1) )
		if len(votelist)==0: return None
		votelist = merge_votes(votelist, maxMinRatio=maxMinRatio, gapRatio=gapRatio, print_str=print_str )
		return get_max_vote(votelist)

	def generate_tls_stages_phases(self, allPhases, tlsid, outdir, print_str=False, write_file=0): 
		generate_stages_phases(allPhases, tlsid, outdir, print_str=print_str, write_file=write_file)

	def __eq__(self, other):
		if self.absTime != other.absTime:
			return False
		if self.moves == other.moves:
			return True
		return False 
	def __ne__(self, other):
		return not self.__eq__(other)
	def __repr__(self):
		idst = str(self.id) if self.id is not None else ""
		tmp = ["%.1f|%d"%(d[0],d[1].id) for d in self.sameCycleNextPhasesDuras]
		return "id=%s t=%.1f Mv=%s Nex=%s"%(idst, self.absTime, str(self.moves), str(tmp))



''' ----------- Tls operation related --------------'''

class Phase: # 1 stage can have multi phases.
	def __init__(self, pid, duration):
		self.id = pid
		self.dura = duration
		self.allowed = [] # TlsMove
	def add_allowed_move(self,mov):
		if mov not in self.allowed: # using def __eq__ 
			self.allowed.append(mov)
	def __repr__(self):
		return "pid=%d T=%.1f %s.."%(self.id,self.dura,str(self.allowed[0] if self.allowed else ""))


class Stage: # 1 for each NS/WE direction.
	def __init__(self, direction, duration):
		self.d = direction
		self.dura = duration
		self.acc2time, self.pas2time ={},{} # { mov: {time: cnt}, .. } 
		self.fromEdgeSet , self.tlsid = set(),""
		self.transition={} # next stage {d1:cnt, d2:cnt2 ..}
		self.full_print=False
		self.phases, self.cycleLength = [], None

	def setFromEdges(self,edges):
		for e in edges:
			self.fromEdgeSet.add(e)
	def setTransition(self, transdic):
		self.transition = transdic

	def append_acc_move(self,mov): 
		t, acc = mov
		if acc.split(",")[0] not in self.fromEdgeSet: 
			return False
		if acc not in self.acc2time: self.acc2time[acc]=dict()
		t = int(t)
		self.acc2time[acc][t] = self.acc2time[acc].get(t,0)+1
		return True

	def append_pas_move(self,mov): #  as string.
		t, pas = mov
		if pas.split(",")[0] not in self.fromEdgeSet: 
			return False
		if pas not in self.pas2time: self.pas2time[pas]=dict()
		t = int(t)
		self.pas2time[pas][t] = self.pas2time[pas].get(t,0)+1
		return True

	def is_defect_single_stage(self,): # badly learned, only 1 phase there
		return len(self.transition)==0
	def to_dict(self):
		return {"d":self.d, "dura":self.dura, "fromEdgeSet":self.fromEdgeSet, "acc2time":self.acc2time, "pas2time":self.pas2time,"transition":self.transition, 'tlsid':self.tlsid, "phases":self.phases,"cycleLength":self.cycleLength }
	def select_to_str(self, ):
		dic = self.to_dict()
		if self.full_print: return dic
		concise = ["d","dura","fromEdgeSet","transition",'tlsid',"phases","cycleLength"]
		tmp = {}
		for k in concise: tmp[k]=dic[k]
		return tmp
	def __repr__(self):
		return pprint.pformat( self.select_to_str() )




class Tls:
	traci= None
	ofid = None
	TlsSameAccTimeDiff = 2.1 # regard as same acc time.
	TlsSamePhaseTimeSpan = 20.1 # regard as in same phase
	FutureTimeGapTrig = 200 # request time > current max pred time, trigger new pred.
	EventTimeGapTrig = 30 # new event time> last event, trigger new pred.
	MaxPlanAhead = 700 # don't dead reckon too far.

	def __init__(self,tlsid, **kwargs):
		do_loadStages = kwargs.get("loadStages",False)
		if do_loadStages: stageFolder = kwargs.get("stageFolder")
		self.iprint, self.output_file = 1, False
		self.id = tlsid
		self.em = EventManager()
		self.version  = 0
		self.em.add_event(Event("updatePhase",0, ev_reached_time(0), ["time"]))
		self.link2index = {}# (frLane,toLane,via):index
		if Tls.traci: 
			self.ctrlanes = Tls.traci.trafficlights.getControlledLanes(self.id)
			list_tup = self.traci.trafficlights.getControlledLinks(self.id)
			for i in range(len(list_tup)):
				assert len(list_tup[i])==1 , list_tup
				self.link2index[list_tup[i][0]] = i 
		self.predPhaseSeq , self.maxFutureTime = None,0
		self.nowphase , self.nextPhaseStartTime = None,0
		if self.ctrlanes:
			self.inEdges=[]
			for lane in self.ctrlanes: 
				e=lane.split("_")[0]
				if e not in self.inEdges: self.inEdges.append(e) 
		if do_loadStages:
			self.loadStages(folder = stageFolder)
			self.events={"acc":[], "pas":[]}
		self.registeredCarSet = set()
		self.keepPredicting= True


	def process(self,time):
		self.time = time
		if self.iprint>=2: 
			if self.time<=1: print(self)
			print(self.id, self.em)
		eventQueue = self.em.get_events_to_exec({"time":time})
		while not eventQueue.is_empty():
			name = eventQueue.pop().name
			if self.iprint>=2: print("exec event: "+name)
			if name == "updatePhase": 
				self.updatePhase()
			elif name == "updatePrediction": 
				self.gen_prediction()
				self.inform_cars()
			else:
				raise Exception("Unknown event "+name)


	def registerCar(self, car):
		self.registeredCarSet.add(car)
	def deregisterCar(self, car):
		try: #sometimes too fast, no register but already request deregistered
			self.registeredCarSet.remove(car)
		except: pass
	def inform_cars(self,):
		if self.keepPredicting:
			for car in self.registeredCarSet:
				try:
					car.add_event(Event("RequestTls",1, ev_reached_time(self.time), ["time"], allow_duplicate=True, data_dic={"tlsid":self.id}))
				except: print("inform_cars dead?", car)


	def set_prediction_behavior(self, keepPredicting):
		if self.keepPredicting != keepPredicting:
			print("[  set_prediction_behavior  ] %s %s"%(str(keepPredicting),self.id))
			self.keepPredicting = keepPredicting


	def ingestEvent(self, event):
		if not hasattr(self,'stageList') or len(self.stageList)==0 or event.etype!="acc": 
			return False  
		self.gen_event_stage_d(event)
		if self.iprint>=2: print("[ingestEvent] %s d%s"%(str(event),str(event.d)) )
		queue = self.events[event.etype]
		trigger_predict = 0
		if len(queue)==0:
			trigger_predict = 1
		else:
			lastMove = queue[-1] 
			if event.time-lastMove.time > Tls.EventTimeGapTrig:
				trigger_predict=1# time gap after last received event is large.
			if event.d is not None and event.d!=lastMove.d:
				trigger_predict=1
		i=len(queue)
		while i>=0:
			if i==0: 
				queue.insert(i, event)
				break
			if event.time >= queue[i-1].time:
				queue.insert(i, event)
				break
			else: i-=1 
		if trigger_predict>0:
			if self.iprint>=2: 
				print("[ingestEvent] trigger %.1f, queue:"%event.time) 
				pprint.pprint(queue)
			self.em.add_event( Event("updatePrediction",1, ev_reached_time(self.time), ["time"]) , print_str = self.iprint>=2) # update asap

	

	def getFuturePhaseSeq(self, futuretime, who=""):
		if self.iprint>=2: print("[getFuturePhaseSeq] futuretime %.1f by %s"%(futuretime,who)) 
		if futuretime > self.maxFutureTime :
			if self.iprint>=2: print("[getFuturePhaseSeq] add event updatePrediction") 
			self.em.add_event( Event("updatePrediction",1, ev_reached_time(self.time), ["time"]) , print_str = self.iprint>=2)  
			self.maxFutureTime = futuretime + Tls.FutureTimeGapTrig  
		self.maxFutureTime = min(self.maxFutureTime, self.time + Tls.MaxPlanAhead)
		if self.keepPredicting:
			if self.predPhaseSeq is None: status = "pending"
			else: status  = "valid"
		else: status = "invalid"
		dic = { "seq": self.predPhaseSeq,
				"version": self.version,
				"status": status,
		}
		return dic


	def gen_prediction(self,): # return nothing
		if len(self.stageList)==0: 
			return
		if not self.keepPredicting:
			self.predPhaseSeq = None
			return
		queue = self.events["acc"]
		i=len(queue)-1
		if i<0: 
			return
		acct = queue[i].time
		accd = queue[i].d # latest stage dir, could be None.
		while i>0 and accd is None:
			i-=1
			acct = queue[i].time
			accd = queue[i].d
		if accd is None: 
			return
		collectEvents=[queue[i],]
		while i>0:
			i-=1
			if queue[i].d is None:
				if abs(queue[i].time-acct) < Tls.TlsSameAccTimeDiff:
					acct = queue[i].time
					collectEvents.append(queue[i])
			elif accd == queue[i].d:
				if abs(queue[i].time-acct) < Tls.TlsSamePhaseTimeSpan:
					collectEvents.append(queue[i]) # gether more simultaneous moves
			else: break
		tList, pool = [],[]
		for ev in collectEvents: # last recorded stage's from edges.
			tList.append(ev.time)
		tList.sort()
		accTime = tList[0]
		for i in range(1,len(tList)):
			if tList[i]-accTime<Tls.TlsSameAccTimeDiff: pool.append(tList[i])
			else: break
		pool.append(accTime)
		accTime = np.median(pool)
		stage = self.stageList[collectEvents[0].d]
		seq=[ [accTime, stage], ]
		while accTime < self.maxFutureTime:
			accTime+= stage.dura
			stage = self.get_next_stage(stage.d)
			seq.append([accTime, stage])
		self.predPhaseSeq = seq
		self.version += 1
		if self.iprint>=2: 
			print(accd,"=d  collectEvents",collectEvents) 
			print("seq",seq, "v",self.version)


	def gen_event_stage_d(self,event): # set event.d
		eSet = set()
		eSet.add(event.fromEdge)
		matchNumStage = []
		for stage in self.stageList:# which stage is that last record? 
			inters = stage.fromEdgeSet & eSet
			if len(inters)>0:
				matchNumStage.append([ len(inters) , stage.d ])
		matchNumStage.sort(reverse=True)
		if len(matchNumStage)>1:
			# print("! ! ! Ambiguous match: acc event fromE set", eSet, event)
			# pprint.pprint(self.stageList)
			matchNumStage = [] # re-match to check least leftovers.
			for stage in self.stageList:
				inters = stage.fromEdgeSet & eSet
				if len(inters)>0:
					matchNumStage.append([ len(stage.fromEdgeSet)- len(inters) , stage.d ])
			matchNumStage.sort() # least mis-match is good.
			# print("now chose this stage:", matchNumStage[0][1])
		if len(matchNumStage)==0:
			print("[sumo_tls] tid %s stage.fromEdgeSet does not match eSet"%self.id, stage.fromEdgeSet, eSet)
			return
		event.set_stage_d(matchNumStage[0][1])


	def get_next_stage(self, direction):
		if direction>=len(self.stageList):
			print(self.id, self.stageList, direction)
		curstage= self.stageList[direction]
		trans = curstage.transition  
		if len(trans)==0: return curstage
		maxcnt = 0
		nextind = None
		for d,cnt in trans.items(): # most likely trans to.
			if cnt>maxcnt:
				maxcnt=cnt
				nextind=d
		if nextind<len(self.stageList):
			return self.stageList[nextind]
		return curstage # only 1 stage, despite transition  


	def updatePhase(self,):
		if self.nowphase is None: do_write_log = 0
		else: do_write_log=1
		self.nowphase = Tls.traci.trafficlights.getPhase(self.id)
		self.nextPhaseStartTime = Tls.traci.trafficlights.getNextSwitch(self.id)
		self.redYellowGreenState = Tls.traci.trafficlights.getRedYellowGreenState(self.id)
		if do_write_log: self.writePhaseLog()
		self.em.add_event( Event("updatePhase",0, ev_reached_time(self.nextPhaseStartTime), ["time"]), print_str=self.iprint>=2)
		if self.iprint>=2: print(self.id, self.nowphase, self.time, self.nextPhaseStartTime)


	def get_link_state(self, fromLane, toLane, via):
		ind = self.link2index[(fromLane, toLane, via)]
		return self.redYellowGreenState[ind], ind

	def loadStages(self, folder, use_pkl_stageList=True ): # load pickled stageList.
		if use_pkl_stageList:
			fn = folder+os.sep+self.id+"-sl.txt"
		self.stageList=[]
		if os.path.exists(fn):
			obj = pickle.load(open(fn , "rb"))
			if use_pkl_stageList:
				self.stageList=obj # pickled stage list. 
			else: # not in use
				for direction, sdic in obj.items(): # pickled attr dict.
					if sdic["dura"]>0:
						stage = Stage(direction, sdic["dura"])
						stage.acc2time=sdic["acc2time"]
						stage.pas2time=sdic["pas2time"]
						stage.fromEdgeSet =sdic["fromEdgeSet"]
						stage.tlsid =sdic["tlsid"]
						stage.sumoPhase =sdic["sumoPhase"]
						stage.transition =sdic["transition"]
						self.stageList.append(stage)
			self.fix_loaded_stages()
			if len(self.stageList)<=1:
				if self.iprint: print("%s  missing stage ?"%self.id)
				if self.iprint>=2: pprint.pprint(obj)
		else:
			print("\n%s  Stage Not Found !!!! "%self.id + fn)

	def fix_loaded_stages(self,):
		if len(self.stageList)>1:
			eset = set()
			for d in range(len(self.stageList)):
				stage = self.stageList[d]
				assert stage.d == d
				if len(stage.fromEdgeSet & eset)>0:
					self.stageList = []
					return
				eset |= stage.fromEdgeSet
				trans = stage.transition
				if len(trans)==0:  
					tostages = range(len(self.stageList))
					tostages.remove(d)
					for todir in tostages:
						trans[todir]=1  
		elif len(self.stageList)==1:
			stage = self.stageList[0]
			if stage.d != 0: 
				stage.d=0
			if len(stage.transition)>0:
				stage.transition={}

	def is_defect_single_stage(self,):
		return len(self.stageList)<=1  
	def setProgram(self,pid):
		Tls.traci.trafficlights.setProgram(self.id, pid)
	def writePhaseLog(self, ):
		if not self.output_file: return
		wstr= "p id=%s s=%d st=%d nt=%d rs=%s\n"%(self.id, self.nowphase, self.time, self.nextPhaseStartTime, self.redYellowGreenState)
		if self.iprint>=3: print(wstr)
		if Tls.ofid:
			try:
				Tls.ofid.write(wstr)
			except: 
				print("Tls.ofid Fail",Tls.ofid , wstr)
				Tls.ofid = open(Tls.ofidname, 'a')
				Tls.ofid.write('\n')
				Tls.ofid.write(wstr)
		else: pass
	def __repr__(self):
		return self.id+":  S="+pprint.pformat(self.stageList) 



''''''''' ------------------- util -----------------'''

LV_strict_mode=1
LV_at_least_one_move=2
LV_fromEdge_only=3
LV_guess_edge=4
LV_direction_guess=5

Correct_3_Stages_TlsIDs = []
Unlearnable_0_Stage_TlsIDS=[]

def generate_stages_phases(allPhases, tlsid, outdir, print_str=False, write_file=0): 
	# Input move phases list, output tls stage phases.
	if print_str: print("\n[ generate_stages_phases ]")
	if print_str: pprint.pprint(allPhases)
	firstTryPhasesWithNextTransitions = []
	for ph in allPhases:
		if len(ph.sameCycleNextPhasesDuras)>0:
			firstTryPhasesWithNextTransitions.append(ph)
	if print_str: print("First try #%d out of #%d"%(len(firstTryPhasesWithNextTransitions),len(allPhases)))
	if len(firstTryPhasesWithNextTransitions)<2:
		if print_str: print("too few, use ori allPhases")
		firstTryPhasesWithNextTransitions = allPhases
	mlevel = LV_strict_mode
	mergedPhases= merge_phases(firstTryPhasesWithNextTransitions,mlevel, print_str=print_str)
	while len(mergedPhases)>2:
		mlevel+=1
		if mlevel<= LV_direction_guess:
			if print_str: print("merge_level",mlevel)
			mergedPhases= merge_phases(mergedPhases,mlevel, print_str=print_str)
		else: break
	if len(mergedPhases)>2:
		if print_str: print("\nThen  try ori #%d"%(len(allPhases)))
		mlevel = LV_strict_mode
		mergedPhases= merge_phases(allPhases,mlevel, print_str=print_str)
		while len(mergedPhases)>2:
			mlevel+=1
			if mlevel<= LV_direction_guess:
				mergedPhases= merge_phases(mergedPhases,mlevel, print_str=print_str)
			else: break

	# merge same phase, which happened far apart.
	if print_str: 
		print("\nAfter merged:")
		pprint.pprint(mergedPhases)
	stageList = None
	if len(mergedPhases)==1: # just one phase
		raw = mergedPhases[0]
		stage = Stage(0, raw.cycleLength )
		phase = Phase(0, raw.cycleLength/2 ) # assume half on.
		phase1 = Phase(1, raw.cycleLength/2 ) # half off.
		stage.phases.append(phase)
		stage.phases.append(phase1)
		stage.tlsid=tlsid
		stage.cycleLength=raw.cycleLength
		for mov in raw.moves:
			phase.add_allowed_move(mov)
			stage.fromEdgeSet.add(mov.fromEdge)
		print(stage)
		stageList=[stage]
	elif len(mergedPhases)==2: # most likely, two phases 
		raw,raw2 = mergedPhases
		dura = raw.get_estimated_duration(print_str=print_str)
		dura2 = raw2.get_estimated_duration(print_str=print_str)
		LearnedCycle = raw.cycleLength
		if dura is None and dura2 is None: dura,dura2 = LearnedCycle/2,LearnedCycle/2
		elif dura is None: dura = LearnedCycle-dura2
		elif dura2 is None: dura2 = LearnedCycle-dura
		if abs(LearnedCycle -(dura+dura2)) > LearnedCycle*0.1:
			if abs(dura+dura2-config.TlsMostLikelyCycleDura)<abs(LearnedCycle-config.TlsMostLikelyCycleDura): # change cycle back 
				ratio = LearnedCycle%(dura+dura2)
				if min(ratio,1.-ratio)<0.07:
					LearnedCycle = dura+dura2
				elif abs(dura+dura2-config.TlsMostLikelyCycleDura)<5:
					LearnedCycle = dura+dura2
			if abs(dura+dura2-LearnedCycle) > LearnedCycle*0.1 : # still not good
				dura = raw.get_estimated_duration(gapRatio=0.1, print_str=print_str)
				dura2 = raw2.get_estimated_duration(gapRatio=0.1, print_str=print_str)
				LearnedCycle = raw.cycleLength
			if abs(dura+dura2-LearnedCycle) > LearnedCycle*0.1 : # try strict merge vote 
				dura = raw.get_estimated_duration(gapRatio=0.05, print_str=print_str)
				dura2 = raw2.get_estimated_duration(gapRatio=0.05, print_str=print_str)
				LearnedCycle = raw.cycleLength
			if abs(dura+dura2-LearnedCycle) > LearnedCycle*0.1 : # most strict, no gap allowed
				dura = raw.get_estimated_duration(gapRatio=0.01, print_str=print_str)
				dura2 = raw2.get_estimated_duration(gapRatio=0.01, print_str=print_str)
				LearnedCycle = raw.cycleLength
				
		stage = Stage(0, dura )
		stage2= Stage(1, dura2 )
		phase = Phase(0, dura )
		phase2 = Phase(0, dura2 )
		stage.phases.append(phase)
		stage2.phases.append(phase2)
		stage.tlsid=tlsid
		stage2.tlsid=tlsid
		stage.cycleLength=LearnedCycle
		stage2.cycleLength=LearnedCycle
		stage.transition ={1:999}
		stage2.transition={0:999}
		for mov in raw.moves:
			phase.add_allowed_move(mov)
			stage.fromEdgeSet.add(mov.fromEdge)
		for mov in raw2.moves:
			phase2.add_allowed_move(mov)
			stage2.fromEdgeSet.add(mov.fromEdge)
		if print_str:
			print(stage)
			print(stage2)
		if tlsid in a3LearnTlsTolerate:
			assert abs(dura+dura2-LearnedCycle)<LearnedCycle*0.2 , tlsid
		else:
			assert abs(dura+dura2-LearnedCycle)<LearnedCycle*0.1 , tlsid
		stageList=[stage,stage2]
	else:
		for ph1 in mergedPhases:
			print("")
			print(ph1)
			for duraph2 in ph1.sameCycleNextPhasesDuras:
				ph2=duraph2[1]
		if tlsid not in Correct_3_Stages_TlsIDs and tlsid not in Unlearnable_0_Stage_TlsIDS:
			raise Exception("Unknown %d stages.."%len(mergedPhases)+tlsid)
	if write_file and stageList is not None:
		fn = outdir+tlsid+"-sl.txt"
		print("Writing "+fn)
		pickle.dump(stageList, open(fn,"wb"))





def merge_phases( raw_phases, gran_level=LV_strict_mode , print_str=False):
	''' merge [MovesOfPhase,  ..] 
	- gran_level: criteria granularity, following levels:  
		- strict_mode: only merge when fully match moves.
		- fromEdge_only: merge as long as one move shares from-edge.
		- guess_edge: merge if from-edges are in opposite direction.
		- direction_guess: merge if edge heading is opposite direction.
	'''
	if print_str: print('\n[ merge_phases ] in #%d'%len(raw_phases))
	allPhases = raw_phases[:]
	idmap = {}
	while 1:
		happened=0
		mergedPhases=[]
		while allPhases:
			mergedPhases.append(allPhases.pop(0))
			basePhase = mergedPhases[-1]
			j=0
			while j< len(allPhases):
				check_phase = None
				if check_phase is None and gran_level>=LV_strict_mode: # full all moves match valid.
					if basePhase.moves==allPhases[j].moves:
						check_phase = allPhases.pop(j)
						happened+=1
				if check_phase is None and gran_level>=LV_at_least_one_move: # at least one move matches.
					inter = [value for value in basePhase.moves if value in allPhases[j].moves] 
					if len(inter)>0:
						check_phase = allPhases.pop(j)
						happened+=1
				if check_phase is None:   # from same edge will merge.
					fe1 = set([ mv.fromEdge for mv in basePhase.moves ])
					fe2 = set([ mv.fromEdge for mv in allPhases[j].moves ])
				if check_phase is None and gran_level>=LV_fromEdge_only: # guess if edge on same street/oppo dir.
					if fe1 & fe2:
						check_phase = allPhases.pop(j)
						happened+=1
				if check_phase is None and gran_level>=LV_guess_edge:
					fedge1 = set([e.lstrip("-").split("#",1)[0] for e in fe1])# extract main edge name.
					fedge2 = set([e.lstrip("-").split("#",1)[0] for e in fe2])
					if fedge1 & fedge2:
						check_phase = allPhases.pop(j)
						happened+=1
				if check_phase is None and gran_level>=LV_direction_guess:# guess if edge on oppo dir.
					try: # query edge server.
						de1 = [ query_sumo_edge(e, Server_IP_Port)['heading'] for e in fe1 ]
						de2 = [ query_sumo_edge(e, Server_IP_Port)['heading'] for e in fe2 ]
						if all_same_heading(de1, allow_180=True, threshold=15.) and all_same_heading(de2, allow_180=True, threshold=15.):
							if same_angle(de1[0],de2[0], allow_180=True, threshold=15.):
								check_phase = allPhases.pop(j)
								happened+=1
						if check_phase is None :
							if print_str: print(fe1,de1,"   !=   ",fe2,de2)
					except: 
						print("[sumo_tls] Server %s is Not On...."%Server_IP_Port)
				if check_phase: # successfully merged.
					basePhase.merge_phases_from_other(check_phase)
					basePhase.merge_move_from_other(check_phase)
					idmap[check_phase.id] = basePhase.id
				else: 
					j+=1
		allPhases = mergedPhases 
		if happened==0: break # one scan may not be enough!
	# change id to reflect merge.
	for ph in mergedPhases:
		if ph.id in idmap: pd.id = idmap[ph.id] 
		for duraPhase in ph.sameCycleNextPhasesDuras:
			pid = duraPhase[1].id
			if pid in idmap: duraPhase[1].id = idmap[pid]
	# re-check/remove self-loop in next list:
	for ph in mergedPhases:
		i=0
		while i< len(ph.sameCycleNextPhasesDuras):
			duraPhase = ph.sameCycleNextPhasesDuras[i]
			if ph.id == duraPhase[1].id:
				ph.sameCycleNextPhasesDuras.pop(i)
			else: i+=1
	return mergedPhases
