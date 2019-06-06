#!/usr/bin/env python
from common import *
from common.constants import *
from common.imp_analysis import *
from common.model import Global_model,VTCPFEM_features # from GreenRoute 
from common.util import get_target_xy_given_lane_change,get_moved_xy,get_2xy_dist
from common.simulation import feedback_incross_stops, feedback_tls_stops
from sumo_tls import TlsMove,EventTime
from sumo_event import *
from sumo_road import Node,Lane,Edge,Segment,Route
from a4loadServer import query_sumo_out_edges
from sumo_sol import TlsSolution,FuelCost
from sumo_sol import Group, DT, INF
from configure.params import addr2Same_Group_Dist,addr2Coor_Period,addr2Coor_Max_Time_Gap

from configure.runtime import * # IgnoreVids, IgnoreTimes,etc
config, Server_IP_Port, R = None , None , None # set by b*.py 

G_Allow_Prompt = 1  # global GUI ctr vars.
G_Do_Show_GUI = 1 # if has gui.
G_removed_cars = []

def do_interrupt(car): # alert, ask you to press key
	if G_Allow_Prompt and car.time < IgnoreAfterTime and car.id not in IgnoreVids and car.curEdge.id not in IgnoreEdges and car.time not in IgnoreTimes and car.id.startswith('a'):
		return True
	return False

def GUI_show(obj , avoid_GUI_shift=True): 
	if G_Do_Show_GUI==0 or avoid_GUI_shift: return # set to 0 
	if isinstance(obj, list) or isinstance(obj, tuple):
		Car.traci.gui.setOffset('View #0',obj[0],obj[1])
	elif isinstance(obj, Car):
		Car.traci.gui.setOffset('View #0',obj.xy[0],obj.xy[1])
	elif isinstance(obj, Node):
		Car.traci.gui.setOffset('View #0',obj.x,obj.y)
	elif isinstance(obj, Edge):
		n1 = obj.endNode
		n0 = obj.prevEdge.endNode if obj.prevEdge is not None else n1
		x = (n0.x+n1.x)/2
		y = (n0.y+n1.y)/2
		Car.traci.gui.setOffset('View #0',x,y)
	elif isinstance(obj, Lane):
		GUI_show(obj.edge)


class SpeedController:
	def __init__(self,car):
		self.car = car
		self.minSpeed = config.CarMinSpeed
		self.maxSpeed = config.CarMaxSpeed
		self.cmdHistory = [] # [ [command,observed], ..], not every second 
		self.speedLog = [] # [v, ..] # every second
		self.cmdLogSize, self.speedLogSize = 30, 30
		self.lastcmd = None
		self.lastv = None
	def starting_acceleration(self, after_wait=3):# after add_current_speed()
		if len(self.speedLog)<=1: return False
		if self.speedLog[-2]<config.StopSpeedThresh and self.speedLog[-1]>=config.StopSpeedThresh: 
			i=len(self.speedLog)-3
			while i>=0:
				if self.speedLog[i]>=config.StopSpeedThresh: break
				i-=1
				if len(self.speedLog)-2 - i >= after_wait: return True
			return len(self.speedLog)-2 - i >= after_wait
		return False
	def add_current_speed(self,):
		self.speedLog.append(self.car.nowSpeed)
		if len(self.speedLog)>self.speedLogSize: self.speedLog.pop(0)
	def setMinSpeed(self,spd):
		self.minSpeed=spd
	def setMaxSpeed(self,spd):
		self.maxSpeed=spd
	def get_acc(self,):
		if len(self.speedLog)<=1: return 0.
		i=len(self.speedLog)-1
		return self.speedLog[i]-self.speedLog[i-1]
	def clampSpeed(self, spd):
		return max(min(self.maxSpeed,spd),self.minSpeed)
	def cannot_reach_command_speed(self, how_long=1, print_str=False):
		latestVdiff = None
		for i in range(len(self.cmdHistory)-1,-1,-1):
			cmdv, realv = self.cmdHistory[i]
			if latestVdiff is None: 
				latestVdiff= cmdv- realv
				if latestVdiff > 9: return True
			if print_str: print(self.car.id, 'cmdv, realv',cmdv, realv)
			if cmdv< realv+1:  return False
			if len(self.cmdHistory)-i >= 3+how_long or i==0: return True
			if len(self.cmdHistory)-i >= 2+how_long and cmdv- realv>2 and cmdv-realv<latestVdiff: return True
			if len(self.cmdHistory)-i >= 1+how_long and cmdv- realv>8: return True
		return False
	def applySpeed(self, spd):
		nowSpeed = self.car.nowSpeed
		spd = self.clampSpeed(spd)
		self.car.applySpeedRaw(spd)
		if self.lastcmd: self.cmdHistory.append( [self.lastcmd, nowSpeed] )
		self.lastcmd = spd
		while len(self.cmdHistory)>self.cmdLogSize: self.cmdHistory.pop(0)
		return spd
	def applySpeedLimit(self,):
		self.applySpeed(self.car.speedLimit)
	def tryAppendToFile(self, fid, wstr):
		fid.write(wstr)
		if not wstr.endswith('\n'): fid.write('\n')
	def flush_speed_stats(self, fid):
		if self.car.output_file and fid:
			if len(self.speedLog)>=self.speedLogSize/2: 
				wstr = ",".join( map(lambda x: "%.2f"%x, self.speedLog[0:4]) )
				wstr = "d id=%s v=%s\n"%(self.car.id, wstr )
				self.tryAppendToFile( fid, wstr)
			if len(self.cmdHistory)>=self.cmdLogSize/2:
				cmdstr = " ".join( map(lambda x: "%.1f,%.1f"%tuple(x), self.cmdHistory[0:4]) )
				wstr = "c id=%s v=%s\n"%(self.car.id, cmdstr)
				self.tryAppendToFile( fid, wstr)




class Car:
	traci=None
	id2car = None
	id2group = None # coor
	ofid = None # car stats output file 
	mvfd = None # car tls movement output file
	spdfd = None # output speed distrib
	lane2vq=None 
	feedbackData,congestedAreas = None, None
	kQueMoving,kQueStill =1,0
	RemoveWongLaneThresh = 2 # if get stuck beyond this

	def __init__(self,vid, **kwargs):
		self.modelPredictFuel = kwargs.get("modelPredictFuel",False)
		self.id =vid
		# static vars:
		self.VLen=5.0
		self.minGap = 2.0
		self.tDischarge = 1.0 # 1s discharge delay of wait queue 
		self.maxAccel = 2.0
		self.maxDecel = 6.0
		# dynamic var:
		self.em = EventManager()
		self.trafficStatusDic = {}
		self.gas , self.travelDist , self.rou, self.total_wait_time = 0., 0., [], 0
		self.stopped_tls_num, self.flush_v_period, self.log_tls_pred_period = 0 , 200, 15 
		self.iprint, self.iprintSet,  self.output_file = 1,set([1]), False
		self.wrongLane, self.stuck_time, self.nowLaneId = 0, 0, None 
		self.isDead, self.subscribed , self.inCross= False,False,False
		self.e2tls  , self.route = None , None # set by main simulation b*.py.
		self.lane2vq  , self.id2car, self.id2group = Car.lane2vq  , Car.id2car, Car.id2group
		self.speedControl = SpeedController(self)
		if self.modelPredictFuel:
			self.fuel = FuelModel(Global_model)
		self.EV_TrafficStatus_T , self.EV_OptimSpeed_T = 5, 10 # event period.
		self.wrongLaneOffNum, self.othersWrongLane = 0, set()
		self.tls1, self.tls2 , self.fuelCost = TlsSolution(self), TlsSolution(self), FuelCost(self)
		self._calc_fuel_cost, self._do_transaction, self.coorArriveT1 = False, False, -1


	def add_event(self, event, print_str = False):
		if 3 in self.iprintSet or print_str:
			print('[ %s add_event ] '%self.id +str(event))
		self.em.add_event(event, print_str= False)

	''' ----------- main loop, once per step: ----------- '''

	def process(self,time):
		self.time=time
		if not self.subscribed or self.isDead:
			return
		res = self.getSubscriptionResults()
		bugcnt = 0
		while res[Car.traci.constants.VAR_LANE_ID]=='':
			res = self.getSubscriptionResults() # traci bug
			print('bad traci.res?',res)
			bugcnt+=1
			if bugcnt>50:
				self.remove(output=False)
				if 3 in self.iprintSet:raw_input("t%d  Removed2 myself .. %s"%(self.time, self.id))
				return
		try:
			self.nowSpeed = res[Car.traci.constants.VAR_SPEED]
			self.nowLaneId = res[Car.traci.constants.VAR_LANE_ID]
			if len(self.nowLaneId)<2: print('bad traci.res?',res)
			self.lanePos = res[Car.traci.constants.VAR_LANEPOSITION]
			self.travelDist = res[Car.traci.constants.VAR_DISTANCE]
			self.xy = res[Car.traci.constants.VAR_POSITION]
		except:
			print('imcomplete traci res...',res) # bug
			self.remove(output=False)
			if 3 in self.iprintSet:raw_input("t%d  Removed3 myself .. %s"%(self.time, self.id))
			return
		if self.modelPredictFuel: 
			self.sumoGas = self.fuel.get_fuel(self.nowSpeed)
		else: self.sumoGas = res[Car.traci.constants.VAR_FUELCONSUMPTION]
		self.gas += self.sumoGas
		self.speedControl.add_current_speed()

		self.route.update_0_time(self.time)
		try:
			self.curEdge = self.route.update_1_edge(self.nowLaneId, self.lanePos, print_str=False)
		except:
			print('bad route no match...')
			self.remove(output=False)
			if 3 in self.iprintSet:raw_input("t%d  Removed4 myself .. %s"%(self.time, self.id))
			return
		self.curLane = self.curEdge.get_lane(laneID = self.nowLaneId) # could be wrong inCross
		self.inCross = self.curEdge.isInternal
		self.remainDist = self.route.remainDist
		if self.inCross:
			self.curLaneLen = Car.traci.lane.getLength(self.nowLaneId)
		else: self.curLaneLen = self.curLane.len
		self.laneRemDist = self.curLaneLen -self.lanePos
		self.speedLimit = self.curEdge.get_speed_limit(laneID = self.nowLaneId)
		self.route.update_2_speed(self.nowSpeed)
		self.route.update_3_data( gas = self.sumoGas)
		if 3 in self.iprintSet:
			if G_Do_Show_GUI: GUI_show(self,  avoid_GUI_shift=False)
			print("\n%d"%self.time)

		# Highest priority: must put before speed control:
		self.checkLane() 
		self.check_teleport() # will call remove() 
		if self.isDead or not self.subscribed:
			print("t%d  Removed myself .. %s"%(self.time, self.id))
			return
		self.updateLaneQueue() # will enter only after nowLaneId is changed
		# mark. avoid repeated calls:
		self._requestTls1,self._requestTls2,self._optimizeSpeed,self._getTrafficStatusAhead = False,False,False,False
		# fill feedback data 
		if self.nowSpeed < config.StopSpeedThresh:
			if self.do_optimizeSpeed and self.inCross: # could be non-tls node.
				if not hasattr(self,'lastFeedbackEdge') or self.curEdge.id != self.lastFeedbackEdge:
					junction = self.curEdge.endNode
					if 3 in self.iprintSet:  print("\n %s report congestion at %s"%(self.id,str(junction)))
					feedback_incross_stops(Car.feedbackData, self.time, junction)
					self.lastFeedbackEdge = self.curEdge.id 
			if self.do_Tls1 and self.remainDist < self.minGap and not self.inCross and self.curEdge.ends_with_tls() and self.wrongLane==0 and self.stuck_time==0 and self.tls1.is_valid():
				if not hasattr(self,'lastFeedbackEdge') or self.curEdge.id != self.lastFeedbackEdge:
					fromLaneID, toLaneID, via = self.get_via_link( self.curEdge , self.curLane.ind )
					state,_ = self.get_signal_given_link(fromLaneID, toLaneID, via)
					if state.lower()=='r':
						tls = self.e2tls[self.curEdge.id]
						if 3 in self.iprintSet:  print("\n %s report tls pred err at %s, state %s"%(self.id, tls.id, state))
						feedback_tls_stops(Car.feedbackData, self.time, tls)
						self.lastFeedbackEdge = self.curEdge.id # no overlap with previous, in cross edge.
		# gather statistics: wait time, halt tls num.
		if self.nowSpeed < config.StopSpeedThresh:
			self.total_wait_time+=1
			if self.remainDist < 40 and not self.inCross and self.curEdge.ends_with_tls() and self.wrongLane==0 and self.stuck_time==0:
				if not hasattr(self,'lastTlsStopEdge') or self.curEdge.id != self.lastTlsStopEdge:
					self.stopped_tls_num+=1
					self.lastTlsStopEdge = self.curEdge.id
				if 3 in self.iprintSet: 
					print("%s stop at tls lane %s"%(self.id,self.nowLaneId),'wt',self.total_wait_time,'halt',self.stopped_tls_num)
		if not hasattr(self,'last_flush_time') or self.time-self.last_flush_time> self.flush_v_period:
			try:
				self.speedControl.flush_speed_stats(Car.spdfd )
			except:
				Car.spdfd = open(Car.spdfdname, 'a')
				Car.spdfd.write('\n')
				self.speedControl.flush_speed_stats(Car.spdfd )
			self.last_flush_time = self.time


	def optimize(self):
		if self.isDead or not self.subscribed:return
		if 3 in self.iprintSet: 
			print("%s T%d rem %.2f,"%(self.id, self.time, self.remainDist)+ " gas%.2f, tG%.1f, tD%.1f,"%(self.sumoGas, self.gas, self.travelDist)+ " v%.2f,"%self.nowSpeed + " lim%.2f"%self.speedLimit+ " 1vo%.2f"%(self.tls1.optimSpeed or -1)+ " Lrem%.1f"%self.laneRemDist)
			print("now E=%s li%d L%s"%(self.curEdge.id, self.curLane.ind, self.nowLaneId) +" wrongl %d"%self.wrongLane, "stuckt %d"%self.stuck_time, "hd %.1f"%self.curEdge.heading, "turn "+self.curEdge.turn, "segStopCnt", self.curEdge.seg.stopCounter.data['cnt'])
			print("_calc_fuel_cost?",self._calc_fuel_cost,'_do_transaction?',self._do_transaction,'coT1',self.coorArriveT1)
		# High priority:
		self.handleLaneProblem()
		self.handleEndOfSeg() # also for not using our sys.
		self.handleBeginOfSeg()
		self.handleBeginOfEdge() # after handleBeginOfSeg !

		if not self.do_optimizeSpeed: return

		if 3 in self.iprintSet: 
			print("Tls1 ?%s %s"%(self.curEdge.seg.ends_with_tls(),self.tls1.tlsid[-4:] if self.tls1.tlsid else "")+" v:%d %s"%(self.tls1.version,self.tls1.status) +", Tls2 ?%s %s"%(self.curEdge.seg.nextSeg.ends_with_tls() if self.curEdge.seg.nextSeg else "False",self.tls2.tlsid[-4:] if self.tls2.tlsid else "")+" v:%d %s"%(self.tls2.version,self.tls2.status) )
		# Event driven dynamic priority:
		eventQueue = self.em.get_events_to_exec({"time":self.time})
		while not eventQueue.is_empty():
			event = eventQueue.pop()
			name = event.name
			if 3 in self.iprintSet:  print("-- exec event: "+name)
			if name == "RequestTls": # pri=1 
				tlsid = event.data["tlsid"]
				if tlsid == self.tls1.tlsid: 
					self.requestTls1()
					self.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]))
				elif tlsid == self.tls2.tlsid: 
					self.requestTls2()
			elif name == "TrafficStatus": # pri=2
				self.getTrafficStatusAhead()
				self.add_event(Event("TrafficStatus",2, ev_reached_time(self.time+self.EV_TrafficStatus_T), ["time"], only_once_in_em=True) )
				self.add_event(Event("OptimV",3, ev_reached_time(self.time+self.EV_OptimSpeed_T), ["time"]))
			elif name == "OptimV": # pri=3
				self.optimizeSpeed()
			elif name == "CoorTrans": # pri=5 low
				pass # called somewhere else.
			else:
				raise Exception("Unknown event "+name)
		# situations that need to call frequently:
		if self.inCross:
			self.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]))
		elif self.speedControl.starting_acceleration():
			if 3 in self.iprintSet: print("[ starting_acceleration ]",self.speedControl.speedLog)
			self.optimizeSpeed( redo=True, starting_acceleration=True )
		elif self.nowSpeed < config.StopSpeedThresh:
			self.preferStableTrafficEstimation = False # allow update.
			if 3 in self.iprintSet: print("    Turn off StableTrafficEstimation")
			if self.nowSpeed>0: # why extremely slow ???
				self.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]))
		elif self.nowSpeed <= config.CarMinSpeed and self.nowSpeed>0: 
			self.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]))


	''' ----------- only call once, exec each intent once per step: ------- '''

	def handleLaneProblem(self,):
		if not self.do_optimizeSpeed or self.wrongLane==0 or self.inCross: 
			return
		if not hasattr(self,'lastWrongLane'): self.lastWrongLane = self.wrongLane
		nowLaneIndex = self.curLane.ind
		self.bestLaneIndices = self.curEdge.get_lane_choices("best")
		targetIndex = sorted([(abs(i-nowLaneIndex),i) for i in self.bestLaneIndices])[0][1]
		myQue = self.lane2vq[self.nowLaneId]
		self.wrongLaneOffNum = targetIndex - nowLaneIndex # off by 1,2,3..?
		if abs(self.wrongLaneOffNum)>1: # need to change multi-lanes...
			if targetIndex>nowLaneIndex:
				targetIndex = nowLaneIndex +1
			else: targetIndex = nowLaneIndex -1
		targetLane = self.curEdge.id+"_%d"%targetIndex
		rightQue = self.lane2vq[targetLane] if targetLane in self.lane2vq else []
		if 3 in self.iprintSet: 
			print("\n[ handleLaneProblem ] w+%d to %d, off by %d, myQ, tQ:"%(self.wrongLane,targetIndex,self.wrongLaneOffNum))
			print(myQue)
			print(rightQue)
		# find out gap to squeeze in.
		i = len(rightQue)-1
		if i<0: # cars on diff lanes in front or behind. 
			if self.wrongLane > self.lastWrongLane: # counter still inc, still wrong after last call.
				changedSpd = self.lastWrongLaneChangedSpd
			else:
				changedSpd = max(1., self.nowSpeed)
			self.applySpeedRaw(changedSpd)
			self.lastWrongLaneChangedSpd = changedSpd
			if self.remainDist < 29: # bad situation, just force a move.
				x,y = get_target_xy_given_lane_change(self.xy[0], self.xy[1], self.curEdge.heading, self.nowSpeed+self.speedControl.get_acc(), self.wrongLaneOffNum)
				if 3 in self.iprintSet:  print(self.xy, "  moveToXY  %.1f %.1f"%(x,y))
				self.moveToXY(self.curEdge.id, targetIndex, x, y)
			if 3 in self.iprintSet: 
				print("No target, apply v=%.2f"%changedSpd)
			return
		firstCarPos = rightQue[-1].lanePos
		if firstCarPos < self.lanePos -20 and firstCarPos > self.lanePos -30: # is far behind, but can see.
			if rightQue[-1].nowSpeed <= self.nowSpeed:
				if 3 in self.iprintSet: print("  Overtake ", rightQue[-1])
				self.lastWrongLane = self.wrongLane # if trying multi-times 
				self.lastWrongLaneChangedSpd = self.nowSpeed # if keep trying multi-times 
				return
		if len(rightQue)==1 and firstCarPos > self.lanePos + 30: #1st car, is far ahead
			if firstCarPos + rightQue[-1].nowSpeed * 2. > self.lanePos + 2* self.nowSpeed:
				if 3 in self.iprintSet: print("  He's too far ahead ", rightQue[-1])
				self.lastWrongLane = self.wrongLane 
				self.lastWrongLaneChangedSpd = self.nowSpeed 
				return
		while i>=0:
			if rightQue[i].lanePos > self.lanePos + 20:
				i-=1
				continue
			if i>0:
				space = rightQue[i].VLen + rightQue[i].get_space_gap_behind(High_Speed_Gap_Offset=3.)
				if rightQue[i].lanePos -rightQue[i-1].lanePos > 2*space: break
			else: break
			i-=1
		targetCar = rightQue[i] # move to his behind
		self.othersWrongLane.add(targetCar.id)
		tarPos = targetCar.lanePos - targetCar.VLen - targetCar.get_space_gap_behind(High_Speed_Gap_Offset=3., Low_Speed_Least_Gap = 5.)
		if self.nowSpeed<12: 
			vDiff = max(0, self.lanePos - tarPos)/2 # milder, tighter
		else: 
			tarPos -= 2
			vDiff = max(0, self.lanePos - tarPos)/1.5 # Too fast, dec lower faster.
		hisSpeed = targetCar.nowSpeed + targetCar.speedControl.get_acc()
		if 3 in self.iprintSet: print("targetCar %s v%.2f"%(targetCar.id,targetCar.nowSpeed),"acc",targetCar.speedControl.get_acc())
		changedSpd = max(1., min(self.nowSpeed, hisSpeed - vDiff) )
		# if he is correct, or we move same direction across lanes, no interfere
		iSacrifice = 0
		if targetCar.wrongLane==0 or self.wrongLaneOffNum*targetCar.wrongLaneOffNum>0: 
			if 3 in self.iprintSet: print("targetCar good",targetCar,"tarPos %.1f"%tarPos,"myPos %.1f"%self.lanePos,"vDiff %.1f"%vDiff,"changedSpd %.2f"%changedSpd)
			iSacrifice = 1
		if iSacrifice==0 and self.id not in targetCar.othersWrongLane: # both wrong, both aware?
			if 3 in self.iprintSet: print("targetCar not aware, wait",targetCar,"tarPos %.1f"%tarPos,targetCar.othersWrongLane,"myPos %.1f"%self.lanePos,self.othersWrongLane)
			iSacrifice = -1 # skip this step.
		if iSacrifice==0 and abs(targetCar.wrongLaneOffNum)<abs(self.wrongLaneOffNum): # let him go
			if 3 in self.iprintSet: print("Let him go",targetCar,"He is less off",targetCar.wrongLaneOffNum,"tarPos %.1f"%tarPos,"myPos %.1f"%self.lanePos,"vDiff %.1f"%vDiff,"changedSpd %.2f"%changedSpd)
			iSacrifice = 1
		if iSacrifice==0 and abs(targetCar.wrongLaneOffNum) == abs(self.wrongLaneOffNum):
			if self.nowSpeed < targetCar.nowSpeed -0.1:# let him go first
				if 3 in self.iprintSet: print("I'll slow down, he is faster:",targetCar,"tarPos %.1f"%tarPos,"myPos %.1f"%self.lanePos,"vDiff %.1f"%vDiff,"changedSpd %.2f"%changedSpd)
				iSacrifice = 1
			if iSacrifice==0 and abs(self.nowSpeed - targetCar.nowSpeed)< 0.1:
				if self.lanePos<targetCar.lanePos:
					if 3 in self.iprintSet: print("I'll slow down, he is in front:",targetCar,"tarPos %.1f"%tarPos,"myPos %.1f"%self.lanePos,"vDiff %.1f"%vDiff,"changedSpd %.2f"%changedSpd)
					iSacrifice=1
		# I should take over him fast.
		if iSacrifice==0:
			changedSpd = max(self.tls1.optimSpeed , self.nowSpeed + 0.5)
			self.applySpeedRaw(changedSpd)
			if 3 in self.iprintSet: print("I'll take over:",targetCar,"myPos %.1f"%self.lanePos,"changedSpd %.2f"%changedSpd)
		elif iSacrifice==1:
			self.applySpeedRaw(changedSpd)
			if 3 in self.iprintSet: print("Slowing down",targetCar,"tarPos %.1f"%tarPos,"myPos %.1f"%self.lanePos,"vDiff %.1f"%vDiff,"changedSpd %.2f"%changedSpd)
		self.lastWrongLane = self.wrongLane # if trying multi-times 
		self.lastWrongLaneChangedSpd = changedSpd # if keep trying multi-times 


	def handleBeginOfEdge(self,):
		if not self.do_optimizeSpeed: return
		intent = self.curEdge.pop_intent("speed") # run optimize
		if intent is not None: 
			if 3 in self.iprintSet: print("[ handleBeginOfEdge ] intent 1: speed")
			if not self.approaching_seg_end_queue(): self.preferStableTrafficEstimation = False 
			if 3 in self.iprintSet: print("    Turn off StableTrafficEstimation")
			self.getTrafficStatusAhead()
			self.optimizeSpeed()


	def handleBeginOfSeg(self,):
		if not self.do_optimizeSpeed: return
		intent = self.curEdge.seg.pop_intent("TlsRequest") # only 1 intent per seg.
		if intent is not None: 
			self.preferStableTrafficEstimation = False # allow update.
			if 3 in self.iprintSet: print("    Turn off StableTrafficEstimation")
			self.tls1.clear() 
			if self.curEdge.seg.ends_with_tls():
				tlsFromEdgeID = self.curEdge.seg.get_last_non_internal_edge().id
				if 3 in self.iprintSet: print("[ handleBeginOfSeg ] TlsRequest intent. tls1 config %s"%tlsFromEdgeID)
				if tlsFromEdgeID not in self.e2tls: 
					print(self.id)
					print("\nStale cache? reset time to 0!!\n")
				self.tls1.configure(tlsFromEdgeID, self.e2tls[tlsFromEdgeID] )
				self.e2tls[tlsFromEdgeID].registerCar(self)
				self.requestTls1()
				self.add_event(Event("TrafficStatus",2, ev_reached_time(self.time+self.EV_TrafficStatus_T), ["time"], only_once_in_em=True) )
				self.add_event(Event("OptimV",3, ev_reached_time(self.time+self.EV_OptimSpeed_T),["time"]))
			else:
				if 3 in self.iprintSet: print("[ handleBeginOfSeg ] TlsRequest intent. No tls1")
			self.tls2.clear() 
			nextSeg = self.curEdge.seg.nextSeg
			if self.do_Tls2 and nextSeg and nextSeg.ends_with_tls():
				tlsFromEdgeID = nextSeg.get_last_non_internal_edge().id
				if 3 in self.iprintSet: print("[ handleBeginOfSeg ] TlsRequest intent. tls2 config %s"%tlsFromEdgeID)
				self.tls2.configure(tlsFromEdgeID, self.e2tls[tlsFromEdgeID] )
				self.e2tls[tlsFromEdgeID].registerCar(self)
				self.requestTls2()
			else:
				if 3 in self.iprintSet: print("[ handleBeginOfSeg ] TlsRequest intent. No tls2")

			self.getTrafficStatusAhead( inCross_update = True )
			self.optimizeSpeed() # do it now, as well as schedule later.
			self.reset_coor_arr_t1(msg="in [ handleBeginOfSeg ]")# clear coor on new seg.


	def handleEndOfSeg(self,): # clear intents of previous segment.
		prevSeg = self.curEdge.seg.prevSeg
		if prevSeg is None: return
		if self.do_Tls1 and prevSeg.pop_intent("TlsMove") is not None:
			if 3 in self.iprintSet: print("[ handleEndOfSeg ] intent 1: TlsMove")
			lastt = prevSeg.lastTime.t # time of last update at seg.
			etime = prevSeg.minSpeed.t # time at minspd.
			etype = None
			if prevSeg.stopped_at_tls():
				if prevSeg.stopped_count()==1:
					etype, remdist = "acc", prevSeg.minSpeed.data["rem"]
					est_delay = max(remdist, 0.)/(self.VLen+self.minGap) * self.tDischarge	
					etime -= est_delay 
					if est_delay > 10:
						etype = None # too back in line, do not report.
			else: 
				etype,etime = "pas",lastt # not time of min speed, but last time.
				if etime is None: etime= self.time
			# upload to tls: fromEdge, toEdge, eventTime, etype
			if etype is not None:
				inedge = prevSeg.get_last_non_internal_edge()
				outedge = prevSeg.nextSeg.get_first_non_internal_edge()
				mov= TlsMove(inedge.id, outedge.id, EventTime(etime,relative=-1e4), etype)
				if inedge.id not in self.e2tls:
					pprint.pprint(self.route)
				if self.do_optimizeSpeed: 
					self.e2tls[inedge.id].ingestEvent(mov) # only cars using our system 
				if self.id.startswith('a') or self.do_optimizeSpeed:
					tlsid = self.e2tls[inedge.id].id
					wstr = "m tid=%s e=%s ne=%s vm=%.2f wt=%.1f tx=%.1f vid=%s ty=%s dl=%.1f\n"%(tlsid, inedge.id, outedge.id, prevSeg.minSpeed.data["v"], prevSeg.waitTime.data["wt"], etime, self.id, etype, est_delay if etype=='acc' else -1)  
					self.appendMoveFile(wstr) 
					if 2 in self.iprintSet:
						print(mov, mov.etype)
						print(wstr.rstrip())
		# remove from tls's inform list
		if self.do_optimizeSpeed and prevSeg.pop_intent("TlsDeReg") is not None:
			frE = prevSeg.get_last_non_internal_edge().id
			self.e2tls[ frE ].deregisterCar(self)
			if 5 in self.iprintSet:print("[ handleEndOfSeg ] intent 2: TlsDeReg tid %s"%self.e2tls[frE].id)
		# leave, also for not using ours:
		if prevSeg.pop_intent("leaveGroup") is not None: # both vtypes here.
			if self.id2group is not None and self.id in self.id2group:
				gr = self.id2group[self.id]
				gr.delete(self)
				self._calc_fuel_cost = False
				self._do_transaction = False
				if 9 in self.iprintSet: 
					print("[ handleEndOfSeg ] intent 3: leave gr",gr)
					gr.print_str = False
				if self.do_coor:
					self.initiate_coor(gr, msg='. And Left.')



	''' ------------- called multi times, enter only once: --------- '''

	def optimizeSpeed(self, **kwargs):
		if not self.do_optimizeSpeed: return
		redo = kwargs.get("redo",False)
		if not redo and self._optimizeSpeed: return
		self._optimizeSpeed = True
		if self.remdist_adapted_is_small():
			starting_acceleration = kwargs.get("starting_acceleration",False)
			# stopped at tls, just starting to move again:
			if starting_acceleration:
				tlsNode = self.curEdge.seg.get_last_non_internal_edge().endNode
				congested = self.is_intersection_congested(tlsNode.id)
				if 3 in self.iprintSet: 
					print("starting_acceleration tlsNode",tlsNode,"congested?",congested)
				if self.wrongLane>0:
					if 3 in self.iprintSet: print('   Gotta fix wrong lane !')
					return
				if congested: # hurry up
					speedCmd = max(config.InCrossMinSpeed, self.get_optimSpeed_2or1())
				else: 
					speedCmd = max(3 , self.get_optimSpeed_2or1())
				vcmd = self.speedControl.applySpeed(speedCmd)
				if 3 in self.iprintSet: 
					print("     after stop, vcmd",vcmd)
			# if stopping at tls line.
			if self.nowSpeed < config.StopSpeedThresh:# pred err stop
				self.tls1.predictedWrong = True 
			return # let go at last few seconds to tls.
		if not self.tls1.is_valid():
			if 3 in self.iprintSet: 
				print("Signal tls1 pred Seq is None...")
				print("congested?",self.is_intersection_congested(self.curEdge.endNode.id) )
			if self.wrongLane ==0: 
				self.applySpeedRaw(-1)
			self.tls1.optimSpeed = self.speedLimit
			return

		# select a proper stage:
		travelTime1 = self.get_time_to_seg_end_at_speedLimit()
		earliestArrivalTime1 = self.time + travelTime1
		if 3 in self.iprintSet: 
			print("I can arrive1 no earlier than %.1f "%(earliestArrivalTime1))
		found1 = self.tls1.findTargetInterval(self.time, travelTime1, self.preferStableTrafficEstimation)
		
		if found1 >0:
			tmpqStatus, tmpqDelay, tmpnumStop, tmpnumCars = self.trafficStatusDic['qStatus'],self.trafficStatusDic['delay'],self.trafficStatusDic['nstop'],self.trafficStatusDic['numCars']
			if self.preferStableTrafficEstimation and hasattr(self,'qStatus') and self.qStatus == Car.kQueMoving and tmpnumCars < self.numQueCars:
				if 3 in self.iprintSet: print("Avoid fluctuate traffic status, use old:",self.qStatus, self.qDelay, self.numStopCars, self.numQueCars)
			else: 
				self.qStatus, self.qDelay, self.numStopCars, self.numQueCars = tmpqStatus, tmpqDelay, tmpnumStop, tmpnumCars
			if self.approaching_seg_end_queue(): 
				if 3 in self.iprintSet: print("   Turn on preferStableTrafficEstimation")
				self.preferStableTrafficEstimation = True # do not get affected by discharging
			if 3 in self.iprintSet: 
				print('new qStat %d'%tmpqStatus, 'dt %.2f'%tmpqDelay, 'nstp %d'%tmpnumStop, 'ncar %d'%tmpnumCars)
				print("use tInc/qDelay: %.2f"%self.qDelay, 'stable?%s'%self.preferStableTrafficEstimation)
				
			# do not go slower due to queue, if too far :
			v0 = self.remainDist / max(0.1, self.tls1.targetStartTime + config.TlsStartTimeBuf + self.qDelay - self.time)
			if self.remainDist>300 and self.qStatus == Car.kQueMoving and self.qDelay>10 and v0<self.nowSpeed:
				timeIncrease = 10
				if 3 in self.iprintSet: print("Not go slower despite que, if too far. dt%.2f"%timeIncrease)
			elif self.qDelay>40:
				timeIncrease = 40
				if 3 in self.iprintSet: print("qDelay %.2f>40... dt <- %.2f"%(self.qDelay,timeIncrease))
			else:
				timeIncrease = self.qDelay

			delayedGreenStartTime1 = self.tls1.targetStartTime + config.TlsStartTimeBuf + timeIncrease 
			if delayedGreenStartTime1 >self.tls1.targetEndTime - config.TlsEndTimeBuf:
				timeIncrease = self.tls1.targetEndTime - config.TlsEndTimeBuf-(self.tls1.targetStartTime + config.TlsStartTimeBuf) # just for print later.
				if 3 in self.iprintSet: 
					print("Green time1 %.2f set to ph End !! %.2f"%(delayedGreenStartTime1,self.tls1.targetEndTime-config.TlsEndTimeBuf))
				delayedGreenStartTime1 = self.tls1.targetEndTime-config.TlsEndTimeBuf
			if 3 in self.iprintSet: 
				print("T1 phase stt %.1f, endt %.1f, ph1 stt+dt= %.1f"%(self.tls1.targetStartTime, self.tls1.targetEndTime, delayedGreenStartTime1))
				pprint.pprint(self.tls1.targetStage)
			earliestArrivalTime1 = max(earliestArrivalTime1, delayedGreenStartTime1)

			found2 = 0
			nextSeg = self.curEdge.seg.nextSeg
			if self.do_Tls2 and nextSeg:
				nextSegTime = nextSeg.get_seg_tavel_time()
				nextSegLen = nextSeg.get_seg_length()
				nextVLimit = nextSeg.get_seg_avg_speed_limit()
				found2 = self.tls2.findTargetInterval(earliestArrivalTime1, nextSegTime) 

			if found2>0:
				cannot_reach_vcmd = 0
				if self.tls1.targetEndTime - config.TlsEndTimeBuf- earliestArrivalTime1 < 5 and self.speedControl.cannot_reach_command_speed(how_long=1):
					if 3 in self.iprintSet: print("Can't reach cmd speed (ph end)", self.speedControl.cmdHistory)
					cannot_reach_vcmd = 1
				if self.speedControl.cannot_reach_command_speed(how_long=3):
					if 3 in self.iprintSet: print("Can't reach cmd speed (long)")
					cannot_reach_vcmd = 1
				if self.nowSpeed>config.CarMinSpeed and cannot_reach_vcmd>0 and self.do_coor and self.id in self.id2group: 
					gr = self.id2group[self.id]
					self.initiate_coor(gr, msg = 'cannot_reach_vcmd')


				self.fuelCost.set_max_speed(self.remainDist/travelTime1, nextVLimit) # set vlimit for seg1, seg2.

				intv1 = [earliestArrivalTime1, max(earliestArrivalTime1+0.5, self.tls1.targetEndTime-config.TlsEndArrivalTimeBuf) ] # arrive earlier than feasibility -buffer time.
				intv1quick = [earliestArrivalTime1, self.tls1.targetEndTime -config.TlsEndTimeBuf ] # nearly catch end, small buf, go fast if so.
				
				intv2 = [self.tls2.targetStartTime+config.TlsStartTimeBuf, max(self.tls2.targetStartTime+config.TlsStartTimeBuf+0.5 , self.tls2.targetEndTime-config.TlsEndArrivalTimeBuf)]
				intv2quick = [self.tls2.targetStartTime+config.TlsStartTimeBuf, self.tls2.targetEndTime-config.TlsEndTimeBuf ] # nearly catch end, go fast if so.

				attemptSpeed1, t1, v2, t2, fastest, catchLateIntv12 = self.fuelCost.find_v1t1v2t2_arrival_time_given_tls12(self.time, self.remainDist, intv1,intv1quick, nextSegLen, intv2,intv2quick, nextSegTime, stressed=cannot_reach_vcmd, print_str=3 in self.iprintSet)

				if 3 in self.iprintSet: 
					print("T2 ph stt %.1f, endt %.1f, v2=%.2f t2=%.1f"%(self.tls2.targetStartTime, self.tls2.targetEndTime, v2,t2))
					pprint.pprint(self.tls2.targetStage)
					print("t0 %.1f"%self.time, "v0 %.1f"%self.nowSpeed, "d1 %.1f"%self.remainDist, "intv1 %.1f-%.1f"%(intv1[0],intv1[1]), "d2 %.1f"%nextSegLen, "intv2 %.1f-%.1f"%(intv2[0],intv2[1]), "dT2 %.1f"%nextSegTime  )
					print("catch late intv end? intv1: %d,  intv2: %d"%tuple(catchLateIntv12))

				if cannot_reach_vcmd>0 and self.coorArriveT1>0 and abs(self.coorArriveT1 - t1)> 2:
					self.reset_coor_arr_t1(msg="disrupt, reset")
				if earliestArrivalTime1> self.coorArriveT1 + 20 and self.coorArriveT1>0:
					self.reset_coor_arr_t1(msg="que bigger, reset")


				if self._calc_fuel_cost:
					if self.trafficStatusDic['nstop'] >0 and self.trafficStatusDic['qStatus']==Car.kQueStill: # use fresh stats. avoid using stable stats
						hardDelay = timeIncrease
					else:
						hardDelay = 0. # no red light, try push.
					intv1st = max(self.time + travelTime1, self.tls1.targetStartTime + config.TlsStartTimeBuf + hardDelay)
					intv1NoQue = [intv1st, self.tls1.targetEndTime - config.TlsEndTimeBuf] # no que, late end allow.
					intv2NoQue =intv2quick # allow late intv2 end

					_, t1NoQue, _, _, _,_ = self.fuelCost.find_v1t1v2t2_arrival_time_given_tls12(self.time, self.remainDist, intv1NoQue,intv1NoQue, nextSegLen, intv2NoQue,intv2NoQue, nextSegTime, stressed=cannot_reach_vcmd, print_str= False)
					self.arrT1NoQue = t1NoQue # if no que.
					self.rear_delay_gap = self.get_rear_delay_gap()
					self.fuelCost.get_cost_tls1_tls2(self.time, self.nowSpeed, self.remainDist, intv1NoQue , nextSegLen, intv2NoQue, tminSeg2=nextSegTime)
					if 9 in self.iprintSet: 
						print('nstop:%d'%self.numStopCars,'fc dt:%.2f'%hardDelay,'st1:%.2f'%intv1st)
						print('calc_cost t1noQ %.2f'%t1NoQue,'rear gap %2f'%self.rear_delay_gap)
						if self.fuelCost.minInd is not None:print('calc_cost min t,c', self.fuelCost.tcosts[self.fuelCost.minInd])
				
				if 9 in self.iprintSet: 
					print('fuel_cost  t1 ori: %.2f,  coT1 %.2f'%(t1, self.coorArriveT1))
				if self.coorArriveT1>0:
					t1 = self.coorArriveT1
					attemptSpeed1 = self.remainDist / max(0.1, t1 - self.time)
					if 9 in self.iprintSet: print("  ori tls2 attemptSpeed1: %.5f"%attemptSpeed1)
				self.tls2.optimSpeed = v2 
				attemptArriveTime1 = t1 
				timeToArrive1 = t1 
				if fastest>0: 
					attemptSpeed1 = self.speedLimit # has to stick to limit all the way.
					if 3 in self.iprintSet: print("  go fastest %d"%fastest)
				self.arrT1que = timeToArrive1
				self.arrT1NoQue = self.tls1.targetStartTime + config.TlsStartTimeBuf

			else:
				attemptArriveTime1 = delayedGreenStartTime1 # allow  < curtime. could be very early
				if 3 in self.iprintSet: print("T2 not found"+ ", nextSeg is None" if nextSeg is None else "")
				timeToArrive1 = earliestArrivalTime1 # true arrival time.
				attemptSpeed1 = self.remainDist / max(0.1, attemptArriveTime1 - self.time)# could be large
				if 3 in self.iprintSet: print("ori tls1 attemptSpeed1:",attemptSpeed1)

			# modify attempt speed if inside congested cross.
			if self.inCross and self.curEdge.is_inside_tls():
				curNode = self.curEdge.endNode
				congested = self.is_intersection_congested(curNode.id)
				if 3 in self.iprintSet: print("  is_inside_tls, node",curNode,"congested?",congested)
				if attemptSpeed1 < self.nowSpeed: # do not slow down
					if 3 in self.iprintSet:print("inCross Not slow down fr %.3f to %.3f"%(self.nowSpeed,attemptSpeed1))
					attemptSpeed1 = self.nowSpeed
				if congested: # hurry up if necessary.
					if 3 in self.iprintSet: print("congested, attemptSpeed1 = max(%.1f,%.1f)"%(config.InCrossMinSpeed, attemptSpeed1) )
					attemptSpeed1 = max(config.InCrossMinSpeed, attemptSpeed1)

			if self.wrongLane==0:
				vcmd = self.speedControl.applySpeed(attemptSpeed1)
				self.tls1.optimSpeed = min(self.speedLimit,vcmd)
			else: 
				if 3 in self.iprintSet: print("Fixing wrong lane. No applySpeed.")
				vcmd = None
				self.tls1.optimSpeed = self.speedControl.clampSpeed(attemptSpeed1)

			if 3 in self.iprintSet:
				print("remainDist %.1f. "%(self.remainDist) + "tInc %.1f, t1|tls2 %.1f, earliest t1 %.1f"%(timeIncrease, timeToArrive1, earliestArrivalTime1) )
				print("Tls1 vcalc",attemptSpeed1,"vcmd",vcmd,"vo1",self.tls1.optimSpeed,"vr",self.nowSpeed)
			
			# log signal pred timing. 
			if not hasattr(self,'last_tls_log_time') or self.time-self.last_tls_log_time> self.log_tls_pred_period:
				self.last_tls_log_time = self.time
				self.writeTlsPredLog(self.curEdge, self.tls1, timeToArrive1, self.remainDist)
				if found2>0:
					nextnextSeg = self.curEdge.seg.nextSeg.nextSeg
					if nextnextSeg:
						self.writeTlsPredLog(self.curEdge.seg.nextSeg.get_last_non_internal_edge(), self.tls2, -1, self.remainDist+nextSegLen)

		else:
			if 3 in self.iprintSet: 
				print("-- ? ? ? tls1 fromEdge1 no match...")
				pprint.pprint(self.tls1.tlsPredSeq)
			if self.tls1.tlsPredSeq[-1][0]<self.time+50:
				self.add_event(Event("RequestTls",1, ev_reached_time(self.time), ["time"], allow_duplicate=True, data_dic={"tlsid":self.tls1.tlsid}))
			

	def requestTls1(self,):
		if not self.do_Tls1: return
		if self._requestTls1: return
		self._requestTls1 = True
		if self.wrongLane>0: return
		dic = self.requestTlsPrediction(self.tls1.tlsFromEdgeID, self.remainDist/max(4,self.nowSpeed) + 90. ) 
		self.tls1.load_prediction(dic)
		if 3 in self.iprintSet: print("[ requestTls1 ] v%d, %s"%(self.tls1.version, self.tls1.status))
		if self.tls1.is_valid():
			self.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]))


	def getTrafficStatusAhead(self, **kwargs): 
		if not self.do_optimizeSpeed: return
		if self._getTrafficStatusAhead: return
		self._getTrafficStatusAhead = True
		if self.wrongLane>0: return
		# in cross: only begin of seg does init update here.
		if self.inCross and not kwargs.get("inCross_update",False): return
		# get possible cars in front leading to slow down. Return nothing.
		if 3 in self.iprintSet: print("[getTrafficStatusAhead]")
		delayTime = 0. 
		nstop = 0 # cars not moving in front, to delay time
		numCars = 0 # total num ahead
		isMoving = 1 # first car stopped or moving.
		tmpE = self.curEdge
		while 1: # iter through edges till end of seg:
			maxDelay, maxNStop, maxNc = 0.,0, 0 # take max among correct lanes
			hasStop = 0 # check at last seg edge.
			for lane in tmpE.get_traffic_status_lanes(): # all best/via lanes.
				if tmpE==self.curEdge:
					if self.curLane != lane: # if right lane, care only cur lane.
						if 7 in self.iprintSet: print("skipping lane",lane,"because mine is",self.curLane)
						continue 
				if tmpE==self.curEdge:
					dt,ns,ncar = self.getQueueDelay(lane, minLanePos= self.lanePos)
				else: 
					dt,ns, ncar = self.getQueueDelay(lane)
				if dt>= maxDelay:
					maxDelay,maxNStop,maxNc = dt,ns , ncar
				hasStop = max(ns,hasStop)
			nstop += maxNStop
			delayTime += maxDelay
			numCars += maxNc
			if 8 in self.iprintSet: print("Edge que,dt,Ns,#car",tmpE.id,maxDelay,maxNStop,maxNc)
			if tmpE.is_last_in_seg(): 
				if hasStop>0: isMoving = 0 # last edge is tls, stop means red light.
				break
			tmpE = tmpE.nextEdge
		if isMoving >0: # cars moving 
			qStatus = Car.kQueMoving
		else: qStatus = Car.kQueStill
		if 3 in self.iprintSet: 
			print("qStatus",qStatus," delay %.2f"%delayTime,"nstop",nstop,"numCars",numCars)
		self.trafficStatusDic["qStatus"], self.trafficStatusDic["delay"], self.trafficStatusDic["nstop"], self.trafficStatusDic["numCars"] = qStatus, delayTime, nstop, numCars


	def requestTls2(self,):
		if not self.do_Tls2: return
		if self._requestTls2: return
		self._requestTls2 = True
		if self.wrongLane>0: return
		nextSegLen = self.curEdge.seg.nextSeg.get_seg_length()
		dic = self.requestTlsPrediction(self.tls2.tlsFromEdgeID, (nextSegLen+self.remainDist)/max(3,self.nowSpeed) + 60. )
		self.tls2.load_prediction(dic)
		if 3 in self.iprintSet: print("[ requestTls2 2 ] v%d, %s"%(self.tls2.version, self.tls2.status))


	''' ------------- critical, called by main every step: ------------- '''

	def checkLane(self,):
		if 3 in self.iprintSet: print("[ checkLane ]")
		fromE = self.curEdge.seg.get_last_non_internal_edge()
		nextSegE = fromE.get_next_non_internal_edge() # none if last segment.
		nowLaneIndex = self.curLane.ind
		self.bestLaneIndices = self.curEdge.get_lane_choices("best") #on current edge 
		if 3 in self.iprintSet: 
			print("curE:", self.curEdge.id, self.curEdge.laneIndices)
			if 6 in self.iprintSet: 
				if nextSegE: print("Tls move ahead: %s %s"%(fromE.id,nextSegE.id))
				print("to next lane map:", self.curEdge.lid2next)
				print("At tls:",fromE.conns)
		if self.inCross:  
			self.wrongLane, self.wrongLaneOffNum = 0,0
			return
		if self.do_optimizeSpeed: 
			targetIndex = sorted([(abs(i-nowLaneIndex),i) for i in self.bestLaneIndices])[0][1]
			canUnLock = False
			if self.curEdge.has_intent("lockLane"):
				self.curEdge.pop_intent("lockLane") # once per edge, not inCross
				if 3 in self.iprintSet:
					print("  Pop intent (lockLane)")
					if nowLaneIndex != targetIndex: 
						print("\a")
						print("    Wrong Lane! change %d -> %d "%(nowLaneIndex,targetIndex))
				self.tryChangeLane(targetIndex,1e3)
			else: canUnLock=True
			if not canUnLock:
				if self.laneRemDist/max(0.1, self.nowSpeed) <1.999: # BUG 
					if 3 in self.iprintSet: print("Too fast! ! Unlock lane now! rL", self.laneRemDist)
					canUnLock=True
			if canUnLock and self.curEdge.has_intent("releaseLane"):
				if self.wrongLane==0 and self.about_to_finish_lane(time_ahead = 3): # >=3 seconds ahead !!!
					self.curEdge.pop_intent("releaseLane") # must free lane no matter what.
					if 3 in self.iprintSet: print("  End of lane, free %d"%nowLaneIndex)
					self.tryChangeLane(nowLaneIndex,0 )
		if nowLaneIndex in self.bestLaneIndices:
			if self.wrongLane>0: # wrongLane needs update even if not using our system.
				if self.do_optimizeSpeed: 
					if self.tls1.optimSpeed:
						vcmd = max(config.CarMinSpeed ,max(self.nowSpeed, self.tls1.optimSpeed))
						self.speedControl.applySpeed(vcmd)
					else: 
						vcmd = -1
						self.applySpeedRaw(vcmd)
					self.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]))
					if 3 in self.iprintSet: print("    Clear wrongLane=0 best,  v =%.1f"%vcmd)
			self.wrongLane = 0
			self.othersWrongLane.clear()
			return
		self.correctLaneIndices = self.curEdge.get_lane_choices("correct") # current edge 
		if nowLaneIndex in self.correctLaneIndices:
			canTolerateLane, msg = 0,"" # can i go in correct lane instead of best lane?
			if self.remainDist > config.RemDistEnforceBestLane:
				canTolerateLane, msg = 1, "still far"
			if canTolerateLane==0:
				remainEdges = self.curEdge.get_num_remain_edges_in_seg()
				if remainEdges==0: # can go with suboptimal lane.
					canTolerateLane, msg = 1, "last edge correct"
				else:
					eventualCorrectLaneInds= self.curEdge.get_correct_lane_inds_at_seg_end()
					if nowLaneIndex in eventualCorrectLaneInds:
						canTolerateLane, msg = 1, "last edge allows"
			if canTolerateLane>0:
				if 3 in self.iprintSet: print("But in correct lanes:", self.correctLaneIndices, msg)
				if self.wrongLane>0:
					if self.do_optimizeSpeed: 
						if self.tls1.optimSpeed:
							vcmd = max(config.CarMinSpeed ,max(self.nowSpeed, self.tls1.optimSpeed))
							self.speedControl.applySpeed(vcmd)
						else: 
							vcmd = -1
							self.applySpeedRaw(vcmd)
						self.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]))
						if 3 in self.iprintSet: print("    Clear wrongLane = 0 cor,  v =%.1f"%vcmd)
				self.wrongLane = 0
				self.othersWrongLane.clear()
				return
			if 3 in self.iprintSet: print("Still wrong lane, remD, remE",self.remainDist,remainEdges)
		self.wrongLane +=1
		if self.do_optimizeSpeed: 
			self.tryChangeLane( targetIndex,1e3)
			if 3 in self.iprintSet: 
				print("Wrong+%d! force %d -> %d "%(self.wrongLane,nowLaneIndex,targetIndex))


	def check_teleport(self,):
		if self.remainDist > self.minGap or self.nowSpeed> config.StopSpeedThresh:
			self.stuck_time=0
			return
		if self.inCross or not self.curEdge.ends_with_tls(): return # not tls.
		fromLaneID = self.curLane.id # now lane id 
		if 3 in self.iprintSet: 
			print("[ check_teleport ] "+self.id)
		if self.curLane.ind in self.curEdge.lid2next: # correct lane
			fromLaneID, toLaneID, via = self.get_via_link( self.curEdge )
			state,_ = self.get_signal_given_link(fromLaneID, toLaneID, via)
			if state.lower()=='g':
				if self.stuck_time>9 or 3 in self.iprintSet: print("\nStuck alert %d !!!! "%self.stuck_time +self.id+" "+state+" "+str((fromLaneID, toLaneID, via)))
				self.stuck_time+=1
				self.curEdge.seg.pop_intent("TlsMove") # do not report move.
				if self.stuck_time>=11: 
					if 3 in self.iprintSet:  print(self.id+" report congestion.")
					junction = self.curEdge.endNode
					feedback_incross_stops(Car.feedbackData, self.time, junction)
					if G_Do_Show_GUI: GUI_show(self)
					G_removed_cars.append([self.id , self.time] )
					if do_interrupt(self):
						raw_input("Enter key... %d"%self.stuck_time)
					self.remove(output=False)
					return
		else: # wrong link, any green link will cause this kill !
			if 3 in self.iprintSet: print("[ check_teleport ] Wrong lane %d !"%self.wrongLane, self.id)
			if self.wrongLane>= Car.RemoveWongLaneThresh:
				ret = query_sumo_out_edges(self.curEdge.id, Server_IP_Port)
				for toEdge, conlst in ret.items():
					for conn in conlst:
						if self.curLane.ind == int(conn["fromLane"]):
							toLaneID = toEdge+"_"+str(conn["toLane"])
							via = str(conn["via"])
							state,_ = self.get_signal_given_link(fromLaneID, toLaneID, via)
							if 3 in self.iprintSet:  
								print(fromLaneID, toLaneID, via, "state",state)
							if state.lower()=='g':
								print("\nStuck dead "+self.id+" "+state+" "+str((fromLaneID, toLaneID, via)))
								if 3 in self.iprintSet:  print(self.id+" report congestion.")
								junction = self.curEdge.endNode
								feedback_incross_stops(Car.feedbackData, self.time, junction)
								if G_Do_Show_GUI: GUI_show(self)
								G_removed_cars.append([self.id, self.time ] )
								if  do_interrupt(self): # bad , give up.
									raw_input("Enter key...")
								self.remove(output=False)
								return
		

	''' ------------- util, called by other func: ------------- '''


	def getQueueDelay(self, laneObj, minLanePos=-1): 
		delay = 0. # cars in front 
		nstop = 0 # cars not moving in front
		num = 0
		que = self.lane2vq.get(laneObj.id,[]) # sorted by lanePos
		if 5 in self.iprintSet and len(que)>0: 
			print("Queue "+ laneObj.id, que)
		firstAffectCar, qHeadStopped = None,0  
		for i in range(len(que)-1,-1,-1):
			if que[i].lanePos > minLanePos: # in front of me.
				car = que[i]
				# tls1 available? if not, do not skip. skip only if both tls1 pred valid, and he is aiming at previous tls cycle:
				pendingTls = not self.tls1.has_found_target() or not car.tls1.has_found_target()
				shouldConsider = pendingTls or self.tls1.predictedWrong or car.tls1.predictedWrong
				try:
					if (not shouldConsider  and  car.tls1.targetStartTime < self.tls1.targetStartTime-10 and car.nowSpeed> config.StopSpeedThresh ): 
						# he will be long gone in a cycle ahead.
						if 5 in self.iprintSet: print(" ignore",car,car.tls1.targetStartTime," mine st1",self.tls1.targetStartTime)
						continue
				except: 
					print(self, self.tls1, 'getQueueDelay err')
					print(car, car.tls1)
					sys.exit(1)
				# check if we will eventually meet at same lane?:
				if car.inCross and len(car.curEdge.get_best_lane_inds_at_seg_end() & self.curEdge.get_best_lane_inds_at_seg_end())==0:
					if 5 in self.iprintSet: print(" ignore other lane",car)
					continue

				if firstAffectCar is None: 
					firstAffectCar = car
					if firstAffectCar.remainDist<25 and firstAffectCar.nowSpeed< config.StopSpeedThresh:
						qHeadStopped = 1
						if 5 in self.iprintSet : print("firstAffectCar stopped",firstAffectCar)
				
				if car.nowSpeed < config.StopSpeedThresh:
					if qHeadStopped: 
						nstop +=1 
					tmpv = car.get_optimSpeed_2or1()
					if tmpv<= config.CarMinSpeed: tmpv = max(config.MinSpeedForAcc, config.CarMinSpeed )
					reachSpeed= min( tmpv, 10.)
					moveSpace = car.VLen + car.get_space_gap_behind(High_Speed_Gap_Offset=4., use_speed=reachSpeed)
					delay += moveSpace/reachSpeed
					if 5 in self.iprintSet: 
						print("  stopped",car,"d%.1f/v%.1f"%(moveSpace,reachSpeed),"+d %.1f, dt= %.1f"%(moveSpace,delay))
				else: # cars in front of me moving
					tmpv = max(car.nowSpeed, car.tls1.optimSpeed)
					if tmpv<= config.CarMinSpeed: tmpv = max(config.MinSpeedForAcc , config.CarMinSpeed )
					reachSpeed = tmpv
					moveSpace = car.VLen + car.get_space_gap_behind(High_Speed_Gap_Offset=4., use_speed=reachSpeed)
					delay += moveSpace/reachSpeed
					if 5 in self.iprintSet: 
						print("  moving",car,"d%.1f/v%.1f"%(moveSpace,reachSpeed),"+d %.1f, dt= %.1f"%(moveSpace,delay))
				num+=1
		if 5 in self.iprintSet and len(que)>0: 
			print("    :dt %.1f, ns %d, nc %d"%(delay, nstop, num))
		return [delay, nstop, num]


	def requestTlsPrediction(self, tlsFromEdgeID, travelTime):
		if tlsFromEdgeID in self.e2tls:
			tls = self.e2tls[tlsFromEdgeID]
			dic = tls.getFuturePhaseSeq(self.time + travelTime, who=self.id)
			if 4 in self.iprintSet: print("tls seq", dic["seq"],"v",dic["version"],dic["status"])
			if 3 in self.iprintSet: 
				print("Requesting future time %.1f"%(self.time + travelTime))
				print(tls.id +" maxFutureTime %.1f, keepPredicting?%s"%(tls.maxFutureTime,str(tls.keepPredicting)))
		else:
			if 3 in self.iprintSet: print(tlsFromEdgeID+' not ending w/ tls.')
		return dic


	def updateLaneQueue(self,): # fill in vehicle queues on lanes. # assume they know/see other cars.
		if self.lane2vq is None: return
		if hasattr(self,"lastQueueLane") and self.lastQueueLane == self.nowLaneId:
			return
		if self.nowLaneId not in self.lane2vq:
			self.lane2vq[self.nowLaneId] = []
		self.lane2vq[self.nowLaneId].append(self)
		if hasattr(self,"lastQueueLane") and self.lastQueueLane in self.lane2vq:
			self.lane2vq[self.lastQueueLane].remove(self)
		self.lastQueueLane = self.nowLaneId


	def formGroup(self, ): 
		enter, msg = self.can_do_coor()
		if not enter: 
			if 9 in self.iprintSet: print("[ formGroup ] skip... "+msg)
			self._do_transaction = False
			self._calc_fuel_cost = False
			return
		if 9 in self.iprintSet: print("[ formGroup ]")
		if not self.id in self.id2group: 
			endNodeID = self.curEdge.seg.get_node_id_at_seg_end()
			gr = Group(endNodeID, self)
			gr.append(self) # added to id2group too.
		else: 
			gr = self.id2group[self.id]
			endNodeID = gr.endNodeID
		if 9 in self.iprintSet: gr.print_str = True
		changed = 0
		while 1: # iter through edges till far dist
			for lane in [self.curLane]:
				minLanePos = self.lanePos 
				minDelay = 0.
				que = self.lane2vq.get(lane.id,[]) # sorted by lanePos
				if 9 in self.iprintSet: print('que',que)
				for i in range(len(que)):# scan from behind to leading front.
					car = que[i]
					if car.lanePos <= minLanePos: 
						continue # not in my front 
					minDelay += car.get_rear_delay_gap()
					if car.wrongLane>0: 
						if 9 in self.iprintSet: print('- wrongLane',car.wrongLane)
						break
					if not car.curEdge.seg.ends_with_tls(): 
						if 9 in self.iprintSet: print('- not ends_with_tls',car)
						break
					if endNodeID != car.curEdge.seg.get_node_id_at_seg_end(): 
						if 9 in self.iprintSet: print('- endNodeID !=',car.curEdge.seg.get_node_id_at_seg_end())
						break
					if car.do_coor:
						if not hasattr(car,'arrT1NoQue'):
							if 9 in self.iprintSet: print('- no arrT1NoQue',car)
							break
						if self.arrT1NoQue > car.arrT1NoQue +0.5:
							if 9 in self.iprintSet: 
								print('my t1 noQ %.2f'%self.arrT1NoQue+', his t1 noQ %.2f'%car.arrT1NoQue)
								print('- dt %.1f  not pushing if noQ'%minDelay,car)
								print('But my t1Q %.2f'%self.arrT1que+', his t1Q %.2f'%car.arrT1que)
							break
					if 'coD' in R: Same_Group_Dist = R['coD']
					else: Same_Group_Dist = addr2Same_Group_Dist[config.osmfile]
					if get_2xy_dist(self.xy, car.xy) < Same_Group_Dist:# include not using our sys.
						if car.id in self.id2group: 
							hisGroup = self.id2group[car.id]
							if gr.id == hisGroup.id:
								continue # scan front next, don't break.
							if hisGroup.endNodeID == gr.endNodeID:
								if 9 in self.iprintSet: print(self.id, 'Merging hisGroup', hisGroup)
								gr.merge_front_into_mine(hisGroup, print_str=False)  
								changed = 1
							else:
								if 9 in self.iprintSet: 
									print('Cut at Group',car.id, hisGroup, 'nd',hisGroup.endNodeID)
									print('But my Group',self.id, 'nd', gr.endNodeID)
							break
						else:
							gr.append(car)
							changed  =1
							if 9 in self.iprintSet: print('+ Group Append', car)
					else:
						if 9 in self.iprintSet: print('- get_2xy_dist >', car)
						break
			break # just current lane

		if 9 in self.iprintSet: print('group:',gr,"changed?",changed)
		if 'coG' in R: Coor_Max_Time_Gap = R['coG']
		else: Coor_Max_Time_Gap = addr2Coor_Max_Time_Gap[config.osmfile]
		if changed>0 or self.time - gr.last_success_time >= Coor_Max_Time_Gap:
			self.initiate_coor(gr)


	def transaction(self,): # CoDrive
		if not self._do_transaction: return
		enter, msg = self.can_do_coor()
		if not enter: 
			if 9 in self.iprintSet: print("[ transaction ] skip... "+msg)
			self._do_transaction = False
			self._calc_fuel_cost = False
			return
		if not self.id in self.id2group: return
		gr = self.id2group[self.id]
		if len(gr.gr)==1: return
		print_str =  9 in self.iprintSet or gr.print_str
		if 'coT' in R: Coor_Period = R['coT']
		else: Coor_Period = addr2Coor_Period[config.osmfile]
		if self.time - gr.last_success_time < Coor_Period:
			if print_str: print("[ transaction ] < Coor_Period. %s skip..."%self.id)
			return
		gr.last_success_time = self.time
		if print_str: print("[ transaction ] ")
		for i in range(len(gr.gr)):
			if 9 in gr.gr[i].iprintSet and not print_str: 
				print("[ transaction ] ")
				print_str =True
			if gr.gr[i].do_coor:
				if not gr.gr[i].tls1.has_found_target():
					if print_str: print(gr.gr[i],"no tls1 ...")
					return
				if not gr.gr[i].tls2.has_found_target(): 
					if print_str: print(gr.gr[i],"no tls2 ...")
					return
				if not gr.gr[i]._calc_fuel_cost: 
					if print_str: print(gr.gr[i],"no _calc_fuel_cost ...")
					return
				if gr.gr[i].remdist_adapted_is_small():
					if print_str: print(gr.gr[i],"remd too small ...")
					return
		if print_str: print("gr: "+str(gr)+"  | I'm "+self.id)
		cars, delays, costs = [],[],[]
		tmpdelay= 0.
		i = len(gr.gr)-1
		while i>=0:
			car = gr.gr[i]# need to check if using our sys!
			if not car.do_coor:
				i -=1
				if len(cars)==0: continue
				tmpdelay += car.get_rear_delay_gap() # not using ours
				continue
			if len(cars)>0: # append delay before proc next car
				delays.append(tmpdelay)
				tmpdelay = 0.
			if not hasattr(car, 'rear_delay_gap'):
				tmpdelay += car.get_rear_delay_gap()
			else: 
				tmpdelay += car.rear_delay_gap
			cars.append(car)
			costs.append(car.fuelCost.tcosts)
			i-=1
		if len(cars)<=1:
			if print_str: print("gr just me, skip... "+self.id)
			return
		leaderArrT1que = cars[0].arrT1que
		leaderEarliestArrT1 = cars[0].arrT1NoQue
		if print_str: 
			print(cars)
			print(delays)
			print('1st car arrT1 noQ %.2f'%leaderEarliestArrT1, 't1Q %.2f'%leaderArrT1que)
		# assume optimal is at one of individual's local optimal.
		choices = [] 
		for i in range(len(cars)): # fix one car at its optimal, shift others.
			cid2arrt = {} # log arrival time1
			car = cars[i]
			try:
				t1i, csum = car.fuelCost.get_min_time_cost()
			except:
				print(car,'get_min_time_cost err')
				break # regard as INF, no solution for this.
			cid2arrt[car.id] = t1i
			if print_str: 
				print('---- fix %d '%i + car.id+' minc,t (%.2f, %.2f)'%(csum, t1i)+ ' t1Q:%.2f'%car.arrT1que + ' t1noQ:%.2f'%car.arrT1NoQue)
			# front cars
			t1 = t1i
			j=1
			while i-j>=0:
				tmpc = cars[i-j]
				t1 -= delays[i-j]
				tc = tmpc.fuelCost.get_cost_at_time(t1)
				cid2arrt[tmpc.id] = t1
				if print_str: print(i-j, tmpc.id,'car front', tc, t1)
				csum +=tc
				j+=1
			car1arrt1 = t1
			if car1arrt1 < leaderArrT1que:
				# need to push later
				dt = leaderArrT1que - car1arrt1
				t1i += dt
				cid2arrt[cars[i].id] = t1i
				if print_str: print("-- adjust back dt %.2f"%dt)
				t1 = t1i
				csum = cars[i].fuelCost.get_cost_at_time(t1)
				if print_str: print(i, cars[i].id,'car fix', csum, t1)
				j=1
				while i-j>=0:
					tmpc = cars[i-j]
					t1 -= delays[i-j]
					tc = tmpc.fuelCost.get_cost_at_time(t1)
					cid2arrt[tmpc.id] = t1
					if print_str: print(i-j, tmpc.id,'car front', tc, t1)
					csum +=tc
					j+=1
			car1arrt1 = t1
			# behind ones
			t1 = t1i
			j=1
			while i+j<len(cars):
				tmpc = cars[i+j]
				t1 += delays[i+j-1]
				tc = tmpc.fuelCost.get_cost_at_time(t1)
				cid2arrt[tmpc.id] = t1
				if print_str: print(i+j, tmpc.id,'car behind', tc, t1)
				csum += tc
				j+=1
			dic ={'c':csum, 't':car1arrt1, 'i':i, 'at':cid2arrt}
			choices.append(dic)
			if print_str: 
				print('leader %s arrt1'%cars[0].id , car1arrt1, '>=',leaderArrT1que)
				print('choice',choices[-1])
		# find best:
		minc , dic = 1e6 , None
		for d in choices:
			if d['c']>=INF: continue
			if d['c']<minc:
				minc = d['c']
				dic = d
		if minc < INF:
			optimDelay = dic['t']-leaderArrT1que
			if print_str:
				print("Best t1: %.2f"%dic['t'],'mincost %.2f'%minc, 'coDt %.2f'%optimDelay) 
			for car in cars:
				car.set_coor_arr_t1(dic['at'][car.id], print_str=print_str, msg =' me '+car.id+' By '+self.id )
				car.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]), print_str=gr.print_str) #exec at next step.
			self._do_transaction = False
			self._calc_fuel_cost = False
			return
		# g_optim is at mid somewhere. search brute force.
		t1 = leaderArrT1que
		try:
			tmax = cars[0].fuelCost.tcosts[-1][0]
		except:
			print(cars[0],'do not have fuelCost',cars[0].fuelCost.tcosts)
			tmax = 0
		minc , car1arrt1 = 1e6 , None
		while t1 <= tmax:
			tmpsum , tmpdelay = 0,0
			for i in range(len(cars)):
				tmpsum += cars[i].fuelCost.get_cost_at_time(t1+tmpdelay)
				if i<len(delays): tmpdelay += delays[i]
				if tmpsum >= INF:
					break
			if tmpsum < 1.5*INF:
				if print_str: print('t1 %.2f '%t1+ cars[i].id+ ' arrt1 %.2f, csum %.1f'%(t1+tmpdelay, tmpsum))
				if tmpsum<INF and minc > tmpsum:
					minc = tmpsum
					car1arrt1 = t1
			t1+= DT
		if minc< INF:
			optimDelay = car1arrt1-leaderArrT1que
			if print_str:
				print("Best t1: %.2f"%car1arrt1,'mincost %.2f'%minc, 'coDt %.2f'%optimDelay) 
			tmpdelay =0
			for i in range(len(cars)):
				car = cars[i]
				car.set_coor_arr_t1(car1arrt1+tmpdelay, print_str=print_str, msg =' me '+car.id+' By '+self.id )
				car.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]), print_str=gr.print_str) 
				if i<len(delays): tmpdelay += delays[i]
			self._do_transaction = False
			self._calc_fuel_cost = False
		else:
			if print_str: print('Coor fail again ...' + self.id)
		self._calc_fuel_cost = False
		self._do_transaction = False



	def initiate_coor(self, gr, msg=''):
		if 9 in self.iprintSet: 
			gr.print_str = True
		if gr.print_str:
			print("%s initiate coor gr %s "%(self.id,gr)+msg)
		for car in gr.gr:
			if not car.do_coor: continue
			car._calc_fuel_cost=True # allow update in next for loop at same time/step
			car._do_transaction= True
			car.add_event(Event("OptimV",3, ev_reached_time(self.time), ["time"]), print_str=gr.print_str) #exec at same step.

	def can_do_coor(self,):
		if not self.do_optimizeSpeed: return False, 'No using ours'
		if not self.do_coor: return False, 'No coor'
		if self.wrongLane>0: return False, 'Wronglane'
		if self.nowSpeed< config.CarMinSpeed: return False, 'Low speed'
		if not self.curEdge.seg.ends_with_tls(): return False, 'No tls'
		if not self.tls1.is_valid(): return False, 'invalid tls1'
		if not self.tls2.is_valid(): return False, 'invalid tls2'
		if not hasattr(self,'arrT1que'): return False, 'no arrT1que'
		return True,''

	def get_rear_delay_gap(self, print_str=False):
		reachSpeed= self.nowSpeed
		if self.tls1.optimSpeed is not None:
			reachSpeed = max(reachSpeed, self.tls1.optimSpeed)
		if reachSpeed<= config.CarMinSpeed: reachSpeed = max(config.MinSpeedForAcc , config.CarMinSpeed )
		moveSpace = self.VLen + self.get_space_gap_behind(High_Speed_Gap_Offset=4., use_speed=reachSpeed)
		if print_str: print(self.id+ " space %.2f"%moveSpace,"v %.2f"%reachSpeed )
		self.rear_delay_gap = moveSpace/reachSpeed
		return self.rear_delay_gap

	def set_coor_arr_t1(self, t1, print_str=False, msg=''):
		self.coorArriveT1 = t1
		if 9 in self.iprintSet or print_str: print('coorArriveT1 set to %.2f. '%t1 + msg)

	def reset_coor_arr_t1(self, print_str=False, msg=''):
		self.coorArriveT1 = -1
		if 9 in self.iprintSet or print_str: print('coorArriveT1 reset -1. '+msg)

	def get_optimSpeed_2or1(self,):  
		if self.tls2.optimSpeed:
			return self.tls2.optimSpeed
		else: return self.tls1.optimSpeed 

	def get_signal_given_link(self, fromLaneID, toLaneID, via):
		tls = self.e2tls[ fromLaneID.rsplit("_",1)[0] ]
		state, linkIndex = tls.get_link_state(fromLaneID, toLaneID, via)
		return state, linkIndex

	def about_to_finish_lane(self, time_ahead=1): # will change edge in next few steps.
		estAcc = max(0.,self.speedControl.get_acc())
		dist_next_second = time_ahead* (self.nowSpeed+ estAcc)
		if dist_next_second> self.laneRemDist:
			if 3 in self.iprintSet: print("   About_to_finish_lane() can go %.1f in %.1f sec."%(dist_next_second,time_ahead))
			return True
		return False

	def approaching_seg_end_queue(self,):
		return self.remainDist<100 or (self.remainDist<140 and self.curEdge.get_num_remain_edges_in_seg()==0)
	def remdist_adapted_is_small(self,):
		return self.remainDist < min(config.TlsLetGoRemDist, 5+6*self.nowSpeed)

	def get_time_to_seg_end_at_speedLimit(self,):
		tavel_time_from_next_edge_to_seg_end = self.curEdge.get_tavel_time_from_next_edge_to_seg_end()
		remainTime = self.laneRemDist/self.speedLimit + tavel_time_from_next_edge_to_seg_end
		if 3 in self.iprintSet:
			print('L %s'%self.nowLaneId, 'LremD%.2f'%self.laneRemDist, 'nexEtime%.1f'%tavel_time_from_next_edge_to_seg_end, 'fasttToTls%.1f'%remainTime)
		return remainTime

	def get_space_gap_behind(self, High_Speed_Gap_Offset=4., Low_Speed_Least_Gap = 2., use_speed=None):
		if use_speed is None:
			use_speed = self.nowSpeed
		return get_adjusted_car_gap(self, min( High_Speed_Gap_Offset+ use_speed, max(Low_Speed_Least_Gap, config.CarGapPerSpeed*use_speed) ) )

	def getVia(self,): # Bug, got empty
		return Car.traci.vehicle.getVia(self.id)
	def getNextTLS(self,):
		return Car.traci.vehicle.getNextTLS(self.id)
	def getLastActionTime(self,):
		return Car.traci.vehicle.getLastActionTime(self.id)
	def moveTo(self, laneID, pos): # Bug, not actually moved.
		return Car.traci.vehicle.moveTo(self.id, laneID, pos)
	# https://sumo.dlr.de/pydoc/traci._vehicle.html#VehicleDomain-moveToXY
	def moveToXY(self,edgeID,laneInd, x, y): # Can actually move, but will read stale nowSpeed. LanePos is updated good.
		return Car.traci.vehicle.moveToXY(self.id, edgeID, laneInd, x, y, keepRoute=1)
	def setActionStepLength(self, vehID, actionStepLength, resetActionOffset=True):
		Car.traci.vehicle.setActionStepLength(self.id,actionStepLength,resetActionOffset=resetActionOffset)
	def setRouteID(self, rid):
		Car.traci.vehicle.setRouteID(self.id, rid)
	def tryChangeLane(self, targetIndex, duration):
		try:
			Car.traci.vehicle.changeLane(self.id,targetIndex,duration)
		except:
			print(self.id, self.nowLaneId, "cannot change lane to",targetIndex,'dura',duration)


	''' ---------- start life --------------'''
	def subscribe(self,varlist):
		Car.traci.vehicle.subscribe(self.id, varlist)
		self.subscribed=True
		self.getRoute()
	def setStartTime(self,time):
		self.startTime = time
	def getRoute(self, refresh=False):
		if self.rou==[] or refresh:
			self.rou = Car.traci.vehicle.getRoute(self.id)
		return self.rou
	def getRouteID(self , refresh=False ):
		if not hasattr(self, 'RouteID')  or refresh:
			self.RouteID = Car.traci.vehicle.getRouteID(self.id)
		return self.RouteID



	''' ---------- end of life --------------'''
	def remove(self,time=-1, output=True):
		try:
			Car.traci.vehicle.remove(self.id)
		except: print("Err: traci.vehicle.remove  "+self.id)
		if self.id in Car.id2car: Car.id2car.pop(self.id)
		self.setDead(time, output=output)
	def setDead(self,time=-1, output=True):
		self.subscribed=False
		self.isDead=True
		self.appendToFile("i id=%s stt=%d\n"%(self.id, time))
		if output:
			self.endTime= max(self.time,time)
			wstr="v id=%s st=%d ed=%d en=%d"%(self.id, self.startTime, self.endTime, len(self.rou))
			if 2 in self.iprintSet: print(wstr)
			self.appendToFile(wstr)
			self.writeGasResult()
			self.writeTlsResult()
		if self.do_optimizeSpeed :
			try:
				if self.curEdge.seg.ends_with_tls():
					tlsFromEdgeID = self.curEdge.seg.get_last_non_internal_edge().id
					self.e2tls[tlsFromEdgeID].deregisterCar(self)
			except: pass
			if self.lane2vq:
				try:
					self.lane2vq[self.lastQueueLane].remove(self)
				except: pass


	''' ---------------------- logging/util --------------'''

	def get_via_link(self, edgeObj, fromLaneIndex=None):
		if edgeObj.seg.nextSeg is None: return None
		endFromEdge = edgeObj.seg.get_last_non_internal_edge() # In 
		endToEdge = edgeObj.seg.nextSeg.get_first_non_internal_edge() # Out
		if fromLaneIndex is None:
			fromLaneIndex = list(endFromEdge.get_best_lane_indices())[0] # from lid 
		fromLaneIndex = int(fromLaneIndex)
		for conn in endFromEdge.conns:
			if int(conn["fromLane"])== fromLaneIndex:
				toLaneIndex = int(conn["toLane"])# any lane on next seg edge
				via = conn["via"]
				break
		return (endFromEdge.id+"_%d"%fromLaneIndex , endToEdge.id+"_%d"%toLaneIndex, via)


	def writeTlsPredLog(self, fromEdgeObj, tlsSolution, timeToArrive, dist):
		# write tls pred stats:
		if tlsSolution.targetStartTime and tlsSolution.targetStartTime>self.time and fromEdgeObj.seg.nextSeg:
			fromLaneId, toLaneId, via = self.get_via_link( fromEdgeObj)
			state, linkIndex = self.get_signal_given_link(fromLaneId, toLaneId, via)
			wstr = "p id=%s t=%d at=%.1f pt=%.1f S=%s li=%d rd=%.1f tls=%s\n"%(self.id, self.time, timeToArrive, tlsSolution.targetStartTime, state, linkIndex , dist, tlsSolution.tlsid)
			if self.output_file and Car.mvfd: self.appendMoveFile(wstr)
			if 10 in self.iprintSet:
				print("Tls Pred stats, state",state,"link#",linkIndex)
				print(wstr.strip())

	def writeGasResult(self,):
		wstr="g id=%s f=%.2f d=%.1f tt=%.1f wt=%.1f"%(self.id, self.gas, self.travelDist, self.endTime-self.startTime, self.total_wait_time)
		if 2 in self.iprintSet: print(wstr)
		self.appendToFile(wstr)

	def writeTlsResult(self,):
		numTls =  len(self.route.segs)
		numStopSign = self.route.get_num_stop_signs()
		wstr="s id=%s ntl=%d nss=%d ntp=%d"%(self.id, numTls, self.stopped_tls_num, numStopSign)
		if 2 in self.iprintSet: print(wstr)
		self.appendToFile(wstr)
	
	def appendToFile(self, wstr):
		if self.output_file and Car.ofid is not None:
			try:
				Car.ofid.write(wstr)
				if not wstr.endswith("\n"): Car.ofid.write("\n")
			except: 
				print("Write Fail!", Car.ofid, wstr)
				Car.ofid = open(Car.ofidname,'a') # NFS closed, need reopen
				Car.ofid.write("\n")
				Car.ofid.write(wstr)
				if not wstr.endswith("\n"): Car.ofid.write("\n")

	def appendMoveFile(self, wstr):
		try: 
			Car.mvfd.write(wstr)
			if not wstr.endswith("\n"):
				Car.mvfd.write("\n")
		except: 
			print("File write Fail.", Car.mvfd, wstr)
			Car.mvfd = open(Car.mvfdname, 'a')
			Car.mvfd.write("\n")
			Car.mvfd.write(wstr)
			if not wstr.endswith("\n"):
				Car.mvfd.write("\n")

	def getSubscriptionResults(self,):
		return Car.traci.vehicle.getSubscriptionResults(self.id)
	def setColor(self,color): # e.g. (255,255,0)
		Car.traci.vehicle.setColor(self.id, color)
	def applySpeedRaw(self,spdcmd): # -1 means release/free.
		if 3 in self.iprintSet: print("[applySpeedRaw] v=%.4f"%spdcmd)
		Car.traci.vehicle.setSpeed(self.id, spdcmd)

	def is_intersection_congested(self, junctionID):
		if junctionID in Car.congestedAreas["cross"]:
			return True
		return False


	def __eq__(self, other):
		return self.id == other.id
	def __ne__(self, other):
		return not self.__eq__(other)
	def __repr__(self):
		return "%s,v%.2f,p%.1f"%(self.id,self.nowSpeed,self.lanePos)
	def __hash__(self):
		return hash(self.id)

	


''' ---------------- other utils ----------------------'''


def get_adjusted_car_gap(car, proposed):
	return min(config.MaxCarGap , max(car.minGap, proposed) )


class FuelModel(object):
	def __init__(self,  model):
		self.model = model # Lasso from GreenRoute: 6*.py
		self.vsize = 7 # Order of poly v^0-6
		self.lastv = None
		self.avsize= 4 # Order of poly acc*v^1-4

	def get_fuel(self, v):
		vs = [0. for _ in range(self.vsize)]
		avs = [0. for _ in range(self.avsize)]
		aavv = 0. # a^2*v^2
		tmp=1.
		for i in range(len(vs)): # 0 -> constant term.
			vs[i]+=tmp
			tmp*=v
		acc = max(0., v-self.lastv) if (self.lastv is not None and v is not None) else 0.
		aavv +=  acc*acc*v*v
		tmp= acc*v
		for i in range(len(avs)):
			avs[i]+=tmp
			tmp*=v
		self.lastv = v
		sample={}
		sample[VTCPFEMv0]=vs[0]
		sample[VTCPFEMv1]=vs[1]
		sample[VTCPFEMv2]=vs[2]
		sample[VTCPFEMv3]=vs[3]
		sample[VTCPFEMv4]=vs[4]
		sample[VTCPFEMv5]=vs[5]
		sample[VTCPFEMv6]=vs[6]
		sample[VTCPFEMa2v2]=aavv
		sample[VTCPFEMav1]=avs[0]
		sample[VTCPFEMav2]=avs[1]
		sample[VTCPFEMav3]=avs[2]
		sample[VTCPFEMav4]=avs[3]
		x = []
		for feat in VTCPFEM_features:
			x.append(sample[feat])
		y = self.model.predict(np.asarray(x).reshape(1,-1))
		return y

