#!/usr/bin/env python
# Solution related classes:
from common import *
import numpy as np
config = None 

DT = 0.5 # time granularity 
INF = 1e4 # largest cost
dV = 0.5 # speed deviation tolerance
tTolerate = 0.5 # do not throw except when diff within this.
DelayCost= 1.5 # could arrive earlier, but go slow prolong time, add delay cost. 

def div(d,t):
    return float(d)/max(0.1,t)


class FuelCost:
	def __init__(self, car ): 
		self.car = car
		self.minSpeed = config.CarMinSpeed
		self.tcosts = []
		self.minInd = None # self.tcosts[ind] is smallest

	def set_max_speed(self, vlimit1, vlimit2, print_str=False):
		self.vlimit1 = vlimit1
		self.vlimit2 = vlimit2
		if print_str: print("[ FuelCost ] set_max_speed( %.2f, %.2f )"%(vlimit1,vlimit2))


	def find_v1t1v2t2_arrival_time_given_tls12(self, t0,d1,intv1,intv1late, d2,intv2,intv2late, tminSeg2=None, stressed=0, print_str=False, allow_exception=False, alert=True ):
		fastest = 0 # run fastest to stick to varying speed limit?
		catchLateIntv12 = [0,0]
		if tminSeg2 is None and div(d2, intv2[1]-intv1[0])> self.vlimit2 +dV:
			if div(d2, intv2late[1]-intv1[0])> self.vlimit2 +dV: # arrive t2 later, still not possible
				if print_str: 
					print('t0 %.2f'%t0,'d1 %.1f'%d1, 'intv1',intv1, 'd2 %.1f'%d2, 'intv2',intv2)
				raise Exception("Cannot catch tls2 end1!! "+self.car.id)
			if print_str:
				print(self.car.id, "catch late tls2","intv1[0] %.2f"%intv1[0],"intv2late[1] %.2f"%intv2late[1],'d2 %.1f'%d2,'vlim2 %.2f'%self.vlimit2)
			catchLateIntv12[1] = 1 # use intv2late

		if tminSeg2 is not None and intv2[1]-intv1[0] < tminSeg2 -tTolerate:
			if intv2late[1]-intv1[0] < tminSeg2 -tTolerate: # arrive t2 late, still miss...
				if print_str: 
					print('intv1[0] %.2f'%intv1[0],'intv2late[1] %.2f'%intv2late[1], 'tminSeg2 %.1f'%tminSeg2, self.car.id)
				raise Exception("Cannot catch tls2 end2!! "+self.car.id)
			if print_str:
				print(self.car.id, "catch late tls2","t0 %.2f"%t0,"d1 %.1f"%d1, "intv1",intv1, "d2 %.1f"%d2,"intv2",intv2, 'intv2late',intv2late, "tminSeg2 %.2f"%tminSeg2)
			catchLateIntv12[1] = 1 # use intv2late

		v1x = min(self.vlimit1, div(d1, intv1[0]-t0))
		v1m = div(d1, intv1[1]-t0)
		if v1m > self.vlimit1 +dV:
			needV1 =div(d1, intv1late[1]-t0) # aim at later end2
			if needV1 - self.vlimit1 > -0.5:
				fastest = 1
			if needV1 > self.vlimit1 +dV: # arrive late t1, still miss end time.
				if alert: print("t0",t0,"d1",d1, "intv1",intv1,'intv1late',intv1late,'vlim1',self.vlimit1)
				if stressed==0: 
					if alert: print(self.car.id, 'vr',self.car.nowSpeed, 'remd', self.car.remainDist)
					if alert: print("vcmd:", self.car.speedControl.cmdHistory)
					if alert: print(self.car.speedControl.cannot_reach_command_speed(True))
					if allow_exception: 
						raise Exception(self.car.id +" Cannot catch tls1 end !! v1: %.2f"%needV1,'intv1late', intv1late)
				if alert: print("stressed",stressed, 'suppress Exception')
			needV1 = min(needV1, self.vlimit1)
			if print_str:
				if alert: print("nowt",t0,"cannot catch intv1 endt",intv1[1], 'd1: %.1f'%d1)
				if alert: print("aim at itv1late[1] %.2f"%intv1late[1] ,'vlimit1 %.2f'%self.vlimit1, 'v1 too big',v1m, 'use v1', needV1)
			v1m = needV1
			catchLateIntv12[0] = 1 # use intv1late
		v2 = div(d1+d2, intv2[0]-t0) # care only about start time2

		if v2>= v1x: 
			v1 = v1x
			t1 = t0 + d1/v1 
			v2 = min(self.vlimit2, div(d2 , intv2[0] - t1))
			t2 = t1 + d2 / v2
			if print_str: print("[ FuelCost ] Use v1_max %.2f t1=%.1f, acc to v2=%.2f t2=%.1f"%(v1,t1,v2,t2))
		elif v2>self.vlimit2: # tls2 has lower limit, use fastest
			v1 = min(self.vlimit1, div(d1, max(intv1[0], intv2[0]-tminSeg2) - t0) )
			if v1 < v1m: v1 = v1m
			t1 = t0 + d1/v1
			t2 = max(intv2[0], t1 + tminSeg2)
			v2 = min(self.vlimit2 , div(d2,t2-t1))
			if print_str: print("[ FuelCost ] v2lim=%.2f is slower, v1=%.2f t1=%.1f, t2=%.1f"%(v2,v1,t1,t2))
		elif v2> v1m:
			v1 = v2
			t1 = t0 + d1/v1
			t2 = t1 + d2 / v2
			if print_str: print("[ FuelCost ] Use v2=%.2f t1=%.1f, smooth to t2=%.1f"%(v1,t1,t2))
		else:
			v1 = v1m
			t1 = t0 + d1/v1 
			v2 = min(self.vlimit2, div(d2 , intv2[0] - t1))
			t2 = t1 + d2 / v2
			if print_str: print("[ FuelCost ] Use v1_min %.2f t1=%.1f, dec to v2=%.2f t2=%.1f"%(v1,t1,v2,t2))
		if abs(v1 - self.vlimit1)<0.5:
			fastest = 1 # has to run fastest, since vlimit fluctuates. 
			if print_str:
				print(self.car.id, "fastest! calc v1, vlim1:",v1 , self.vlimit1)
		return v1,t1,v2,t2,fastest, catchLateIntv12


	def get_cost_tls1_tls2(self, t0, v0, d1, intv1, d2, intv2, tminSeg2=None, print_str=False):
		del self.tcosts[:]
		mind, minv = 0, 1e6
		earliestT2 = None
		for t1 in np.arange(intv1[0], intv1[1]+DT, DT):
			try:
				v1,arrivet1,v2,t2,fastest,_ = self.find_v1t1v2t2_arrival_time_given_tls12(t0,d1, [t1,t1], [t1,t1], d2,intv2,intv2, tminSeg2=tminSeg2, print_str=print_str, allow_exception=True, alert=False)
				cost1 = self.calc_fuel_from_v0_to_d1_at_t1(t0, v0, d1, arrivet1, print_str=print_str)
				cost2 = self.calc_fuel_from_v1_to_d2_at_t2(arrivet1, v1, d2, t2, print_str=print_str)
				if cost1<INF and cost2<INF:
					if earliestT2 is None: # for delay cost.
						earliestT2 = t2
					else:
						cost2 += (t2-earliestT2)*DelayCost
				costsum= cost2+cost1
				if costsum <INF: 
					self.tcosts.append([t1, costsum ])
					if costsum< minv:
						minv = costsum
						mind = len(self.tcosts)-1
					if print_str:
						print("cost1 %.2f, t0 %.1f, v0 %.2f, d1 %.1f, t1 %.2f, v1 %.2f"%(cost1, t0, v0, d1, arrivet1,v1 ))
						print(" cost2 %.2f, v2 %.2f, d2 %.1f, t2 %.2f. t2min %.1f"%(cost2, v2, d2, t2,earliestT2 or -1))
				elif cost1>=INF:
					if print_str: print('  INF? t0, v0, d1, arrivet1',t0, v0, d1, arrivet1)
				elif cost2>=INF:
					if print_str: print('  INF? arrivet1, v1, d2, t2',arrivet1, v1, d2, t2)
			except Exception as e:
				if print_str: print(e)
		self.minInd = mind
		return self.tcosts


	def get_min_time_cost(self,):
		if self.minInd is None or self.minInd>=len(self.tcosts):
			print(self.car.id, self.minInd, self.tcosts)
		return self.tcosts[self.minInd][0], self.tcosts[self.minInd][1]

	def calc_fuel_from_v1_to_d2_at_t2(self, t1, v1, d2, t2, print_str=False): 
		v2 = div(d2 , (t2-t1))
		if v2 > self.vlimit2+dV: 
			return INF
		dt = d2 / v2
		return self.cost_const_cruise(v2)*dt + self.cost_change_v12(v1,v2)
	
	def calc_fuel_from_v0_to_d1_at_t1(self, t0, v0, d1, t1, print_str=False): 
		v1 = div(d1 , (t1-t0))
		if v1 > self.vlimit1+dV: 
			return INF
		dt = d1 / v1
		return self.cost_const_cruise(v1)*dt + self.cost_change_v12(v0,v1)


	def cost_const_cruise(self, v): # per second
		return -0.0001* v**2 + 0.0517*v + 0.2458

	def cost_const_dec(self, v): # per second
		return -0.0003* v**2 + 0.0102* v + 0.23

	def cost_change_v12(self, v1, v2):
		return 0.1631* abs(v2 - v1) + 0.0361* abs(v2**2 - v1**2)


	def search_min_ind(self, costs, should_be=None):
		print_str = False
		low = 0
		high = len(costs)-1
		while low<high:
			mid = (low+high)/2
			if costs[mid][1]<costs[mid+1][1]:
				if print_str: print(low,mid,high, "%.1f < %.1f"%(costs[mid][1],costs[high][1]))
				high = mid
			else:
				if print_str: print(low,mid,high, "%.1f >= %.1f"%(costs[mid][1],costs[high][1]))
				low= mid+1
		if should_be and abs(costs[low][0]-should_be)>=3:
			low = self.search_min_ind_linear(costs)
		return low

	def search_min_ind_linear(self, costs):
		minCost= INF
		for i in range(len(costs)):
			t,c = costs[i]
			if c<minCost:
				minCost=c
				ind = i
		return ind

	def get_cost_at_time(self, t, costs=None):
		if costs is None: costs=self.tcosts
		try:
			if t - costs[0][0] < -DT: return INF
			if t - costs[0][0] <= 0: return costs[0][1]
			if t - costs[-1][0] >DT: return INF
			if t - costs[-1][0] >=0: return costs[-1][1]
			pos = max(0, int((t - costs[0][0]-DT)/DT) )
			while 1:
				l,h = pos-1, pos+1
				if l>=0 and h<len(costs):
					if abs(t - costs[l][0])>abs(t - costs[pos][0]) and abs(t - costs[h][0])>abs(t - costs[pos][0]):
						return costs[pos][1]
				if h<len(costs) and abs(t - costs[h][0])<abs(t - costs[pos][0]):
					pos+=1
					continue
				if l>=0 and abs(t - costs[l][0])<abs(t - costs[pos][0]):
					pos-=1
					continue
				return costs[pos][1]
		except Exception as e:
			print(e, self.car.id)
			sys.exit(1)




class Group:
	def __init__(self, nid, init_car ): 
		self.endNodeID = nid
		self.id = nid+'('+init_car.id+')'
		self.id2group = init_car.id2group
		self.gr = []
		self.print_str = False
		self.last_success_time = 0
		

	def append(self, car):
		self.gr.append(car)
		self.id2group[car.id]=self

	def delete(self, car):
		self.gr.remove(car)
		self.id2group.pop(car.id)

	def merge_front_into_mine(self, front, print_str=False):
		if print_str: print('[merge_front_into_mine]',front,'into',self)
		if front==self: return
		for car in front.gr:
			if print_str: print("    append %s"%car )
			self.append(car)

	def __eq__(self, other):
		if self.id == other.id: return True
		return False 
	def __ne__(self, other):
		return not self.__eq__(other)
	def __repr__(self):
		wstr = self.id
		for car in self.gr:
			wstr += ','+car.id
		return wstr



class TlsSolution:
	TimeDiffTolerable = 2.

	def __init__(self, car ):
		self.car = car
		self.iprintSet = self.car.iprintSet
		self.clear()

	def configure(self, tlsFromEdgeID, tlsObj ):# called once at seg begin
		self.tlsFromEdgeID = tlsFromEdgeID
		self.tlsObj = tlsObj
		self.tlsid = tlsObj.id

	def load_prediction(self, tls_dic):
		self.tlsPredSeq = tls_dic["seq"]
		self.version = tls_dic["version"]
		self.status = tls_dic["status"]
		self.timeIntervals = self.gen_intervals_from_tls_seq(self.tlsPredSeq)

	def findTargetInterval(self, timeOffset, travelTime, ask_for_stability = False, print_str=False):  
		self.foundTarget =0
		if not self.is_valid(): return self.foundTarget
		ind = 0
		while ind<len(self.timeIntervals): # proper time intv.
			starttime , endtime, stageDir = self.timeIntervals[ind] # ind'th stage in pred seq.
			if endtime -timeOffset -config.TlsEndTimeBuf > travelTime:
				self.foundTarget=1
				if print_str: print("[findTarget] *%s, found"%self.tlsid[-4:],"%.1f + %.1f"%(timeOffset,travelTime), "< %.1f"%(endtime-config.TlsEndTimeBuf))
				break
			ind+=1
		if self.foundTarget:
			if print_str: print("[findTarget] *%s, %.1f - %.1f"%(self.tlsid[-4:],starttime,endtime), "stable?",ask_for_stability)
			if self.targetStartTime is not None and ask_for_stability:
				if abs(starttime - self.targetStartTime) < TlsSolution.TimeDiffTolerable:
					return self.foundTarget # do not change it for now.
			self.targetStartTime = starttime
			self.targetEndTime = endtime
			self.targetStage = self.tlsObj.stageList[stageDir]
		else:
			if print_str: print("[findTarget] Not found. *%s"%self.tlsid[-4:])
			self.targetStartTime = None
			self.targetEndTime = None
			self.targetStage = None
		return self.foundTarget


	def gen_intervals_from_tls_seq(self, seq): # not called frequently
		intervals = []
		if seq is None: return
		normal_2_stages = not self.tlsObj.is_defect_single_stage()
		i=0
		while i<len(seq): # find proper phase.
			stage_starttime , stage = seq[i] # ind'th stage in pred seq.
			if normal_2_stages: # 2 stages there
				starttime = stage_starttime
				endtime = stage_starttime + stage.phases[0].dura
				if self.tlsFromEdgeID in stage.fromEdgeSet:
					intervals.append([ starttime, endtime, stage.d ])
			else: # fail to learn signal, single 1 stage:
				if self.tlsFromEdgeID in stage.fromEdgeSet:
					starttime = stage_starttime
					endtime = stage_starttime + stage.phases[0].dura
				else: # assume later half of stage allows this move 
					starttime = stage_starttime + stage.phases[0].dura
					endtime = stage_starttime + stage.dura
				intervals.append([ starttime, endtime, stage.d ])
			i+=1
		return intervals


	def has_found_target(self,):
		return self.foundTarget>0 and self.status == "valid"

	def is_valid(self,):
		return self.status == "valid"

	def clear(self,):
		self.version = -1
		self.status = ""
		self.tlsPredSeq = None
		self.targetStartTime = None
		self.targetEndTime = None
		self.foundTarget=0
		self.tlsObj = None
		self.tlsid = None
		self.tlsFromEdgeID = None
		self.optimSpeed = None
		self.predictedWrong = False

	def __eq__(self, other):
		if self.version == other.version and self.tlsObj.id == other.tlsObj.id: return True
		return False 
	def __ne__(self, other):
		return not self.__eq__(other)
	def __repr__(self):
		return "%s-%s,%s,%s"%(self.car,self.tlsObj.id,self.status, str(self.targetStartTime))


