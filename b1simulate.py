#!/usr/bin/env python
from common import *
from configure.assemble import *
from configure.params import Mytools_Rel_Dir,region2serverIP
import optparse
from netcode.func import get_sumo_vars
import sumo_car,sumo_tls,sumo_road,sumo_sol
from sumo_car import Car,GUI_show
from sumo_tls import Tls
from sumo_road import construct_segment
from a4loadServer import query_sumo_junction,query_sumo_edge
from a6InductionLoop import import_a6_func_gen_loop_xml

addpath= mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)
from util import beep
from sumolib import checkBinary
import traci
from configure.runtime import * 

sumo_tls.config,sumo_car.config,sumo_road.config,sumo_sol.config = config,config,config,config
sumo_road.traci = traci
sumo_car.Server_IP_Port = Server_IP_Port
sumo_car.R = R

if Has_GUI_Device ==0: 
	sumo_car.G_Do_Show_GUI=0
	sumo_car.G_Allow_Prompt=0

_Do_Output = True

OutputDir = SumoOutBaseDir +Use_Ip+ SUMO_PORT+ ThisCodeDir_suffix+ Outdir_suffix +os.sep
if Outdir_suffix=="":
	print("\n   Turning off  output,  due to empty suffix   ... ")
	_Do_Output = False
if _Do_Output:
	if not os.path.exists(OutputDir): os.makedirs(OutputDir)
	else: raw_input("Out Dir already exists! will overwrite... %s"%OutputDir)

# TLs learned schedules:
TlsLoadStageDir = SumoTlsBaseDir +config.TLS_INFO_DIR+ "%s"%(Tls_Learn_Custom_suffix) +os.sep
if not os.path.exists(TlsLoadStageDir): raise Exception("Not found: TlsLoadStageDir "+TlsLoadStageDir)


tmp, last_cache_time_file =0, LastCachedRumTimeFile
if os.path.exists(last_cache_time_file):
	with open(last_cache_time_file,"r") as f: 
		try: tmp = int(f.readline())
		except: pass
SetCacheLastTime = raw_input("\nOverwrite Last Cached Rum Time %d ? If so, Enter a number ('e'=endt)"%tmp)
if SetCacheLastTime: 
	if SetCacheLastTime=='e': SetCacheLastTime = config.simulationEndTime
	else: SetCacheLastTime = int(SetCacheLastTime)
else: SetCacheLastTime = None


print('My_Ip '+My_Ip, 'Use_Ip '+Use_Ip, 'Platform '+My_Platform, "GUI", Has_GUI_Device, "SetCacheLastTime",SetCacheLastTime )
print("\nOutputDir: %s"%OutputDir)
print("\nCheck   tls stages load dir: "+TlsLoadStageDir)
print("\nCheck sumo-cfg file: "+SumoCfgFilePath)

print(R)

import_a6_func_gen_loop_xml(R['lsuf'])
num_rou_splits_to_use = R['split']
num_use_our_system  = R['pene']
compare_replace_indices = [0,1] # use /x-*.repcp.xml  instead of /x-*.xml

add_route_files = [RoutesEdgesDefXmlFile,]
i=0
while i< num_rou_splits_to_use:
	if num_use_our_system > 0:
		num_use_our_system-=1
		vpref = 'a'
	else: 
		vpref = 'b'
	if i in compare_replace_indices:
		fn = SplitRoutesDir+ "%d-%s.repcp.xml"%(i, vpref)
	else:
		fn = SplitRoutesDir+ "%d-%s.xml"%(i, vpref)
	add_route_files.append(fn)
	i+=1

feedbackData = {}
congestedAreas = {"cross":set(), }

# gather stats for TLS errors.
def get_bad_list(feedbackData, label, rate, historyPeriod=300, id2tls={}, startTime=500, print_str=False):
	if label not in feedbackData: return []
	dic = feedbackData[label]
	res = []
	for obj, lst in dic.items():
		lasttime = lst[-1]
		if lasttime < startTime: continue # not enough time
		cnt=0
		for i in range(len(lst)-1,-1,-1):
			if lasttime-lst[i]>historyPeriod: break
			cnt+=1
		if cnt >= historyPeriod*rate: 
			if label=="tls" and obj in id2tls:
				tls = id2tls[obj]
				normalizedWidth = float(len(tls.ctrlanes))/len(tls.inEdges)
				if cnt/normalizedWidth < historyPeriod*rate: 
					continue # tls having wider lanes  
			res.append(obj)
	return res


def show_obj(typ, obj): # be sure to turn on server.py.
	if typ == 'tls':
		eid= obj.inEdges[0]
		nid = query_sumo_edge(eid, Server_IP_Port)['to']
		show_obj('nodeID', nid)
	elif typ == 'nodeID':
		res = query_sumo_junction(obj, Server_IP_Port)
		GUI_show([res['x'],res['y']])

def which_should_i_show(fn=Show_Obj_Signal_File):
	show_types = []
	with open(fn,"r") as f:
		for l in f:
			l = l.strip()
			if l=="t": show_types.append('tls') 
			if l=="c": show_types.append('cross') 
	return show_types



def run():
	lastRunMaxTime=0 # for cache overwrite after this
	last_cache_time_file = LastCachedRumTimeFile
	if os.path.exists(last_cache_time_file):
		with open(last_cache_time_file,"r") as f: 
			try:
				lastRunMaxTime = int(f.readline())
			except: pass
	if SetCacheLastTime is not None: lastRunMaxTime = SetCacheLastTime

	print(My_Ip, SUMO_PORT)
	traci.init(int( SUMO_PORT)) 
	var2str, varlist = get_sumo_vars(traci)
	badTls, badCross= [],[]
	Car.traci = traci

	tlslist=traci.trafficlights.getIDList()
	print('# tls', len(tlslist))
	tlsSet = set(tlslist)
	id2tls=dict()
	e2tls =dict() # controlled edge to tls.
	Tls.ofid = open(OutputDir+"tlslog.txt","w") if _Do_Output else None
	Tls.ofidname =  OutputDir+"tlslog.txt"
	Tls.traci = traci
	Tls.feedbackData = feedbackData
	unlearnedTls =[]
	for tlsid in tlsSet:
		tmp = Tls(tlsid, loadStages = True, stageFolder= TlsLoadStageDir) 
		id2tls[tlsid]=tmp
		ctrlanes = tmp.ctrlanes
		tmp.output_file = _Do_Output # writePhaseLog?
		for lane in ctrlanes:
			e2tls[lane.split("_",1)[0]]=tmp
		if len(tmp.stageList)==0: # not learned
			unlearnedTls.append(tmp)

	alledges = [eid for eid in traci.edge.getIDList() if not eid.startswith(":")] 
	edgeSet = set(alledges)
	print('# edges', len(edgeSet), "tls-ctr edges:",len(e2tls))
	if _Do_Output:
		with open(OutputDir+"netinfo.txt","w") as f:
			f.write("nedge=%d nctredge=%d ntls=%d"%(len(edgeSet),len(e2tls),len(id2tls)))
	lane2vq={} # vehicle queue at cross.

	id2car = dict()
	id2group = dict()
	Car.ofid = open(OutputDir+"carlog.txt","w") if _Do_Output else None
	Car.ofidname = OutputDir+"carlog.txt"
	Car.mvfd = open(OutputDir+"carmov.txt","w") if _Do_Output else None
	Car.mvfdname = OutputDir+"carmov.txt"
	Car.spdfd = open(OutputDir+"carspds.txt","w") if _Do_Output else None
	Car.spdfdname = OutputDir+"carspds.txt"
	Car.lane2vq = lane2vq
	Car.id2car = id2car
	Car.id2group = id2group
	Car.feedbackData = feedbackData
	Car.congestedAreas = congestedAreas
	Car.RemoveWongLaneThresh = 2

	print("!!! [cache] lastRunMaxTime",lastRunMaxTime)
	curtime=0
	sumoRunTime=0.
	manageRunTime=0.
	processRunTime=0.
	is_showing = None
	if Has_GUI_Device: 
		Car.traci.gui.setZoom('View #0',1000)
		vprint = set([]) #  to track this vehicle
	else: vprint=set([]) 

	while curtime<config.simulationEndTime+1:
		startTime = time.time()
		traci.simulationStep() 
		sumoRunTime += time.time()-startTime

		curtime = traci.simulation.getTime() -1 
		if Has_GUI_Device and curtime == -1:
			beep(5)
			raw_input("\nEnter any to Continue...") # debug purpose

		if curtime%30 ==0 or curtime==config.simulationEndTime:  
			print("\nT=%d  Vnum=%d sT=%.2f mT=%.2f pT=%.2f"%(curtime,len(id2car),1000*sumoRunTime/(curtime+1),1000*manageRunTime/(curtime+1),1000*processRunTime/(curtime+1)))
			if curtime>lastRunMaxTime: 
				lastRunMaxTime = curtime
				with open(last_cache_time_file,"w") as f: 
					f.write("%d"%lastRunMaxTime)

		if curtime%31==0 or is_showing is not None: 
			badTls= get_bad_list(feedbackData, "tls",TlsBadRateThresh, id2tls=id2tls)
			badCross= get_bad_list(feedbackData, "cross", CrossBadRateThresh)
			for tid in badTls: 
				print("Checkout bad tls "+tid)
			for nid in badCross: 
				print("bad cross "+nid)
				congestedAreas["cross"].add(nid)


		for vid in traci.simulation.getStartingTeleportIDList():# stuck
			try:
				id2car[vid].remove(curtime,output=False) # traci remove + setDead 
			except: pass
			try:
				del id2car[vid]
			except: pass

		startTime = time.time()
		for vid in traci.simulation.getArrivedIDList():
			if vid in id2car:
				id2car[vid].setDead(curtime) # already auto removed.
				del id2car[vid]

		for vid in traci.simulation.getDepartedIDList(): # new born cars 
			tmp = Car(vid, modelPredictFuel=True)
			tmp.subscribe(varlist)
			tmp.setStartTime(curtime)
			rid = tmp.getRouteID()
			tmp.route = construct_segment(tmp.rou, e2tls, route_id=rid, print_str=vid in vprint, overwrite_cache=curtime>=lastRunMaxTime, prefix=config.osmfile ) 
			tmp.route.id = rid
			tmp.route.car = tmp
			tmp.e2tls= e2tls
			if vid.startswith('a'):
				tmp.do_Tls1 = R['tls1']>0 # query 1st tls ahead
				tmp.do_Tls2 = R['tls2']>0 # query two tls ahead
				tmp.do_coor = R['co']>0 # codrive
				tmp.do_optimizeSpeed = True
			else:
				tmp.do_Tls1 = False
				tmp.do_Tls2 = False
				tmp.do_coor = False
				tmp.do_optimizeSpeed = False # false will not do lane change.
			tmp.output_file = _Do_Output 
			id2car[vid]=tmp
			if vid in vprint: 
				tmp.iprintSet |= set([2,3,9]) # track this car, and print more
				tmp.route.print_str = True
				tmp.setColor((255,255,0))
				print(tmp.rou)
				pprint.pprint(tmp.route)
		manageRunTime += time.time()-startTime

		startTime = time.time()
		for tlsid in tlsSet: # proc tls, called before car.process
			id2tls[tlsid].process(curtime)

		for vid,car in id2car.items(): 
			car.process(curtime) # fill in basic variables

		for lid,que in lane2vq.items():
			que.sort(key=lambda car: car.lanePos) # sort by lane-pos.

		for vid,car in id2car.items(): 
			car.formGroup()

		for vid,car in id2car.items(): 
			car.optimize() # run optimization.

		for vid,car in id2car.items(): 
			car.transaction() # if codrive

		processRunTime += time.time()-startTime

	

	traci.close()
	Car.ofid.close()
	Tls.ofid.close()
	Car.mvfd.close()
	Car.spdfd.close()





if __name__ == "__main__":
	def get_options():
		optParser = optparse.OptionParser()
		optParser.add_option("-n","--nogui", action="store_true", default=False, help="run the commandline version of sumo")
		options, args = optParser.parse_args()
		return options

	options = get_options()
	if options.nogui or Has_GUI_Device==0:
		sumoBinary = checkBinary('sumo')
	else:
		sumoBinary = checkBinary('sumo-gui')

	additional_files=[VTypeDefXmlFile,]
	if os.path.exists(LoopXmlFile): additional_files.append(LoopXmlFile)
	if os.path.exists(TaxiRouteEdgesFile): additional_files.append(TaxiRouteEdgesFile)
	if os.path.exists(ComparisonRouteDefFile): additional_files.append(ComparisonRouteDefFile)
	for fn in add_route_files: additional_files.append(fn)

	cmd = [sumoBinary, "-c", SumoCfgFilePath , "--remote-port", SUMO_PORT, '--additional-files', ','.join(additional_files)]
	pprint.pprint(cmd)
	
	assert R['tls1']>0 or R['tls2']>0, "at least using one of our systems?"
	if R['co']>0: assert R['tls1']>0 and R['tls2']>0
	raw_input("\nPlease confirm settings ...") 

	sumoProcess = subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr)
	run()
	sumoProcess.wait()

	R['run_status']='done'
	write_task_R(R)
	tmp = load_task_R(R['task_key'])
	print(tmp)
