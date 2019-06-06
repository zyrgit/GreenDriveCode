#!/usr/bin/env python
from common import *
from configure.assemble import *
from configure.params import Mytools_Rel_Dir,region2serverIP
from netcode.func import get_sumo_vars
from common.imp_network import *
import optparse

addpath= mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)
from util import beep
from sumolib import checkBinary
import traci
from sumo_car import Car
from sumo_tls import Tls
import sumo_car, sumo_road
import sumo_tls,sumo_sol
from sumo_road import construct_segment
from a6InductionLoop import import_a6_func_gen_loop_xml

sumo_tls.config,sumo_car.config,sumo_road.config,sumo_sol.config = config,config,config,config
sumo_road.traci = traci
sumo_car.Server_IP_Port = Server_IP_Port

if Has_GUI_Device ==0: 
	sumo_car.G_Do_Show_GUI=0
sumo_car.G_Allow_Prompt = 0 # not asking for key press 
_Do_Output = True 


OutputDir = SumoOutBaseDir + Use_Ip +SUMO_PORT + ThisCodeDir_suffix + Outdir_suffix +os.sep
if Outdir_suffix=="": # Outdir_suffix defined in configure/options.py
	print("\n   Turning off  output,  due to empty suffix   ... ")
	_Do_Output = False
if _Do_Output:
	if not os.path.exists(OutputDir): os.makedirs(OutputDir)
	else: raw_input("Out Dir already exists! overwrite? ... %s"%OutputDir)


tmp, last_cache_time_file = 0, LastCachedRumTimeFile
if os.path.exists(last_cache_time_file):
	with open(last_cache_time_file,"r") as f: 
		try: tmp = int(f.readline())
		except: pass
SetCacheLastTime = raw_input("\nOverwrite Last Cached Rum Time %d ? If so, Enter a number ('e'=end)"%tmp)
if SetCacheLastTime: 
	if SetCacheLastTime=='e': SetCacheLastTime = config.simulationEndTime
	else: SetCacheLastTime = int(SetCacheLastTime)
else: SetCacheLastTime = None


print('My_Ip '+My_Ip, 'Use_Ip '+Use_Ip, 'Platform '+My_Platform, "GUI",Has_GUI_Device, "SetCacheLastTime",SetCacheLastTime )
print("\nOutputDir: %s"%OutputDir)
print("\nCheck sumo-cfg file: "+SumoCfgFilePath)

print(R)

import_a6_func_gen_loop_xml(R['lsuf'])

num_rou_splits_to_use = R['split'] # split number to use, controls density.
num_use_our_system  = R['pene'] # not in use, none using our systems.
compare_replace_indices = [0,1] 

add_route_files = [RoutesEdgesDefXmlFile, ]
for i in range(num_rou_splits_to_use):
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

blacklist = set() # tls id 
if os.path.exists(TlsIdsBlacklistFile):
	with open(TlsIdsBlacklistFile,"r") as f:
		for l in f:
			l=l.strip()
			if l: blacklist.add(l)
	print("Blacklisted tlsid #", len(blacklist))

blacklistEdges=set([]) # remove tls on this edge. Add to blacklist yourself after seeing the tlsid !

tls_info_outdir = SumoTlsBaseDir +config.TLS_INFO_DIR+ ThisCodeDir_suffix +os.sep # write tls info  
if not os.path.exists(tls_info_outdir): os.makedirs(tls_info_outdir) # just to write in-edge list.


def run():
	lastRunMaxTime=0 # for cache overwrite after this
	last_cache_time_file = LastCachedRumTimeFile
	if os.path.exists(last_cache_time_file):
		with open(last_cache_time_file,"r") as f: 
			try:
				lastRunMaxTime = int(f.readline())
			except: pass
	if SetCacheLastTime is not None: lastRunMaxTime = SetCacheLastTime

	server_working = 1 # assume data server is online.
	print(My_Ip,SUMO_PORT)
	traci.init(int(SUMO_PORT)) 
	var2str, varlist = get_sumo_vars(traci)
	Car.traci = traci

	tlslist=traci.trafficlights.getIDList()
	print('# tls', len(tlslist))
	tlsSet = set(tlslist)
	id2tls=dict()
	e2tls =dict() # controlled in-edge to tls.
	Tls.ofid = open(OutputDir+"tlslog.txt","w") if _Do_Output else None
	Tls.ofidname = OutputDir+"tlslog.txt"
	Tls.traci = traci
	badjunc = set()
	tls2inEdges = {}
	for tlsid in tlsSet:
		tmp = Tls(tlsid, loadStages = False)
		id2tls[tlsid]=tmp
		ctrlanes = tmp.ctrlanes
		tmp.output_file = _Do_Output # writePhaseLog ? 
		tls2inEdges[tlsid]=[]
		for lane in ctrlanes:
			e=lane.split("_",1)[0]
			e2tls[e]=tmp
			if e not in tls2inEdges[tlsid]:
				tls2inEdges[tlsid].append(e)
				if e in blacklistEdges:
					print(tlsid+" Add this tlsID to blacklist yourself! ")
		if len(tls2inEdges[tlsid])<=1:
			print(tlsid+": Only 1 in-Edge, blacklist!")
			if tlsid not in blacklist:
				blacklist.add(tlsid)
				with open(TlsIdsBlacklistFile,"a") as f:
					f.write(tlsid+"\n")
		if server_working>0 and tlsid in blacklist:
			print("--- Will remove blacklisted tls: "+tlsid)
			badedges = set([lane.split("_",1)[0] for lane in ctrlanes])
			for eid in badedges:
				try:
					ret = requests.post(Server_IP_Port+'/data/sumo_edges', data = {'id':eid}).json()
					junc = ret['to']
					print(" Remove tls on e="+eid+" at j="+junc)
					badjunc.add(junc)
				except:
					server_working=0
					print("\nserver.py Not On....\n")
					raw_input("server.py not working, cannot remove tls! Enter...")
					break
	if badjunc and server_working>0:
		if os.path.exists(JunctionBlacklistFile): 
			with open(JunctionBlacklistFile,"r") as f: 
				for l in f:
					l=l.strip()
					if l in badjunc: badjunc.remove(l) # do not append dups.
		if badjunc:
			with open(JunctionBlacklistFile,"a") as f: # must append only. 
				for jid in badjunc:
					f.write(jid+"\n")
					print("write to junc blacklist "+jid)


	alledges = [eid for eid in traci.edge.getIDList() if not eid.startswith(":")] 
	edgeSet = set(alledges)
	print('# edges', len(edgeSet), "ctr edges:",len(e2tls))
	if _Do_Output:
		with open(OutputDir+"netinfo.txt","w") as f:
			f.write("nedge=%d nctredge=%d ntls=%d"%(len(edgeSet),len(e2tls),len(id2tls)))
	for tlsid in tls2inEdges:
		fn = tls_info_outdir+tlsid+"ie.txt"
		if os.path.exists(fn): continue
		pickle.dump(tls2inEdges[tlsid],open(fn,"wb"))

	id2car = dict()
	Car.ofid = open(OutputDir+"carlog.txt","w") if _Do_Output else None
	Car.ofidname = OutputDir+"carlog.txt" 
	Car.mvfd = open(OutputDir+"carmov.txt","w") if _Do_Output else None
	Car.mvfdname = OutputDir+"carmov.txt"
	Car.spdfd = open(OutputDir+"carspds.txt","w") if _Do_Output else None
	Car.spdfdname = OutputDir+"carspds.txt"
	Car.id2car = id2car
	Car.RemoveWongLaneThresh = 10 
	if Has_GUI_Device: 
		Car.traci.gui.setZoom('View #0',1000)
		vprint = set([]) # if you want to track a vehicle id.
	else: vprint = set() # on cluster 

	curtime=0
	while curtime< config.simulationEndTime+1:  
		traci.simulationStep() 
		curtime = traci.simulation.getTime() -1 

		if curtime%30==0 or curtime== config.simulationEndTime: # log cache time.
			print("T=%d  Vnum=%d"%(curtime,len(id2car)),config.osmfile)
			if curtime>lastRunMaxTime: 
				lastRunMaxTime = curtime
				with open(last_cache_time_file,"w") as f: 
					f.write("%d"%lastRunMaxTime)

		for vid in traci.simulation.getStartingTeleportIDList():
			try:
				id2car[vid].remove(curtime,output=False)
			except: pass
			try:
				del id2car[vid]
			except: pass
		for vid in traci.simulation.getArrivedIDList():
			if vid in id2car:
				id2car[vid].setDead(curtime)
				del id2car[vid]

		for tlsid in tlsSet: # proc tls
			id2tls[tlsid].process(curtime)

		for vid in traci.simulation.getDepartedIDList():
			tmp = Car(vid, modelPredictFuel=True)
			tmp.subscribe(varlist)
			tmp.setStartTime(curtime)
			rid = tmp.getRouteID()
			tmp.route = construct_segment(tmp.rou, e2tls, route_id=rid, overwrite_cache=curtime>=lastRunMaxTime, prefix=config.osmfile )# prefix is mem cache prefix.
			tmp.route.id = rid
			tmp.route.car = tmp
			tmp.e2tls= e2tls
			tmp.do_Tls1 = False
			tmp.do_Tls2 = False
			tmp.do_optimizeSpeed = False # not using our system.
			tmp.output_file = _Do_Output 
			id2car[vid]=tmp
			if vid in vprint: # if you are tracking this vehicle in GUI
				tmp.iprintSet |= set([2,3])
				tmp.route.print_str = True
				tmp.setColor((255,255,0))
				pprint.pprint(tmp.route)
			
		for vid,car in id2car.items():
			car.process(curtime)
			car.optimize()
		


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
	
	assert R['tls1']==0 and R['tls2']==0, "wrong settings in tasks.py"
	raw_input("\nPlease confirm settings ...")

	sumoProcess = subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr)
	run()
	sumoProcess.wait()

	R['run_status']='done'
	write_task_R(R)
	tmp = load_task_R(R['task_key'])
	print(tmp)
