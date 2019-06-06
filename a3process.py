#!/usr/bin/env python
from common import *
from configure.assemble import *
from configure.params import region2serverIP,Mytools_Rel_Dir
from common.imp_analysis import *
from sumo_tls import Stage,EventTime,TlsMove,MovesOfPhase
import sumo_tls
from common.util import VoteCluster,merge_votes,get_max_vote,get_period_given_candidates

addpath= mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)

sumo_tls.config = config
_Do_Write = 1 



def get_car_stats(CarLogFile, NetInfoFile, OutputDir, selected_vids=None, retrieve=[]): # get car density, fuel, time, etc.
	print("[ get_car_stats ]")
	numEdge,vrate =None,-1
	if os.path.exists(NetInfoFile):
		with open(NetInfoFile,"r") as f:
			l=f.readline()
		numEdge =float( l.split("nedge=",1)[1].split(" ",1)[0] )
		print("numEdge",numEdge)
	
	car2stats = dict()
	our2stats = dict()# use our system
	not2stats = dict()# not using.
	totalcarset = set()
	with open(CarLogFile,"r") as f:
		for l in f:
			try:
				if l.startswith("i id="):
					vid = l.split(" id=",1)[1].split(" ")[0]
					totalcarset.add(vid)
				# v id=b8 st=10 ed=138 en=8
				elif l.startswith("v id="):
					vid = l.split(" id=",1)[1].split(" ")[0]
					if selected_vids and vid not in selected_vids:  continue
					stt = int(l.split(" st=",1)[1].split(" ")[0])
					edt = int(l.split(" ed=",1)[1].split(" ")[0])
					nedge = int(l.split(" en=",1)[1].split(" ")[0])

					if vid not in car2stats:  car2stats[vid] = dict()
					car2stats[vid]["time"] = edt-stt
					car2stats[vid]["nedge"] = nedge
					if vid.startswith('a'):
						if vid not in our2stats:  our2stats[vid] = dict()
						our2stats[vid]["time"] = edt-stt
						our2stats[vid]["nedge"] = nedge
					else:
						if vid not in not2stats:  not2stats[vid] = dict()
						not2stats[vid]["time"] = edt-stt
						not2stats[vid]["nedge"] = nedge

				# g id=b8 f=98.01 d=1087.3 tt=128.0 wt=29.0
				elif l.startswith("g id="):
					vid = l.split(" id=",1)[1].split(" ")[0]
					if selected_vids and vid not in selected_vids:  continue
					fuel = float(l.split(" f=",1)[1].split(" ")[0])
					dist = float(l.split(" d=",1)[1].split(" ")[0])
					wait = float(l.split(" wt=",1)[1].split(" ")[0])

					if vid not in car2stats:  car2stats[vid] = dict()
					car2stats[vid]["fuel"] = fuel
					car2stats[vid]["dist"] = dist
					car2stats[vid]["wait"] = wait
					if vid.startswith('a'):
						if vid not in our2stats:  our2stats[vid] = dict()
						our2stats[vid]["fuel"] = fuel
						our2stats[vid]["dist"] = dist
						our2stats[vid]["wait"] = wait
					else:
						if vid not in not2stats:  not2stats[vid] = dict()
						not2stats[vid]["fuel"] = fuel
						not2stats[vid]["dist"] = dist
						not2stats[vid]["wait"] = wait

				#"s id=%s ntl=%d nss=%d ntp=%d"%(self.id, numTls, self.stopped_tls_num, numStopSign)
				elif l.startswith("s id="):
					vid = l.split(" id=",1)[1].split(" ")[0]
					if selected_vids and vid not in selected_vids:  continue
					ntl = float(l.split(" ntl=",1)[1].split(" ")[0])
					nss = float(l.split(" nss=",1)[1].split(" ")[0])
					ntp = float(l.split(" ntp=",1)[1].split(" ")[0])

					if vid not in car2stats:  car2stats[vid] = dict()
					car2stats[vid]["ntls"] = ntl
					car2stats[vid]["nhalt"] = nss
					car2stats[vid]["nstopsign"] = ntp
					if vid.startswith('a'):
						if vid not in our2stats:  our2stats[vid] = dict()
						our2stats[vid]["ntls"] = ntl
						our2stats[vid]["nhalt"] = nss
						our2stats[vid]["nstopsign"] = ntp
					else:
						if vid not in not2stats:  not2stats[vid] = dict()
						not2stats[vid]["ntls"] = ntl
						not2stats[vid]["nhalt"] = nss
						not2stats[vid]["nstopsign"] = ntp
			except:
				print("bad line? "+l)
	if _Do_Write and selected_vids is None: 
		pickle.dump(car2stats, open(OutputDir+"car2stats","wb"))
		pickle.dump(our2stats, open(OutputDir+"our2stats","wb"))
		pickle.dump(not2stats, open(OutputDir+"not2stats","wb"))
	# get avg stats
	total_num_cars = max(len(car2stats),len(totalcarset))
	print("total_num_cars",total_num_cars)

	avggasPerMeter,avggas,avgtime,avgnedges,vrate,avgdist,avgwait,ntlsPkm,nhaltPkm,nstopsignPkm = proc_avg_states(car2stats, total_num_cars,numEdge, OutputDir+"rescar.txt", selected_vids=selected_vids, msg='all' )
	print('')
	if len(our2stats)>0:
		proc_avg_states(our2stats, total_num_cars,numEdge, OutputDir+"resour.txt", selected_vids=selected_vids, msg='our',retrieve=retrieve)
	if len(not2stats)>0:
		proc_avg_states(not2stats, total_num_cars,numEdge, OutputDir+"resnot.txt", selected_vids=selected_vids, msg='not',retrieve=retrieve)
	print('')
	return avggasPerMeter,avggas,avgtime,avgnedges,vrate,avgdist


def proc_avg_states(car2stats, total_num_cars, numEdge, ofname, selected_vids=None, msg='', retrieve=[]):
	avggasPerMeter,avggas,avgtime,avgnedges,vrate,avgdist,avgwait,ntlsPkm,nhaltPkm,nstopsignPkm = calc_avg_stats(car2stats, numEdge)
	wstr = "gasPm=%.4f avggas=%.2f avgtime=%.2f avgnedges=%.2f vrate=%.4f avgdist=%.1f avgwait=%.1f ntlsPkm=%.2f nhaltPkm=%.2f nstopsignPkm=%.2f"%(avggasPerMeter,avggas,avgtime,avgnedges,vrate,avgdist,avgwait,ntlsPkm,nhaltPkm,nstopsignPkm)
	dic = { 'gasPm':float("{0:.4f}".format(avggasPerMeter)),  'avggas':float("{0:.2f}".format(avggas)),  'avgtime':float("{0:.2f}".format(avgtime)),  'avgnedges':float("{0:.2f}".format(avgnedges)),  'vrate':float("{0:.4f}".format(vrate)),  'avgdist':float("{0:.1f}".format(avgdist)),  'avgwait':float("{0:.1f}".format(avgwait)),  "total_num_cars":total_num_cars , 'ntlsPkm':float("{0:.2f}".format(ntlsPkm)), 'nhaltPkm':float("{0:.2f}".format(nhaltPkm)), 'nstopsignPkm':float("{0:.2f}".format(nstopsignPkm)),  'carnum':len(car2stats),  'msg':msg}
	print("'"+msg+"':"+str(dic)+',')
	retrieve.append("'"+msg+"':"+str(dic)+',')
	if _Do_Write and selected_vids is None: 
		with open(ofname,"w") as f: f.write(wstr)
	return avggasPerMeter,avggas,avgtime,avgnedges,vrate,avgdist,avgwait,ntlsPkm,nhaltPkm,nstopsignPkm


def calc_avg_stats(car2stats, numEdge):
	gas=0.
	gasPerMeter=0.
	ti, wt =0., 0.
	ne, ntlsPkm, nhaltPkm, nstopsignPkm = 0.,0.,0.,0.,
	dist=0.
	for _, dic in car2stats.items():
		if dic["dist"]<=0:
			print(_, dic)
			continue
		gas += dic["fuel"]
		ti += dic["time"]
		wt += dic["wait"]
		ne += dic["nedge"]
		dist += dic["dist"]
		gasPerMeter += dic["fuel"]/dic["dist"]
		ntlsPkm += 1000.* dic["ntls"]/dic["dist"]
		nhaltPkm += 1000.* dic["nhalt"]/dic["dist"]
		nstopsignPkm += 1000.* dic["nstopsign"]/dic["dist"]
	num = max(1,len(car2stats))
	avggas, avgtime = gas/num, ti/num
	avgwait,avgnedges = wt/num, ne/num
	avgdist,avggasPerMeter = dist/num, gasPerMeter/num
	ntlsPkm, nhaltPkm, nstopsignPkm = ntlsPkm/num, nhaltPkm/num, nstopsignPkm/num
	if numEdge: 
		vrate = (ne)/numEdge/(config.simulationEndTime) 
	return avggasPerMeter,avggas,avgtime,avgnedges,vrate,avgdist,avgwait,ntlsPkm,nhaltPkm,nstopsignPkm



def parseMoveLog(CarMovLogFile):
	print("\n[ parseMoveLog ] reading "+CarMovLogFile)
	t2ee2mov=dict()
	with open(CarMovLogFile,"r") as f:
		for l in f:
			if l.startswith("m tid"):
				tlsid = l.split(" tid=",1)[1].split(" ",1)[0]
				e1 = l.split(" e=",1)[1].split(" ",1)[0]
				e2 = l.split(" ne=",1)[1].split(" ",1)[0]
				vmin = float(l.split(" vm=",1)[1].split(" ",1)[0])
				wait = float(l.split(" wt=",1)[1].split(" ",1)[0])
				acct = float(l.split(" tx=",1)[1].split(" ",1)[0])
				try:
					est_delay = float(l.split(" dl=",1)[1].split(" ",1)[0])
				except: est_delay = 0.
				typ = l.split(" ty=",1)[1].split(" ",1)[0].strip()
				if tlsid not in t2ee2mov: t2ee2mov[tlsid]=dict()
				mov = e1+","+e2
				if mov not in t2ee2mov[tlsid]: 
					t2ee2mov[tlsid][mov]=dict()
					t2ee2mov[tlsid][mov]["acc"]=[]
					t2ee2mov[tlsid][mov]["pas"]=[]
				if typ=='acc' and vmin<0.1 and wait>0:
					if est_delay<=8: # back in wait-line causes trouble 
						t2ee2mov[tlsid][mov]["acc"].append(acct) 
				else:
					t2ee2mov[tlsid][mov]["pas"].append(acct)
	for tlsid in t2ee2mov:
		for mov in t2ee2mov[tlsid]:
			t2ee2mov[tlsid][mov]["acc"].sort()
			t2ee2mov[tlsid][mov]["pas"].sort()
	print("tls#%d"%len(t2ee2mov))
	return t2ee2mov


_MergeTimeThresh= 3. 

def mergeTimestampsAcc(t2ee2mov): # merge same acc move timestamps.
	print("\n[ mergeTimestampsAcc ]")
	for tlsid in t2ee2mov:
		for mov in t2ee2mov[tlsid]:
			arr = t2ee2mov[tlsid][mov]["acc"]  
			if arr!=[]:
				lst = []
				i=0
				j=i
				tmp=[]
				while i<len(arr):
					while j<len(arr) and abs(arr[i]-arr[j])< _MergeTimeThresh:
						tmp.append(arr[j])
						j+=1
					i=j
					lst.append(np.median(tmp))  
					tmp=[]
				t2ee2mov[tlsid][mov]["acc"] = lst
	return t2ee2mov


def mergeMovements(t2ee2mov):
	print("\n[ mergeMovements ]")
	t2accmov=dict()
	for tlsid in t2ee2mov:
		if tlsid not in t2accmov: 
			t2accmov[tlsid]=[]
		res= t2accmov[tlsid] # list of [[acct,times],set[moves] ]...
		for mov in t2ee2mov[tlsid]: # mov = ["acc"] = list of time
			arr = t2ee2mov[tlsid][mov]["acc"] # list of time
			for acct in arr:
				i=0
				inserted=0 
				while i<len(res):
					if abs(acct-res[i][0][0])< _MergeTimeThresh:
						res[i][0][1].append(acct)
						res[i][0][0]=np.median(res[i][0][1])
						res[i][1].add(mov)  
						inserted=1
						break
					i+=1
				if inserted==0: # not merged, it's new timestamp 
					j=0
					while j<=len(res):
						if j==len(res) or acct<res[j][0][0]:
							res.insert(j, [ [acct,[acct]] , set([mov]) ])
							break
						j+=1
	# add passing events to acc events:
	for tlsid in t2ee2mov:
		res = t2accmov[tlsid] 
		for i in range(len(res)): 
			if len(res[i])==2: 
				res[i].append(set(res[i][1]))# another set of all pass/acc moves
				res[i].append([])# another lst of [time,pass-mov],...
		for mov in t2ee2mov[tlsid]:
			arr = t2ee2mov[tlsid][mov]["pas"]
			for ti in arr:
				i=len(res)-1
				while i>0:
					if ti >= res[i][0][0]:
						if ti<= config.TlsMaxPassTimeAfterAcc +res[i][0][0]:
							res[i][2].add(mov)
						if ti<= config.TlsMaxPossiblePhaseDura +res[i][0][0]:
							res[i][3].append([ti,mov])
						break
					i-=1	
	return t2accmov




_tls_blackList = set([ ]) # bad formed tls, need to remove
_tls_not_learned =   set([]) # not learned, but don't remove. 


def analyzePhases(t2accmov, t2s2dura, t2timePhase, tls2inEdges, tls_info_outdir, print_str="p" in sys.argv):
	print("\n[ analyzePhases ]")
	problem = [] 
	print("#tls", len(t2accmov))
	for tlsid in t2accmov:
		try:
			if print_str and tlsid!= "" : continue
			if tlsid in _tls_blackList or tlsid in _tls_not_learned: continue
			print('Proc '+tlsid)
			res = t2accmov[tlsid] # list of [ [t,tlist], set(acc), set(acc+pas), lst[t,mov] ] 
			tphases = t2timePhase[tlsid] 
			ph2dura = t2s2dura[tlsid] 
			inEdges = tls2inEdges[tlsid] 
			if print_str or len(res)<2: 
				print("tid = "+tlsid)
				pprint.pprint(res)
				pprint.pprint(ph2dura)
			assert len(res)>2, tlsid
			# Find cycle length first, by indiv move type:
			mov2cyc = {} # occur time for same movement
			mov2time= {}
			for i in range(len(res)):
				tt,accset,_,_ = res[i]
				for mov in accset:
					if mov not in mov2cyc:
						mov2cyc[mov],mov2time[mov] =[],tt[0]
						continue
					cyct = tt[0] - mov2time[mov]
					if cyct>config.TlsMinCycleDura and cyct<config.TlsMaxCycleDura:
						mov2cyc[mov].append(cyct)
					mov2time[mov]=tt[0]
			e2cyc = {} # occur time for same fromEdge
			e2time = {} 
			for i in range(len(res)):
				tt,accset,_,_ = res[i]
				for mov in accset:
					fromEdge=mov.split(",",1)[0]
					if fromEdge not in e2cyc:
						e2cyc[fromEdge],e2time[fromEdge] =[],tt[0]
						continue
					cyct = tt[0] - e2time[fromEdge]
					if cyct>config.TlsMinCycleDura and cyct<config.TlsMaxCycleDura:
						e2cyc[fromEdge].append(cyct)
					e2time[fromEdge]=tt[0]
			mov2vote = {}
			for mov,cyclst in mov2cyc.items(): # get cycle period from moves candidates
				newcyclist = get_period_given_candidates(cyclst, filter_lb=config.TlsMostLikelyCycleDura * 0.5, period_diff_max_ratio=0.07, lb_diff_ratio=0.2,  print_str=False)
				if newcyclist:
					mov2vote[mov]= VoteCluster(np.median(newcyclist), len(newcyclist))
					if print_str: print("newcyclist",newcyclist,mov2vote[mov])
			if len(mov2vote)==0: # failed?
				if print_str: pprint.pprint(e2cyc)
				for fromE,cyclst in e2cyc.items():# get cycle period from fromEdge candidates
					newcyclist = get_period_given_candidates(cyclst, filter_lb=config.TlsMostLikelyCycleDura * 0.5, period_diff_max_ratio=0.07, lb_diff_ratio=0.2,  print_str=print_str)
					if newcyclist:
						mov2vote[fromE]= VoteCluster(np.median(newcyclist), len(newcyclist))
						if print_str: print("newcyclist from Edge",newcyclist,mov2vote[fromE])
			assert len(mov2vote)>0, tlsid
			votelist = mov2vote.values() 
			votelist = merge_votes(votelist, maxMinRatio=3, print_str=print_str)
			CycleLen = get_max_vote(votelist)
			if print_str: print("\nCycleLen: %.2f"%CycleLen )
			# After cycle length found, group moves into phases:
			res = t2accmov[tlsid] # list of [ [t,tlist], set(acc), set(acc+pas), lst[t,mov] ] 
			movelist = []
			for i in range(len(res)):
				tt,accset,_,paslst = res[i]
				movephase = MovesOfPhase(absTime= tt[0], pid=i) # construct MovesOfPhase
				for ee in accset: # ['from-edge,to-edge',...]
					e0,e1 = ee.split(",",1)
					movephase.add_move(TlsMove(e0,e1,EventTime(tt[0],relative=0.)))
				for t,mov in paslst:
					if t-tt[0]< config.TlsMaxPassTimeAfterAcc:
						e0,e1 = mov.split(",",1)
						movephase.add_move(TlsMove(e0,e1,EventTime(t,relative=t-tt[0])))
				movelist.append(movephase)
			if print_str: 
				print("\nmovelist #%d, 0-9:"%len(movelist))
				pprint.pprint(movelist[0:min(10,len(movelist))])
			# Link/merge movephases which has gap of cycles:
			parent = range(len(movelist))
			for i in range(len(movelist)-1):
				mvp1 = movelist[i]
				pa1 = find(parent, mvp1.id)
				for j in range(i+1,len(movelist)):
					mvp2 = movelist[j]
					linked = 0
					timegap = mvp2.absTime-mvp1.absTime 
					for n in [1,2]: #  try a few cycle-len
						if abs(timegap-n*CycleLen) < _MergeTimeThresh:
							pa2 = find(parent, mvp2.id)
							if pa1!=pa2:
								parent[pa2]=pa1 # union find.
							linked = 1
							break
					if linked: break
					if timegap < CycleLen: 
						inter = [x for x in mvp1.moves if x in mvp2.moves] 
						if len(inter)==0:
							mvp1.add_next_dura_phase([timegap,mvp2]) # link next phase
			# merge phase id to root parent id
			pa2id = {}
			pid = 0
			for i in range(len(movelist)):
				pa = find(parent, movelist[i].id)
				if pa not in pa2id: 
					pa2id[pa]=pid
					pid+=1
				movelist[i].id = pa2id[pa]
			# merge linked phases.
			mergedId2Phases={}
			for i in range(len(movelist)):
				mvph = movelist[i]
				if mvph.id not in mergedId2Phases:
					phase = MovesOfPhase(0, mvph.id)
					phase.set_cycle_len(CycleLen)
					mergedId2Phases[mvph.id] = phase
				else: 
					phase = mergedId2Phases[mvph.id]
				phase.merge_move_from_other(mvph)
				phase.merge_phases_from_other(mvph)
			# infer and write tls schedule.
			phase.generate_tls_stages_phases(mergedId2Phases.values(), tlsid, tls_info_outdir, print_str=print_str, write_file=_Do_Write)
		
		except Exception as e:
			print(e)
			problem.append(tlsid)
	print(problem,len(problem),'# tls',len(t2accmov))



def parseTlsLog(fn , tls_info_outdir): 
	print("\n[ parseTlsLog ] reading "+fn)
	# get tls states, state transition, duration
	t2s=dict()
	t2trans=dict()
	t2s2dura=dict()
	t2timePhase=dict()
	t2lastph = dict()
	t2laststart = dict()
	with open(fn,"r") as f:
		for l in f:
			if l.startswith("p id="):
				ind = l.split("id=",1)[1].split(" ",1)[0]
				ph = l.split("s=",1)[1].split(" ",1)[0]
				startt = float(l.split("st=",1)[1].split(" ",1)[0])
				if ind not in t2s:
					t2s[ind]=set()
					t2trans[ind]=dict()
					t2s2dura[ind]=dict()
					t2timePhase[ind]=[]
				t2timePhase[ind].append([startt, ph])
				if ind in t2lastph: # skip first line/state
					ph0 = t2lastph[ind] # prev phase 
					# add a valid phase:
					t2s[ind].add(ph0)
					# add transition from prev ph to cur ph:
					if ph0 not in t2trans[ind]:
						t2trans[ind][ph0]=dict()
					t2trans[ind][ph0][ph] = t2trans[ind][ph0].get(ph,0)+1
					# add ph0 duration to a list:
					if ph0 not in t2s2dura[ind]:
						t2s2dura[ind][ph0]=[]
					t2s2dura[ind][ph0].append(startt - t2laststart[ind])
				t2lastph[ind] = ph
				t2laststart[ind] = startt
	print("t2s#%d"%len(t2s))
	for tlsid in t2s2dura:
		for ph in t2s2dura[tlsid]:
			t2s2dura[tlsid][ph]= np.median(t2s2dura[tlsid][ph])
		tmp = {"phases":t2s[tlsid], "transition":t2trans[tlsid], "dura":t2s2dura[tlsid]}
		outf = tls_info_outdir + tlsid
		if _Do_Write: pickle.dump(tmp, open(outf,"wb"))

	return [t2s,t2trans,t2s2dura,t2timePhase]


def lookup(tlsid , tls_info_outdir):
	print("Look Up "+tlsid)
	pprint.pprint(pickle.load(open(tls_info_outdir + tlsid,"rb")) )
	fn = tls_info_outdir+tlsid+"-sl.txt"
	print("Learned at "+fn)
	pprint.pprint(pickle.load(open(fn,"rb")) )


def find(parent, i): # Union find. Merge phases. 
	if parent[i] != i:
		parent[i] = find(parent, parent[i])
	return parent[i]




if __name__ == "__main__": 

	OutputDir = SumoOutBaseDir + Use_Ip + SUMO_PORT+ ThisCodeDir_suffix+ Outdir_suffix +os.sep
	TlsInfoDir = SumoTlsBaseDir + config.TLS_INFO_DIR+ ThisCodeDir_suffix +Tls_Learn_Custom_suffix +os.sep # ground truth written there

	if not os.path.exists(TlsInfoDir): os.makedirs(TlsInfoDir)
	tlsfn = OutputDir+"tlslog.txt"
	carfn = OutputDir+"carlog.txt"
	movfn = OutputDir+"carmov.txt"
	netfn = OutputDir+"netinfo.txt"
	
	print("\nMy_Ip: %s"%My_Ip + "  Use_Ip: "+Use_Ip)
	print("ThisCodeDir_suffix: %s"%ThisCodeDir_suffix)
	print("Outdir_suffix: %s"%Outdir_suffix)
	print("Tls_Learn_Custom_suffix: %s"%Tls_Learn_Custom_suffix)
	print("TlsInfoDir: %s"%TlsInfoDir)
	print(OutputDir, "_Do_Write/pickle =",_Do_Write)
	

	if 'learn' in sys.argv: # learn the tls stages
		t2s,t2trans,t2s2dura,t2timePhase = parseTlsLog(tlsfn, TlsInfoDir)
		tls2inEdges = load_tls_to_edges(t2s)
		ret = parseMoveLog(movfn)
		ret = mergeTimestampsAcc(ret)
		mov = mergeMovements(ret)
		analyzePhases(mov,t2s2dura,t2timePhase, tls2inEdges, TlsInfoDir)
		sys.exit(0)

	if 1 or "carstats" in sys.argv: # show fuel/time, etc.
		get_car_stats(carfn, netfn, OutputDir)



