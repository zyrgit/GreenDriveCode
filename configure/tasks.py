#!/usr/bin/env python
from .settings import * # mypydir My_Ip My_Platform Has_GUI_Device
import cPickle as pickle
import glob
addpath= mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)
from mem import AccessRestrictionContext


tasks = [ # a.k.a. R:

{'addr':'chicagoL','cfg':'net.sumo.cfg-plain.xml', 'sfx':'', 'tls1':0, 'tls2':0, 'co':0, 'split':22, 'pene':0 , 'coT':10, 'coG':30, 'coD':300 , 'sumo_port':9996 },

]


# allocate tasks via a redis lock:
_lock= AccessRestrictionContext(prefix="sumo~task1~", persistent_restriction=True, persist_seconds=864000,)
_Log_Dir = mypydir+'logs/'
R = None

#  maybe already in log file?
if R is None: 
	outdir = _Log_Dir+ My_Ip + os.sep
	log_flist = glob.glob(outdir+"*")
	log_keys = set()
	for fn in log_flist: # previously run 
		log_keys.add(fn.rsplit(os.sep,1)[1])
	for dic in tasks:
		task_key = get_key_from_task_dic(dic) # sfx
		if task_key in log_keys:
			log_file = outdir + task_key
			R = pickle.load( open(log_file,'rb'))
			print('\n'+ My_Ip+" [ resume ] "+task_key+',  '+log_file)
			break

# tasks not assigned before
if R is None: 
	for dic in tasks: 
		task_key = get_key_from_task_dic(dic) 
		with _lock:
			_lock.Access_Or_Skip(task_key)
			outdir = _Log_Dir+My_Ip + os.sep
			if not os.path.exists(outdir): os.makedirs(outdir)
			log_file = outdir + task_key
			dic['task_key'] = task_key
			dic['ip'] = My_Ip
			dic['log_file'] = log_file
			dic['run_status'] = 'starting'
			pickle.dump(dic, open(log_file,'wb'))
			R = dic
			print'\n'+ (My_Ip+" [ got ] "+task_key+',  '+log_file)
			break


if R is None: 
	print(My_Ip+"  No task allocated...")
	sys.exit(0)

if 'run_status' in R and R['run_status']=='done':
	print(My_Ip+"  Already done:")
	print(R)
	sys.exit(0)

# assemble R to conform to older version:
R['csuf'] = ''
R['osuf'] = R['task_key'] # output suffix
R['code name']= R['task_key']
if 'lsuf' not in R: R['lsuf'] = R['csuf']+R['osuf']

R['run_type'] = 'tasks'


