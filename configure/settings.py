#!/usr/bin/env python
import sys,os,inspect,socket
import cPickle as pickle

_thisDir =os.path.abspath(os.path.dirname(inspect.getfile(inspect.currentframe())))
mypydir = _thisDir.rsplit(os.sep,1)[0]+os.sep

try:
	My_Ip = ([l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0])
except:
	HomeDir = os.path.expanduser("~")
	_cmd = "bash "+HomeDir+os.sep+"get_my_ip.sh"
	_output = subprocess.check_output(_cmd.split())
	My_Ip = _output


import platform
_p = platform.platform().lower()

My_Platform = ''
if "darwin" in _p: 
	My_Platform = "mac"
	My_Ip = 'localhost' 
elif "linux" in _p and "debian" in _p: My_Platform = "ubuntu"
elif "linux" in _p and "centos" in _p: My_Platform = "centos"

if My_Platform == 'centos': 
	Has_GUI_Device = 0
else: 
	Has_GUI_Device = 1


Mytools_Rel_Dir = "../zyrcode/mytools" # if you install greenroute.


''' ---------- R func --------------'''

def load_task_R(task_key):
	log_file = mypydir +'logs/' + My_Ip + os.sep + task_key
	if not os.path.exists(log_file): 
		print(task_key+": No log file found")
		return None
	return pickle.load( open(log_file,'rb'))

def write_task_R(R):
	if 'log_file' not in R:
		print(R)
		print("No log file requested.")
		return
	pickle.dump(R, open(R['log_file'],'wb'))


def get_key_from_task_dic(dic): # suffix name for task
	addr = dic.get('addr','')
	sfx = dic.get('sfx','')
	tls1 = dic.get('tls1',0)
	tls2 = dic.get('tls2',0)
	split = dic.get('split',0)
	pene = dic.get('pene',0)
	acmp = dic.get('acmp',0)
	bcmp = dic.get('bcmp',0)
	coor = dic.get('co',0)
	key=addr
	if tls1==0 and tls2==0: key+='.no'
	elif tls1==1 and tls2==0: key+='.t1'
	elif tls1==1 and tls2==1: key+='.t2'
	if split>0: key += '.s%d'%split
	if pene>0:  key += '.p%d'%pene
	if acmp>0 or bcmp>0: 
		key+='.c'
		if acmp>0: key += 'a%d'%acmp
		if bcmp>0: key += 'b%d'%bcmp
	if coor>0: 
		key+= '.co.'
		if 'coD' in dic: key+='d%d'%dic['coD']
		if 'coT' in dic: key+='t%d'%dic['coT']
		if 'coG' in dic: key+='g%d'%dic['coG']
	return key + sfx



