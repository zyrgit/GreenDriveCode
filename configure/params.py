#!/usr/bin/env python
from .settings import * # mypydir My_Ip My_Platform Has_GUI_Device


region2serverIP = { # server.py
		'chicagoL': {'host':My_Ip,  'port':9876},
		'chyde':    {'host':My_Ip,  'port':9875},
		'cwest':    {'host':My_Ip,  'port':9874},
		'clake':    {'host':My_Ip,  'port':9873},
	}


addr2Same_Group_Dist ={ } # not in use
addr2Coor_Period ={ }
addr2Coor_Max_Time_Gap ={ }
