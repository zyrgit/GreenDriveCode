#!/usr/bin/env python
import sys

from .tasks import * # R mypydir My_Ip My_Platform Has_GUI_Device
from .params import region2serverIP

ThisCodeDir_suffix = R['csuf']
Outdir_suffix = R['osuf']
Use_Ip = R['ip']
LoopOutSuffix = R['lsuf']
SumoCfgXmlFile = R['cfg']
SUMO_PORT = str(R['sumo_port']) if 'sumo_port' in R else "9999"
if 'co' not in R: R['co']=0 # coor

if R['addr'] == 'chicagoS':
	from .chicagoS import base as config
elif R['addr'] == 'chicagoL':
	from .chicagoL import base as config
elif R['addr'] == 'chicagoloop':
	from .chicagoloop import base as config
elif R['addr'] == 'clake':
	from .clake import base as config
elif R['addr'] == 'cwest':
	from .cwest import base as config
elif R['addr'] == 'chyde':
	from .chyde import base as config


Server_IP_Port = "http://"+ region2serverIP[R['addr']]['host'] +":%d"%region2serverIP[R['addr']]['port']

''' ---------- File Names: -----------'''

mapFolder = config.osmfile # addr
AddTlsNetfile= mypydir +mapFolder+os.sep+"%s.tls.net.xml"%config.osmfile # final osm to use.

''' ---------- a0.py -----------'''
DownloadedOsmFile = mypydir +mapFolder+os.sep+"%s.osm"%config.osmfile # addr/addr.osm
OriOutNetfile =  mypydir +mapFolder+os.sep+"%s.net.xml"%config.osmfile # before change node tls.
NoTlsNodefile = mypydir +mapFolder+os.sep+"%s.notls.txt"%config.osmfile # plain nodes.
PatchNetworkFile =  mypydir +mapFolder+os.sep+"%s.patch.nod.xml"%config.osmfile # node property patch.
TlsIdsBlacklistFile =  mypydir +mapFolder+os.sep+"tls_blacklist.txt" # tls ids != junc ids.
JunctionBlacklistFile =  mypydir +mapFolder+os.sep+"junc_blacklist.txt" # junc to avoid tls.
StopsignBlacklistFile =  mypydir +mapFolder+os.sep+"stop_blacklist.txt" # X,Y to avoid stop sign.

''' ---------- a1.py -----------'''
DuaOutRoutesFile =  mypydir +mapFolder+os.sep+"%s.rou.xml"%config.osmfile # original base traffic
EdgeWeightFile=  mypydir +mapFolder+os.sep+"%s.weight.xml"%config.osmfile
VTypeDefXmlFile =  mypydir +"vtype.add.xml"
DUA_OUT_DIR_Prefix =  mypydir + mapFolder+os.sep+"dua"
SplitRoutesDir =  mypydir + mapFolder+os.sep+"routes"+os.sep
RoutesEdgesDefXmlFile = SplitRoutesDir+"routes.xml"
DownloadsDir = "/Users/yiranzhao/Downloads/"
TaxiRawDownloadFile = DownloadsDir+ 'Taxi_Trips.csv.download/Taxi_Trips.csv'
UsbStorageDir = "/Volumes/usb/" 
TaxiStorageFile = UsbStorageDir+ 'taxi-raw.csv' # 15GB on usb
TaxiDataDir =  mypydir + mapFolder +os.sep+"taxi" +os.sep
TaxiYearMonTripDumpFile = TaxiDataDir +"taxi2016.txt" 

''' ---------- a3.py -----------'''
Trace_Dir = mypydir +mapFolder+os.sep+"trace"+os.sep
TraceRouteStatsDir = Trace_Dir+os.sep+"stats/"
Tls_Learn_Custom_suffix = "-4" 

''' ---------- a5.py -----------'''
GpsTraceFile = Trace_Dir + "trace.txt" # input for tracemapper
IndGpsTraceFile = Trace_Dir + "itrace.txt" # input for tracemapper
RouteStatsDir = Trace_Dir +"stats/"
IndRouteStatsDir = Trace_Dir +"istats/"
TracemapperOutEdgesFile = Trace_Dir +"compare.routes.xml"
ComparisonRouteDefFile = Trace_Dir +"compare.routes.fix.xml"

''' ---------- a6.py -----------'''
FlowRouterDir = mypydir+mapFolder+os.sep+"flow/" 
LoopSpeedDataDir = mypydir+mapFolder+os.sep+"dump/" 
SharedSmallDataDir = mypydir+"data/"
TrafficCountsFile = SharedSmallDataDir+"Average_Daily_Traffic_Counts.csv"
LoopCntDataDumpFile = LoopSpeedDataDir+"loop.txt" 
SpeedDataDumpFile = LoopSpeedDataDir+"speed.txt" 
SpeedSampleDatasetFile = UsbStorageDir+"Chicago_Traffic_Tracker_-_Historical_Congestion_Estimates_by_Segment_-_2018-Current.csv.crdownload" # 12GB
SumoLoopOutDir = mypydir+ mapFolder+os.sep+"loop"+os.sep # output loop stats
LoopXmlFile =  SumoLoopOutDir +"%s.loop%s.xml"%(config.osmfile, LoopOutSuffix)

''' ---------- a7.py -----------'''
TaxiTripXmlFile = TaxiDataDir+"taxi.trip.xml"
TaxiRouteEdgesFile =  TaxiDataDir+"taxi.rou.xml" 
TaxiTripToStatsFile = TaxiDataDir+"taxi.stats.txt"

''' ---------- b*.py -----------'''
SumoOutBaseDir = mypydir +mapFolder+os.sep+ config.OUTPUT_DIR+ os.sep # car mov output
SumoTlsBaseDir = mypydir +mapFolder+os.sep+ config.TLS_DIR +os.sep # tls true phase output
Show_Obj_Signal_File = mypydir+ "logs/signal.txt" # to check GUI 
LastCachedRumTimeFile = mypydir+ "logs/lasttime-%s.txt"%R['addr'] # last cached time
SumoLoopOutputFile = SumoLoopOutDir + "out%s.txt"%LoopOutSuffix
LoopFeedbackLowerSpeedLimitFile = SumoLoopOutDir + "edges-keep-vlimit.txt"
LoopFeedbackMovePositionFile = SumoLoopOutDir + "edges-backoff-loop.txt"

SumoCfgFilePath = mypydir +mapFolder+os.sep+"sumo-cfg"+os.sep+ SumoCfgXmlFile



