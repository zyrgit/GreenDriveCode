#!/usr/bin/env python
from common import *
from configure.assemble import *
from configure.params import Mytools_Rel_Dir
import optparse

addpath=mypydir+ Mytools_Rel_Dir
if addpath not in sys.path: sys.path.append(addpath)
from sumolib import checkBinary
import traci
from sumo_tls import Tls,Stage,Phase,MovesOfPhase,TlsMove,EventTime

''' --------------- just a test for GUI ------------'''
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("-n","--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

options = get_options()
if options.nogui:
    sumoBinary = checkBinary('sumo')
else:
    sumoBinary = checkBinary('sumo-gui')

_cfg_file = mapFolder+os.sep+"sumo-cfg/net.sumo.cfg-a2.xml"

if 'loop' in sys.argv: 
	_cfg_file = mapFolder+os.sep+"sumo-cfg/net.sumo.cfg-a2-loop.xml"

sumoProcess = subprocess.Popen([sumoBinary, "-c", _cfg_file, "--remote-port", SUMO_PORT], stdout=sys.stdout, stderr=sys.stderr)

traci.init(int(SUMO_PORT))

tlslist=traci.trafficlights.getIDList()
print '>>> # tlslist\n',len(tlslist)


curtime=0
while curtime < config.simulationEndTime :
	traci.simulationStep()
	curtime = traci.simulation.getTime() -1  
	if tls: tls.process(curtime)

traci.close()
sumoProcess.wait()
