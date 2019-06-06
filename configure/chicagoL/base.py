#!/usr/bin/env python

addr = "Illinois,US" # mem cache only. 
osmfile='chicagoL' # addr

tls_m2_low = 1./500/500 # suburban
allway_stop_m2_low = 6./500/500 # 6*tls
priority_stop_m2_low = 3./500/500 # 3*tls
tls_m2_high = 1./100/100 # downtown 
allway_stop_m2_high = 0.1 /200/200
priority_stop_m2_high = 1./200/200

Tls_Density = tls_m2_high
Stop_All_Density = allway_stop_m2_high
Stop_Pri_Density = priority_stop_m2_high
SpeedLimitRaiseRatio = 1. 

OUTPUT_DIR = "output"
OSM_QUOTE='"'
TLS_DIR = "tls"
TLS_INFO_DIR = "info"

simulationStartTime=0
simulationEndTime = 4000
routePoolFile = "%s_019.rou.xml"% osmfile
tripDepartPeriod = 0.9
tripAttribute='"type=\\"%s\\" departLane=\\"best\\" departSpeed=\\"max\\" departPos=\\"base\\" arrivalPos=\\"max\\""'
tripMinDistance = 2500
fringeFactor = 10

StopSpeedThresh = 0.1 

TlsMaxPassTimeAfterAcc = 5. # regard ACC as in same phase.
TlsMaxPossiblePhaseDura= 100. 
TlsYellowDuraMax = 4.1 # maxi yellow phase length
TlsMinCycleDura = 15. # min cycle length
TlsMostLikelyCycleDura = 90. 
TlsMaxCycleDura = 200. # max cycle length

TlsQueueMinSpeed = 3. # to be considered in queue.
TlsStopPosMaxDist = 100. # if stop remdist > this, then not an event.
TlsStartTimeBuf= 3. # time buffer
TlsEndTimeBuf= 4. # covers yellow light >3.
TlsEndArrivalTimeBuf= 6. # try more buf than previous

TlsLetGoRemDist = 40. # do not apply speed anymore within this close to tls.
TlsSkipPredictionDist = 30. # bug. may contain too short edge, already incross but still show 2 edges behind.
RemDistEnforceBestLane = 110. # avoid lane deadlock.

CarMinSpeed = 1.8 # spd cmd at least this .
CarMaxSpeed = 50. # also check with <vtype/> 
CarGapPerSpeed = 2. # 'two' second rule of car following keep dist. Affect when low speed.
MaxCarGap = 20. # space no matter how fast, exclude VLen+Gap
ReachSpeedLimitRatio = 0.96 # max go this amount of limit. 
MinSpeedForAcc = 4. # calc delay usage
InCrossMinSpeed = 4. # do not block intersection.

GreenRoute_IP_Port = 'http://0.0.0.0:7784' # for greenroute only.

