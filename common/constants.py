#!/usr/bin/env python

#VT-CPFEM:
VTCPFEMv0='vtv0'
VTCPFEMv1='vtv1'
VTCPFEMv2='vtv2'
VTCPFEMv3='vtv3'
VTCPFEMv4='vtv4'
VTCPFEMv5='vtv5'
VTCPFEMv6='vtv6'
VTCPFEMa2v2='vta2v2'
VTCPFEMav1='vtav1'
VTCPFEMav2='vtav2'
VTCPFEMav3='vtav3'
VTCPFEMav4='vtav4'

indMass=0 # for greenroute
indArea=1
indDrag=2

GramPerGallon = 2834
MetersPerMile = 1609.344
Mph2MetersPerSec = 0.44704

Kdist="dis"
Kelevation="ele"
Kgas="gas" # rescaled/normalized
Krealgas="realgas"
KincSpeed2="incV2" # all segs in a trip.
Kv2d="v2d"
Kvd="vd"
Ktime="time" # OSRM estimated time, dist/spd
KSmjcrs="mjn"
# turn features:
TSelfV="t0v" # U turn leg speed
TLeftV="t1v" # left turn leg speed
TRightV="t3v" # right turn leg speed
TStraightV="t2v" # Straight leg speed
TminV="tminv" # min speed seen around cross.
TspdDec="tvdec" # speed dec percent at cross.
TNleft="tn1" # number of left turns in trip.
TNright="tn3"
TNstraight="tn2"
Ttype="ttyp"
TstopLeft="ts1" # number of truely stopped turns in trip.
TstopRight="ts3"
TstopStraight="ts2"
TnidFrom="tfr" # nid of turn from 
TnidAt="tat" # nid of turn at cross
TnidLeft="tle"
TnidRight="tri"
TnidStraight="tst"
TPleft="tp1" #cumu prob of stopping at cross. 
TPright="tp3"
TPstraight="tp2"
TPspd2inc="pv2inc" # from speed dec distrib 
Ttime="tti" # true
KaddWaitTime="wtime"
TPwtime="pwti" # trip time plus prob * wait time.
KtripTime="tripti" # true whole trip time.
KtagType="tag" # slow/medium/fast
KelevDecNeg="eleDec" # downhill, negative of elevation decrease
KelevInc="eleInc" # just increment, after adjust end-to-end (may decrease)
RealSegSpeedinc='rvinc' # like greengps, real block v^2 inc. 

KMmass="mass"
KMair="drag"
KMarea="area"
KMmdv2="mdv2"
KMmelev="mele"
KMav2d ="av2d"
KMdragv2d ="drav2d"
KMmd = "md"
KMvd = "v*d"
KMmleft="ms1"
KMmright="ms3"
KMmstraight="ms2"
KMmtime="mtime"

CMEMv0='cmv0' # not in use
CMEMv1='cmv1'
CMEMv2='cmv2'
CMEMv3='cmv3'
CMEMv4='cmv4'


Turn_Left=1
Turn_Right=3
Turn_Straight=2
T_turn01=4
T_turn02=5
T_turn12=6
T_turn10=7
T_turn20=8
T_turn21=9
kMinTurnAngleDiff=10.0
kMinTurnGasPenalty=0.0
