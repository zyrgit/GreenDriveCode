#!/usr/bin/env python

import os,sys

if 'SUMO_HOME' in os.environ:
    SUMO_HOME=os.environ['SUMO_HOME']
else: SUMO_HOME=os.path.expanduser("~")+os.sep+"sumo"+os.sep+"sumo-1.1.0/"

SUMO_Tools_dir = SUMO_HOME+os.sep+"tools"
if SUMO_Tools_dir not in sys.path: 
	sys.path.append(SUMO_Tools_dir)

