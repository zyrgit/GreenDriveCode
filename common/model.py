#!/usr/bin/env python

import os, sys, getpass, glob
import subprocess
import random, time
import inspect
from sklearn.externals import joblib
import cPickle as pickle
import pprint
from .constants import *
mypydir= os.path.abspath(os.path.dirname(inspect.getfile(inspect.currentframe())))

_iprint = 0   

VTCPFEM_features = [VTCPFEMv0,VTCPFEMv1,VTCPFEMv2,VTCPFEMv3,VTCPFEMv4,VTCPFEMv5,VTCPFEMv6,VTCPFEMav1,VTCPFEMav2,VTCPFEMav3,VTCPFEMav4,VTCPFEMa2v2] # VT-CPFEM model 

''' greenroute: ../zyrcode/6*.py out model: '''
try:
	Global_model_pickle = joblib.load("%s/model_lr"%mypydir)
	Global_model= Global_model_pickle["model"]
	Global_model_coef= Global_model.coef_
	if _iprint:
		print('From 6.py Coefficients:', Global_model_coef, "intercept: ", Global_model.intercept_)
	Global_model_features = Global_model_pickle["features"]
	Global_feat2coef = {}
	for i in range(len(Global_model_features)):
		Global_feat2coef[Global_model_features[i]]= Global_model_coef[i]
	if _iprint:
		print("Global_model_features",Global_model_features)
except:
	print("\nException loading "+"%s/model_lr !!\n"%mypydir)


if __name__ == "__main__":
	
	arglist=sys.argv[1:]




