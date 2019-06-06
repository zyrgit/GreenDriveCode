#!/usr/bin/env python

TlsBadRateThresh =  2.8/300. # feedback only
CrossBadRateThresh = 3./300.

IgnoreVids = set([]) # for GUI only

IgnoreEdges = set([]) # for GUI only

IgnoreTimes = set([]) #for GUI only

IgnoreAfterTime = 30 # for GUI only

KeepTlsSet = set([]) 

BadTlsSet = set([]) # unlearned, disable prediction.

a3LearnTlsTolerate = set([]) # badly learned, ok. 

OutputBadTls = [] # feedback output

OutputBadCross = [] # feedback output

