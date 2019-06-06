#!/usr/bin/env python

def feedback_incross_stops(feedbackData, timestamp, junction):
	if feedbackData is None: return
	label = "cross"
	if label not in feedbackData: feedbackData[label]=dict()
	nid = str(junction.id)
	if nid not in feedbackData[label]: feedbackData[label][nid]=[]
	feedbackData[label][nid].append(timestamp)

def feedback_tls_stops(feedbackData, timestamp, tls):
	if feedbackData is None: return
	label = "tls"
	if label not in feedbackData: feedbackData[label]=dict()
	tlsid = str(tls.id)
	if tlsid not in feedbackData[label]: feedbackData[label][tlsid]=[]
	feedbackData[label][tlsid].append(timestamp)