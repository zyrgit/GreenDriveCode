#!/usr/bin/env python
from common import *
from configure.assemble import *
from configure.params import region2serverIP,Mytools_Rel_Dir

print("server.py only needs 'addr' in the task dict.")
print("This should run on: "+region2serverIP[R['addr']]['host'] )
import subprocess, pprint
import random, time
import urllib2, httplib, urlparse, cgi
import requests
import threading, thread, signal
from copy import deepcopy
import glob,json
import socket, SocketServer
from datetime import datetime, timedelta
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
from sumolib import checkBinary
import traci

sys.path.append(mypydir+ Mytools_Rel_Dir) # install greenroute
from namehostip import get_my_ip
from logger import SimpleAppendLogger
from a4loadServer import load_osm_nodes,load_sumo_juntions,load_sumo_edges,load_sumo_connections,load_sumo_out_edges

l = SimpleAppendLogger("./logs/"+__file__.split(os.sep)[-1].split(".",1)[0]+".log",500) # KB 
DateFormat = "%a %b %d %H:%M %Y"
l.lg_list(["server_run", datetime.now().strftime(DateFormat), get_my_ip()])

iprint = 1

class DataManager(object):
	def __init__(self, *args):
		self.osm_nodes = None # osm idstr to {lat,lng}
		self.sumo_junctions = None # idstr to {'id','type','x','y','incLanes'}
		self.sumo_edges= None # idstr to {'id','from','to','priority','type'}
		self.sumo_connections= None # tup(e0,e1) to [{'from','to','via','dir','tl','linkIndex','fromLane','toLane'}, ]
		self.sumo_out_edges=None # eid to {out-eid: ['via','dir',fromLane','toLane']}
		self.OSM_File = mypydir +mapFolder+os.sep +config.osmfile+ ".osm"
		self.AddTlsNetfile=mypydir +mapFolder+os.sep+"%s.tls.net.xml"%config.osmfile
	
	def load_data(self,):
		if self.osm_nodes is None:
			self.osm_nodes = load_osm_nodes(self.OSM_File)
			self.sumo_junctions = load_sumo_juntions(self.AddTlsNetfile)
			self.sumo_edges = load_sumo_edges(self.AddTlsNetfile,self.sumo_junctions)
			self.sumo_connections = load_sumo_connections(self.AddTlsNetfile)
			self.sumo_out_edges= load_sumo_out_edges(self.sumo_connections)


class myHandler(BaseHTTPRequestHandler):

	def do_POST(self):
		if iprint>=2 : print("do_POST, path="+self.path)
		form = cgi.FieldStorage(fp=self.rfile, headers=self.headers,  environ={'REQUEST_METHOD':'POST','CONTENT_TYPE':self.headers['Content-Type'],} )
		if iprint>=3: 
			print self.headers['Content-Type']
			print self.headers['Content-Length']
		res = None
		if self.path=="/data/osm_nodes": 
			idstr = str(form["id"].value)
			res = self.server.dm.osm_nodes.get(idstr,{})
		elif self.path=="/data/sumo_junctions": 
			idstr = str(form["id"].value)
			res = self.server.dm.sumo_junctions.get(idstr,{})
		elif self.path=="/data/sumo_edges": 
			idstr = str(form["id"].value)
			res = self.server.dm.sumo_edges.get(idstr,{})
		elif self.path=="/data/sumo_connections": 
			lst = [Field.value for Field in form["id"]] # fromE, toE  as list
			tup = tuple(lst)
			res = self.server.dm.sumo_connections.get(tup,[])
		elif self.path=="/data/sumo_out_edges": 
			idstr = str(form["from"].value)
			res = self.server.dm.sumo_out_edges.get(idstr,{})

		if res is not None:
			self.send_response(200)
			self.end_headers()
			self.wfile.write(json.dumps(res))
			return	



	def do_GET(self): # Not in use 
		if iprint>=2: print("do_GET, path="+self.path)
		try:
			mimetype='text/plain'
			sendReply = True
			if sendReply:
				self.send_response(200)
				if "%s/healthcheck"%pathbeg == self.path:
					if iprint>=2: print("checking health...")
					content="<html><body><h1>ok</h1></body></html>"
					self.send_header("Content-length", str(len(str(content))))
					self.send_header('Content-type','text/html;charset=utf-8')
					self.end_headers()
					self.wfile.write(content)
					self.wfile.flush()
			return
		except IOError:
			self.send_error(404,'File Not Found: %s' % self.path)


	
	def _set_headers(self):
		self.send_response(200)
		self.send_header('Content-type', 'text/html')
		self.end_headers()

	def do_HEAD(self):
		self._set_headers()
	def log_message(self, format, *args):
		return

if __name__ == "__main__":
	try:
		# create/load data
		dm = DataManager()
		dm.load_data()
		#Create a web server and define the handler to manage the incoming request
		server = HTTPServer((My_Ip, region2serverIP[R['addr']]['port'] ), myHandler)
		server.dm = dm
		print 'Started httpserver addr %s port %d' %(My_Ip, region2serverIP[R['addr']]['port'] )
		server.serve_forever()

	except KeyboardInterrupt:
		print '^C received, shutting down the web server'
		server.socket.close()

