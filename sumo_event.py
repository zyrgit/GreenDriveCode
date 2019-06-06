#!/usr/bin/env python
from common import *
import heapq
config = None
traci = None

def ev_reached_time(baseTime): # trigger func
	return lambda a : a >= baseTime


class Event: 
	def __init__(self, name, priority, trigger_func, input_list, allow_duplicate=False, only_once_in_em=False, data_dic=None):
		self.name = name
		self.priority= priority #int smaller is higher, P0 exec earlier than P1...
		self.trigger_func = trigger_func # return True to trigger
		self.input_list = input_list # input names to trigger_func
		self.allow_duplicate = allow_duplicate # multi events same name same time will exec multi times.
		self.only_once_in_em= only_once_in_em # keep only 1 replica 
		self.data = data_dic

	def is_triggered(self, params):
		args = []
		for arg in self.input_list:
			args.append(params[arg])
		return self.trigger_func(*args)
	
	def __lt__(self, other):
		if self.priority == other.priority:
			return self.name < other.name
		return self.priority < other.priority # smaller in the front
	def __eq__(self, other):
		return self.priority == other.priority and self.name == other.name
	def __ne__(self, other):
		return not self.__eq__(other)
	def __repr__(self):
		return "%s,%d"%(self.name,self.priority)
	def __hash__(self):
		return hash(self.name)


class EventQueue: 
	def __init__(self,):
		self.que = []
	def add(self, event):
		heapq.heappush(self.que, event)
	def pop(self,):
		return heapq.heappop(self.que)
	def is_empty(self,):
		return len(self.que)==0


class EventManager: 
	def __init__(self, ):
		self.event_list = list() # allow_duplicate
		self.event_dict = dict() # only_once_in_em
	
	def add_event(self, event, print_str=False):
		if print_str: print("[add_event] "+str(event))
		if event.only_once_in_em:
			if event.name not in self.event_dict:
				self.event_dict[event.name] = event# same name not twice in whole queue
		else: self.event_list.append(event)

	def get_events_to_exec(self, params):
		eventQueue = EventQueue()
		eventNames = set()
		i=0
		while i < len( self.event_list ):
			event = self.event_list[i]
			if event.is_triggered(params):
				if not event.allow_duplicate and event.name in eventNames: self.event_list.pop(i)
				else: eventQueue.add(self.event_list.pop(i))
				eventNames.add(event.name)
			else: i+=1
		for name,event in self.event_dict.items():
			if event.is_triggered(params):
				if not event.allow_duplicate and name in eventNames: self.event_dict.pop(name)
				else: eventQueue.add(self.event_dict.pop(name))
				eventNames.add(name)
		return eventQueue


	def __repr__(self):
		return "EM%s%s"%(str(self.event_list),str(self.event_dict.values()))




