#!/usr/bin/env python

from common import *
import numpy as np


class WeightDistribution:
	def __init__(self, numSlot, name=''): 
		self.name = name
		self._d = [1e-6 for i in range(numSlot)]
		self._norm = [0. for i in range(numSlot)]
	def set_weight_at_index(self, ind, wt):
		self._d[ind] = wt
	def normalize(self,):
		s = sum(self._d)
		for i in range(len(self._d)):
			self._norm[i] = self._d[i]/s
	def get_percentage_at(self, ind):
		return self._norm[ind]

	def __repr__(self):
		wstr = " ".join(map(lambda x: "%.3f"%x, self._norm))
		return "%s: %s"%(self.name, wstr)


class YearData:
	def __init__(self, name , maxSizePerSlot=40): 
		self.name = name
		self.size = maxSizePerSlot
		self.d = {}

	def ingest(self, val, hour, weekday, day, month, year):
		d = self.d
		if year not in d: d[year]={}
		if month not in d[year]:d[year][month]= SampleData("%d-%d"%(year,month), self.size)
		d[year][month].ingest(val, hour, weekday)

	def has_enough_data(self, ):
		d = self.d
		for year in d.keys():  
			for mon in d[year].keys():
				if not d[year][mon].has_data_every_weekday():
					return False
			return True
		return True

	def get_mean_at_hour_wkd_mon_year(self, h, weekday, month, year):
		return self.d[year][month].get_mean_at_day_hour(weekday,h)

	def show_some_samples(self):
		d = self.d
		for year in d.keys():  
			for mon in d[year].keys():
				d[year][mon].show_some_samples()
				return

	def __repr__(self):
		d , wstr = self.d , self.name+":\n    "
		for year in d.keys():
			for mon in d[year].keys():
				wstr += str(d[year][mon])+"\n    "
		return wstr


class SampleData: # in a week 
	def __init__(self, name='' , maxSizePerSlot=40): 
		self.name = name
		self.size = maxSizePerSlot
		self.da = [ [ [] for i in range(24) ] for j in range(7) ]# weekday[24hour]
		self.mean = [ [ [] for i in range(24) ] for j in range(7) ] 

	def ingest(self, val, hour, weekday):
		lst = self.da[weekday][hour]
		if len(lst)<self.size:
			lst.append(val)

	def has_data_every_weekday(self,):
		for d in range(7):
			good=0
			for h in range(24):
				if len(self.da[d][h])>0:
					good=1
					break
			if good==0:
				return False
		return True

	def has_data_at_day_hour(self, d, h):
		try:
			if len(self.da[d][h])>0: return True
		except:
			print(self.da)
			print("Wrong at d,h",d,h)
			sys.exit(1)
		return False

	def get_mean_at_day_hour(self, d,h):
		if self.has_data_at_day_hour( d, h):
			return sum( self.da[d][h] )/len(self.da[d][h])
		for i in range(1,7):
			dd = (d+i)%7
			if len(self.da[dd][h])>0:
				return sum( self.da[dd][h] )/len(self.da[dd][h])

	def summary(self,):
		wstr = ""
		for wkday in range(7):
			wstr += "d%d{"%wkday
			for hour in range(24):
				self.mean[wkday][hour] = sum(self.da[wkday][hour])/max(1,len(self.da[wkday][hour]))
				if self.mean[wkday][hour]>0: wstr += "%d:%.1f "%(hour,self.mean[wkday][hour])
			wstr += "} "
		return wstr

	def show_some_samples(self,):
		print(self.da)
		print(np.std(self.da[2][10]))

	def __repr__(self):
		return "%s,%s"%(self.name, self.summary())


