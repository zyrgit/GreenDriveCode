#!/usr/bin/env python
import numpy as np
import operator
import math
import datetime


def get_weekday_index_given_dateStr(dateStr):# 03/19/2006 # mon = 0
	month,day, year = (int(x) for x in dateStr.split('/')) 
	return datetime.date(year, month, day).weekday()# mon = 0
def get_weekday_index_given_ymdhms(dateStr):# 2010-07-21 14:51:10.0
	p1 , p2 = dateStr.split(' ',1)
	if '-' in p1: sep = '-'
	elif '/' in p1: sep = '/'
	year, month,day = (int(x) for x in p1.split(sep))
	return datetime.date(year, month, day).weekday()
def get_weekday_index_given_mdyhms12(dateStr):# 04/07/2019 02:40:36 PM
	p1 , p2 = dateStr.split(' ',1)
	if '/' in p1: sep = '/'
	elif '-' in p1: sep = '-'
	month,day,year = (int(x) for x in p1.split(sep))
	return datetime.date(year, month, day).weekday()

def get_hour_index_given_str(dateStr):# 04/07/2019 02:40:36 PM ; 2010-07-21 14:51:10.0
	_ , st = dateStr.split(' ',1)
	st = st.strip()
	hourOffset = 0
	if st.endswith("PM"):
		hourOffset = 12
	hour = int(st.split(":",1)[0])
	if hour == 12:
		if hourOffset==12: 
			hourOffset=0
		elif hourOffset==0: 
			hourOffset= -12
	return hour + hourOffset

def get_minute_given_str(dateStr):# 04/07/2019 02:40:36 PM ; 2010-07-21 14:51:10.0
	h, minute, s = dateStr.split(' ',1)[1].split(" ")[0].split(":")
	return int(minute)

def in_bbox_latlng(lat,lng, minLat,minLon,maxLat,maxLon):
	return lat>minLat and lat<maxLat and lng>minLon and lng<maxLon # assume not crossing 0^o line.

def get_heading_given_dirStr(dirstr):# East Bound/ NB / Oneway
	dirstr = dirstr.strip()
	s= dirstr.lower()
	if s.startswith("east"): return 90.
	if s.startswith("west"): return 270.
	if s.startswith("north"): return 0.
	if s.startswith("south"): return 180.
	if dirstr.startswith("EB"): return 90.
	if dirstr.startswith("WB"): return 270.
	if dirstr.startswith("NB"): return 0.
	if dirstr.startswith("SB"): return 180.
	if dirstr.startswith("NE"): return 45.
	if dirstr.startswith("NW"): return 315.
	if dirstr.startswith("SE"): return 135.
	if dirstr.startswith("SW"): return 225.
	return None # one way


def get_target_xy_given_lane_change(x,y,heading,dist,laneOffset):
	ldx,ldy = 0.,0. # adding lane shift
	if laneOffset!=0:
		laneWidth = 3.
		laneDir = heading+90 if laneOffset<0 else heading-90
		ldx,ldy = get_moved_xy(0,0, laneDir%360. ,laneWidth)
	x,y = get_moved_xy(x,y,heading,dist)
	return x+ldx, y+ldy

def get_moved_xy(x,y,heading,dist):
	assert heading>=0
	rad = heading/180.* math.pi
	newx = x + dist* math.sin(rad)
	newy = y + dist* math.cos(rad)
	return newx,newy


def get_2xy_dist(xy1,xy2):
	dx = abs(xy1[0]-xy2[0])
	dy = abs(xy1[1]-xy2[1])
	return (dx**2+dy**2)**0.5


def get_period_given_candidates(cyclst, filter_lb=None, period_diff_max_ratio=0.07, lb_diff_ratio=0.2,  print_str=False):
	''' Try to turn cycle lengths into basic periods.
	- filter_lb: period too small, suspect to be noise. 
	- period_diff_max_ratio: regard as 1 or multi periods if relative diff ratio smaller than this.
	- lb_diff_ratio: start from 1st suspect, keep skipping if not exceeding this diff.
	'''
	if not cyclst: return None
	cyclst.sort()
	if print_str: print(" ----  Cycle list",cyclst)
	while filter_lb is not None and cyclst[0] < filter_lb:
		if len(cyclst)==1 : return None
		first,k = cyclst[0],1
		while k<len(cyclst):
			if abs(cyclst[k]-first)/first < lb_diff_ratio: k+=1
			else: break
		cyclst2 = cyclst[k:] # remove too short cyc
		if len(cyclst2)>0: # do not rm if leading to empty
			cyclst = cyclst2
		if print_str: print("cyc-list cut",cyclst)
	# check multiple spanning cycles:
	i,maxvote,bestT = 0,0,None
	while i<len(cyclst): # vote for a base cycTime
		baset = cyclst[i]
		vote = 0
		for j in range(0,len(cyclst)):
			if j==i: continue
			quotient = (cyclst[j]//baset)
			remainder= (cyclst[j]%baset)
			ratio = min(remainder/baset, 1-remainder/baset)
			if ratio < period_diff_max_ratio: 
				vote+=1
		if vote > maxvote:
			maxvote=vote
			bestT=baset
		i+=1
	baset = bestT
	if bestT is None:
		baset = cyclst[0]
		if print_str: print("[common/util.py] Failed to find base cycle, return None")
		return None

	if print_str: print("Base cycle period %f"%baset)
	newcyclist=[]
	for i in range(len(cyclst)):# split up multi-span according to baseT
		m = 1
		err = abs(cyclst[i]-m*baset)
		while 1:
			m+=1
			if abs(cyclst[i]-m*baset)>err:
				m-=1
				break
			err = abs(cyclst[i]-m*baset)
		if abs(cyclst[i]-m*baset)/cyclst[i] < period_diff_max_ratio:
			newcyclist.append(cyclst[i]/m) # divide spanned cyc into single
	assert len(newcyclist)>0
	return newcyclist # list of period candidates


def merge_votes(votelist, maxMinRatio=3., gapRatio=0.2,  print_str=False): 
	''' - votelist: [VoteCluster(),..]
	 - maxMinRatio: clamp input votes to avoid large diff.
	 - gapRatio: merge medians within this ratio.'''
	minnum = 1e6
	maxnum = 0
	maxvc = None
	for c in votelist:
		if minnum>c.get_size():
			minnum = c.get_size()
		if maxnum<c.get_size():
			maxnum = c.get_size()
			maxvc=c
	for c in votelist: # adjust if vote size diff is too large. 
		if c.get_size()> maxMinRatio *minnum:
			c.resize(maxMinRatio *minnum)
	votelist.sort(key=operator.attrgetter('median'))
	maxgap = maxvc.get_val() * gapRatio # merge until this gap
	orisize = len(votelist) # ori size of a majority voting pool
	if print_str: print(votelist)
	while len(votelist)>1: # merging [(cyclen,num),...] votes
		mingap = 1e6
		ind = 0
		for i in range(len(votelist)-1): # this is sorted:
			gap = votelist[1+i].get_val()-votelist[i].get_val()
			if gap<mingap:
				mingap=gap
				ind=i
		if mingap>maxgap:
			break
		votelist[ind ].dissolve_other(votelist.pop(ind+1))
	if print_str: print(votelist)
	return votelist

def get_max_vote(votelist):
	maxsize = 0
	vc = None
	for c in votelist:
		if c.get_size()>maxsize: # the majority vote cluster.
			vc=c
			maxsize=c.get_size()
	return vc.get_val()


class VoteCluster:
	def __init__(self, val, size=1):
		self.vals = [val for _ in range(size)]
		self.size = size
		self.median = np.median(self.vals)
	def get_val(self,):
		return self.median
	def get_size(self,):
		return self.size

	def dissolve_other(self, other): # dissolve the other one.
		self.vals += other.vals
		self.size = len(self.vals)
		self.median = np.median(self.vals)
		other.vals = None
		other.size = 0
		other.median = None
	def resize(self, newsize):
		assert newsize>0
		self.vals.sort()
		while len(self.vals)>=2+newsize:
			self.vals.pop(0) # keep middle median
			self.vals.pop(-1)
		while len(self.vals)>newsize:
			self.vals.pop(-1)
		self.size= len(self.vals)
		
	def __repr__(self):
		return "v=%.2f,n=%d"%(self.median , self.size)


def gzipencode(content):
    import StringIO
    import gzip
    out = StringIO.StringIO()
    f = gzip.GzipFile(fileobj=out, mode='w', compresslevel=5)
    f.write(content)
    f.close()
    return out.getvalue()
