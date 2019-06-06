# GreenDriveCode

This is the code for SUMO simulation of GreenDrive, CoDrive and GreenRoute. Some code will use my GreenRoute code at https://github.com/zyrgit/GreenRouteCode. I assume that you also installed that. I also assume you put this repo in `~/syncdir/GreenDriveCode/` to run.

Note, I have an alias in Linux command for Python, as specified in `~/.bashrc`:
```
alias py='$HOME/anaconda2/bin/python'
```

---
## Setup
Download and install `sumo-1.1.0`. 
Make sure you have `SUMO_HOME` defined in your environment, so that in Python, `os.environ['SUMO_HOME']` can be resolved. 


---
## Download osm data from bbbike website
https://extract.bbbike.org/

Then create a folder for the osm file, name the folder and osm file the same name, such as `./addr/addr.osm`.


---
## Generate net file
Edit and run `py a0NetProcess.py`.

You will get `*.tls.net.xml`, then run `py server.py` to load network data structures into memory. If you think that SUMO generates less lane numbers than expected. You can edit `SUMO_HOME+os.sep+"data"+os.sep+"typemap"+os.sep+"osmNetconvert.typ.xml"` file to increase lane number per edge.


---
## Generate random trips
Run these commands to generate traffic, re-organize route/trip files. 
```
py a1GenRoutes.py trips duarouter iter # generate
py a1GenRoutes.py cp  # copy, move
py a1GenRoutes.py splitduaroutes
```

If you downloaded taxi dataset, then you can run:
```
py a1GenRoutes.py procrawtaxi
py a1GenRoutes.py extracttaxi
```


---
## Run initial simulation and check GUI
Edit config files in `./addr/sumo-cfg/net.sumo.cfg*.xml` and edit `a2RunSumo.py`, run and see. 


---
## Run SUMO to learn traffic signal schedule
Now we can run first round of simulation to generate logs to learn TLS (a.k.a. traffic signal) schedules.

### If you use my format to run tasks, then define a task:
Edit `./configure/tasks.py`, add your configuration. For now only 'split' matters, give a number that leads to a medium vehicle density. 

### If you are using cache, like Redis, to speed up initializing trip/road structures:
For the first time running, when prompted to enter "Overwrite Last Cached Rum Time", key-in `0`. After the previous run, it will log down the last cached simulation time, and next run will read from cache until reaching that time. To overwrite cached timestamp, enter another number when prompted. 
Without Redis cache, it will query road structure from `server.py`, which is very slow. 

### Run SUMO
SUMO will use config file defined in `./configure/tasks.py: 'cfg':'net.sumo.cfg-plain.xml'` for this. 
The dict in `tasks.py` will be loaded as `R` in python main, since for now we are not using any speed-advisory service, make sure `R['tls1']==0 and R['tls2']==0`.

Edit and run: `py b0simuCollect.py`.

It will generate car movements and tls logs, for SPaT (signal phase and timing) learning.


---
## Process output, learn TLS, get car stats
After running SUMO simulation, we can run this to learn tls SPaT:
`py a3process.py learn`.

You can also run this to get car stats about fuel, time, etc.:
`py a3process.py carstats`.


---
## Insert Google's vs. Green routes into route files
We can compare with Google's routes in terms of fuel efficiency by having vehicles running on their routes. It will use GreenRouteCode with Google Maps API keys, set this up yourself. 

Generate comparing routes: `py a5MappingCompareTraces.py genroutes`.

Map routes to SUMO edges: `py a5MappingCompareTraces.py tracemapper`.

Replace original routes: `py a5MappingCompareTraces.py replaceroutes`.


---
## Use traffic count data and road speed data
Process downloaded datasets, traffic counts and road speeds. Map them to SUMO edge, and place an induction loop/detector there. It also generates routes according to the induction loop counts. 
```
py a6InductionLoop.py speed loop xml
```

---
## Use taxi trip data
Process the extracted taxi trips from `a1GenRoutes.py`, and map the trips' origin-destinations (ODs) to SUMO edges, and do the routing. Also you can find those trips that pass by the SUMO network, and get the overlapped part as ODs.
```
py a7Taxi.py map route
```

---
## Run main simulations
After routes are generated, and TLS learning is done, we can start run main simulations here. Similarly, edit `./configure/tasks.py` and config file, and the task dict in this task file will be loaded as `R` in main code. 

To run GreenDrive, set `R['tls1']=1 and R['tls2']=1` in task dict. To run CoDrive on top of GreenDrive, set `R['tls1']=1 and R['tls2']=1 and R['co']=1` in task dict. 

To control vehicle density, set `'split'` value to an desired one according to `a3*.py` analysis. Set `'pene'` value to control system penetration ratio, `R['pene']/R['split']` will be this percentage.

When ready, run:
```
py b1simulate.py
```


