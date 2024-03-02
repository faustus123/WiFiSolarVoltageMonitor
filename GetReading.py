#!/usr/bin/env python3
#
# Influx DB server may be started with:
#
# docker run -d -p 8086:8086 --name influxdb2 -v ${HOME}/HomeSolar/etc/influxdb2:/etc/influxdb2 -v ${HOME}/HomeSolar/DB:/var/lib/influxdb2 influxdb:2.7.1
#
#
# Start this with:
#
#  source venv/bin/activate.csh
#  ./WiFiSolarVoltageMonitor.py

import urllib.request, json
import argparse
import sys
import time
from datetime import datetime

vmon_url = "http://192.168.1.82/status.json"
#vmon_url = "http://192.168.1.81/status.json"


try:
	data = json.load( urllib.request.urlopen(vmon_url) )
	print(data)
except:
	print("Error getting data from: " + vmon_url)



