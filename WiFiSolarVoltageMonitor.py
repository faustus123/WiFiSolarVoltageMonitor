#!/usr/bin/env python3
#
# Influx DB server may be started with:
#
# docker run -d -p 8086:8086 --name influxdb2 -v ${HOME}/HomeSolar/etc/influxdb2:/etc/influxdb2 -v ${HOME}/HomeSolar/DB:/var/lib/influxdb2 influxdb:2.7.11
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
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.domain.write_precision import WritePrecision

vmon_url = "http://192.168.1.17/status.json"
if len(sys.argv)>1 : vmon_url = f"http://{sys.argv[1]}/status.json"
#vmon_url = "http://192.168.1.81/status.json"
influxdb_host = "http://localhost:8086"
WRITE_TO_DB = True

host='localhost'
port=8086
db_token = 'OQoSwarVK9qtnN9GvZ818IL7Su1VlZoJdqrSeRH8kreFyWdIqsAH8b0zpOpdinnbjuh4GpvM3gH7x956QnDo3A=='
db_org = 'Hill-Lawrence'
db_bucket = 'HL_Solar'

print(f"Data will be read from device at: {vmon_url}")

print("Connecting to DB ...")
# client = InfluxDBClient(host, port, dbuser, dbuser_password, dbname)
client = InfluxDBClient(influxdb_host, token=db_token, org=db_org)
health = client.health()
if health.status == 'pass':
	print("Connected to server running InfluxDB {}".format(health.version))
else:
	print("Failed to connect to DB at \"{}\"".format(influxdb_host))
	sys.exit(-1)

# Get write_api from DB client
write_api = client.write_api(write_options=SYNCHRONOUS)

# Loop forever, updating the DB
period_seconds = 10.0
last_time = datetime.now()
while True:
    
    # Get measurement from remote device as JSON record
	try:
		data = json.load( urllib.request.urlopen(vmon_url) )
		# Convert to dictionary more suitable to InfluxDB
		fields = {}
		for key, value in data.items():
			if key == 'date' : continue
			try:
				fields[key] = float(value)
			except ValueError:
				fields[key] = value

		influxdb_dict = {
			"measurement": "Solar_Vmon",
			"tags": {"location": "back_porch"},
			"fields": fields,
			"time": data['date']}
		print(influxdb_dict)

		# Write to InfluxDB database
		point = Point.from_dict(influxdb_dict, WritePrecision.S)
		if WRITE_TO_DB : write_api.write(bucket=db_bucket, record=point)
	except:
		print("Error getting data or writing to DB. Skipping data point.")
  
	# Sleep for remaining time
	tleft = period_seconds - (datetime.now() - last_time).total_seconds()
	if tleft>0.0 : time.sleep(tleft)
	last_time = datetime.now()

