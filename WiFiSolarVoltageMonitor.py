#!/usr/bin/env python3

import urllib.request, json

url = "http://192.168.1.78/status.json"

while True:
	data = json.load( urllib.request.urlopen(url) )
	print(data)
