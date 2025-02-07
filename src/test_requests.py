#!/usr/bin/env python

import requests
import traceback


url = 'https://w3schools.com/python/demopage.php'
myobj = {'somekey': 'somevalue'}

#to demonstrate the 'timeout' parameter, we set the timeout to 0.001 to guarantee that the connection will be timed out:

try:
	x = requests.post(url, data = myobj, timeout=0.001)
	print(x.text )
	print(x.status_code)
	print(x.ok)
except Exception as inst:
	traceback.print_exc()


