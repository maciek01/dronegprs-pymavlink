#!/usr/bin/env python

import httplib2
import traceback
import json


url = 'https://w3schools.com/python/demopage.php'
myobj = {'somekey': 'somevalue'}

#to demonstrate the 'timeout' parameter, we set the timeout to 0.001 to guarantee that the connection will be timed out:

try:

	httplib2.debuglevel     = 0
	http                    = httplib2.Http(timeout=0.001)

	response, content = http.request(url , 'POST', json.dumps(myobj), headers={"Content-Type": "application/json"})

	print(content)
	print(response.status)
except Exception as inst:
	traceback.print_exc()


