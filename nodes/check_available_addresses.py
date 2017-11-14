#!/usr/bin/env python
import gclib

if __name__ == '__main__':
  # Instance of the gclib python class
  g = gclib.py()
  print('gclib version: {}'.format(g.GVersion()))
  # Get Ethernet controllers requesting IP addresses
  print('Listening for controllers requesting IP addresses...')
  try:
    ip_requests = g.GIpRequests()
    for id in ip_requests.keys():
      print('{0} at mac {1}'.format(id, ip_requests[id]))
  except:
    pass
  print('\nAvailable addresses:')
  available = g.GAddresses()
  for a in sorted(available.keys()):
    print('{0}:{1}'.format(a, available[a]))
