import cereal.messaging as messaging

# in subscriber
sm = messaging.SubMaster(['carControl'])
while 1:
  sm.update()
  print(sm['carControl'])
