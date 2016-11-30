def newVertex(stdscr):
  done = 0
  while done == 0:
    stdscr.addstr(1, 0, "vertex: index?")
    stdscr.clrtoeol()
    stdscr.refresh()
    c = chr(stdscr.getch())
    try:
      int(c)
      done = 1
    except:
      pass
  done = 0
  vertex_index = int(c)
  x_coord = 0
  while done == 0:
    stdscr.addstr(1, 0, "vertex: x_coord? {}".format(x_coord))
    stdscr.clrtoeol()
    stdscr.refresh()
    c = chr(stdscr.getch())
    try:
      int(c)
      x_coord = x_coord*10 + int(c)
    except:
      if(c == '\n' or c == '\r'):
        done = 1
  y_coord = 0
  done = 0
  while done == 0:
    stdscr.addstr(1, 0, "vertex: y_coord? {}".format(y_coord))
    stdscr.clrtoeol()
    stdscr.refresh()
    c = chr(stdscr.getch())
    try:
      int(c)
      y_coord = y_coord*10 + int(c)
    except:
      if(c == '\n' or c == '\r'):
        done = 1
  stdscr.addstr(1, 0, "VERTEX MESSAGE: {} {} {}".format(vertex_index, x_coord, y_coord))
  outputMsgBody = ("{{\"RESPONSE\" : {{ \"SOURCE\" : \"SERVER\" , \"DEST\" : \"TargetLocator\" , \"ID\" : 0 \"TYPE\" : \"VERTEX\" , {{\"VERTEX_ID\" : {0} , \"X_COORD\" : {1} , \"Y_COORD\" : {2} }}}}}}".format(vertex_index, x_coord, y_coord))
  return outputMsgBody
  
def newLocation(stdscr):
  done = 0
  x_coord = 0
  while done == 0:
    stdscr.addstr(1, 0, "Location: x_coord? {}".format(x_coord))
    stdscr.clrtoeol()
    stdscr.refresh()
    c = chr(stdscr.getch())
    try:
      int(c)
      x_coord = x_coord*10 + int(c)
    except:
      if(c == '\n' or c == '\r'):
        done = 1
  y_coord = 0
  done = 0
  while done == 0:
    stdscr.addstr(1, 0, "Location: y_coord? {}".format(y_coord))
    stdscr.clrtoeol()
    stdscr.refresh()
    c = chr(stdscr.getch())
    try:
      int(c)
      y_coord = y_coord*10 + int(c)
    except:
      if(c == '\n' or c == '\r'):
        done = 1
  signTheta = 1
  done = 0
  while done == 0:
    stdscr.addstr(1, 0, "Location: ANGLE +-?")
    stdscr.clrtoeol()
    stdscr.refresh()
    c = chr(stdscr.getch())
    if c == '+':
      signTheta = 1
      done = 1
    elif c == '-':
      signTheta = -1
      done = 1
  theta = 0
  done = 0
  while done == 0:
    stdscr.addstr(1, 0, "Location: theta? {}".format(theta))
    stdscr.clrtoeol()
    stdscr.refresh()
    c = chr(stdscr.getch())
    try:
      int(c)
      theta = signTheta * (abs(theta*10) + int(c))
    except:
      if(c == '\n' or c == '\r'):
        done = 1
  stdscr.addstr(1, 0, "LOCATION MESSAGE: {} {} {}".format(x_coord, y_coord, theta))
  outputMsgBody = ("{{\"RESPONSE\" : {{ \"SOURCE\" : \"SERVER\" , \"DEST\" : \"TargetLocator\" , \"ID\" : 0 \"TYPE\" : \"LOCATION\" , {{\"X_COORD\" : {0} , \"Y_COORD\" : {1} , \"THETA\" : {2}}}}}}}".format(x_coord, y_coord, theta))
  return outputMsgBody

def reqObjects(stdscr):
  outputMsgBody = ("{\"REQUEST\" : { \"SOURCE\" : \"SERVER\" , \"DEST\" : \"TargetLocator\" , \"ID\" : 0 \"TYPE\" : \"OBSTACLES\" }}")
  return outputMsgBody

def inputThread(stdscr, q):
  while 1:
    c = chr(stdscr.getch())
    stdscr.addstr(0, 0, "{}".format(c))
    stdscr.clrtoeol()
    stdscr.refresh()
    if(c == 'v'):
      outputMsgBody = newVertex(stdscr)
    elif(c == 'l'):
      outputMsgBody = newLocation(stdscr)
    elif(c == 'o'):
      outputMsgBody = reqObjects(stdscr)
    else:
      continue
    outputMsgHeader = b'L2\xa5\xbd\x01\x01\x01\x04'
    checkSum = 0
    for c in outputMsgBody:
      checkSum = checkSum + ord(c)
    #newMsgLength = bytearray(3)
    #newMsgLength.append(len(outputMsgBody))
    newMsgLength = (len(outputMsgBody) & 0xFFFF)
    msgLength = "{:02x} ".format(len(outputMsgBody))
    outputMsgHeader = outputMsgHeader + chr(newMsgLength & 0xFF) + chr((newMsgLength & 0xFF00) >> 8)
    #newMsgCheck = bytearray(3)
    #newMsgCheck.append(checkSum)
    newMsgCheck = (checkSum & 0xFFFF)
    msgCheck = "{:02x} ".format(checkSum)
    outputMsgHeader = outputMsgHeader + chr(newMsgCheck & 0xFF) + chr((newMsgCheck & 0xFF00) >> 8)
    outputMsg = outputMsgHeader + outputMsgBody
    stdscr.addstr(2, 0, "{}".format(outputMsgBody))
    stdscr.addstr(1, 0, "msgLENGTH: {} msgCheck: {}".format(newMsgLength, newMsgCheck))
    q.put(outputMsg)
   