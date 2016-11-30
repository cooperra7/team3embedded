def msgParse(data, stdscr):
  msgParse.msgLoc['ADC_A']
  msgParse.msgCount = msgParse.msgCount + 1
  stdscr.addstr(msgParse.msgLoc['PIXY_COUNT'], 0, "Count: {}".format(msgParse.counter))
  stdscr.clrtoeol()
  parsed = data.strip(b'\xaa')
  parsed_array = parsed.split(b'L2\xa5\xbd')
  for i in parsed_array:
    i = i.strip(b'\xaa')
    split = i.partition(b'{')
    seq = (split[1], split[2])
    jsonVal = b''
    jsonVal = jsonVal.join(seq)
    try:
      string_Json = jsonVal.decode()
      json_data = json.loads(string_Json)
      for j in json_data:
        #print(j)
        if(j == "DEBUG"):
          try:
            if(msgParse.counter == 60):
              msgParse.counter = 0
            DetectionCode = json_data["ID"]
            stdscr.addstr(msgParse.msgLoc['PIXY_UNMOD'], 0, "UNModified ID: {}       ".format(json_data["ID"]))
            msgParse.pixyCount = msgParse.pixyCount + 1
            stdscr.clrtoeol()
            #stdscr.addstr(msgParse.msgLoc['PIXY_COORD']+msgParse.counter, 0, "ID: {:4d} xCoord: {:3d} yCoord: {:3d} Width: {:3d} Height: {:3d}".format(json_data["ID"], json_data["xCoord"], json_data["yCoord"], json_data["Width"], json_data["Height"]))
            #stdscr.addstr(msgParse.msgLoc['PIXY_PROCC']+msgParse.counter, 0, "ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["XCoord"], json_data["YCoord"], json_data["ANGLE"]))
            stdscr.clrtoeol()
            ID1 = 0
            ID2 = 0
            ID3 = 0
            ID4 = 0
            if (DetectionCode == 0):
              for i in range (0, msgParse.counter):
                # stdscr.setsyx(i+10, 0)
                # stdscr.clrtoeol()
                # msgParse.counter = 0;
                pass
            else:
              try:
                stdscr.addstr(msgParse.msgLoc['PIXY_ID'], 0, "ID1: {:1d} ID2: {:1d} ID3: {:1d} ID4: {:1d}".format(ID1, ID2, ID3, ID4))
                stdscr.clrtoeol()
                ID1 = DetectionCode % 8;
                stdscr.addstr(msgParse.msgLoc['PIXY_ID'], 0, "ID1: {:1d} ID2: {:1d} ID3: {:1d} ID4: {:1d}".format(ID1, ID2, ID3, ID4))
                stdscr.clrtoeol()
                Detection234 = ((DetectionCode - ID1)/8)
                ID2 = Detection234%8
                stdscr.addstr(msgParse.msgLoc['PIXY_ID'], 0, "ID1: {:1d} ID2: {:1d} ID3: {:1d} ID4: {:1d}".format(ID1, ID2, ID3, ID4))
                stdscr.clrtoeol()
                Detection34 = ((Detection234 - ID2)/8)
                ID3 = Detection34%8
                stdscr.addstr(msgParse.msgLoc['PIXY_ID'], 0, "ID1: {:1d} ID2: {:1d} ID3: {:1d} ID4: {:1d}".format(ID1, ID2, ID3, ID4))
                stdscr.clrtoeol()
                Detection4 = ((Detection34 - ID3)/8)
                ID4 = Detecion4%8
                stdscr.addstr(msgParse.msgLoc['PIXY_ID'], 0, "ID1: {:1d} ID2: {:1d} ID3: {:1d} ID4: {:1d}".format(ID1, ID2, ID3, ID4))
                stdscr.clrtoeol()
              except:
                pass
              try:
                stdscr.addstr(msgParse.msgLoc['PIXY_COUNT'], 0, "Count: {}".format(msgParse.counter))
                stdscr.clrtoeol()
              except:
                pass
              try:
                readableID = ID1*1000 + ID2*100 + ID3*10 + ID4
              except:
                readableID = DetectionCode
              #stdscr.addstr(msgParse.msgLoc['PIXY_COORD']+msgParse.counter, 0, "ID: {:4d} xCoord: {:3d} yCoord: {:3d} Width: {:3d} Height: {:3d}".format(readableID, json_data["xCoord"], json_data["yCoord"], json_data["Width"], json_data["Height"]))
              #stdscr.addstr(msgParse.msgLoc['PIXY_PROCC']+msgParse.counter, 0, "ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
              stdscr.clrtoeol()
              msgParse.counter = msgParse.counter + 1
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX1'], 0, "~~~~~~~")
            if(DetectionCode == 2):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX1'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 3):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 4):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX3'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 5):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX4'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 6):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX5'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 7):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX6'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 8):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX7'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 9):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX8'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 10):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX9'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 11):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX10'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 12):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX11'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 13):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX12'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 14):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX13'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 15):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX14'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 16):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX15'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 17):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX16'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 18):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX17'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 19):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX18'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 20):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX19'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
            if(DetectionCode == 21):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX20'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            DetectionCode = json_data["ID"]
            if(DetectionCode == 0):
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX1'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX3'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX4'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX5'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX6'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX7'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX8'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX9'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX10'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX11'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX12'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX13'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX14'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX15'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX16'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX17'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX18'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX19'], 0, "~~~~~~~")
              stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX20'], 0, "~~~~~~~")
              
            #if(DetectionCode == 3):
              #stdscr.addstr(msgParse.msgLoc['PIXY_VERTEX2'], 0, "VALID  ID: {:4d} AngleTo: {:4d} xLoc: {:4d} yLoc: {:4d}".format(json_data["ID"], json_data["ANGLE"], json_data["XCoord"], json_data["YCoord"]))
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['ADC_A'], 0, "ADC_DATA__A: {:4d} in Inches".format(json_data["ADC_DATA__A"]))
            stdscr.clrtoeol()
            msgParse.ADCCount = msgParse.ADCCount + 1
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['ADC_B'], 0, "ADC_DATA__B: {:4d} in Inches".format(json_data["ADC_DATA__B"]))
            stdscr.clrtoeol()
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['ADC_C'], 0, "ADC_DATA__C: {:4d} in Inches".format(json_data["ADC_DATA__C"]))
            stdscr.clrtoeol()
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['ADC_D'], 0, "ADC_DATA__D: {:4d} in Inches".format(json_data["ADC_DATA__D"]))
            stdscr.clrtoeol()
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['ADC_E'], 0, "ADC_DATA__E: {:4d} in Inches".format(json_data["ADC_DATA__E"]))
            stdscr.clrtoeol()
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['ADC_F'], 0, "ADC_DATA__F: {:4d} in Inches".format(json_data["ADC_DATA__F"]))
            stdscr.clrtoeol()
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['PING_A'], 0, "PING_DATA__A: {:4d} in Inches".format(json_data["LEFT"]))
            stdscr.clrtoeol()
            msgParse.pingCount = msgParse.pingCount + 1
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['PING_B'], 0, "PING_DATA__B: {:4d} in Inches".format(json_data["RIGHT"]))
            stdscr.clrtoeol()
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['PING_C'], 0, "PING_DATA__C: {:4d} in Inches".format(json_data["CENTER"]))
            stdscr.clrtoeol()
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['PING_D'], 0, "PING_DATA__D: {:4d} in Inches".format(json_data["BACK"]))
            stdscr.clrtoeol()
          except:
            pass
          try:
            stdscr.addstr(msgParse.msgLoc['ROVER_LOC'], 0, "ROVER_LOCATION X:{:4d} Y:{:4d} theta:{:4d} theta_1:{:4f} theta_2:{:4f} tan_1:{:4f} tan_2:{:4f} x_1:{:4d} rad_1:{:4f} x_2:{:4d} rad_2:{:4f}".format(json_data["X_LOCATION"], json_data["Y_LOCATION"], json_data["ANGLE"], json_data["THETA_1"], json_data["THETA_2"], json_data["TAN_1"], json_data["TAN_2"], json_data["X_1"], json_data["THETA_1_RAD"], json_data["X_2"], json_data["THETA_2_RAD"]))
            stdscr.clrtoeol()
            calc_theta_1 = json_data["THETA_1"]
            calc_theta_1_rad = math.radians(calc_theta_1)
            calc_tan_1 = math.tan(calc_theta_1_rad)
            calc_theta_2 = json_data["THETA_2"]
            calc_theta_2_rad = math.radians(calc_theta_2)
            calc_tan_2 = math.tan(calc_theta_2_rad)
            calculated = ((json_data["Y_1"] - json_data["Y_2"]) - (json_data["X_1"]*json_data["TAN_1"] - json_data["X_2"]*json_data["TAN_2"]))/(json_data["TAN_2"] - json_data["TAN_1"])
            calculated2 = ((json_data["Y_1"] - json_data["Y_2"]) - (json_data["X_1"]*calc_tan_1 - json_data["X_2"]*calc_tan_2))/(calc_tan_2 - calc_tan_1)
            stdscr.addstr(msgParse.msgLoc['ROVER_LOC']+2, 0, "CALCULATED VALUE: {} calc_theta_1_rad:{} calc_tan_1:{} calc_theta_2_rad:{} calc_tan_2:{} calculated_2:{}".format(calculated, calc_theta_1_rad, calc_tan_1, calc_theta_2_rad, calc_tan_2, calculated2))
            stdscr.clrtoeol()
            msgParse.locationCount = msgParse.locationCount + 1
          except:
            pass
          try:
            # stdscr.addstr(msgParse.msgLoc['VERTEX_REC']+1, 0, string_Json)
            # stdscr.clrtoeol()
            stdscr.addstr(msgParse.msgLoc['VERTEX_REC'], 0, "FOUND VERTEX {}".format(json_data["VERTEX"]))
            stdscr.clrtoeol()
          except:
            pass
          stdscr.refresh()
        elif(j == "RESPONSE"):
          stdscr.addstr(msgParse.msgLoc['OBJECT'], 0, "Got a Response: ")
          try:
            temp_obj = json_data["RESPONSE"]
            stdscr.clrtoeol()
            stdscr.addstr(msgParse.msgLoc['OBJECT'], 0, "Got a Response: {}".format(temp_obj["DATA"]))
            stdscr.clrtoeol()
            # stdscr.addstr(msgParse.msgLoc['OBJECT'], 0, "PING_DATA__D: {:4d} in Inches".format(json_data["RESPONSE"]))
            stdscr.clrtoeol()
          except:
            pass
      
    except:
      pass
    
  stdscr.addstr(0, 10, "Message Count: {} ADC Count: {} Pixy Count: {} PING Count: {} Location Count: {}".format(msgParse.msgCount, msgParse.ADCCount, msgParse.pixyCount, msgParse.pingCount, msgParse.locationCount))
  return parsed
msgParse.counter = 0
msgParse.msgCount = 0
msgParse.ADCCount = 0
msgParse.pixyCount = 0
msgParse.pingCount = 0
msgParse.locationCount = 0
msgParse.msgLoc = {'ROVER_LOC' : 2,
                  'ADC_A' : 3, 
                  'ADC_B' : 4, 
                  'ADC_C' : 5, 
                  'ADC_D' : 6, 
                  'ADC_E' : 7, 
                  'ADC_F' : 8, 
                  'PING_A' : 9, 
                  'PING_B' : 10, 
                  'PING_C' : 11, 
                  'PING_D' : 12, 
                  'PIXY_ID' : 13, 
                  'PIXY_COUNT' : 14, 
                  'PIXY_UNMOD' : 15, 
                  'PIXY_COORD' : 16, 
                  'PIXY_PROCC' : 16,
                  'PIXY_TARGET' : 17,
                  'VERTEX_REC' : 18,
                  'PIXY_VERTEX1' : 19,
                  'PIXY_VERTEX2' : 20,
                  'PIXY_VERTEX3' : 21,
                  'PIXY_VERTEX4' : 22,
                  'PIXY_VERTEX5' : 23,
                  'PIXY_VERTEX6' : 24,
                  'PIXY_VERTEX7' : 25,
                  'PIXY_VERTEX8' : 26,
                  'PIXY_VERTEX9' : 27,
                  'PIXY_VERTEX10' : 28,
                  'PIXY_VERTEX11' : 29,
                  'PIXY_VERTEX12' : 30,
                  'PIXY_VERTEX13' : 31,
                  'PIXY_VERTEX14' : 32,
                  'PIXY_VERTEX15' : 33,
                  'PIXY_VERTEX16' : 34,
                  'PIXY_VERTEX17' : 35,
                  'PIXY_VERTEX18' : 36,
                  'PIXY_VERTEX19' : 37,
                  'PIXY_VERTEX20' : 38,
                  'OBJECT' : 39
                  }