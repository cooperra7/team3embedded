import thread
import socket
import threading
import codecs
import json
#from queue import Queue
import Queue
#import unicurses
import curses
import math

from target_locator_disp import *
from target_locator_input import *

global_data = ''
client1 = ''
client2 = ''
client3 = ''
client4 = ''

def clientReceive(s, size, q):
  global client1
  global client2
  global client3
  global client4
  while(1):
    client, address = s.accept()
    if (client1 == ''):
      client1 = client
    elif (client2 == ''):
      client2 = client
    elif (client3 == ''):
      client3 = client
    elif (client4 == ''):
      client4 = client
    while (client != ''):
      try:
        data = client.recv(size)
      except:
        if (client1 == client):
          client1 = ''
        elif (client2 == client):
          client2 = ''
        elif (client3 == client):
          client3 = ''
        elif (client4 == client):
          client4 = ''
        client = ''
      q.put(data)
    
def parseThread(q, parsedQ, stdscr):
  while 1:
    parsedQ.put(msgParse(q.get(), stdscr))
    
   

def main(stdscr):   
  host = ''
  port = 2000
  backlog = 5
  size = 1024  

  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  s.bind((host, port))
  s.listen(backlog)

  q = Queue.Queue()
  parserQ = Queue.Queue()
  parsedQ = Queue.Queue()

  stdscr.addstr(0, 0, "started")
  stdscr.refresh()
  
  target_window = curses.newwin(50, 50)

  clientThread1_handle = thread.start_new_thread( clientReceive, (s, size, q))
  clientThread2_handle = thread.start_new_thread( clientReceive, (s, size, q))
  clientThread3_handle = thread.start_new_thread( clientReceive, (s, size, q))
  clientThread4_handle = thread.start_new_thread( clientReceive, (s, size, q))
  parserThread_handle = thread.start_new_thread( parseThread, (parserQ, parsedQ, stdscr))
  inputThread_handle = thread.start_new_thread( inputThread, (target_window, q))

  while 1:
    global_data = q.get()
    parserQ.put(global_data)
    if (client1 != ''):
      client1.send(global_data)
    if (client2 != ''):
      client2.send(global_data)
    if (client3 != ''):
      client3.send(global_data)
    if (client4 != ''):
      client4.send(global_data)
      
      
curses.wrapper(main)
if (client1 != ''):
  client1.close
if (client2 != ''):
  client2.close
if (client3 != ''):
  client3.close
if (client4 != ''):
  client4.close
clientThread1_handle.exit()
clientThread2_handle.exit()
clientThread3_handle.exit()
clientThread4_handle.exit()
parserThread_handle.exit()
inputThread_handle.exit()
s.release()
s.close()