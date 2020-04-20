import serial
import socket
socket = socket.socket()          
print "Socket successfully created"
port = 12345
socket.bind(('localhost',12345))         
print "socket binded to %s" %(port) 
from time import sleep
import sys
import time
from math import *
serialPortWorks = True
users={'Pi':['/dev/ttyUSB0'],'aaron':['COM6','COM3','COM7','COM4','COM10']}
ser=None
f=None
f1=None
start=0
stop=0
duration=0
data=[]
#Array Denotation
alt=0
#barom=1
#GPS
#GPS_LA=2
#GPS_LO=3
#gyroscope
Gx=1
Gy=2
Gz=3
#accelerometer
Ax=4
Ay=5
Az=6
#magnemometer
#Mx=10
#My=11
#Mz=12
#time
t=7
#Quaterion Preset
qa=1
qb=0
qc=0
qd=0
alpha=0
cmd=0
#File
file=open("Baruh.txt","w")
def Start():
    global start
    sleep(0.02)
    start=time.time()
def FindData():
    global ser
    global f
    global f1
    global serialPortWorks
    global data
    try:
        user=sys.argv[1]
    except:
        print ('No user given, cannot check serial ports. '+
               'Function call should be \"py interpretData.py {user} {file (optional)}\"')
        sys.exit()


    if user in users.keys() and len(sys.argv)<3:
       for port in users.get(user):
            try:
               ser=serial.Serial(port,115200)
               return
            except:
                pass
        
    if ser==None:
        serialPortWorks=False
        try:
            fName=sys.argv[2]
        except:
            print ('No serial port or file detected')
            sys.exit()
        f1=open(fName, "r")

def GetData():
    global ser
    global data
    if serialPortWorks==True:
        while (ser.in_waiting <1):
            pass
        line = ser.readline().decode('utf-8')[:-1]
        strData = line.split()
        if (len(strData) <10):
            return
    if serialPortWorks==False:
        line=f1.readline()
        if len(line)<1:
            sys.exit()
        strData=line.split()
        if len(strData)<3:
            return
        try:
            data = [float(i) for i in strData]
        except:
            return
    f = open('ard_log.txt', 'a+')
    f.write(line + '/n')
    f.close()
    #print(strData)
def QCF():
    global qa
    global qb
    global qc
    global qd
    global cmd
    if len(data)<6:
        return
    else:
        gx=((data[Gx]*3.14)/180)
        gy=((data[Gy]*3.14)/180)
        gz=((data[Gz]*3.14)/180)

        wa=0.0
        wb=gx
        wc=gy
        wd=gz
        
        qdota = (wa*qa - wb*qb - wc*qc - wd*qd) * -0.5
        qdotb = (wa*qb + wb*qa + wc*qd - wd*qc) * -0.5
        qdotc = (wa*qc - wb*qd + wc*qa + wd*qb) * -0.5
        qdotd = (wa*qd + wb*qc - wc*qb + wd*qa) * -0.5
        
        qwa = qa + qdota * duration
        qwb = qb + qdotb * duration
        qwc = qc + qdotc * duration
        qwd = qd + qdotd * duration
        
        norm = sqrt(qwa*qwa + qwb*qwb + qwc*qwc + qwd*qwd)
        
        qa = qwa/norm
        qb = qwb/norm
        qc = qwc/norm
        qd = qwd/norm

        '''aa=0
        ab=data[Ax]
        ac=data[Ay]
        ad=data[Az]
        
        norma = sqrt(ab*ab + ac*ac + ad*ad)
        
        ab = ab/norma
        ac = ac/norma
        ad = ad/norma
        
        gb = (2*qwa*qwa - 1 + 2*qwb*qwb)*ab + (2*qwc*qwb + 2*qwa*qwd)*ac + (2*qwd*qwb - 2*qwa*qwc)*ad
        gc = (2*qwb*qwc - 2*qwa*qwd)*ab + (2*qwa*qwa - 1 + 2*qwc*qwc)*ac + (2*qwd*qwc + 2*qwa*qwb)*ad
        gd = (2*qwb*qwd + 2*qwa*qwc)*ab + (2*qwc*qwd - 2*qwa*qwb)*ac + (2*qwa*qwa - 1 + 2*qwd*qwd)*ad
        
        normb = sqrt(gb*gb + gc*gc + gd*gd)
        gb = gb/normb
        gc = gc/normb
        gd = gd/normb
        
        if (gd >= 0):
            q_acca = sqrt((gd + 1)*0.5)
            q_accb = -gc/2*q_acca
            q_accc = gb/2*q_acca
            q_accd = 0.0
            
            normc = sqrt(q_acca*q_acca + q_accb*q_accb + q_accc*q_accc + q_accd*q_accd)
            q_acca = q_acca/normc
            q_accb = q_accb/normc
            q_accc = q_accc/normc
            q_accd = q_accd/norm
        else:
            q_acca = -gc/sqrt(2 - 2*gd)
            q_accb = sqrt((1 - gd)/2)
            q_accc = 0.0
            q_accd = gb/sqrt(2 - 2*gd)
            
            normc = sqrt(q_acca*q_acca + q_accb*q_accb + q_accc*q_accc + q_accd*q_accd)
            q_acca = q_acca/normc
            q_accb = q_accb/normc
            q_accc = q_accc/normc
            q_accd = q_accd/normc
        if(q_acca > 0.9):
            q_acca = (1 - alpha) + alpha * q_acca 
            q_accb = alpha * q_accb
            q_accc = alpha * q_accc
            q_accd = alpha * q_accd
            
            normc = sqrt(q_acca*q_acca + q_accb*q_accb + q_accc*q_accc + q_accd*q_accd)
            q_acca = q_acca/normc
            q_accb = q_accb/normc
            q_accc = q_accc/normc
            q_accd = q_accd/normc
        else:
            angle = acos(q_acca)
            q_acca = sin((1 - alpha) * angle)/sin(angle) + sin(alpha * angle) * q_acca/sin(angle)
            q_accb = sin(alpha * angle) * q_accb/sin(angle)
            q_accc = sin(alpha * angle) * q_accc/sin(angle)
            q_accd = sin(alpha * angle) * q_accd/sin(angle)
            
        qa = qwa * q_acca - qwb * q_accb - qwc * q_accc - qwd * q_accd
        qb = qwa * q_accb + qwb * q_acca + qwc * q_accd - qwd * q_accc
        qc = qwa * q_accc - qwb * q_accd + qwc * q_acca + qwd * q_accb
        qd = qwa * q_accd + qwb * q_accc - qwc * q_accb + qwd * q_acca
        
        normd = sqrt(qa*qa + qb*qb + qc*qc + qd*qd)
        qa = qa/normd
        qb = qb/normd
        qc = qc/normd
        qd = qd/normd
        '''
        val0=2*acos(qa)
        if(val0<=0.01):
            val1 = qb
            val2 = qc
            val3 = qd
            cmd=(str(val0)+" "+str(val1)+" "+str(val2)+" "+str(val3))
        else:
            val1 = qb/sin(val0/2)
            val2 = qc/sin(val0/2)
            val3 = qd/sin(val0/2)
            cmd=(str(val0)+" "+str(val1)+" "+str(val2)+" "+str(val3))
def SerWrite():
    if serialPortWorks==True:
        ser.write(cmd.encode())
    else:
        print(cmd)
def SocketSend():
    while True:
        # Wait for a connection
        print('waiting for a connection')
        connection, client_address = sock.accept()
        try:
            print('connection from', client_address)
            print('sending data to the client')
            connection.sendall(str(cmd))            
        finally:
            # Clean up the connection
            connection.close()
            break
def FileWrite():
    file.write(str(cmd) +"\n")
    print(str(cmd))
def Stop():
    global start
    global stop
    global duration
    stop=time.time()
    duration=stop-start
    print(duration)
FindData()
while True:
    Start()
    GetData()
    QCF()
    #SerWrite()
    #FileWrite()
    SocketSend()
    Stop()
