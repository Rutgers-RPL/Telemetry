import serial
import socket
socket = socket.socket()          
print("Socket successfully created")
port = 12345
socket.bind(('0.0.0.0',12345))         
print ("socket binded to %s" %(port))
socket.listen(1)
# Wait for a connection
print('waiting for a connection')
connection, client_address = socket.accept()
print('connection from', client_address)
from time import sleep
import sys
import time
import subprocess
from math import *
serialPortWorks = True
users={'Pi':['/dev/ttyUSB0'],'aaron':['COM6','COM3','COM7','COM4','COM10']}
ser=None
f=None
f1=None



####Array Denotation(Real time)
###Gyroscope
##Gx=0
##Gy=1
##Gz=2
###accelerometer
##Ax=3
##Ay=4
##Az=5
###time
##t=6

##Array Denotion(Past Data[November])
#Gyroscope
Gx=4
Gy=5
Gz=6
#Accelerometer
Ax=7
Ay=8
Az=9

#Quaternion initialization
qa=1.0
qb=0.0
qc=0.0
qd=0.0
alpha=0.15
cmd=0.0
dt=0.00
data=[]
start=0
stop=0
duration=0


#camera control
camIsOn = False
def CheckCamera():
    global camIsOn
    if data[cam]==1 and not camIsOn:
        try:
            #run camera program
            p = subprocess.Popen(['python','/rasp_record.py'])
            camIsOn = True
        except:
            print("oof")
    elif data[cam]==0 and camIsOn:
        try:
            #end camera program
            p.terminate()
            camIsOn = False
        except:
            print("oof 2")

            
#File
file=open("Quat.txt","w")
def Start():
    global start
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
               ser=serial.Serial(port,9600)
               serialPortWorks=True
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
        try:
            line = ser.readline().decode('utf-8')[:-1]
        except:
            return
        try:
            strData = line.split()
        except:
            return
        if (len(strData) <4):
            return
        try:
            data = [float(i) for i in strData]
        except:
            return
    if serialPortWorks==False:
        line=f1.readline()
        if len(line)<1:
            sys.exit()
        strData=line.split()
        if len(strData)<4:
            return
        try:
            data = [float(i) for i in strData]
        except:
            return
    f = open('ard_log.txt', 'a+')
    f.write(line + '/n')
    f.close()
    #print(strData)

def SerWrite():
    if serialPortWorks==True:
        ser.write(cmd.encode())
    else:
        print(cmd)

def FileWrite():
    file.write(str(cmd) +"\n")
    print(str(cmd))

def Stop():
    global start
    global stop
    global dt
    stop=time.time()
    dt=stop-start
    

def QCF1():
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
        ax= data[Ax]
        ay= data[Ay]
        az= data[Az]
        #dt=(data[t])
        dt=0.25
        
        wa = 0.0
        wb = gx
        wc = gy
        wd = gz
      
        qdota = (wa*qa - wb*qb - wc*qc - wd*qd) * -0.5
        qdotb = (wa*qb + wb*qa + wc*qd - wd*qc) * -0.5
        qdotc = (wa*qc - wb*qd + wc*qa + wd*qb) * -0.5
        qdotd = (wa*qd + wb*qc - wc*qb + wd*qa) * -0.5
        
        qwa = qa + qdota * dt
        qwb = qb + qdotb * dt
        qwc = qc + qdotc * dt
        qwd = qd + qdotd * dt
        
        norm = sqrt(qwa*qwa + qwb*qwb + qwc*qwc + qwd*qwd)
        
        if norm==0:
            norm=0.01 
        qwa = qwa/norm
        qwb = qwb/norm
        qwc = qwc/norm
        qwd = qwd/norm

        norma= sqrt(ax*ax + ay*ay + az*az)
        if norma==0:
            norma=0.01 
        ax = ax/norma
        ay = ay/norma
        az = az/norma
      
        aa = 0
        ab = ax
        ac = ay
        ad = az
      
        normb = sqrt(ab*ab + ac*ac + ad*ad)
        if normb==0:
            normb=0.01 
        ab = ab/normb
        ac = ac/normb
        ad = ad/normb
        
        gb = (2*qwa*qwa - 1 + 2*qwb*qwb)*ab + (2*qwc*qwb + 2*qwa*qwd)*ac + (2*qwd*qwb - 2*qwa*qwc)*ad
        gc = (2*qwb*qwc - 2*qwa*qwd)*ab + (2*qwa*qwa - 1 + 2*qwc*qwc)*ac + (2*qwd*qwc + 2*qwa*qwb)*ad
        gd = (2*qwb*qwd + 2*qwa*qwc)*ab + (2*qwc*qwd - 2*qwa*qwb)*ac + (2*qwa*qwa - 1 + 2*qwd*qwd)*ad
        
        normc = sqrt(gb*gb + gc*gc + gd*gd)
        if normc==0:
          normc=0.01
        gb = gb/normc
        gc = gc/normc
        gd = gd/normc

        if gd >= 0:
            q_acca = sqrt((gd + 1)*0.5)
            q_accb = -gc/2*q_acca
            q_accc = gb/2*q_acca
            q_accd = 0.0

            normd = sqrt(q_acca*q_acca + q_accb*q_accb + q_accc*q_accc + q_accd*q_accd)
            if normd==0:
                normd=0.01
            q_acca = q_acca/normd
            q_accb = q_accb/normd
            q_accc = q_accc/normd
            q_accd = q_accd/normd
        else:
            q_acca = -gc/sqrt(2 - 2*gd)
            q_accb = sqrt((1 - gd)/2)
            q_accc = 0.0
            q_accd = gb/sqrt(2 - 2*gd)
        
            norme = sqrt(q_acca*q_acca + q_accb*q_accb + q_accc*q_accc + q_accd*q_accd)
            if norme==0:
                norme=0.01
            q_acca = q_acca/norme
            q_accb = q_accb/norme
            q_accc = q_accc/norme
            q_accd = q_accd/norme
      
        if q_acca > 0.9:
            q_acca = (1 - alpha) + alpha * q_acca
            q_accb = alpha * q_accb
            q_accc = alpha * q_accc
            q_accd = alpha * q_accd

            normf = sqrt(q_acca*q_acca + q_accb*q_accb + q_accc*q_accc + q_accd*q_accd)
            if normf==0:
                normf=0.01
            q_acca = q_acca/normf
            q_accb = q_accb/normf
            q_accc = q_accc/normf
            q_accd = q_accd/normf
        else:
            angle = acos(q_acca)
            q_acca = sin((1 - alpha) * angle)/sin(angle) + sin(alpha * angle) * q_acca/sin(angle)
            q_accb = sin(alpha * angle) * q_accb/sin(angle)
            q_accc = sin(alpha * angle) * q_accc/sin(angle)
            q_accd = sin(alpha * angle) * q_accd/sin(angle)
        
        qa1 = qwa * q_acca - qwb * q_accb - qwc * q_accc - qwd * q_accd
        qb1 = qwa * q_accb + qwb * q_acca + qwc * q_accd - qwd * q_accc
        qc1 = qwa * q_accc - qwb * q_accd + qwc * q_acca + qwd * q_accb
        qd1 = qwa * q_accd + qwb * q_accc - qwc * q_accb + qwd * q_acca
        
        normg=sqrt(qa1*qa1 + qb1*qb1 + qc1*qc1 + qd1*qd1)

        if normg==0:
            normg=0.01
        qa = qa1/normg
        qb = qb1/normg
        qc = qc1/normg
        qd = qd1/normg
        val0 = 2*acos(qa)
        if val0 <= 0.01: 
            val1 = qb
            val2 = qc
            val3 = qd
        else:
            val1 = qb/sin(val0/2)
            val2 = qc/sin(val0/2)
            val3 = qd/sin(val0/2)
        #if val0<0.001:
        #        val0=0.00
        #if val1<0.001:
         #       val1=0.00
        #if val2<0.001:
         #       val2=0.00
        #if val3<0.001:
       #         val3=0.00
        #cmd=(str(val0)+" "+str(val1)+" "+str(val2)+" "+str(val3))
        cmd=(str(round(val0,2))+" "+str(round(val1,2))+" "+str(round(val2,2))+" "+str(round(val3,2))+"\n")
        print(str(cmd))
def SocketSend():
    print('sending data to the client')
    a = str(cmd)
    if a != "0.0":
        a = a.encode("ascii")
        connection.sendall(a)
FindData()
while True:
    #Start()
    GetData()
    QCF1()
    #CheckCamera()
    #SerWrite()
    #FileWrite()
    SocketSend()
    #Stop()
