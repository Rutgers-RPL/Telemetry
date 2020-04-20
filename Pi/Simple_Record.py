import os.path
import shlex
import subprocess
from subprocess import call
from subprocess import Popen
import picamera
import time
import datetime as dt
from time import sleep


camera = picamera.PiCamera()
camera.resolution = (1280,720)
camera.framerate = 30
camera.annotate_text = 'Test'

save_name = "/home/pi/Videos/"
filename = dt.datetime.now().strftime("%Y-%m-%d_%H.%M.%S.h264")
completed_video = os.path.join(save_name, filename)
# filename = '%s.%s' % (save_name, get_file_name())

camera.start_recording(completed_video, quality=30)
print('recording!')
sleep(10)
camera.stop_recording()

sleep(5)
print('converting!')
# command = (shlex.split("MP4box -add {f} {f}.mp4".format(f=filename)))
# sleep(5)
# f = filename
# convert = "MP4Box -add {f} {f}.mp4"
# try:
    # output = subprocess.check_output(command, stderr=subprocess.STDOUT, shell= True)
# subprocess.call = (['cd Videos', 'convert'])

command = os.popen('MP4Box -add filename.h264 filename.mp4', mode= 'w')
command.read()

