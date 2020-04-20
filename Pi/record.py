import picamera
import datetime as dt
from time import sleep
import os

def print_c(msg):
    file = open('cam_log', "a+")
    file.write(msg + "\n")
    file.close()

# length of video in secs
# vid_length = int(input('>> how long do you want the video to be? (use integers) $ '))
vid_length = 60

# resolution
height = 1280
width = 720

# framerate
fps = 30

# camera initialization
camera = picamera.PiCamera()
camera.resolution = (height, width)
camera.framerate = fps
camera.brightness = 55

# file location
save_name = "/"

while True:
    filename = dt.datetime.now().strftime("%Y-%m-%d_%H.%M.%S")
    completed_video_name = os.path.join(save_name, filename + ".h264")

    # start recoding
    print_c('>> camera is now recording!')
    camera.start_recording(completed_video_name, quality=15)
    sleep(vid_length)
    

    # stop recording
    camera.stop_recording()

    sleep(0.5)
    
    # wraps h264 file in mp4
    print_c('>> converting from h264 to mp4!')
    # exec_cmd = 'MP4Box -add $s.h264 $s.mp4'.replace('$s', filename)
    exec_cmd = 'MP4Box -add ' + filename + '.h264 ' + filename + '.mp4' 
    command = os.popen(exec_cmd)	
    sleep(0.5)

    # removes h264'
    exec_rm = 'sudo rm $s.h264'.replace('$s', filename)
    command1 = os.popen(exec_rm)
    print_c('..resetting!')
