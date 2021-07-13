#!/usr/bin/env python3

"""
This is a little tool to capture videos automatically with the raspberry pi. 
Captured videos will be saved at path `/home/pi/Desktop/videos`. Maximum 20 videos, each of which is 30 seconds.

## To make this script auto run on raspberry pi reboot, need to do the following: 
- Type `crontab -e` in terminal.
- Insert `@reboot python3 /home/pi/projects/fish_vision/tools/video_capture.py` at the bottom. 

"""
import picamera
import os
import datetime as dt
from pathlib import Path

MAX_NUMBER_OF_FILES = 20

def create_dir_if_not_exist(dir_path):
    Path(dir_path).mkdir(parents=True, exist_ok=True)

def delete_old_files(dir_path):
    paths = sorted(Path(dir_path).iterdir(), key=os.path.getmtime, reverse=True)
    for file in paths[MAX_NUMBER_OF_FILES-1:]:
        print(f'removing file at {file}')
        os.remove(file)

def capture_videos(dir_path):
    create_dir_if_not_exist(dir_path)

    with picamera.PiCamera(resolution=(1280, 720), framerate=60) as camera:
        while True:
            delete_old_files(dir_path)

            filename = os.path.join(dir_path, dt.datetime.now().strftime('%Y-%m-%d_%H.%M.%S.h264'))
            camera.start_recording(filename)
            camera.wait_recording(30)
            camera.stop_recording()

if __name__ == '__main__':
    file_directory = '/home/pi/Desktop/videos'
    create_dir_if_not_exist(file_directory)
    capture_videos(file_directory)
