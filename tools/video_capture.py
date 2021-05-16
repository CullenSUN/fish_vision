import picamera
import os
import datetime as dt
from pathlib import Path

file_directory = '/home/pi/Desktop/videos'
max_num_files = 5

def delete_old_files():
    paths = sorted(Path(file_directory).iterdir(), key=os.path.getmtime, reverse=True)
    for file in paths[max_num_files-1:]:
        print(f'removing file at {file}')
        os.remove(file)

def main():    
    with picamera.PiCamera(resolution=(1280, 720), framerate=60) as camera:
        while True:
            delete_old_files()

            filename = os.path.join(file_directory, dt.datetime.now().strftime('%Y-%m-%d_%H.%M.%S.h264'))
            camera.start_recording(filename)
            camera.wait_recording(30)
            camera.stop_recording()


if __name__ == '__main__':
    main()
