import picamera
import os
import datetime as dt
from pathlib import Path

def create_dir_if_not_exist(dir_path):
    Path(dir_path).mkdir(parents=True, exist_ok=True)

def delete_old_files(dir_path, max_num_files=5):
    paths = sorted(Path(dir_path).iterdir(), key=os.path.getmtime, reverse=True)
    for file in paths[max_num_files-1:]:
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
    capture_videos(file_directory, max_num_files=5)
