import time
import picamera

def main():    
    # while True:
    #     time.sleep(6)
    print('Hello fish!')
    with picamera.PiCamera() as camera:
        camera.resolution = (1280, 720)
        camera.framerate = 60
        camera.start_preview()
        camera.start_recording('highres.h264')
       # camera.start_recording('lowres.h264', splitter_port=2, resize=(320, 240))
        camera.wait_recording(30)
        # camera.stop_recording(splitter_port=2)
        camera.stop_recording()
        camera.stop_preview()

if __name__ == '__main__':
    main()
