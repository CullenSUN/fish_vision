import picamera

def main():    
    with picamera.PiCamera(resolution = (1280, 720), framerate=60) as camera:
        camera.start_recording("test.h264")
        camera.wait_recording(30)
        camera.stop_recording()


if __name__ == '__main__':
    main()
