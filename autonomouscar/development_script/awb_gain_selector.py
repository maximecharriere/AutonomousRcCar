
import sys
import time
import picamera
import picamera.array
import pygame


def main(argv):
    camResolution = (640, 480)
    step = 1/100
    pygame.init()
    display = pygame.display.set_mode((camResolution[0],camResolution[1]))
    pygame.display.set_caption('AWB Selector')

    print("Commands:")
    print("  UP:    Add blue")
    print("  DOWN:  Remove blue")
    print("  RIGHT: Add red")
    print("  LEFT:  Remove red")

    with picamera.PiCamera(resolution=camResolution) as camera:
        with picamera.array.PiRGBArray(camera, size=camResolution) as rawCapture:
            camera.awb_mode = 'off'
            (rg, bg) = (1, 211/128)
            camera.awb_gains = (rg, bg)
            camera.contrast=50
            camera.saturation=100
            camera.sharpness=0
            # Let time to the camera for color and exposure calibration
            time.sleep(1)

            for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
                frameRGB = frame.array

                surface = pygame.pixelcopy.make_surface(frameRGB.transpose((1,0,2)))
                display.blit(surface, (0,0))
                pygame.display.update()
                
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_UP:
                            bg += step
                        elif event.key == pygame.K_DOWN:
                            bg -= step
                        elif event.key == pygame.K_RIGHT:
                            rg += step
                        elif event.key == pygame.K_LEFT:
                            rg -= step
                        camera.awb_gains = (rg, bg)
                        print(f"Red: {camera.awb_gains[0]}    Blue: {camera.awb_gains[1]}")

                # Reset analised frame
                rawCapture.truncate(0)


    

if __name__ == "__main__":
   main(sys.argv[1:])