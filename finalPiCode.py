import pygame
import time
import serial
import signal
import sys
import glob

ser=serial.Serial('/dev/USB0',9600)#need to have something that checks the port, not major issue but small one
time.sleep(3)#3 seconds
ser.flushInput()
print("Serial OK")

pygame.init()
pygame.joystick.init()
ljoy=pygame.joystick.Joystick(0)
ljoy.init()
rjoy=pygame.joystick.Joystick(0)#one of these should be 1, will have to check left and right
rjoy.init()

#def interrupt_handler(signum,frame):
    #print("/n received")
    #sys.exit(0)

#signal.signal(signal.SIGINT,interrupt_handler)

def run():
    String mode
    while True:
        pygame.event.pump()
        
        xButton=joystick.get_button(0)
        yButton=joystick.get_button(1)
        aButton=joystick.get_button(2)
        bButton=joystick.get_button(3)
        ljoy=ljoy.get_axis(1)*100#mult by 100
        rjoy=rjoy.get_axis(1)*100#mult by 100
        #time.sleep(0.01)
        
        if(abs(ljoy))<1:#these are between -1 to 1, use these as percents of power
           ljoy=0;
        if(abs(rjoy))<1:
            rjoy=0;
        
        if xButton:
            user_a=("sxn")
            ser.write(user_a.encode())
        elif yButton:
            user_a=("syn")
            ser.write(user_a.encode())
        elif aButton:
            user_a=("san")
            ser.write(user_a.encode())
        elif bButton:
            user_a=("sbn")
            ser.write(user_a.encode())
        elif ((abs(ljoy))>1)and ((abs(rjoy)>1)):#forms of values need to be finalize but this is the general code
            user_a=f"mo{ljoy}t{rjoy}\n"#1 should be o and 2 should be t
            ser.write(user_a.encode())
        elif ((abs(ljoy))>1) and (abs(rjoy)<1):#1 not 0.1 for these
            user_a=f"mo{ljoy}t0000\n"#
            ser.write(user_a.encode())
        elif ((abs(ljoy))<1) and (abs(rjoy)>1):
            user_a=f"mo0000t{rjoy}\n"
                      

try:
    run()
    
except KeyboardInterrupt:
        print("Close Serial Communication")
        ser.close()

finally:
        print("closed serial")