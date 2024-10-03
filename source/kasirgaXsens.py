#!/usr/bin/python
"""
KASIRGA AMFIBIC UNMANNED AIR VEHICLE 
Project for Design with XSENS Competition - IMCA Electronics
Gazi University - Cem Avci & Berat Semercioglu
Author: Cem Avci, cemavci97@hotmail.com
Date: 20/11/2020
Ankara, Turkey

"""


import os
os.environ['SDL_AUDIODRIVER'] = 'dsp'
import pygame, sys
from pygame.locals import *
import numpy
import math
import serial 
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.backends.backend_agg as agg
#import matplotlib.gridspec as gridspec
from matplotlib import *
import matplotlib
matplotlib.use("Agg")
import time
#import os
#os.environ['SDL_AUDIODRIVER'] = 'dsp'


#import time
programIcon = pygame.image.load('ksrg.png')

pygame.display.set_icon(programIcon)

pygame.init()
pygame.display.set_caption('KASIRGA GROUND CONTROL UNIT')
class Dial:
   """
   Generic dial type.
   """
   def __init__(self, image, frameImage, x=0, y=0, w=0, h=0):
       """
       x,y = coordinates of top left of dial.
       w,h = Width and Height of dial.
       """
       self.x = x 
       self.y = y
       self.image = image
       self.frameImage = frameImage
       self.dial = pygame.Surface(self.frameImage.get_rect()[2:4])
       self.dial.fill(0xFFFF00)
       if(w==0):
          w = self.frameImage.get_rect()[2]
       if(h==0):
          h = self.frameImage.get_rect()[3]
       self.w = w
       self.h = h
       self.pos = self.dial.get_rect()
       self.pos = self.pos.move(x, y)

   def position(self, x, y):
       """
       Reposition top,left of dial at x,y.
       """
       self.x = x 
       self.y = y
       self.pos[0] = x 
       self.pos[1] = y 

   def position_center(self, x, y):
       """
       Reposition centre of dial at x,y.
       """
       self.x = x
       self.y = y
       self.pos[0] = x - self.pos[2]/2
       self.pos[1] = y - self.pos[3]/2

   def rotate(self, image, angle):
       """
       Rotate supplied image by "angle" degrees.
       This rotates round the centre of the image. 
       If you need to offset the centre, resize the image using self.clip.
       This is used to rotate dial needles and probably doesn't need to be used externally.
       """
       tmpImage = pygame.transform.rotate(image ,angle)
       imageCentreX = tmpImage.get_rect()[0] + tmpImage.get_rect()[2]/2
       imageCentreY = tmpImage.get_rect()[1] + tmpImage.get_rect()[3]/2

       targetWidth = tmpImage.get_rect()[2]
       targetHeight = tmpImage.get_rect()[3]

       imageOut = pygame.Surface((targetWidth, targetHeight))
       imageOut.fill(0xFFFF00)
       imageOut.set_colorkey(0xFFFF00)
       imageOut.blit(tmpImage,(0,0), pygame.Rect( imageCentreX-targetWidth/2,imageCentreY-targetHeight/2, targetWidth, targetHeight ) )
       return imageOut

   def clip(self, image, x=0, y=0, w=0, h=0, oX=0, oY=0):
       """
       Cuts out a part of the needle image at x,y position to the correct size (w,h).
       This is put on to "imageOut" at an offset of oX,oY if required.
       This is used to centre dial needles and probably doesn't need to be used externally.       
       """
       if(w==0):
           w = image.get_rect()[2]
       if(h==0):
           h = image.get_rect()[3]
       needleW = w + 2*math.sqrt(oX*oX)
       needleH = h + 2*math.sqrt(oY*oY)
       imageOut = pygame.Surface((needleW, needleH))
       imageOut.fill(0xFFFF00)
       imageOut.set_colorkey(0xFFFF00)
       imageOut.blit(image, (needleW/2-w/2+oX, needleH/2-h/2+oY), pygame.Rect(x,y,w,h))
       return imageOut

   def overlay(self, image, x, y, r=0):
       """
       Overlays one image on top of another using 0xFFFF00 (Yellow) as the overlay colour.
       """
       x -= (image.get_rect()[2] - self.dial.get_rect()[2])/2
       y -= (image.get_rect()[3] - self.dial.get_rect()[3])/2
       image.set_colorkey(0xFFFF00)
       self.dial.blit(image, (x,y))




class Horizon(Dial):
   """
   Artificial horizon dial.
   """
   def __init__(self, x=0, y=0, w=0, h=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.
       """
       self.image = pygame.image.load('img/Horizon_GroundSky.png').convert_alpha()
       self.frameImage = pygame.image.load('img/Horizon_Background2.png').convert_alpha()
       self.maquetteImage = pygame.image.load('img/Maquette_Avion.png').convert_alpha()
       Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
   def update(self, screen, angleX, angleY):
       """
       Called to update an Artificial horizon dial.
       "angleX" and "angleY" are the inputs.
       "screen" is the surface to draw the dial on.
       """
       angleX %= 360
       angleY %= 360
       if (angleX > 180):
           angleX -= 360 
       if (angleY > 90)and(angleY < 270):
           angleY = 180 - angleY 
       elif (angleY > 270):
           angleY -= 360
       tmpImage = self.clip(self.image, 0, (59-angleY)*720/180, 250, 250)
       tmpImage = self.rotate(tmpImage, angleX)
       self.overlay(tmpImage, 0, 0)
       self.overlay(self.frameImage, 0,0)
       self.overlay(self.maquetteImage, 0,0)
       self.dial.set_colorkey(0xFFFF00)
       screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )

class TurnCoord(Dial):
   """
   Turn Coordinator dial.
   """
   def __init__(self, x=0, y=0, w=0, h=0):
       """
       Initialise dial at x,y.
       Default size of 300px can be overidden using w,h.
       """
       self.image = pygame.image.load('img/HeadingIndicator_Aircraft.png').convert()
       self.frameImage = pygame.image.load('img/HeadingIndicator_Background2.png').convert()
       self.marks = pygame.image.load('img/HeadingWeel.png').convert()
       #self.ball = pygame.image.load('resources/TurnCoordinatorBall.png').convert()
       Dial.__init__(self, self.image, self.frameImage, x, y, w, h)
   def update(self, screen, angleX, angleY):
       """
       Called to update a Turn Coordinator dial.
       "angleX" and "angleY" are the inputs.
       "screen" is the surface to draw the dial on.       
       """
       angleX %= 360 
       angleY %= 360
       if (angleX > 180):
           angleX -= 360 
       if (angleY > 180):
           angleY -= 360
       if(angleY > 14): 
           angleY = 14
       if(angleY < -14): 
           angleY = -14
       tmpImage = self.clip(self.image, 0, 0, 0, 0, 0, -12)
       tmpImage = self.rotate(tmpImage, angleX)
       self.overlay(self.frameImage, 0,0)
       self.overlay(tmpImage, 0, 0)
       tmpImage = self.clip(self.marks, 0, 0, 0, 0, 0, -80)
       self.overlay(tmpImage, 0, 80)
       #tmpImage = self.clip(self.ball, 0, 0, 0, 0, 0, 300)
       #tmpImage = self.rotate(tmpImage, angleY)
       #self.overlay(tmpImage, 0, -220)
       self.dial.set_colorkey(0xFFFF00)
       screen.blit( pygame.transform.scale(self.dial,(self.w,self.h)), self.pos )


fig = figure.Figure(figsize=(15,8))
fig.set_facecolor((0.05,0,0))



ax = fig.add_subplot(231)
ax.set_facecolor('silver')

ax2=fig.add_subplot(232)
ax2.set_facecolor('silver')

ax3=fig.add_subplot(234)
ax3.set_facecolor('silver')

ax4 = fig.add_subplot(235)
ax4.set_facecolor('silver')

#ax5=fig.add_subplot(235)
#ax5.set_facecolor('silver')

#ax6=fig.add_subplot(236)
#ax6.set_facecolor('silver')

canvas = agg.FigureCanvasAgg(fig)
#canvas.set_facecolor('none')



altitude=10

# Initialise screen.
screen = pygame.display.set_mode((1366, 768))
screen.fill((255,255,255,100))
clock=pygame.time.Clock()

# Initialise Dials.
horizon = Horizon(1050,10,200,200)

turn = TurnCoord(1050,220,200,200)
#throttle = Generic(470,255,75,75)
#RXbattery = Battery(470,180,75,75)
#TXbattery = Battery(545,180,75,75)
#rfSignal = RfSignal(470,330,150,150)
img = pygame.image.load('KasirgaOut.png')
screen.blit(img,(610,0))

#pygame.display.update()

a=0


accX= []
accY=[]
accZ=[]
gyroX=[]
gyroY=[]
gyroZ=[]


altitude=[]
throttle=[]
port=input("Please enter the Serial port")
try:
    arduinoData = serial.Serial(port, 115200) #Creating our serial object named arduinoData
    arduinoData.timeout = 0.01
except:
    arduinoData=0

plt.ion() #Tell matplotlib you want interactive mode to plot live data
cnt=0
NolduLanPasinyan=0

acc_X=0
acc_Y=0
acc_Z=0
gyro_X=0
gyro_Z=0
gyro_Y=0

def plot(hist): #Create a function that makes our desired plot

   
   #ax.plot(data)
   ax.cla()
   ax.set_title('ACCELERATION (m/s^2)',color='silver')
   ax.tick_params(axis='x', colors='silver')
   ax.tick_params(axis='y', colors='silver')
   ax.plot(accX,label='X')
   ax.plot(accY,label='Y')
   ax.plot(accZ,label='Z')
   ax.legend(loc='upper left')
   

   ax2.cla()
   ax2.set_title('GYRO (deg/s)',color='silver')
   ax2.tick_params(axis='x', colors='silver')
   ax2.tick_params(axis='y', colors='silver')
   ax2.plot(gyroX,label='X')
   ax2.plot(gyroY,label='Y')
   ax2.plot(gyroZ,label='Z')
   ax2.legend(loc='upper left')
   

   ax3.cla()
   ax3.set_title('THROTTLE (%)',color='silver')
   ax3.tick_params(axis='x', colors='silver')
   ax3.tick_params(axis='y', colors='silver')
   ax3.set_ylim((-5,103))
   ax3.plot(tr)
   

   ax4.cla()
   ax4.set_title('ALTITUDE (m)',color='silver')
   ax4.tick_params(axis='x', colors='silver')
   ax4.tick_params(axis='y', colors='silver')
   ax4.plot(altitude)
   
   plt.pause(0.0001)

   if NolduLanPasinyan==0:
      ax.legend()
      ax2.legend()
      #ax3.legend()
      #ax4.legend()
    
   else:
       pass


   #ax5.set_title('MODE SCREEN',color='silver')
   #ax6.set_title('XSENS MTi-7')


   canvas.draw()
   renderer = canvas.get_renderer()

   raw_data = renderer.tostring_rgb()
   size = canvas.get_width_height()

   return pygame.image.fromstring(raw_data, size, "RGB")

x1,y1=(860,100)
x2,y2=(860,150)
x3,y3=(860,200)
x4,y4=(860,250)
x5,y5=(860,300)
x6,y6=(860,350)
x7,y7=(860,400)
w=165
h=30

def text_objects(text,font):
    textSurface=font.render(text,True,(0,0,0))
    return textSurface,textSurface.get_rect()


smallText=pygame.font.Font("freesansbold.ttf",20)
throttle=100
tr=[]

def button(msg,x,y,w,h,num):
    mouse=pygame.mouse.get_pos()
    click=pygame.mouse.get_pressed()
    #print(click)
    #print(mouse)
        
    pygame.draw.rect(screen,color=(200,200,200),rect=(x,y,w,h))

    pygame.time.delay(1)

    if x+w > mouse[0] > x and y+h > mouse[1] > y:
        pygame.draw.rect(screen,color=(240,240,240),rect=(x,y,w,h))
        if click[0]==1:
            try:
                arduinoData.write(str.encode(num))
            except:
                print("No Device Connected to Serial Port.")
    textSurf,textRect=text_objects(msg,smallText)
    textRect.center=((x+(w/2)),(y+(h/2)))   
    screen.blit(textSurf,textRect)



while True:
    try:
        while (arduinoData.inWaiting()+32==0): #Wait here until there is data
            pass #do nothing
        arduinoString = arduinoData.readline() #read the line of text from the serial port
        dataArray = arduinoString.decode().split(',')   #Split it into an array called dataArray
        try:
            acc_X = float( dataArray[0])            
            acc_Y =    float( dataArray[1]) 
            acc_Z =    float( dataArray[2]) 
            gyro_X =    float( dataArray[3]) 
            gyro_Y =    float( dataArray[4]) 
            gyro_Z =    float( dataArray[5]) 
            akali_Roll =    float( dataArray[6]) 
            akali_Pic =    float( dataArray[7]) 
            akali_Yaw =    float( dataArray[8])       

            accX.append(acc_X)                                         
            accY.append(acc_Y)                     
            accZ.append(acc_Z)
            
            gyroX.append(gyro_X)
            gyroY.append(gyro_Y)
            gyroZ.append(gyro_Z)

    

            #altitude.append(47)

            #throttle=throttle-7
            tr.append(0)
            #altitude=altitude+14
            plt.pause(.000001)                     
            cnt=cnt+1

            if cnt>20:
                accX.pop(0)
                accY.pop(0)
                accZ.pop(0)
                gyroX.pop(0)
                gyroY.pop(0)
                gyroZ.pop(0)
                altitude.pop(0)
                tr.pop(0)
            
            
            history = numpy.array([])

            surf = plot(history)

            screen.blit(surf, (-100, -50))
            img = pygame.image.load('img/kasirgaOut3.png').convert_alpha()
            mode=pygame.image.load('img/flightModes.png').convert_alpha()
            screen.blit(img,(1080,500))
            screen.blit(mode,(845,50))
            NolduLanPasinyan+=1

            #pygame.draw.rect(screen,color=(255,255,255),rect=(550,150,500,100))

            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                #sys.exit()   

            
            
            #print("Akali Roll: {}".format(akali_Roll))
            horizon.update(screen, akali_Roll, akali_Pic )
            turn.update(screen,akali_Yaw,akali_Yaw)

            button("MANUAL",x1,y1,w,h,'1')
            button("STABILIZE",x2,y2,w,h,'2')
            button("ALTITUDE",x3,y3,w,h,'3')
            button("TAKE OFF",x4,y4,w,h,'4')
            button("LAND",x5,y5,w,h,'5')
            button("PAM",x6,y6,w,h,'6')
            button("DIVE",x7,y7,w,h,'7')
            
            pygame.display.update()
            clock.tick(60)
        
        except:
            pass
    
    except:
        history = numpy.array([])

        surf = plot(history)

        screen.blit(surf, (-100, -50))
        img = pygame.image.load('img/kasirgaOut3.png').convert_alpha()
        mode=pygame.image.load('img/flightModes.png').convert_alpha()
        screen.blit(img,(1080,500))
        screen.blit(mode,(845,50))
        NolduLanPasinyan+=1

        #pygame.draw.rect(screen,color=(255,255,255),rect=(550,150,500,100))

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                #sys.exit()   

        
        button("MANUAL",x1,y1,w,h,'1')
        button("STABILIZE",x2,y2,w,h,'2')
        button("ALTITUDE",x3,y3,w,h,'3')
        button("TAKE OFF",x4,y4,w,h,'4')
        button("LAND",x5,y5,w,h,'5')
        button("PAM",x6,y6,w,h,'6')
        button("DIVE",x7,y7,w,h,'7')
        
        horizon.update(screen, 0, 0 )
        turn.update(screen,0,0)  
        pygame.display.update()
        clock.tick(60)


