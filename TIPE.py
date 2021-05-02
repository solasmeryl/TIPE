#TIPE 2017 - Véhicule Autonome (Raspberry Pi)
#Auteurs: -------------

import cv2
import numpy as np
from time import *
import math
import RPi.GPIO as GPIO
import os
import smbus
from picamera.array import PiRGBArray
from picamera import PiCamera

#Variables-----------------------------------------------------------------------
vertices=[np.array([[0,480],[0,200], [640,200], [640, 480]])] #Vertex du masque
count=0
sangle=0
anglelim=10 #Angle limite du mouvement tout droit
nbimages=1 #Nombre d'images pour la moyenne

rho=5 #Incertitude sur la posotion de la ligne de hough
theta=np.pi/90.0 #Incertitude sur l'angle de la ligne de hough
thres=100 #Threshold de validation de la ligne de hough
longmin=20 #Longueur minimale de la ligne de hough
espacemax=10 #Trou maximal dans la ligne de hough
slongueur=0 #Somme des longueurs
d=0.5 #Distance de l'obstacle
orientation="k" #Orientation actuelle du véhicule
lenlines=7 #Longueur de la liste de lignes = nombre de lignes trouvés
nbmin=5 #Nombre minimal de lignes
nbmax=15 #Nombre Maximum de lignes à l'image

bus=smbus.SMBus(1) 
address=0x12 #Adresse Arduino (I2C)

#Initialisation GPIO-------------------------------------------------------------
GPIO.setmode(GPIO.BCM)

pin_son_trig=8 #Trig Ultrason
pin_son_echo=7 #Echo Ultrason
pin_mot_1=13 #Pin 1 ctrl moteur
pin_mot_2=19 #Pin 2 ctrl moteur
pin_mot_7=16 #Pin 7 ctrl moteur

GPIO.setup(pin_led_milieu, GPIO.OUT)
GPIO.setup(pin_led_gauche, GPIO.OUT)
GPIO.setup(pin_led_droite, GPIO.OUT)
GPIO.setup(pin_son_trig, GPIO.OUT)
GPIO.setup(pin_son_echo, GPIO.IN)
GPIO.setup(pin_mot_1, GPIO.OUT)
GPIO.setup(pin_mot_2, GPIO.OUT)
GPIO.setup(pin_mot_7, GPIO.OUT)

pwm = GPIO.PWM(pin_mot_1,50) 
pwm.start(60)
GPIO.output(pin_mot_7,GPIO.HIGH)
GPIO.output(pin_mot_2,GPIO.LOW)
GPIO.output(pin_mot_1,GPIO.HIGH)
m=1 #Moteur en route

#Initialisation Cam--------------------------------------------------------------
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 24
camera.brightness = 50
camera.contrast = 0
rawCapture = PiRGBArray(camera, size=(640, 480))
sleep(0.1)
#Transformee de hough------------------------------------------------------------
def hough(img):
    global lenlines
    lines = cv2.HoughLinesP(img,rho,theta,thres,np.array([]),minLineLength=longmin,maxLineGap=espacemax)
    angle=0
    longueur=0
    slongueur=0
    if str(type(lines))!="<class 'NoneType'>": #Verification que des lignes ont bien ete trouves
        lenlines=len(lines)
        for j in range(len(lines)): #Iteration parmis toutes les lignes
            for x1,y1,x2,y2 in lines[j]: #Recuperation des coordonnes
                longueur=math.sqrt((abs(x2-x1))**2+(abs(y2-y1))**2)#Calcul longueur ligne
                slongueur+=longueur
                if y2-y1>0: #Pente positive
                    angle+=np.arctan(abs(x2-x1)/abs(y2-y1))*longueur #Calcul de l angle
                else: #Pente negative
                    angle+=-(np.arctan(abs(x2-x1)/abs(y2-y1)))*longueur
                cv2.line(imgpure,(x1,y1),(x2,y2),(0,255,0),2) #Affichage des lignes sur l image
        angle=angle/slongueur #Moyenne des angles
    return imgpure,angle,lenlines
    
#Masque--------------------------------------------------------------------------
def masque(img):
    mask = np.zeros_like(img) #Copie de l image en blanc   
    cv2.fillPoly(mask, vertices, 255) #Remplissage du polygone dans l image blanche
    image = cv2.bitwise_and(img, mask) #Superposition des 2 images
    return image

#Traitement des angles-------------------------------------------------------
def traitementangle(alpha):
    global count,sangle,orientation
    if count<nbimages: #Traitement toutes les x images acquises
        count+=1
        sangle+=alpha
    else:
        obstacle()
        cv2.imshow("Frame", hough2)
        count=0
        alpha=sangle/nbimages #Moyenne des angles sur les images
        alpha=alpha*180/np.pi #Conversion en degres
        if alpha>anglelim: #Angle trop a Gauche
            if m!=1: #Si non démaré, on démarre
				GPIO.output(pin_mot_7,GPIO.HIGH) 
				GPIO.output(pin_mot_2,GPIO.LOW)
				GPIO.output(pin_mot_1,GPIO.HIGH)
				m=1
            d=0 
            print("Gauche",alpha)
            if orientation !="g":
                bus.write_byte(address, 0) #Transmission de l'information à l'arduino
            orientation="g"
        elif alpha == 0: #Aucun angle détecté, arrêt
            GPIO.output(pin_mot_7,GPIO.LOW)
            GPIO.output(pin_mot_2,GPIO.LOW)
            GPIO.output(pin_mot_1,GPIO.LOW)
            m=0
            d=0
        elif alpha<-anglelim: #Angle trop a droite
            if m!=1:
				GPIO.output(pin_mot_7,GPIO.HIGH)
				GPIO.output(pin_mot_2,GPIO.LOW)
				GPIO.output(pin_mot_1,GPIO.HIGH)
				m=1
            d=0
            print("Droite",alpha)
            if orientation !="d":
                bus.write_byte(address, 180)
            orientation="d"
        else:
			if m!=1:
				GPIO.output(pin_mot_7,GPIO.HIGH)
				GPIO.output(pin_mot_2,GPIO.LOW)
				GPIO.output(pin_mot_1,GPIO.HIGH)
				m=1
            d=0
            print("Milieu",alpha)
            if orientation !="m":
                bus.write_byte(address, 90)
            orientation="m"
        sangle=0
               
#Ultrason:
def obstacle():
    global m
    GPIO.output(pin_son_trig, True) #Impulsion Ultrason
    sleep(0.00001)
    GPIO.output(pin_son_trig, False)
    t1=time()
    t2=time()
    while GPIO.input(pin_son_echo)==0: #Attente Echo
        t1=time()
    while GPIO.input(pin_son_echo)==1: #Retour Echo
        t2=time()
    dt=t2-t1
    d=dt*340/2 #Calcul distance objet
    if d<0.4: #Arret
        print("Stop, obstacle",round(d,3),"m")
        if m!=0:
            GPIO.output(pin_mot_7,GPIO.LOW)
            GPIO.output(pin_mot_2,GPIO.LOW)
            GPIO.output(pin_mot_1,GPIO.LOW)       
        m=0
    else: #Continue
        if m!=1:
            GPIO.output(pin_mot_7,GPIO.HIGH)
            GPIO.output(pin_mot_2,GPIO.LOW)
            GPIO.output(pin_mot_1,GPIO.HIGH)
            m=1
        print("Obstacle à",round(d,3),"m")
                
#Réglage auto luminosité/contraste-----------------------------------------------
def bricon():
    global nblines
    if nblines<nbmin and camera.brightness<100 and camera.contrast<100: #Sous-exposition
        camera.brightness += 1
        camera.contrast += 2        
        return()
    if nblines>nbmax and camera.brightness>0 and camera.contrast>-100: #Sur-exposition
        camera.brightness += -1
        camera.contrast += -2       
        return()
        
#Partie operative----------------------------------------------------------------
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    imgpure = frame.array
    gray=cv2.cvtColor(imgpure, cv2.COLOR_BGR2GRAY)#Conversion niveau de gris (A test si + opti)
    canny=cv2.Canny(gray,100,200,apertureSize = 3)#Algo canny
    i=masque(canny)#Application du masque
    hough2,angle,nblines=hough(i) #Transformee de hough
    key = cv2.waitKey(1) & 0xFF
    traitementangle(angle) #Traitement de l angle
    if nblines<5 or nblines>15:
        bricon()
    rawCapture.truncate(0)
