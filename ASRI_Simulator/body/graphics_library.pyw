import pygame
import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from pygame.locals import *
import colorsys
import numpy as np
import pandas as pd
import math
from PIL import Image as Image
import time
import sys
from random import random
import scipy.stats as st
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
import os

def GraphicsLibrary(scale_user,interval,trajectory,view,land,state):
    scale=scale_user

    def text(textx, texty, color, text):
        glColor3fv(color)
        glWindowPos2f(textx, texty)
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, text.encode('ascii'))

    def textSmall(textx, texty, color, text):
        glColor3fv(color)
        glWindowPos2f(textx, texty)
        glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_10, text.encode('ascii'))

    def reshape(w, h):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0,w,0,h,-view,view)
        glMatrixMode(GL_MODELVIEW)
        gluLookAt(-1,-5,0,0,0,-1,0.2,0.2,4) #gluLookAt(-1,-5,0,0,0,-1,0.2,0.2,4)
        #gluLookAt(Rotate1,Rotate2,z,0,0,-1,Rotate3,1,0)
        glScalef(1,1,1)

    def load_texture(texture_url):
        tex_id = glGenTextures(1)
        tex = pygame.image.load(texture_url)
        tex_surface = pygame.image.tostring(tex, 'RGBA')
        tex_width, tex_height = tex.get_size()
        glBindTexture(GL_TEXTURE_2D, tex_id)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, tex_width, tex_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_surface)
        glBindTexture(GL_TEXTURE_2D, 0)
        return tex_id
            
    def main(scale_user,scale,interval,land,state):
        pygame.init()
        screen=1.2
        window = (1200*screen,960+50)
        display = pygame.display.set_mode(window,DOUBLEBUF | OPENGL)
        pygame.display.set_caption('PyOpenGL Propulsion and Automation Simulation Software')
        clock = pygame.time.Clock()
        h,w=700,700*screen*(1/(screen-1.5*screen/5))
        reshape(w,h)
        glClearColor(0.7, 0.7, 0.7, 0.7)

        Fade1=0
        Fade2=0
        Fade3=0
        """Unused variable: Fade1z=0"""
        ZoomFactor=0
        Move1=0
        Move2=0
        Move3=0

        texture1 = load_texture(r'body\graphics\map1.tif')
        texture2 = load_texture(r'body\graphics\map2.png')
        texture3 = load_texture(r'body\graphics\map3.png')
        texture4 = load_texture(r'body\graphics\paint1.png')
        texture5 = load_texture(r'body\graphics\paint2.png')

        if state==0:
            Trajectory=pd.read_excel(r'body\OpenGL.xlsx',header=0)
            print('0')
        if state==1:
            Trajectory=pd.read_excel(r'body\OpenGL1.xlsx',header=0)
            print('1')
        if state==2:
            Trajectory=pd.read_excel(r'body\OpenTrajectoryPlot.xlsx',header=0)
            print('2')
        if state==3:            
            Trajectory=pd.read_excel(r'body\OpenGL3.xlsx',header=0)
            print('3')
        if state==4:            
            Trajectory=pd.read_excel(r'body\OpenGL4.xlsx',header=0)
            print('4')

        
        Trajectory2=0
        Quantity1=Trajectory.at[0,'time']
        timeMotion=Quantity1
        Quantity2=round(Trajectory.at[0,'North'],1)
        Quantity3=round(Trajectory.at[0,'East'],1)
        Quantity4=round(Trajectory.at[0,'altitude'],1)
        Quantity5=round(Trajectory.at[0,'roll']*180/math.pi,1)
        Quantity6=-round(Trajectory.at[0,'pitch']*180/math.pi,1)
        """There was a glitch in old software causing yaw at t0 to be default "vehicle creation" orientation"""
        Quantity7=45-(round(abs(360-Trajectory.at[1,'yaw']*180/math.pi),1)-45)
        Quantity8=0
        Quantity9=0
        Quantity10=0
        Quantity11=0
        Quantity12=0
        Quantity13=0
        """Counter 'Counter':"""
        Counter=0
        toggle=0
        
        while True:
            clock.tick(100)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
            Parachute=False
            theZoomFactor=0
            Rotate1,Rotate2,z,ZoomFactor=0,0,0,1
            Rotate3=0
            t9=-1000
            Move1,Move2,Move3=0,0,0
            keypress = pygame.key.get_pressed()
            textx,texty=0,0

            """Time forward (from t0):"""
            if keypress[pygame.K_c]:
                if Counter<len(Trajectory)-interval:
                    Counter+=interval
                    timeMotion=Trajectory.at[Counter,'time']
                    position1=round(Trajectory.at[Counter,'North'],1)
                    position2=round(Trajectory.at[Counter,'East'],1)
                    position3=-round(Trajectory.at[Counter,'altitude'],1)

                    previous=Counter-1
                    position11=round(Trajectory.at[Counter,'North'],1)-round(Trajectory.at[previous,'North'],1)
                    position12=round(Trajectory.at[Counter,'East'],1)-round(Trajectory.at[previous,'East'],1)
                    position13=round(Trajectory.at[Counter,'altitude'],1)-round(Trajectory.at[previous,'altitude'],1)
                    
                    if round(Trajectory.at[Counter,'North'],1)-round(Trajectory.at[previous,'North'],1)>0:
                        Move3-=10*position11
                    else:
                        Move3+=10*position11

                    if round(Trajectory.at[Counter,'East'],1)-round(Trajectory.at[previous,'East'],1)>0:
                        Move1-=10*position12
                    else:
                        Move1+=10*position12
                        
                    if round(Trajectory.at[Counter,'altitude'],1)-round(Trajectory.at[previous,'altitude'],1)>0:
                        Move2-=10*position13
                    else:
                        Move2+=10*position13
                    
                    orientation1=round(Trajectory.at[Counter,'roll']*180/math.pi,1)
                    orientation2=-round(Trajectory.at[Counter,'pitch']*180/math.pi,1)
                    orientation3=round(abs(360-Trajectory.at[Counter,'yaw']*180/math.pi),1)
                    Quantity1=timeMotion
                    Quantity3=position1
                    Quantity2=position2
                    Quantity4=position3
                    Quantity5=orientation1
                    Quantity6=orientation2
                    Quantity7=45-(orientation3-45)
                    """print("ascent:","Counter:",Counter,  "North:",Quantity2,  "East:",Quantity3,  "altitude:",Quantity4,  "roll:",Quantity5,  "pitch:",Quantity6,  "yaw:",Quantity7)"""

                else:
                    """Upper bound of counter:"""
                    Counter=Trajectory.index[-1]
                    timeMotion=Trajectory.at[len(Trajectory)-1,'time']
                    position1=round(Trajectory.at[len(Trajectory)-1,'North'],1)
                    position2=round(Trajectory.at[len(Trajectory)-1,'East'],1)
                    position3=-round(Trajectory.at[len(Trajectory)-1,'altitude'],1)
                    orientation1=round(Trajectory.at[len(Trajectory)-1,'roll']*180/math.pi,1)
                    orientation2=-round(Trajectory.at[len(Trajectory)-1,'pitch']*180/math.pi,1)
                    orientation3=round(abs(360-Trajectory.at[len(Trajectory)-1,'yaw']*180/math.pi),1)
                    Quantity1=timeMotion
                    Quantity3=position1
                    Quantity2=position2
                    Quantity4=position3
                    Quantity5=orientation1
                    Quantity6=orientation2
                    Quantity7=45-(orientation3-45)
                    
            """Time backward:"""
            if keypress[pygame.K_v]:
                """Lower bound of counter:"""
                if Counter==Trajectory.index[-1]: 
                    Counter=(math.floor(Counter/interval))*interval
                    timeMotion=Trajectory.at[Counter,'time']
                    position1=round(Trajectory.at[Counter,'North'],1)
                    position2=round(Trajectory.at[Counter,'East'],1)
                    position3=-round(Trajectory.at[Counter,'altitude'],1)
                    orientation1=round(Trajectory.at[Counter,'roll']*180/math.pi,1)
                    orientation2=-round(Trajectory.at[Counter,'pitch']*180/math.pi,1)
                    orientation3=round(abs(360-Trajectory.at[Counter,'yaw']*180/math.pi),1)
                    Quantity1=timeMotion
                    Quantity3=position1
                    Quantity2=position2
                    Quantity4=position3
                    Quantity5=orientation1
                    Quantity6=orientation2
                    Quantity7=45-(orientation3-45)
                elif Counter>1:
                    Counter-=interval
                    timeMotion=Trajectory.at[Counter,'time']
                    position1=round(Trajectory.at[Counter,'North'],1)
                    position2=round(Trajectory.at[Counter,'East'],1)
                    position3=-round(Trajectory.at[Counter,'altitude'],1)

                    if round(Trajectory.at[Counter,'North'],1)-round(Trajectory.at[previous,'North'],1)>0:
                        Move3-=10*position11
                    else:
                        Move3+=10*position11

                    if round(Trajectory.at[Counter,'East'],1)-round(Trajectory.at[previous,'East'],1)>0:
                        Move1-=10*position12
                    else:
                        Move1+=10*position12
                        
                    if round(Trajectory.at[Counter,'altitude'],1)-round(Trajectory.at[previous,'altitude'],1)>0:
                        Move2-=10*position13
                    else:
                        Move2+=10*position13
                    
                    orientation1=round(Trajectory.at[Counter,'roll']*180/math.pi,1)
                    orientation2=-round(Trajectory.at[Counter,'pitch']*180/math.pi,1)
                    orientation3=round(abs(360-Trajectory.at[Counter,'yaw']*180/math.pi),1)
                    Quantity1=timeMotion
                    Quantity3=position1
                    Quantity2=position2
                    Quantity4=position3
                    Quantity5=orientation1
                    Quantity6=orientation2
                    Quantity7=45-(orientation3-45)
                    """Return to t0:"""
                elif Counter==1:
                    Counter-=1
                    timeMotion=Trajectory.at[Counter,'time']
                    position1=round(Trajectory.at[Counter,'North'],1)
                    position2=round(Trajectory.at[Counter,'East'],1)
                    position3=0.0
                    orientation1=round(Trajectory.at[Counter,'roll']*180/math.pi,1)
                    orientation2=-round(Trajectory.at[Counter,'pitch']*180/math.pi,1)
                    orientation3=round(abs(360-Trajectory.at[1,'yaw']*180/math.pi),1)
                    Quantity1=timeMotion
                    Quantity3=position1
                    Quantity2=position2
                    Quantity4=position3
                    Quantity5=orientation1
                    Quantity6=orientation2
                    Quantity7=45-(orientation3-45)
                elif Counter<1:
                    Counter=Trajectory.index[0]
                    timeMotion=Trajectory.at[0,'time']
                    position1=round(Trajectory.at[0,'North'],1)
                    position2=round(Trajectory.at[0,'East'],1)
                    position3=0.0
                    orientation1=round(Trajectory.at[0,'roll']*180/math.pi,1)
                    orientation2=-round(Trajectory.at[0,'pitch']*180/math.pi,1)
                    orientation3=round(abs(360-Trajectory.at[1,'yaw']*180/math.pi),1)
                    Quantity1=timeMotion
                    Quantity3=position1
                    Quantity2=position2
                    Quantity4=position3
                    Quantity5=orientation1
                    Quantity6=orientation2
                    Quantity7=45-(orientation3-45)
                    """print("descent:","Counter:",Counter,  "North:",Quantity2,  "East:",Quantity3,  "altitude:",Quantity4,  "roll:",Quantity5,  "pitch:",Quantity6,  "yaw:",Quantity7)"""

            """Translation:"""
            if keypress[pygame.K_q]:
                Move2-=30
            if keypress[pygame.K_w]:
                Move3-=30
            if keypress[pygame.K_e]:
                Move2+=30
            if keypress[pygame.K_a]:
                Move1+=30
            if keypress[pygame.K_s]:
                Move3+=30
            if keypress[pygame.K_d]:
                Move1-=30

            """Orientation:"""
            if keypress[pygame.K_i]:
                Rotate2+=0.07
            if keypress[pygame.K_k]:
                Rotate2-=0.07
            if keypress[pygame.K_j]:
                Rotate1+=0.08
            if keypress[pygame.K_l]:
                Rotate1-=0.08
            if keypress[pygame.K_u]:
                Rotate3+=0.05
            if keypress[pygame.K_o]:
                Rotate3-=0.05

            """Scale (rocket):"""
            if keypress[pygame.K_n]:
                if scale<1500:
                    """round(scale_user,0)/10:"""
                    scale+=(scale_user-1)/10
            if keypress[pygame.K_m]:
                if scale>1:
                    """round(scale_user,0)/10:"""
                    scale-=(scale_user-1)/10
                    if scale<1:
                        scale=1
                    else:
                        pass

            """Zoom:"""
            if keypress[pygame.K_z]:
                ZoomFactor+=0.1
            if keypress[pygame.K_x]:
                ZoomFactor-=0.1

            """Fade-Unfade 1:"""
            if keypress[pygame.K_r]:
                if Fade1<0.5:
                    Fade1+=0.1
            """Fade-Unfade 2:"""
            if keypress[pygame.K_t]:
                if Fade2<1:
                    Fade2+=0.2
            """Fade-Unfade 3:"""
            if keypress[pygame.K_y]:
                if Fade3<0.5:
                    Fade3+=0.025
            """Fade-Unfade 1:"""
            if keypress[pygame.K_f]:
                Fade1-=0.1
            """Fade-Unfade 2:"""
            if keypress[pygame.K_g]:
                Fade2-=0.2
            """Fade-Unfade 3:"""
            if keypress[pygame.K_h]:
                Fade3-=0.025
            """
            Fade-Unfade (rocket paint swap)
            if keypress[pygame.K_9]:
                zzz+=0.2
            if keypress[pygame.K_0]:
                zzz-=0.2
            """
            if keypress[pygame.K_1]:
                if toggle==1:
                    toggle-=1
            if keypress[pygame.K_2]:
                if toggle<1:
                    toggle+=1

            gluLookAt(Rotate1,Rotate2,z,0,0,-1,Rotate3,1,0)
            glScalef(ZoomFactor,ZoomFactor,ZoomFactor)
            glTranslatef(Move1,Move2,Move3)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            """Grid:"""
            glPushMatrix()
            glTranslated(150, 100, 0)
            glRotated(0, 0, 0, 1)
            glScale(200, 200, 200)
            glBegin(GL_QUADS)

            Grid1=0
            while Grid1 < 11:
                glColor3f(1, 1,1)
                glVertex3f(Grid1, 0, 0)
                glColor3f(1, 1, 1)
                glVertex3f(Grid1, 10, 0)
                glColor3f(1, 1, 1)
                glVertex3f(Grid1+0.02, 10, 0)
                glColor3f(1, 1, 1)
                glVertex3f(Grid1+0.02,  0, 0)
                
                glColor3f(1, 1,1)
                glVertex3f(0, Grid1, 0)
                glColor3f(1, 1, 1)
                glVertex3f(10, Grid1, 0)
                glColor3f(1, 1, 1)
                glVertex3f(10, Grid1+0.02, 0)
                glColor3f(1, 1, 1)
                glVertex3f(0,  Grid1+0.02, 0)
                Grid1+=1

            Grid2=0
            while Grid2 < 11:
                glColor3f(1, 1,1)
                glVertex3f(Grid2, 0, 0)
                glColor3f(1, 1, 1)
                glVertex3f(Grid2, 0, 10)
                glColor3f(1, 1, 1)
                glVertex3f(Grid2+0.02, 0, 10)
                glColor3f(1, 1, 1)
                glVertex3f(Grid2+0.02,  0, 0)
                glColor3f(1, 1,1)
                glVertex3f(0, 0, Grid2)
                glColor3f(1, 1, 1)
                glVertex3f(10, 0, Grid2)
                glColor3f(1, 1, 1)
                glVertex3f(10, 0, Grid2+0.02)
                glColor3f(1, 1, 1)
                glVertex3f(0,  0, Grid2+0.02)
                Grid2+=1

            Grid3=0
            while Grid3 < 11:
                glColor3f(1, 1,1)
                glVertex3f(0,Grid3, 0)
                glColor3f(1, 1, 1)
                glVertex3f(0,Grid3, 10)
                glColor3f(1, 1, 1)
                glVertex3f(0,Grid3+0.02, 10)
                glColor3f(1, 1, 1)
                glVertex3f(0,Grid3+0.02, 0)

                glColor3f(1, 1,1)
                glVertex3f(0, 0, Grid3)
                glColor3f(1, 1, 1)
                glVertex3f(0,10, Grid3)
                glColor3f(1, 1, 1)
                glVertex3f(0,10, Grid3+0.02)
                glColor3f(1, 1, 1)
                glVertex3f(0,  0, Grid3+0.02)
                Grid3+=1
            
            glEnd()
            glPopMatrix()

            """Fade-Unfade Textures"""      
            glPushMatrix()
            glTranslated(130,140,0)
            glRotated(0, 1, 1, 1)
            glScale(1, 1, 1) 
            
            """Fade-Unfade Texture 1:"""
            glDisable(GL_DEPTH_TEST)
            glDepthMask(GL_FALSE)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)  
            glColor4f(1.0,1.0,1.0,1)
            glEnable(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, texture1)
            glBegin(GL_QUADS)

            glTexCoord(0,0)
            glVertex3f(-0.6*land, -0.97*land,0)
            glTexCoord(0,1)
            glVertex3f(-0.6*land, 1.03*land,0)
            glTexCoord(1,1)
            glVertex3f(2.4*land, 1.03*land,0)
            glTexCoord(1,0)
            glVertex3f(2.4*land, -0.97*land,0)
            
            glEnd()
            glDisable(GL_TEXTURE_2D)
            glEnable(GL_DEPTH_TEST)
            glDisable(GL_BLEND)
            glDepthMask(GL_TRUE)

            """Fade-Unfade Texture 2:"""
            glDisable(GL_DEPTH_TEST)
            glDepthMask(GL_FALSE)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glColor4f(1.0,1.0,1.0,Fade3)
            glEnable(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, texture2)
            glRotated(180, 0, 0, 1)
            glBegin(GL_QUADS)
            glTexCoord(0,1)
            glVertex3f(-land, -land,0)
            glTexCoord(1,1)
            glVertex3f(-land, land,0)
            glTexCoord(1,0)
            glVertex3f(land, land,0)
            glTexCoord(0,0)
            glVertex3f(land, -land,0)
            glEnd()
            glPopMatrix()
            glDisable(GL_TEXTURE_2D)
            glEnable(GL_DEPTH_TEST)
            glDisable(GL_BLEND)
            glDepthMask(GL_TRUE)

            """Rocket body:"""
            glPushMatrix()
            glColor3fv((1, 0, 1))
            glTranslatef(150, 100, 0)
            glTranslated(Quantity2, Quantity3, Quantity4)

            position1=round(Trajectory.at[len(Trajectory)-1,'North'],1)
            position2=round(Trajectory.at[len(Trajectory)-1,'East'],1)
            position3=-round(Trajectory.at[len(Trajectory)-1,'altitude'],1)

            glScale(scale, scale, scale)
            if toggle==0:
                #if Trajectory.at[Counter,'range']==0 or -(Trajectory.at[Counter,'altitude']-Trajectory.at[Counter-1,'altitude'])>0:
                ParachuteTrajectory1=0
                ParachuteTrajectory2=0
                
                glRotated(Quantity7,0,0,1)
                glRotated(90-Quantity6,0,1,0)
                glRotated(Quantity5, 0, 0, 1)
                
                glDisable(GL_DEPTH_TEST)
                glDepthMask(GL_FALSE)
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
                glColor4f(1,1,1,1)
                glEnable(GL_TEXTURE_2D)
                glBindTexture(GL_TEXTURE_2D, texture4)
                qobj = gluNewQuadric()
                gluQuadricTexture(qobj, GL_TRUE)
                gluCylinder(qobj,0.087,0.087,4.07,200,200) 
                gluDeleteQuadric(qobj)      
                glDisable(GL_TEXTURE_2D)
                glEnable(GL_DEPTH_TEST)
                glDisable(GL_BLEND)
                glDepthMask(GL_TRUE)

                glTranslatef(0,0, 4.07)
                glColor4f(1,0.5,0,1)
                glutSolidCone(0.087,0.85,50,50)     
                glTranslatef(0,0,-3.929-0.123)

                glBegin(GL_QUADS)

                glColor3f(0.5,0.5,0.5)
                glVertex3f(0.087,0,0.39)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(0.257,0,0.12)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(0.257,0,0.087)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(0.087,0,0.087)

                glColor3f(0.5,0.5,0.5)
                glVertex3f(-0.087,0,0.39)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(-0.257,0,0.12)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(-0.257,0,0.087)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(-0.087,0,0.087)

                glColor3f(0.5,0.5,0.5)
                glVertex3f(0,0.087,0.39)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(0,0.257,0.12)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(0,0.257,0.087)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(0,0.087,0.087)

                glColor3f(0.5,0.5,0.5)
                glVertex3f(0,-0.087,0.39)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(0,-0.257,0.12)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(0,-0.257,0.087)
                glColor3f(0.5,0.5,0.5)
                glVertex3f(0,-0.087,0.087)
                
                glEnd()

                glTranslatef(0,0,-0.15)

##                else:
##                    previous=Counter-10
##                    ParachuteTrajectory1=0
##                    #ParachuteTrajectory2=-180
##
##                    range1=math.sqrt(round(Trajectory.at[Counter,'North'],1)**2+round(Trajectory.at[Counter,'East'],1)**2)
##                    range2=math.sqrt(round(Trajectory.at[previous,'North'],1)**2+round(Trajectory.at[previous,'East'],1)**2)
##                    altitude1=round(Trajectory.at[Counter,'altitude'],1)-round(Trajectory.at[previous,'altitude'],1)
##                    
##                    ParachuteTrajectory2=180-math.atan2((range1-range2),(altitude1))*(180/math.pi)
##                    
##                    #ParachuteTrajectory1=math.atan((Trajectory.at[Counter,'East']-Trajectory.at[previous,'East'])/(Trajectory.at[Counter,'North']-Trajectory.at[previous,'North']))*(180/math.pi)
##                    #ParachuteTrajectory1=-100+90#-90+math.atan((Trajectory.at[Counter,'North']-Trajectory.at[previous,'North'])/(Trajectory.at[Counter,'East']-Trajectory.at[previous,'East']))*(180/math.pi)
##                    
##                    glRotated(ParachuteTrajectory1,0,0,1)
##                    glRotated(ParachuteTrajectory2,0,1,0)
##                    #glRotated(Quantity7,0,0,1)
##                    #glRotated(90-Quantity6,0,1,0)
##                    glRotated(Quantity5, 0, 0, 1)
##                    Parachute=True
##                    #glRotated(180,0,1,0)
##                    glDisable(GL_DEPTH_TEST)
##                    glDepthMask(GL_FALSE)
##                    glEnable(GL_BLEND)
##                    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
##                    glColor4f(1,1,1,1)
##                    glEnable(GL_TEXTURE_2D)
##                    glBindTexture(GL_TEXTURE_2D, texture4)
##                    qobj = gluNewQuadric()
##                    gluQuadricTexture(qobj, GL_TRUE)
##                    gluDeleteQuadric(qobj)      
##                    glDisable(GL_TEXTURE_2D)
##                    glEnable(GL_DEPTH_TEST)
##                    glDisable(GL_BLEND)
##                    glDepthMask(GL_TRUE)
##                    
##                    glTranslatef(0,0, 2.87)
##                    glColor4f(1,0.5,0,1)
##                    glutSolidCone(0.087,0.85,50,50)     
##                    glTranslatef(0,0,-2)
##                    glColor4f(1,0.75,0.5,1)
##                    glutWireCone(0.9,-1,50,50)
##
##                    glBegin(GL_QUADS)
##                    glColor3f(1,0.75,0.5)
##                    glVertex3f(0,-0.087,2)
##                    glVertex3f(0,-0.9,0)
##                    glVertex3f(0,-0.89,0)
##                    glVertex3f(0,-0.077,2)
##                    glEnd()
##                    while theZoomFactor<360:     
##                        glRotated(theZoomFactor, 0,0,1)
##                        glBegin(GL_QUADS)
##                        glVertex3f(0,-0.087,2)
##                        glVertex3f(0,-0.9,0)
##                        glVertex3f(0,-0.87,0)
##                        glVertex3f(0,-0.057,2)
##                        glEnd()
##                        theZoomFactor+=30
                    
            if toggle==1:

                glDisable(GL_DEPTH_TEST)
                glDepthMask(GL_FALSE)
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
                glColor4f(1,1,1,1)
                glEnable(GL_TEXTURE_2D)
                glBindTexture(GL_TEXTURE_2D, texture5)
                qobj = gluNewQuadric()
                gluQuadricTexture(qobj, GL_TRUE)
                gluCylinder(qobj,0.3,0.3,6.6,200,200) 
                gluDeleteQuadric(qobj)      
                glDisable(GL_TEXTURE_2D)
                glEnable(GL_DEPTH_TEST)
                glDisable(GL_BLEND)
                glDepthMask(GL_TRUE)

                glTranslatef(0,0,6.6)
                glColor4f(1,1,1,1)
                glutSolidCone(0.3,1.8,50, 50)     
                glTranslatef(0,0,-5)

            glPopMatrix()
            
            glPushMatrix()
            """Fade-Unfade Texture 3:"""
            glDisable(GL_DEPTH_TEST)
            glDepthMask(GL_FALSE)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)  
            glColor4f(1.0,1.0,1.0,Fade2)
            """DARKMODE: disable this:"""
            glEnable(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, texture3)
            glBegin(GL_QUADS)
            glTexCoord(1,0)
            glVertex3f(-land, -land,0)
            glTexCoord(0,0)
            glVertex3f(-land, land,0)
            glTexCoord(0,1)
            glVertex3f(land, land,0)
            glTexCoord(1,1)
            glVertex3f(land, -land,0)
            glEnd()
            """DARKMODE: enable this:"""
            glDisable(GL_TEXTURE_2D)
            """DARKMODE: disable this:"""
            glEnable(GL_DEPTH_TEST)
            glDisable(GL_BLEND)
            glDepthMask(GL_TRUE)
            glPopMatrix()
            
            """Trajectory:"""
            plot=1
            if plot==1:
                TrajectoryPlot=0
                while TrajectoryPlot<Counter:
                    glPushMatrix()
                    glTranslated(150, 100, 0)
                    glTranslated(round(Trajectory.at[TrajectoryPlot,'East'],1),round(Trajectory.at[TrajectoryPlot,'North'],1),-round(Trajectory.at[TrajectoryPlot,'altitude'],1))

                    glColor3f(1,1,1)
                    """OpenGL Trajectory Line Thickness"""
                    glPointSize(2.0)
                    glBegin(GL_POINTS)
                    glVertex2f(0.0, 0.0)
                    glEnd()

                    glPopMatrix()
                    TrajectoryPlot+=trajectory
                   
            else:
                pass

            """Text:"""
            if Counter>=1:
                Quantity8=round(Trajectory.at[Counter,'range'],3)
                Quantity9=round(Trajectory.at[Counter,'velocity'],3)
                Quantity10=round(Trajectory.at[Counter,'acceleration'],3)
                Quantity11=round(Trajectory.at[Counter,'alpha'],3)
                Quantity12=round(Trajectory.at[Counter,'beta'],3)
                Quantity13=round(Trajectory.at[Counter,'aoa'],3)
                """Note: be wary of NaN in trajectory input file"""

            else:
                Quantity8=round(Trajectory.at[Counter,'range'],3)
                Quantity9=round(Trajectory.at[Counter,'velocity'],3)
                Quantity10=round(Trajectory.at[Counter,'acceleration'],3)
                Quantity11=round(Trajectory.at[1,'alpha'],3)
                Quantity12=round(Trajectory.at[1,'beta'],3)
                Quantity13=round(Trajectory.at[1,'aoa'],3)
            
            text(30, 310, (1, 1, 1), "Time:")
            string=str(round(timeMotion,4))
            text(130, 310, (1, 1, 1), string)
            
            text(30, 280, (1, 1, 1), "North:")
            string=str(Quantity2)
            text(130, 280, (1, 1, 1), string)
            
            text(30, 250, (1, 1, 1), "East:")
            string=str(Quantity3)
            text(130, 250, (1, 1, 1), string)
            
            text(30, 220, (1, 1, 1), "Altitude:")
            string=str(Quantity4)
            text(130, 220, (1, 1, 1), string)

            text(30, 190, (1, 1, 1), "Roll:")
            string=str(Quantity5)
            text(130, 190, (1, 1, 1), string)

            text(30, 160, (1, 1, 1), "Pitch:")
            if Parachute==False:
                string=str(round(Quantity6,3))
                text(130, 160, (1, 1, 1), string)
            else:
                string=str(round(-(ParachuteTrajectory2-90),1))
                text(130, 160, (1, 1, 1), string)

            text(30, 130, (1, 1, 1), "Yaw:")
            if Parachute==False:
                string=str((round(Quantity7-90,3)))
                text(130, 130, (1, 1, 1), string)
            else:
                HeadingParachute=45-(round(abs(360-Trajectory.at[1,'yaw']*180/math.pi),1)-45)-90
                string=str(HeadingParachute)
                text(130, 130, (1, 1, 1), string)

            textSmall(750, 5, (1, 1, 1), "Scale:")
            string=str(scale)
            textSmall(780, 5, (1, 1, 1), string)

            textSmall(810, 5, (1, 1, 1), "Count:")
            string=str(Counter)
            textSmall(840, 5, (1, 1, 1), string)



            textSmall(810, 5, (1, 1, 1), "Count:")
            string=str(Rotate1)
            textSmall(600, 5, (1, 1, 1), string)

            textSmall(810, 5, (1, 1, 1), "Count:")
            string=str(Rotate2)
            textSmall(660, 5, (1, 1, 1), string)

            textSmall(810, 5, (1, 1, 1), "Count:")
            string=str(Rotate3)
            textSmall(720, 5, (1, 1, 1), string)



            text(30, 80, (1, 1, 1), "Range:")
            string=str(Quantity8)
            text(150, 80, (1, 1, 1), string)

            text(340, 80, (1, 1, 1), "Velocity:")
            string=str(Quantity9)
            text(470, 80, (1, 1, 1), string)

            text(630, 80, (1, 1, 1), "acceleration:")
            string=str(Quantity10)
            text(795, 80, (1, 1, 1), string)

            text(30, 50, (1, 1, 1), "Alpha:")
            string=str(Quantity11)
            text(150, 50, (1, 1, 1), string)

            text(340, 50, (1, 1, 1), "Beta:")
            string=str(Quantity12)
            text(470, 50, (1, 1, 1), string)

            text(630, 50, (1, 1, 1), "Attack Angle:")
            string=str(Quantity13)
            text(795, 50, (1, 1, 1), string)

            text(30, 20, (1, 1, 1), "Roll Rate:")
            string=str(round(Trajectory.iloc[Counter,13],3))
            text(150, 20, (1, 1, 1), string)

            text(340, 20, (1, 1, 1), "Pitch Rate:")
            string=str(round(Trajectory.iloc[Counter,14],3))
            text(470, 20, (1, 1, 1), string)

            text(630, 20, (1, 1, 1), "Yaw Rate:")
            string=str(round(Trajectory.iloc[Counter,15],3))
            text(795, 20, (1, 1, 1), string)
            
            pygame.display.flip()

    """Removed: if __name__ == "__main__":"""
    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA)
    main(scale_user,scale,interval,land,state)
    glutMainloop()


