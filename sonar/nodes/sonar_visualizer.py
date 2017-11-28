#!/usr/bin/env python
import rospy
import rosparam

import numpy as np
from scipy import optimize

#from sub8_msgs.srv import Sonar, SonarResponse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from std_msgs.msg import UInt16, Float32
#from pinger_tracker.msg import *

from advantech_pci1714.srv import *
from pinger_tracker.srv import *

from pinger_tracker.msg import *
from sonar.msg import *

import time
import scipy.fftpack
import operator

import math
import pygame

class plotter():
    def cardinal_sub(self,data):
        self.card_bearing = data.data
    
    def pol2cart(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)        

    def bearing_data(self, data):
        self.P1 = data.p1
        self.P2 = data.p2
        self.cardinal = data.cardinal_bearing
        self.dels = data.dels
        self.psolution = data.psolution

    def position(self,data):

        self.crane_x = data.x_pos
        self.crane_y = data.y_pos
        self.crane_z = data.z_pos

        crane_heading = np.arctan2(self.crane_y,self.crane_x) + np.pi

        crane_horizontal_distance = np.sqrt(self.crane_x**2+self.crane_y**2)
        
        if crane_horizontal_distance != 0:
            calculated_declination = np.arctan(self.crane_z/crane_horizontal_distance)
        else:
            calculated_declination = 0.0

        crane_heading = math.degrees(crane_heading)
        calculated_declination = math.degrees(calculated_declination)

        #rospy.loginfo("heading: %f deg, declination: %f deg" % (crane_heading, calculated_declination))

        heading = float(crane_heading)
        heading = 180-heading
        if heading < 0:
            heading = 360 + heading
        pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.
        myfont = pygame.font.SysFont(None, 20)
        bigfont = pygame.font.SysFont(None, 30)
        bhydro = bigfont.render('Cardinal: %0.1f' % self.card_bearing, False, (255, 255, 255))
        #chydro = myfont.render('B', False, (255, 0, 0))
        #dhydro = myfont.render('A', False, (255, 0, 0))
        north  = myfont.render('0', False, (255, 255, 0))
        west  = myfont.render('270', False, (255, 255, 0))
        east  = myfont.render('90', False, (255, 255, 0))
        south  = myfont.render('180', False, (255, 255, 0))

       
        self.screen.fill((0,0,0))

        if self.P1 != (0,0,0) and self.P2 != (0,0,0):
            P1_legend = bigfont.render('P1', False, (255, 165, 0))
            P2_legend = bigfont.render('P2', False, (255, 0, 255))
            self.screen.blit(P1_legend,(200,250))
            self.screen.blit(P2_legend,(230,250))
            if self.psolution == 1:
                pygame.draw.rect(self.screen, (0,255,0), [197, 247, 30, 25], 1)
            elif self.psolution == 2:
                pygame.draw.rect(self.screen, (0,255,0), [227, 247, 30, 25], 1)

        else:
            P1_legend = bigfont.render('**NOFIX**', False, (255, 0, 0))
            self.screen.blit(P1_legend,(200,250))

        hydro_legend = myfont.render('Hydrophone Layout', False, (255, 255, 255))

        self.screen.blit(bhydro,(40,250))

        self.screen.blit(hydro_legend,(260,300))

        if heading == -1 and calculated_declination == 0.0:
            self.screen.fill((100,0,0))
            badfix  = bigfont.render('!!BADFIX!!', False, (255, 0, 0))
            self.screen.blit(badfix,(45,130))       
            self.screen.blit(badfix,(225,130))     
            heading_number  = bigfont.render(('Bearing: ---'), False, (255, 255, 255))
            declination  = bigfont.render(('Declination: ---'), False, (255, 255, 255))  
        else:
            angle = crane_heading + 180
            pos = 100,150

            arrow=pygame.Surface((5,80))
            arrow.fill((0,255,0))
            pygame.draw.rect(arrow, (0,0,0), pygame.Rect(0, 40, 5, 80))
            #pygame.draw.line(arrow, (0,0,0), (0,50), (25,25))
            arrow.set_colorkey((255,255,255))

            nar=pygame.transform.rotate(arrow,angle)
            nrect=nar.get_rect(center=pos)
            self.screen.blit(nar, nrect)
            pygame.display.update()                
            pygame.draw.circle(self.screen, (255,255,0), (100, 150), 5, 0) #reference hydrophone 0
            angle2 = calculated_declination-90
            pos2 = 220,90

            arrow2=pygame.Surface((5,200))
            arrow2.fill((0,255,0))
            pygame.draw.rect(arrow2, (0,0,0), pygame.Rect(0, 100, 5, 200))
            #pygame.draw.line(arrow, (0,0,0), (0,50), (25,25))
            arrow2.set_colorkey((255,255,255))

            nar2=pygame.transform.rotate(arrow2,angle2)
            nrect2=nar2.get_rect(center=pos2)
            self.screen.blit(nar2, nrect2)
            heading_number  = bigfont.render(('Bearing: %0.1f' % heading), False, (255, 255, 255))
            declination  = bigfont.render(('Declination: %0.1f' % calculated_declination), False, (255, 255, 255))
            pygame.display.update()        

        self.screen.blit(north,(100,90))
        self.screen.blit(east,(143,145))
        self.screen.blit(west,(35,145))
        self.screen.blit(south,(90,195))
        self.screen.blit(heading_number,(10,30))
        self.screen.blit(declination,(220,30))        

        rho = 100
        phi = math.radians(self.card_bearing-109)
        (x,y) = self.pol2cart(rho,phi)

        phi = math.radians(self.card_bearing-71)
        (x2,y2) = self.pol2cart(rho,phi)

        x_cent = 150

        pygame.draw.circle(self.screen, (0,0,255), (350, 410), 5, 0)
        pygame.draw.circle(self.screen, (0, 255, 0), (350, 330), 5, 0)
        pygame.draw.circle(self.screen, (0, 255, 255), (330, 370), 5, 0)
        pygame.draw.circle(self.screen, (255, 0, 0), (285, 370), 5, 0)

        zero = myfont.render('0', False, (255, 255, 255)) 
        one = myfont.render('1', False, (255, 255, 255)) 
        two = myfont.render('2', False, (255, 255, 255)) 
        three = myfont.render('3', False, (255, 255, 255)) 

        self.screen.blit(zero,(348,415))
        self.screen.blit(one,(347,335))
        self.screen.blit(three,(327,375))
        self.screen.blit(two,(282,375))

        pygame.draw.rect(self.screen, (255,255,255), [275, 320, 100, 115], 1)        

        pygame.draw.circle(self.screen, (255,255,255), (x_cent, 380), rho, 1) #reference hydrophone 0  
        pygame.draw.circle(self.screen, (255,255,255), (x_cent, 380), 5, 0) #reference hydrophone 0      

        pygame.draw.polygon(self.screen, (255,0,0), [[x_cent, 380], [x_cent+x, 380+y],[x_cent+x2, 380+y2]], 1)    

        best_heading = np.arctan2(self.crane_y, self.crane_x) + np.pi
        best_heading = math.degrees(best_heading)
        best_heading = 180-best_heading
        if best_heading < 0:
            best_heading = 360 + best_heading        

        p1_heading = np.arctan2(self.P1[1],self.P1[0]) + np.pi
        p1_heading = math.degrees(p1_heading)
        p1_heading = 180-p1_heading
        if p1_heading < 0:
            p1_heading = 360 + p1_heading

        p2_heading = np.arctan2(self.P2[1],self.P2[0]) + np.pi
        p2_heading = math.degrees(p2_heading)
        p2_heading = 180-p2_heading
        if p2_heading < 0:
            p2_heading = 360 + p2_heading            

        #rospy.loginfo("P1: %0.1f P2: %0.1f" % (p1_heading,p2_heading))

        phi = math.radians(best_heading-90.0)
        (bestx,besty) = self.pol2cart(rho,phi)

        phi = math.radians(p1_heading-90.0)
        (p1x,p1y) = self.pol2cart(rho,phi)

        phi = math.radians(p2_heading-90.0)
        (p2x,p2y) = self.pol2cart(rho,phi)        

        if self.P1 != (0,0,0) and self.P2 != (0,0,0):
            pygame.draw.line(self.screen, (255,165,0), [x_cent, 380], [x_cent+p1x, 380+p1y], 3)    
            pygame.draw.line(self.screen, (255,0,255), [x_cent, 380], [x_cent+p2x, 380+p2y], 3)

        pygame.draw.line(self.screen, (0,255,0), [x_cent, 380], [x_cent+bestx, 380+besty], 3)

        pygame.draw.line(self.screen,(255,255,255),(220,90),(220,220),2)  
        pygame.draw.line(self.screen,(255,255,255),(220,220),(340,220),2)     

        first  = myfont.render('1st', False, (255, 255, 255)) 
        self.screen.blit(first,(30,482))  
        first  = myfont.render('last', False, (255, 255, 255)) 
        self.screen.blit(first,(330,482)) 

        #print self.dels
        self.dels = self.dels[::-1]
        #print self.dels
        for x in range(len(self.dels)):
            if self.dels[x] == 'del1':
                pygame.draw.rect(self.screen, (0,255,0), [10+x*100, 500, 80, 30], 0)   
                del1  = bigfont.render('Del1', False, (0, 0, 0)) 
                self.screen.blit(del1,(30+x*100,505))     
            elif self.dels[x] == 'del2':
                pygame.draw.rect(self.screen, (255,0,0), [10+x*100, 500, 80, 30], 0)   
                del2  = bigfont.render('Del2', False, (0, 0, 0)) 
                self.screen.blit(del2,(30+x*100,505)) 
            elif self.dels[x] == 'del3':
                pygame.draw.rect(self.screen, (0,255,255), [10+x*100, 500, 80, 30], 0)   
                del3  = bigfont.render('Del3', False, (0, 0, 0)) 
                self.screen.blit(del3,(30+x*100,505)) 
            elif self.dels[x] == 'del0':
                pygame.draw.rect(self.screen, (0,0,255), [10+x*100, 500, 80, 30], 0)   
                del0  = bigfont.render('Del0', False, (255, 255, 255)) 
                self.screen.blit(del0,(30+x*100,505))                 


    '''def received(self,data):
        rospy.loginfo("Ping received")

        ref = rospy.ServiceProxy('/hydrophones/crane_srv', Crane_pos_service)
        ref = ref()
        #print ref

        self.crane_x = ref.x
        self.crane_y = ref.y
        self.crane_z = ref.z

        crane_heading = np.arctan2(self.crane_y,self.crane_x) + np.pi

        crane_horizontal_distance = np.sqrt(self.crane_x**2+self.crane_y**2)
        
        if crane_horizontal_distance != 0:
            calculated_declination = np.arctan(self.crane_z/crane_horizontal_distance)
        else:
            calculated_declination = 0.0

        crane_heading = math.degrees(crane_heading)
        calculated_declination = math.degrees(calculated_declination)

        rospy.loginfo("heading: %f deg, declination: %f deg" % (crane_heading, calculated_declination))

        pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.
        myfont = pygame.font.SysFont('Comic Sans MS', 20)
        bigfont = pygame.font.SysFont('Comic Sans MS', 30)
        bhydro = myfont.render('C', False, (255, 0, 0))
        chydro = myfont.render('B', False, (255, 0, 0))
        dhydro = myfont.render('A', False, (255, 0, 0))
        north  = myfont.render('180', False, (255, 255, 0))
        west  = myfont.render('270', False, (255, 255, 0))
        east  = myfont.render('90', False, (255, 255, 0))
        south  = myfont.render('0', False, (255, 255, 0))
        heading  = bigfont.render(('Bearing: %0.1f' % crane_heading), False, (255, 255, 255))
        declination  = bigfont.render(('Declination: %0.1f' % calculated_declination), False, (255, 255, 255))
       
        self.screen.fill((0,0,0))

        angle = crane_heading+180
        pos = 100,150

        arrow=pygame.Surface((5,80))
        arrow.fill((0,255,0))
        pygame.draw.rect(arrow, (0,0,0), pygame.Rect(0, 40, 5, 80))
        #pygame.draw.line(arrow, (0,0,0), (0,50), (25,25))
        arrow.set_colorkey((255,255,255))

        nar=pygame.transform.rotate(arrow,angle)
        nrect=nar.get_rect(center=pos)
        self.screen.blit(nar, nrect)
        pygame.display.update()

        self.screen.blit(bhydro,(95,105))
        self.screen.blit(chydro,(60,155))
        self.screen.blit(dhydro,(130,155))
        self.screen.blit(north,(95,90))
        self.screen.blit(east,(143,155))
        self.screen.blit(west,(35,155))
        self.screen.blit(south,(90,195))
        self.screen.blit(heading,(10,30))
        self.screen.blit(declination,(220,30))


        pygame.draw.circle(self.screen, (255,255,0), (100, 150), 5, 0)
        pygame.draw.circle(self.screen, (255,255,255), (100, 125), 5, 0)
        pygame.draw.circle(self.screen, (255,255,255), (78, 163), 5, 0)
        pygame.draw.circle(self.screen, (255,255,255), (122, 163), 5, 0)       

        angle2 = calculated_declination-90
        pos2 = 220,90

        arrow2=pygame.Surface((5,200))
        arrow2.fill((0,255,0))
        pygame.draw.rect(arrow2, (0,0,0), pygame.Rect(0, 100, 5, 200))
        #pygame.draw.line(arrow, (0,0,0), (0,50), (25,25))
        arrow2.set_colorkey((255,255,255))

        nar2=pygame.transform.rotate(arrow2,angle2)
        nrect2=nar2.get_rect(center=pos2)
        self.screen.blit(nar2, nrect2)
        pygame.display.update()        


        pygame.draw.line(self.screen,(255,255,255),(220,90),(220,220),2)  
        pygame.draw.line(self.screen,(255,255,255),(220,220),(340,220),2)  

        return Ping_receivedResponse()'''

    def location_service(self, data):

        #can change x1, x2, x3, y2, y3

        #MIL T-shape layout
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [100.0,   0,     0]
        #hydro2_xyz = [-100.0,  0,     0]
        #hydro3_xyz = [0,  -100.0, 0]      

        # Equilateral layout (actual)
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [100.0,   0,     0]
        #hydro2_xyz = [-50,  86.6,     0]
        #hydro3_xyz = [-50,  -86.6, 0]

        #experimental layout
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [173.2,   0,     0]
        hydro2_xyz = [86.6,  -150,     0]
        hydro3_xyz = [86.6,  -50, -100]

        return Hydrophone_locations_serviceResponse(hydro0_xyz, hydro1_xyz, hydro2_xyz ,hydro3_xyz)

    def __init__(self):

        rospy.init_node('sonar_handler')

        rospy.Subscriber('/hydrophones/bearing_info', Bearing, self.bearing_data)
        rospy.Subscriber('/hydrophones/crane_pos', Crane_pos, self.position)
        rospy.Subscriber('/hydrophones/cardinal', Float32, self.cardinal_sub)

        self.P1 = [0,0,0]
        self.P2 = [0,0,0]
        self.cardinal = 0.0
        self.dels = ['del0','del1','del3','del2']

        self.card_bearing = 0.0

        pygame.init()
        self.screen = pygame.display.set_mode((400, 550))
        done = False   

        #self.pingpub = rospy.Publisher('/hydrophones/ping', , queue_size=1)
        #rospy.Service('hydrophones/hydrophone_position', Hydrophone_locations_service, self.location_service)
        #rospy.Service('hydrophones/ready', Ping_received, self.received)

        rate = rospy.Rate(10)

        '''try:
            rospy.loginfo('waiting for hydrophones/ping service')
            rospy.wait_for_service('hydrophones/ping', timeout = 2)
            rospy.loginfo('detected hydrophones/ping service')
        except rospy.ROSException:
            rospy.logwarn("The hydrophones/ping service never showed up!")
            rospy.signal_shutdown("ROSPy Shutdown")'''
        
        while not rospy.is_shutdown():

            for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                            done = True
            
            pygame.display.update()
            rate.sleep()

def main():
    rospy.init_node('sonar_handler', anonymous=False)

    plotter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()