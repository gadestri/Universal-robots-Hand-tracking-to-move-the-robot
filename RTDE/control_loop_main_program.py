#!/usr/bin/env python
# Copyright (c) 2016, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#Modified by: gadestri. 
# HandTracking inspired by: https://www.youtube.com/watch?v=8KWijNI-vp4&ab_channel=Andr%C3%A9GustavoSchaeffer



import sys
from webbrowser import get
sys.path.append('..')
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import cv2 
import mediapipe as mp  # https://opensource.google/projects/mediapipe
import math

mphands = mp.solutions.hands
hands = mphands.Hands()
mpDraw = mp.solutions.drawing_utils
camera = cv2.VideoCapture(0)    
enviou = 0


ROBOT_HOST = '192.168.1.10'                             #Configurin Controller's IP
ROBOT_PORT = 30004                                      #Configurin Controllers Port
config_filename = 'control_loop_configuration.xml'      

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe('state')
setp_names, setp_types = conf.get_recipe('setp')
watchdog_names, watchdog_types = conf.get_recipe('watchdog')

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)


#defining registrators to write in the robot
setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0

setp.input_int_register_7 = 5                 
setp.input_int_register_8 = 10

# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
watchdog.input_int_register_0 = 0

def setp_to_list(setp):
    list = []
    for i in range(0,6):
        list.append(setp.__dict__["input_double_register_%i" % i])
    return list

def list_to_setp(setp, list):
    for i in range (0,6):
        setp.__dict__["input_double_register_%i" % i] = list[i]
    return setp

#start data synchronization
if not con.send_start():
    sys.exit()




# control loop
while True:
    retval, img = camera.read()
    imgc = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    image_height, image_width, _ = imgc.shape
    result = hands.process(imgc)
    #print(result.multi_hand_landmarks) 

    if result.multi_hand_landmarks: # true if hand datection 
        for handLms in result.multi_hand_landmarks:
            mpDraw.draw_landmarks(img, handLms, mphands.HAND_CONNECTIONS)
        
        indicador_x = handLms.landmark[mphands.HandLandmark.INDEX_FINGER_TIP].x * image_width
        anelar_x = handLms.landmark[mphands.HandLandmark.RING_FINGER_TIP].x * image_width
        dedao_x = handLms.landmark[mphands.HandLandmark.THUMB_TIP].x * image_width
        pulso_x= handLms.landmark[mphands.HandLandmark.WRIST].x * image_width

        indicador_y = handLms.landmark[mphands.HandLandmark.INDEX_FINGER_TIP].y * image_height      
        anelar_y = handLms.landmark[mphands.HandLandmark.RING_FINGER_TIP].y * image_height
        dedao_y = handLms.landmark[mphands.HandLandmark.THUMB_TIP].y * image_height        
        pulso_y= handLms.landmark[mphands.HandLandmark.WRIST].y * image_height

        a = indicador_x - dedao_x
        b = indicador_y - dedao_y
        c = math.sqrt(math.pow(a, 2) + math.pow(b, 2))
        estado_mao = c
        
        #Moving the robot to those cartesian's coordinates
        conv_x = 0.3868
        conv_y = ((-0.0016*pulso_x) + 0.3627)
        conv_z =  ((-0.0013*pulso_y) + 1.0333) 
        conv_rx = -1.06134
        conv_ry = 1.21739
        conv_rz = -1.34316
        
        #Writting coordinates in Robot's registrators 
        setp.input_double_register_0 = conv_x
        setp.input_double_register_1 = conv_y
        setp.input_double_register_2 = conv_z
        setp.input_double_register_3 = conv_rx
        setp.input_double_register_4 = conv_ry
        setp.input_double_register_5 = conv_rz
        
        print (conv_y)

        print(pulso_x)
        
        
        #inclinacao = anelar_y - indicador_y

        #just plotting in screen "Mao baixo" and "Mao cima"
        if (pulso_y > 350):
            cv2.putText(img=img, text='Mao BAIXO', org=(1, 300), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 255),thickness=3)    
            #setp.input_int_register_8 = 10
            
        else:
            if(pulso_y < 200):
                cv2.putText(img=img, text='Mao CIMA', org=(1, 300), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 255, 0),thickness=3)
                #setp.input_int_register_8 = 20      
                


        # Logic for close and open robot's Gripper
        if (c < 30):
            cv2.putText(img=img, text='Mao fechada', org=(1, 50), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 255),thickness=3)    
            setp.input_int_register_7 = 11          #Close Gripper
        else:
            if(c>30):
                cv2.putText(img=img, text='Mao aberta', org=(1, 50), fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 255, 0),thickness=3)
                setp.input_int_register_7 = 21      #Open Gripper
        
    cv2.imshow("Image", img)   
    cv2.waitKey(10)    


    
    # receive the current state             
    # Robô -> PC
    state = con.receive()
    posicao_tcp1 = state.actual_TCP_pose
    #print (posicao_tcp1)
    #state.actual_q()
    #print(posicao_tcp1)
    #print(state.output_int_register_6)      # Reading a variable int from robot
    if state is None:
        break;
    
    # do something...                       
    #PC->Robô
    #if state.output_int_register_0 != 0:        
        new_setp = setp1 if setp_to_list(setp) == setp2 else setp2
        list_to_setp(setp, new_setp)
        # send new setpoint        
        
    con.send(setp)


    # kick watchdog
    con.send(watchdog)

con.send_pause()

con.disconnect()
