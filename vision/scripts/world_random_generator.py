# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""

from __future__ import print_function

import os
import rospkg

# import for world changes
import random
import xml.etree.ElementTree as ET
world_dir = rospkg.RosPack().get_path('ros_impedance_controller') + '/worlds/'

#world random generator

def changeposition (myroot):
    counter = 0  # counter for the number of iterations
    listposition = []; # list of the position of the objects
    # iterating through the price values.
    for position in myroot.iter('pose'):
        #do noting on the first and second iteration
        if counter < 2:
            counter = counter + 1
            continue
        print("old:"+position.text)
        # create nwe random position
    new_position = str(round(random.uniform(0, 0.5), 2)) + ' ' + str(round(random.uniform(0.2, 0.8), 2)) + ' ' + '0.9' + ' 0 0 0'
    while (checkposition(new_position, listposition)):
        new_position = str(round(random.uniform(0, 0.5), 2)) + ' ' + str(round(random.uniform(0.2, 0.8), 2)) + ' ' + '0.9' + ' 0 0 0'

    position.text = new_position        
    listposition.append(position.text)
    print("array:")
    print(listposition)
    counter = counter + 1
    print(position.text)

def checkposition(actpose, listposition):
    print ("check")
    for pose in listposition:
        if (actpose==pose):
            return True
        else:
            return False

if __name__ == '__main__':

    #get user input

    number = int(input("Enter numer of blocks: "))

    #cases from 1 to 11
    if number == 1:
        mytree = ET.parse(world_dir+'legopiece1.world')
        myroot = mytree.getroot()
        changeposition(myroot)
        #apply the changes
        mytree.write(world_dir+'legopiece1.world')
        self.world_name = 'legopiece1.world'
    elif number == 2:
        mytree = ET.parse(world_dir+'legopiece2.world')
        myroot = mytree.getroot()
        changeposition(myroot)
        #apply the changes
        mytree.write(world_dir+'legopiece2.world')
        self.world_name = 'legopiece2.world'
        
    elif number == 3:
        mytree = ET.parse(world_dir+'legopiece3.world')
        myroot = mytree.getroot()
        changeposition(myroot)
        #apply the changes
        mytree.write(world_dir+'legopiece3.world')
        self.world_name = 'legopiece3.world'
        '''
    elif number == 4:

    elif number == 5:

    elif number == 6:

    elif number == 7:

    elif number == 8:

    elif number == 9:

    elif number == 10:

    elif number == 11: 
        '''


    os.system("cd /home/giulio/ros_ws; catkin_make install; source devel/setup.bash")
    #self.world_name = 'lego.world'
    #self.world_name = None # only the workbench
    #self.world_name = 'empty.world'
    #self.world_name = 'palopoli.world'

    print("Initialized ur5 generic  controller---------------------------------------------------------------")

    
        