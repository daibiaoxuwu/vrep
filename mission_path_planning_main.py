#!python3
# -*- coding:utf-8 -*-

# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
from entity import Entity, Search

if __name__ == '__main__':
    #print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5) # Connect to V-REP
    #print (clientID)
    if clientID!=-1:
        print ('Connected to remote API server')
        
        ##########
        objects = {
            'Tree': 'Tree',
            'Tree#0': 'Tree',
            'Cylinder': 'Cylinder',
            'Start_point': 'Start',
            'Target': 'Target',
            'End': 'End',
            'UR3': 'UR',
            'UR3#0': 'UR',
            'GateCounter_55cmX40cm': 'Gate',
            'GateCounter_55cmX40cm#0': 'Gate',
            'GateCounter_55cmX40cm#1': 'Gate',
            'GateCounter_80cmX190cm': 'Gate',
            'GateCounter_80cmX190cm#0': 'Gate',
            'GateCounter_80cmX190cm#1': 'Gate',
            'GateCounter_80cmX190cm#2': 'Gate',
        }
        
        entities = []
        for key, value in objects.items():
            if value in ['Tree', 'UR', 'Cylinder']:
                res, handle = vrep.simxGetObjectHandle(clientID, key, vrep.simx_opmode_blocking)
                res, position = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
                entity = Entity(type='Obstacle', name=key, x=position[0], y=position[1], r=2.0 if value != 'Cylinder' else 1.0)
            elif value == 'Start':
                res, handle = vrep.simxGetObjectHandle(clientID, key, vrep.simx_opmode_blocking)
                res, position = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking)
                name ='Target0' if value == 'Start' else 'Target1' if value == 'Target' else 'Target2' if value == 'End' else 'Error'
                entity = Entity(type='Point', name=name, x=position[0], y=position[1])
            elif value in ['Target', 'End']:
                function_name = "get_target_platform_pos" if value == 'Target' else "get_end_point_pos"
                res, _, position, _, _ = vrep.simxCallScriptFunction(clientID, "util_funcs", vrep.sim_scripttype_customizationscript,function_name, [], [], [],bytearray(), vrep.simx_opmode_blocking)
                name ='Target0' if value == 'Start' else 'Target1' if value == 'Target' else 'Target2' if value == 'End' else 'Error'
                entity = Entity(type='Point', name=name, x=position[0], y=position[1])
            elif value == 'Gate':
                res, handle1 = vrep.simxGetObjectHandle(clientID, key, vrep.simx_opmode_blocking)
                res, handle2 = vrep.simxGetObjectHandle(clientID, 'Tmp', vrep.simx_opmode_blocking)
                
                res, position1 = vrep.simxGetObjectPosition(clientID, handle1, -1, vrep.simx_opmode_blocking)
                vrep.simxSetObjectPosition(clientID, handle2, handle1, (2,0,0), vrep.simx_opmode_blocking)
                res, position2 = vrep.simxGetObjectPosition(clientID, handle2, -1, vrep.simx_opmode_blocking)
                vrep.simxSetObjectPosition(clientID, handle2, handle1, (-2,0,0), vrep.simx_opmode_blocking)
                res, position3 = vrep.simxGetObjectPosition(clientID, handle2, -1, vrep.simx_opmode_blocking)
                entity = Entity(type='Gate', name=key, x1=position2[0], y1=position2[1], x2=position3[0], y2=position3[1])
            else:
                print (key, value)
            entities.append(entity)
        ##########

        search = Search(entities, clientID)
        search.build(divided = 10)
        #search.draw(type='B')
        search.search()
        search.draw(type='D')

        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')
