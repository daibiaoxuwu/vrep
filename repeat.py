import numpy as np
import vrep
for i in range(50):
    try:
        # close any open connections
        vrep.simxFinish(-1)
        # Connect to the V-REP continuous server
        clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)

        if clientID != -1: # if we connected successfully
            print ('Connected to remote API server')

            # --------------------- Setup the simulation

            vrep.simxSynchronous(clientID,True)

            dt = .025
            vrep.simxSetFloatingParameter(clientID,
                    vrep.sim_floatparam_simulation_time_step,
                    dt, # specify a simulation time step
                    vrep.simx_opmode_oneshot)

            # start our simulation in lockstep with our code
            vrep.simxStartSimulation(clientID,
                    vrep.simx_opmode_blocking)

            count = 0
            track_hand = []
            track_target = []
            while count < 60: # run for 1 simulated second


                # move simulation ahead one time step
                vrep.simxSynchronousTrigger(clientID)
                count += dt

            # stop the simulation
            vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

            # Before closing the connection to V-REP,
            #make sure that the last command sent out had time to arrive.
            vrep.simxGetPingTime(clientID)

            # Now close the connection to V-REP:
            vrep.simxFinish(clientID)
        else:
            raise Exception('Failed connecting to remote API server')

    finally:

        # stop the simulation
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

        # Before closing the connection to V-REP,
        # make sure that the last command sent out had time to arrive.
        vrep.simxGetPingTime(clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
        print('connection closed...')

