function sysCall_init() 
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v=sim.getInt32Parameter(sim.intparam_program_version)
    if (v<20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    -- Detatch the manipulation sphere:
    targetObj=sim.getObjectHandle('Quadricopter_target')
    sim.setObjectParent(targetObj,-1,true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

    d=sim.getObjectHandle('Quadricopter_base')
    hand_handle=sim.getObjectHandle('JacoHand')
    quadricopter=sim.getObjectHandle('Quadricopter')
    quadricopter_prop_respondable1=sim.getObjectHandle('Quadricopter_propeller_respondable1')

    particlesAreVisible=sim.getScriptSimulationParameter(sim.handle_self,'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree,'particlesAreVisible',tostring(particlesAreVisible))
    simulateParticles=sim.getScriptSimulationParameter(sim.handle_self,'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree,'simulateParticles',tostring(simulateParticles))

    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    heli=sim.getObjectAssociatedWithScript(sim.handle_self)
    hand_script_handle = sim.getScriptHandle('JacoHand')
    print('hand_script_handle', hand_script_handle)
    
    particlesTargetVelocities={0,0,0,0}

    -- landing gear
    revolute0 = sim.getObjectHandle('Revolute_joint0')
    revolute1 = sim.getObjectHandle('Revolute_joint1')
    sim.setJointTargetPosition(revolute0,math.rad(85))
    sim.setJointTargetPosition(revolute1,math.rad(85))
    
    pParam=6
    iParam=0.04
    dParam=0.08
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    alphaCumul=0
    betaCumul=0
    rotCorrCumul=0
    psp2=0
    psp1=0
    spCumul=0

    prevEuler=0

    maxCorr=2
    initialTime=1
    deltaSpeed=0

    fakeShadow=sim.getScriptSimulationParameter(sim.handle_self,'fakeShadow')
    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end

    -- Prepare 2 floating views with the zed camera views:
    zed_vision0 = sim.getObjectHandle('zed_vision0')
    zed_vision1 = sim.getObjectHandle('zed_vision1')

    zed_v0_View=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    zed_v1_View=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(zed_v0_View,zed_vision0,64)
    sim.adjustView(zed_v1_View,zed_vision1,64)

    end_vector = {0,0,0.14}
    t_sim_start = sim.getSimulationTime()
    grapped = false
    speed = -1 -- m/s
    hold_time = 0.5 -- s
    distance_hold = 0.11
    start_position = sim.getObjectPosition(targetObj, -1)

    ------------------ Initialize position -------------------------------------
    targetPos=sim.getObjectPosition(targetObj,-1)

    ------------------ State counter -------------------------------------
    -- main control flag
    control_flag=0
    -- 0 nothing
    -- 1 found pic: land down
    -- 2 not found: fly up
    -- 3 landing stage: wait for secs, keep in sight, then land down
    control_flag_last_time=0
    -- time remaining for this control flag before another decision
    
    ctrl_angle_x=0
    ctrl_angle_y=0 -- 0, -0.1, 0.1. angle.

    ctrl_angle_max=0.1 -- max angle allowed for flight(rad)
    descend_flag_max=0.05 -- max descend speed

    target_height=targetPos[3]
    target_height_change_speed=0 --landing or rising speed
    last_target_height=targetPos[3]

end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(zed_v0_View)
    sim.floatingViewRemove(zed_v1_View)
end 

function sysCall_actuation() 
    ------------------ Get position -------------------------------------
    s=sim.getObjectSizeFactor(d)
    pos=sim.getObjectPosition(d,-1)
    euler=sim.getObjectOrientation(d,targetObj)
    linearSpeed, angularSpeed=sim.getObjectVelocity(d)

    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end


    ------------------ Visual Controller -------------------------------------
    if(control_flag_last_time<1) then
        if(math.abs(euler[1]) < 0.1 and math.abs(euler[2]) < 0.1) then
            
            --visual
            imageBuffer = sim.getVisionSensorImage(zed_vision0)
            maxx=0
            minx=100000
            maxy=0
            miny=100000
            maxd=0
            xlen = 720
            ylen = 3840
            for i=1,xlen,2 do  
                maxy2=0
                miny2=100000
                for j=100,ylen-100,30 do
                    if (imageBuffer[i*ylen+j]>0.9 and imageBuffer[i*ylen+j+1]>0.9 and imageBuffer[i*ylen+j+2]>0.9) then
                        maxx=math.max(maxx,i)
                        minx=math.min(minx,i)
                        maxy2=math.max(maxy2,j)
                        miny2=math.min(miny2,j)
                    end
                end
                if(maxy2 - miny2 < 10000 and maxy2 - miny2 > maxd) then
                    maxd = maxy2 - miny2;
                    maxy = maxy2;
                    miny = miny2;
                end
            end  
            print("---------------------------------------visual---------------------------------------")
            print(maxx,minx,maxy,miny);
            if(minx < 10000 and miny < 10000)then

                -- descending
                ctrl_angle_x = ((maxx + minx)/xlen-1)*ctrl_angle_max
                ctrl_angle_y = ((maxy + miny)/ylen-1)*ctrl_angle_max

                -- descend faster if centered (-0.2, 0.8)
                descend_flag = 0.8 - math.max(math.abs(ctrl_angle_x), math.abs(ctrl_angle_y)) / ctrl_angle_max
                print(descend_flag)

                -- if about to land: descend slower
                if(target_height < 0.8) then
                    if(descend_flag < 0.3) then
                        descend_flag = math.min(0, descend_flag)
                    end
                end
                control_flag_last_time=5
            else
                -- not found: pull up and decelerate
                descend_flag = -1
                ctrl_angle_x = ctrl_angle_x * 0.7
                ctrl_angle_y = ctrl_angle_y * 0.7
                control_flag_last_time=5
            end
     --   end
    end        
    control_flag_last_time=control_flag_last_time-1
    print(control_flag_last_time)

    --vertical speed:
    target_height = target_height - descend_flag_max * descend_flag
    print(target_height)
    ------------------ PID Controller -------------------------------------

    -- Vertical control:
    l=sim.getVelocity(heli)
    e_z=(target_height - pos[3])
    cumul=cumul+e_z
    thrust=9+pParam*e_z+iParam*cumul+dParam*(e_z-lastE)+l[3]*vParam
    lastE=e_z
    
    -- Rotational control:
    alphaCumul = alphaCumul + euler[1]
    alphaCorr=0.00323 + (euler[1]-ctrl_angle_y)*0.225 + 1.4*(euler[1]-pAlphaE)-- + 0.005 * alphaCumul
    pAlphaE=euler[1]

    betaCumul = betaCumul + euler[2]
    betaCorr=(euler[2]-ctrl_angle_x)*0.225 + 1.4*(euler[2]-pBetaE)-- + 0.001 * betaCumul
    pBetaE=euler[2]

    rotCorrCumul = rotCorrCumul + euler[3]
    rotCorr=euler[3]*4 + 1*(euler[3]-prevEuler) + 0.001 * rotCorrCumul
    prevEuler=euler[3]
    
    -- Decide of the motor velocities:
    particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)

    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end
end 



