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
    --  sim.setJointTargetPosition(revolute0,math.rad(85))
    --  sim.setJointTargetPosition(revolute1,math.rad(85))

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
    ctrl_flag=0
    -- 0 nothing
    -- 1 found pic: land down
    -- 2 not found: fly up
    -- 3 landing stage: wait for secs, keep in sight, then land down
    ctrl_flag_last_time=0
    -- time remaining for this control flag before another decision


    ctrl_speed_x=0
    ctrl_speed_y=0 -- [-2, 2]. speed.

    ctrl_speed_max=0.05

    ctrl_angle_x=0
    ctrl_angle_y=0 -- [-0.1, 0.1]. angle.

    ctrl_angle_max=0.3 -- max angle allowed for flight(rad):0.3

    linearSpeed_midway={0,0,0}

    descend_flag_max=0.02 -- max descend speed

    target_height=targetPos[3]
    target_height_change_speed=0 --landing or rising speed
    last_target_height=targetPos[3]

    target_diff={0,0,0}

    ------------------ Obstacles position -------------------------------------
   -- target_platform = sim.getObjectHandle('GateCounter_80cmX190cm')
   -- target_diff[1] = 3
   -- target_platform_pos = sim.getObjectPosition(target_platform, -1)

    -- tree = sim.getObjectHandle('Tree')
    -- tree_pos = sim.getObjectPosition(tree, -1)

 
    -----------------------------------------add code------------------------------
    --get handles
    targetObj0=sim.getObjectHandle('Quadricopter_target0')
    targetObj1=sim.getObjectHandle('Quadricopter_target1')
    targetObj2=sim.getObjectHandle('Quadricopter_target2')
    targetObj3=sim.getObjectHandle('Quadricopter_target3')
    targetObj4=sim.getObjectHandle('Quadricopter_target4')
    targetObj5=sim.getObjectHandle('Quadricopter_target5')
    targetObj6=sim.getObjectHandle('Quadricopter_target6')
sim.setObjectParent(targetObj0,-1,true)
sim.setObjectParent(targetObj1,-1,true)
sim.setObjectParent(targetObj2,-1,true)
sim.setObjectParent(targetObj3,-1,true)
sim.setObjectParent(targetObj4,-1,true)
sim.setObjectParent(targetObj5,-1,true)
sim.setObjectParent(targetObj6,-1,true)
    ctrl_flag_last_time=0
    stable_time = 0
    destx=0
    desty=0
   -- target_height = 3

   -- landing gear
    revolute0 = sim.getObjectHandle('Revolute_joint0')
    revolute1 = sim.getObjectHandle('Revolute_joint1')
    sim.setJointTargetPosition(revolute0,math.rad(-30))
    sim.setJointTargetPosition(revolute1,math.rad(-30))

    imageBuffer = sim.getVisionSensorImage(zed_vision0)
    total_time = 0
    last_pos = {0,0,0}
    last_speed = {0,0,0}
    last_pos_time = 0
    avg_speed = {0,0,0}
    avg_count = -2
    -----------------------------------------add code------------------------------

   
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(zed_v0_View)
    sim.floatingViewRemove(zed_v1_View)
end 

function normalization(vector, length)
    temp=math.sqrt(math.pow(vector[1],2)+math.pow(vector[2],2))
    vector[1]=vector[1]/temp*length
    vector[2]=vector[2]/temp*length
    return vector
end


function sysCall_actuation() 
    ------------------ Get position -------------------------------------
    s=sim.getObjectSizeFactor(d)
    pos=sim.getObjectPosition(d,-1)


    --sim.setObjectPosition(targetObj,target_platform,target_diff)
    target_platform_pos = sim.getObjectPosition(targetObj, -1)

    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end


    ------------------ add code -------------------------------------
    
    ------------------ Get position -------------------------------------
    m=sim.getObjectMatrix(d,-1)
    sim.invertMatrix(m)
    m[4]=0
    m[8]=0
    m[12]=0

    euler=sim.getObjectOrientation(d,targetObj)
    linearSpeed_resolute, angularSpeed=sim.getObjectVelocity(d)
    linearSpeed = sim.multiplyVector(m,linearSpeed_resolute)
    
    ------------------ Visual -------------------------------------
    --wait a while
    ctrl_flag_last_time = ctrl_flag_last_time - 1
    if(pos[3] > 1 and math.abs(euler[1]) < 0.07 and math.abs(euler[2]) < 0.07 and ctrl_flag_last_time < 0) then
        ctrl_flag_last_time = 3

        imageBuffer = sim.getVisionSensorImage(zed_vision0)
        print(#imageBuffer)
        maxx=0
        minx=100000
        maxy=0
        miny=100000
        maxd=0
        xlen = 720
        ylen = 3840
        ylock = 0
        out = {}
        for i=1,xlen,2 do  
            maxy2=0
            miny2=100000
            for j=1,ylen,30 do
                if (imageBuffer[i*ylen+j]>0.9 and imageBuffer[i*ylen+j+1]>0.9 and imageBuffer[i*ylen+j+2]>0.9) then
                    maxx=math.max(maxx,i)
                    minx=math.min(minx,i)
                    maxy=math.max(maxy,j)
                    miny=math.min(miny,j)
                end
            end
        end  
      --  print(maxx,minx,maxy,miny);
        if(maxx > 720-7)then
            maxx = minx + math.max(560 / (pos[3]-0.28), (maxy - miny)/3)
        end
        if(minx < 7) then
            minx = maxx - math.max(560 / (pos[3]-0.28), (maxy - miny)/3)
        end
        if(maxy > 3840-38)then
            maxy = miny + 3 * math.max(560 / (pos[3]-0.28), maxx - minx)
        end
        if(miny < 38) then
            miny = maxy - 3 * math.max(560 / (pos[3]-0.28), maxx - minx)
        end        
        deltax=0
        deltay=0
        destx1=0
        desty1=0
        destx2=0
        desty2=0

       -- print("diff",maxx-minx)
        
        fixx = 0
        fixy = 0

        anglex = 1
        angley=  1.8

        if(minx < 10000 and miny < 10000)then
            deltax = (maxx + minx)/2/xlen-0.5+fixx;
            destx = deltax * anglex * (pos[3]-0.28)
         --   maxx = xlen
        --    minx = 0
            destx1 = (maxx/xlen-0.5+fixx)*anglex * (pos[3]-0.28);
            destx2 = (minx/xlen-0.5+fixx)*anglex * (pos[3]-0.28);

      
          --  maxy = ylen
          --  miny = 0
            deltay = (maxy + miny)/2/ylen-0.5+fixy;
            desty = -deltay * angley * (pos[3]-0.28)

            desty1 = -(maxy/ylen-0.5+fixy)*angley * (pos[3]-0.28);
            desty2 = -(miny/ylen-0.5+fixy)*angley * (pos[3]-0.28);
            
            last_pos = sim.getObjectPosition(targetObj, -1)

            print("delta-----------------",deltax,deltay);
            sim.setObjectPosition(targetObj5, d, {0.1, -0.08, -pos[3]+0.28})        
            sim.setObjectPosition(targetObj, targetObj5, {(destx1+destx2)/2, (desty1+desty2)/2, 0})
            sim.setObjectPosition(targetObj1, targetObj5, {destx1, desty1, 0})
            sim.setObjectPosition(targetObj2, targetObj5, {destx1, desty2, 0})
            sim.setObjectPosition(targetObj3, targetObj5, {destx2, desty1, 0})
            sim.setObjectPosition(targetObj4, targetObj5, {destx2, desty2, 0})


            
            --sim.setObjectPosition(targetObj, targetObj5, {(destx1+destx2)/2, (desty1+desty2)/2, 0})
            target_platform_pos = sim.getObjectPosition(targetObj, -1)

            -- record last
            
            if(last_pos[3] ~= 0) then
                new_speed = {(target_platform_pos[1] - last_pos[1]) / last_pos_time,
                              (target_platform_pos[2] - last_pos[2]) / last_pos_time,
                               0}
                if(avg_count > 0) then
                    avg_speed[1] = avg_speed[1] + new_speed[1]
                    avg_speed[2] = avg_speed[2] + new_speed[2]      
                end
                avg_count = avg_count + 1
            end
            last_pos_time = 0
            last_pos = target_platform_pos
            
            
       --     targetPos[1] = targetPos[1] + 0.06 * math.max(-1, math.min(1,(destx1+destx2)/2 - targetPos[1]))
       --     targetPos[2] = targetPos[2] + 0.06 * math.max(-1, math.min(1,(desty1+desty2)/2 - targetPos[2]))

            target_platform_pos[3] = target_height
            sim.setObjectPosition(targetObj, -1, target_platform_pos)

            --descending
            if(target_height < 1)then
                max_descend_error = 0.35
            else
                max_descend_error = 0.4
            end
            print("-----------diff", pos[1] - target_platform_pos[1],pos[2] - target_platform_pos[2])
            if(math.abs(pos[2] - target_platform_pos[2]) < 0.3 and math.abs(pos[1] - target_platform_pos[1]) < 0.3) then
                if(math.abs(deltax)<max_descend_error and math.abs(deltay)<max_descend_error) then                    
                    target_height = pos[3] - 0.1
                    --risky!!!!!!!!!!!!!!!!!!!!!!1
                    --if(target_height < 1)then target_height = 0 end

                    if(math.abs(pos[2] - target_platform_pos[2]) < 0.15 and math.abs(pos[1] - target_platform_pos[1]) < 0.15) then
                        
                        --risky!!!!!!!!!!!!!!!!!!!!!!1
                    --    if(target_height < 1)then target_height = 0 end

                        target_height = pos[3] - 0.4
                            
                    end  
                    
                elseif(target_height < 1)then
                    target_height = pos[3] + 0.2
                end
            end
                
            --calculate speed
            

        end
    end
    
        
    dest_linearSpeed = {0,0,0}

    last_pos_time = last_pos_time + 1
    if(avg_count > 2) then
        dest_linearSpeed[1] = avg_speed[1] / avg_count / 0.025
        dest_linearSpeed[2] = avg_speed[2] / avg_count / 0.025
        target_platform_pos[1] = target_platform_pos[1] + avg_speed[1] / avg_count * last_pos_time
        target_platform_pos[2] = target_platform_pos[2] + avg_speed[2] / avg_count * last_pos_time
    end
    target_platform_pos[3] = 0.3
    sim.setObjectPosition(targetObj6, -1,target_platform_pos)

   

    -- do not set position!




    ------------------ add code -------------------------------------
    ------------------ Path Planning -------------------------------------

    --control flag = 0: plan path
    --control flag = 1: accelerate
    --control flag = 2: decelerate 
    --control flag = 3: slower decelerate
    --control flag = 4: stableize
    linearSpeedTotal=math.sqrt(math.pow(linearSpeed_resolute[1],2)+math.pow(linearSpeed_resolute[2],2))
    target_distance=math.sqrt(math.pow(pos[1]-target_platform_pos[1],2)+math.pow(pos[2]-target_platform_pos[2],2))

    print("v",linearSpeedTotal,ctrl_flag,"dist",target_distance, "euler", euler)
    target_pushforce=normalization({pos[1]-target_platform_pos[1], pos[2]-target_platform_pos[2]}, 1)
    total_force_resolute = {target_pushforce[1],target_pushforce[2],0,0}        
    total_force=sim.multiplyVector(m,total_force_resolute)



    ctrl_angle_x = 0
    ctrl_angle_y = 0

    -- direction control
    if(stable_time <= 0)then
        --common flight
        angle = math.max(0.3, math.min(1,target_distance / 2))
        speed = math.min(2.5, target_distance)
    else
        --stablization  
        angle = math.max(0.1, math.min(1,target_distance / 4))
        speed = math.min(2.5, target_distance)
    end
    direction_r = {total_force[1] * speed + linearSpeed[1] - dest_linearSpeed[1], total_force[2] * speed + linearSpeed[2] - dest_linearSpeed[2]}
    direction = normalization(direction_r, 1)
    ctrl_angle_x = direction[1] * ctrl_angle_max * angle
    ctrl_angle_y = direction[2] * ctrl_angle_max * angle    

        
        
   -- end
       
   
    print(ctrl_angle_x, ctrl_angle_y)
    if(stable_time > 0 or pos[3] < 1 and math.abs(pos[2] - target_platform_pos[2]) < 0.3 and math.abs(pos[1] - target_platform_pos[1]) < 0.3    
        and math.abs(linearSpeed[1] - dest_linearSpeed[1]) < 0.1 and math.abs(linearSpeed[2] - dest_linearSpeed[2]) < 0.1) then       
        ctrl_angle_x = 0
        ctrl_angle_y = 0
        target_height = 0.8
        stable_time = stable_time + 1
        print('~~~~~~~~~~stable')
        if(stable_time > 6) then
            target_height = 0
        end
    

    end
    ------------------ PID Controller -------------------------------------

    -- Vertical control:
    l=sim.getVelocity(heli)
    e_z=(target_height - pos[3])
    cumul=cumul+e_z
    thrust=9+pParam*e_z+iParam*cumul+dParam*(e_z-lastE)+l[3]*vParam
    lastE=e_z

    -- Rotational control:
    alphaCumul = alphaCumul + euler[1]
    alphaCorr=0.00323 + (euler[1]-ctrl_angle_y)*0.225 + 1.4*(euler[1]-pAlphaE) + 0.005 * alphaCumul
    pAlphaE=euler[1]

    betaCumul = betaCumul + euler[2]
    betaCorr=(euler[2] + ctrl_angle_x)*0.225 + 1.4*(euler[2]-pBetaE) + 0.001 * betaCumul
    pBetaE=euler[2]

    rotCorrCumul = rotCorrCumul + (euler[3])
    rotCorrP = (euler[3])*1
    rotCorrD = 10*(euler[3] - prevEuler)
    rotCorr = rotCorrP + rotCorrD  + 0.001 * rotCorrCumul
    --print('rotCorr value: d,', 8*(euler[3] - prevEuler), 'p,', (euler[3]-math.pi/6)*8)
    prevEuler=euler[3]

    --land
    --if(pos[3] < 0.5) then thrust = 0 end
    print('height', pos[3])


    -- Decide of the motor velocities:
    particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)
    -- print('alphaCorr, betaCorr:', alphaCorr, betaCorr, rotCorr)
    -- print('particlesTargetVelocities:', particlesTargetVelocities)
    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end
end 






