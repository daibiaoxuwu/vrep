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
    ----- the commented part is the decision logic to grap a 'Sphere'
    --hold_target_handle = sim.getObjectHandle('Sphere')
    --hold_target_position = sim.getObjectPosition(hold_target_handle, -1)
    
    
    -----------------------------------------add code------------------------------
    --get handles
    targetObj0=sim.getObjectHandle('Quadricopter_target0')
    targetObj1=sim.getObjectHandle('Quadricopter_target1')
    targetObj2=sim.getObjectHandle('Quadricopter_target2')
    targetObj3=sim.getObjectHandle('Quadricopter_target3')
    targetObj4=sim.getObjectHandle('Quadricopter_target4')
    targetObj5=sim.getObjectHandle('Quadricopter_target5')

    ctrl_flag_last_time=0
    destx=0
    desty=0
    target_height = 3

   -- landing gear
   -- revolute0 = sim.getObjectHandle('Revolute_joint0')
   -- revolute1 = sim.getObjectHandle('Revolute_joint1')
   -- sim.setJointTargetPosition(revolute0,math.rad(-30))
   -- sim.setJointTargetPosition(revolute1,math.rad(-30))
    -----------------------------------------add code------------------------------

end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(zed_v0_View)
    sim.floatingViewRemove(zed_v1_View)
end 

function sysCall_actuation() 
    s=sim.getObjectSizeFactor(d)
    pos=sim.getObjectPosition(d,-1)
    target_pos = sim.getObjectPosition(targetObj, -1)
--    z_distance = target_pos[3] - hold_target_position[3]
--    print('z_distance', z_distance)
--    if (math.abs(z_distance) <  0.21) then
--        sim.setScriptSimulationParameter(hand_script_handle, 'close_hand', 'true')
--        print('Closing hand')
--    end
--    print('simulation time', sim.getSimulationTime())

    pos_z_delta = 0
--    if grapped == false then
--        if (z_distance > distance_hold) then
--                pos_z_delta = speed * sim.getSimulationTimeStep() 
--                hold_start_time = sim.getSimulationTime()
--                print('start', pos_z_delta)
--        elseif z_distance < distance_hold then
            -- hold for a while
--            if (sim.getSimulationTime() - hold_start_time) > hold_time then 
--                grapped = true
--                speed = 1
--            end
--        end
--    else
--        end_delta = start_position[3] - target_pos[3]
--        if (end_delta > 0.01) then
--            pos_z_delta = speed * sim.getSimulationTimeStep() 
--        end
--    end
    
    sim.setObjectPosition(targetObj, -1, {target_pos[1], target_pos[2], target_pos[3] + pos_z_delta})
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
    if(math.abs(euler[1]) < 0.01 and math.abs(euler[2]) < 0.01 and ctrl_flag_last_time < 0) then
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
        print(maxx,minx,maxy,miny);
        if(maxx > 65)then
            maxx = minx + math.max(560 / (pos[3]-0.28), (maxy - miny)/3)
        end
        if(minx < 7) then
            minx = maxx - math.max(560 / (pos[3]-0.28), (maxy - miny)/3)
        end
        if(maxy > 345)then
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

        print("diff",maxx-minx)
        
        fixx = 0
        fixy = 0

        anglex = 1
        angley=  1.8

        if(minx < 10000)then
            deltax = (maxx + minx)/2/xlen-0.5+fixx;
            destx = deltax * anglex * (pos[3]-0.28)
         --   maxx = xlen
        --    minx = 0
            destx1 = (maxx/xlen-0.5+fixx)*anglex * (pos[3]-0.28);
            destx2 = (minx/xlen-0.5+fixx)*anglex * (pos[3]-0.28);

        end
        if(miny < 10000) then
          --  maxy = ylen
          --  miny = 0
            deltay = (maxy + miny)/2/ylen-0.5+fixy;
            desty = -deltay * angley * (pos[3]-0.28)

            desty1 = -(maxy/ylen-0.5+fixy)*angley * (pos[3]-0.28);
            desty2 = -(miny/ylen-0.5+fixy)*angley * (pos[3]-0.28);
        end
        print(deltax,deltay);
        sim.setObjectPosition(targetObj5, d, {0.1, -0.08, -pos[3]+0.28})        
        sim.setObjectPosition(targetObj, targetObj5, {(destx1+destx2)/2, (desty1+desty2)/2, 0})
        sim.setObjectPosition(targetObj1, targetObj5, {destx1, desty1, 0})
        sim.setObjectPosition(targetObj2, targetObj5, {destx1, desty2, 0})
        sim.setObjectPosition(targetObj3, targetObj5, {destx2, desty1, 0})
        sim.setObjectPosition(targetObj4, targetObj5, {destx2, desty2, 0})

        
        --sim.setObjectPosition(targetObj, targetObj5, {(destx1+destx2)/2, (desty1+desty2)/2, 0})
        targetPos = sim.getObjectPosition(targetObj, -1)
        
   --     targetPos[1] = targetPos[1] + 0.06 * math.max(-1, math.min(1,(destx1+destx2)/2 - targetPos[1]))
   --     targetPos[2] = targetPos[2] + 0.06 * math.max(-1, math.min(1,(desty1+desty2)/2 - targetPos[2]))

        targetPos[3] = target_height
        sim.setObjectPosition(targetObj, -1, targetPos)

        --descending
        if(math.abs(deltax)<0.1 and math.abs(deltay)<0.1)then
            target_height = target_height - 0.1
            print(target_height)
        end
        
    end
    





    ------------------ add code -------------------------------------










    
    ------------------ Controller -------------------------------------
    -- Vertical control:
    targetPos=sim.getObjectPosition(targetObj,-1)
    pos=sim.getObjectPosition(d,-1)
    l=sim.getVelocity(heli)
    e_z=(targetPos[3]-pos[3])
    cumul=cumul+e_z
    thrust=9+pParam*e_z+iParam*cumul+dParam*(e_z-lastE)+l[3]*vParam
    lastE=e_z
    
    -- Horizontal control: 
    sp=sim.getObjectPosition(targetObj,d)
    m=sim.getObjectMatrix(d,-1)
    -- print(m)
    vx={1,0,0}
    vx=sim.multiplyVector(m,vx)
    -- print('vx',vx)
    vy={0,1,0}
    vy=sim.multiplyVector(m,vy)
    alphaE=(vy[3]-m[12])
    alphaCumul=alphaCumul + alphaE
    alphaCorr=0.3*alphaE+1.1*(alphaE-pAlphaE)-- + alphaCumul*0.001
    betaE=(vx[3]-m[12])
    betaCumul=betaCumul + betaE
    betaCorr=-0.6*betaE-1.1*(betaE-pBetaE) - betaCumul*0.001
    pAlphaE=alphaE
    pBetaE=betaE
    spCumul=spCumul + sp[2]
    alphaCorr=alphaCorr+sp[2]*0.15+4*(sp[2]-psp2) + 0.005 * spCumul
    betaCorr=betaCorr-sp[1]*0.08-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]
    
    -- Rotational control:
    euler=sim.getObjectOrientation(d,targetObj)
    rotCorrCumul = rotCorrCumul + euler[3]
    rotCorr=euler[3]*4 + 1*(euler[3]-prevEuler) + 0.001 * rotCorrCumul
    prevEuler=euler[3]
    if(target_height < 1.3)then
        thrust = 0
    end
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

