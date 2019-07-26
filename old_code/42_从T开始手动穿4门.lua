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
    -- path finding flag 
    path_ctrl_flag=0
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

    avg_pos_cnt = -2
    avg_pos = {0,0,0}
    ctrl2_cnt = 0
    ctrl2_fnd = 0

    ------------------ Obstacles position -------------------------------------


    -- tree = sim.getObjectHandle('Tree')
    -- tree_pos = sim.getObjectPosition(tree, -1)
    util_funcs = sim.getObjectHandle('util_funcs')
    utilScriptHandle = sim.getCustomizationScriptAssociatedWithObject(util_funcs)
    print("utilScriptHandle",utilScriptHandle)
    target_platform_pos = sim.callScriptFunction("get_target_platform_pos@util_funcs",utilScriptHandle)
    endpos = sim.callScriptFunction("get_end_point_pos@util_funcs",utilScriptHandle)

    targetObj0=sim.getObjectHandle('Quadricopter_target0')
    targetObj1=sim.getObjectHandle('Quadricopter_target1')
    targetObj2=sim.getObjectHandle('Quadricopter_target2')
    targetObj3=sim.getObjectHandle('Quadricopter_target3')
    targetObj4=sim.getObjectHandle('Quadricopter_target4')
    targetObj5=sim.getObjectHandle('Quadricopter_target5')
    targetObj6=sim.getObjectHandle('Quadricopter_target6')

    sim.setObjectParent(targetObj0,-1,true)
    target_platform_pos[3]=0.2
    sim.setObjectPosition(targetObj0,-1,target_platform_pos)
    target_pos = {0,0,0}
    target_pos[1] = target_platform_pos[1]
    target_pos[2] = target_platform_pos[2]
    target_pos[3] = 0.4
    sim.setObjectParent(targetObj1,-1,true)
    sim.setObjectParent(targetObj2,-1,true)
    sim.setObjectParent(targetObj3,-1,true)
    sim.setObjectParent(targetObj4,-1,true)
    sim.setObjectParent(targetObj5,-1,true)
    sim.setObjectParent(targetObj6,-1,true)

    door = {0,0,0,0,0,0,0}
    door[1]=sim.getObjectPosition(sim.getObjectHandle('GateCounter_55cmX40cm'),-1)
    door[2]=sim.getObjectPosition(sim.getObjectHandle('GateCounter_55cmX40cm#0'),-1)
    door[3]=sim.getObjectPosition(sim.getObjectHandle('GateCounter_55cmX40cm#1'),-1)
    door[4]=sim.getObjectPosition(sim.getObjectHandle('GateCounter_80cmX190cm'),-1)
    door[5]=sim.getObjectPosition(sim.getObjectHandle('GateCounter_80cmX190cm#0'),-1)
    door[6]=sim.getObjectPosition(sim.getObjectHandle('GateCounter_80cmX190cm#1'),-1)
    door[7]=sim.getObjectPosition(sim.getObjectHandle('GateCounter_80cmX190cm#2'),-1)

    tree = {0,0,0,0,0,0}
    tree[1]=sim.getObjectPosition(sim.getObjectHandle('Tree'),-1)
    tree[2]=sim.getObjectPosition(sim.getObjectHandle('Tree#0'),-1)
    tree[3]=sim.getObjectPosition(sim.getObjectHandle('smoke'),-1)
    tree[4]=sim.getObjectPosition(sim.getObjectHandle('Cylinder'),-1)
    tree[5]=sim.getObjectPosition(sim.getObjectHandle('UR3'),-1)
    tree[6]=sim.getObjectPosition(sim.getObjectHandle('UR3#0'),-1)

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

function CircleLineCross(tree1, tree2, radius, pos1, pos2, target_platform_pos1, target_platform_pos2, newPoint)

    radius = 2
    dist1 = math.pow(pos1 - target_platform_pos1,2)+math.pow(pos2 - target_platform_pos2,2)
    dist2 = math.pow(pos1 - tree1,2)+math.pow(pos2 - tree2,2)
    dist3 = math.pow(tree1 - target_platform_pos1,2)+math.pow(tree2 - target_platform_pos2,2)

    if(dist1 + dist2 < dist3 or dist1 + dist3 < dist2) then
        return false
    end

    diff = {pos2 - target_platform_pos2, -(pos1 - target_platform_pos1)}
    normalization(diff, radius)

    newPoint[1]= tree1 + diff[1]
    newPoint[2]= tree2 + diff[2]
    print("new", newPoint)

    u=(pos1-tree1)*(newPoint[2]-tree2)-(newPoint[1]-tree1)*(pos2-tree2);
    v=(target_platform_pos1-tree1)*(newPoint[2]-tree2)-(newPoint[1]-tree1)*(target_platform_pos2-tree2);
    w=(tree1-pos1)*(target_platform_pos2-pos2)-(target_platform_pos1-pos1)*(tree2-pos2);
    z=(newPoint[1]-pos1)*(target_platform_pos2-pos2)-(target_platform_pos1-pos1)*(newPoint[2]-pos2);
    if (u*v<=0 and w*z<=0) then return true end

    newPoint[1]= tree1 - diff[1]
    newPoint[2]= tree2 - diff[2]
    print("new2", newPoint)

    u=(pos1-tree1)*(newPoint[2]-tree2)-(newPoint[1]-tree1)*(pos2-tree2);
    v=(target_platform_pos1-tree1)*(newPoint[2]-tree2)-(newPoint[1]-tree1)*(target_platform_pos2-tree2);
    w=(tree1-pos1)*(target_platform_pos2-pos2)-(target_platform_pos1-pos1)*(tree2-pos2);
    z=(newPoint[1]-pos1)*(target_platform_pos2-pos2)-(target_platform_pos1-pos1)*(newPoint[2]-pos2);
    if (u*v<=0 and w*z<=0) then return true end
    return false
end    


function sysCall_actuation() 
    ------------------ Get position -------------------------------------
    s=sim.getObjectSizeFactor(d)
    pos=sim.getObjectPosition(d,-1)


    m=sim.getObjectMatrix(d,-1)
    sim.invertMatrix(m)
    m[4]=0
    m[8]=0
    m[12]=0

    euler=sim.getObjectOrientation(d,targetObj)
    linearSpeed_resolute, angularSpeed=sim.getObjectVelocity(d)
    linearSpeed = sim.multiplyVector(m,linearSpeed_resolute)

    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end

    ------------------ Path Planning -------------------------------------



    if(path_ctrl_flag == 0)then
        path_ctrl_flag = 1
        wayPoint={pos, target_platform_pos, endpos}
        --table.insert(wayPoint, #wayPoint - 1, {1,3,2})
        print(wayPoint)
        for i=1,6,1 do  
            newPoint={0,0,1}
            if(CircleLineCross(tree[i][1], tree[i][2], 1, pos[1], pos[2], target_platform_pos[1], target_platform_pos[2], newPoint)) then
                table.insert(wayPoint, #wayPoint - 1, newPoint)
                print("find cross_tree", i, newPoint)
            end  
        end
        for i=1,7,1 do  
            newPoint={0,0,0.2}
            if(CircleLineCross(door[i][1], door[i][2], 1, pos[1], pos[2], target_platform_pos[1], target_platform_pos[2], newPoint)) then

                m=sim.getObjectMatrix(d,-1)
                newPoint={0,0,3}
                temp = sim.multiplyVector(m,newPoint)
                temp[3]=0.2
                table.insert(wayPoint, #wayPoint - 1, temp)

                newPoint={0,0,-3}
                temp = sim.multiplyVector(m,newPoint)
                temp[3]=0.2                
                table.insert(wayPoint, #wayPoint - 1, temp)
                print("find cross_door", i, temp)
            end             
        end
        length = #wayPoint - 1
        for i=1,6,1 do      
            newPoint={0,0,1}       
            if(CircleLineCross(tree[i][1], tree[i][2], 1, target_platform_pos[1], target_platform_pos[2], endpos[1], endpos[2], newPoint)) then
                for j=length+1, #wayPoint, 1 do
                    if(math.pow(newPoint[1] - target_platform_pos[1], 2) + math.pow(newPoint[2] - target_platform_pos[2], 2) < 
                        math.pow(wayPoint[j][1] - target_platform_pos[1], 2) + math.pow(tree[i][2] - target_platform_pos[2], 2)) then
                        table.insert(wayPoint, j, newPoint)
                        break
                    end
                end
                print("find cross2_tree", i, newPoint)
            end  
        end
        for i=1,7,1 do  
            newPoint={0,0,0.2}
            if(CircleLineCross(door[i][1], door[i][2], 1, target_platform_pos[1], target_platform_pos[2], endpos[1], endpos[2], newPoint)) then
                m=sim.getObjectMatrix(d,-1)
                newPoint={0,0,3}
                temp = sim.multiplyVector(m,newPoint)
                temp[3]=0.2

                for j=length+1, #wayPoint, 1 do
                    if(math.pow(temp[1] - target_platform_pos[1], 2) + math.pow(temp[2] - target_platform_pos[2], 2) < 
                        math.pow(wayPoint[j][1] - target_platform_pos[1], 2) + math.pow(tree[i][2] - target_platform_pos[2], 2)) then
                        table.insert(wayPoint, j, newPoint)
                        break
                    end
                end

                newPoint={0,0,-3}
                temp = sim.multiplyVector(m,newPoint)
                temp[3]=0.2                
                for j=length+1, #wayPoint, 1 do
                    if(math.pow(temp[1] - target_platform_pos[1], 2) + math.pow(temp[2] - target_platform_pos[2], 2) < 
                        math.pow(wayPoint[j][1] - target_platform_pos[1], 2) + math.pow(tree[i][2] - target_platform_pos[2], 2)) then
                        table.insert(wayPoint, j, newPoint)
                        break
                    end
                end
                print("find cross2_door", i, newPoint)
            end             
        end
        print(wayPoint)
        target_pos[1] = wayPoint[path_ctrl_flag + 1][1]
        target_pos[2] = wayPoint[path_ctrl_flag + 1][2]        
        target_pos[3] = 1
        print(target_pos, path_ctrl_flag)
    end

    --sim.setObjectPosition(targetObj0, -1, target_pos)

    --control flag = 0: plan path
    --control flag = 1: accelerate
    --control flag = 2: decelerate 
    --control flag = 3: slower decelerate
    --control flag = 4: stableize

    linearSpeedTotal=math.sqrt(math.pow(linearSpeed_resolute[1],2)+math.pow(linearSpeed_resolute[2],2))
    target_distance=math.sqrt(math.pow(pos[1]-target_pos[1],2)+math.pow(pos[2]-target_pos[2],2))
    print("pos", pos, "tpos", target_pos)
    print("v",linearSpeedTotal,"ctrl", ctrl_flag,"dist",target_distance, "euler", euler, "targetheight", target_height)
    temp = {3*(pos[1]-target_pos[1]), pos[2]-target_pos[2]}
    target_pushforce=normalization(temp, 1)
    total_force_resolute = {target_pushforce[1],target_pushforce[2],0,0}        
    total_force=sim.multiplyVector(m,total_force_resolute)

    ctrl_angle_x = 0
    ctrl_angle_y = 0



    angle = math.max(0.1, math.min(1,target_distance/2))
    speed = math.min(2.5, target_distance) - 0.1      
    ctrl_angle_x = (total_force[1] + linearSpeed[1]/speed) * ctrl_angle_max * angle
    ctrl_angle_y = (total_force[2] + linearSpeed[2]/speed) * ctrl_angle_max * angle

   

    if(ctrl_flag==0) then
        target_height = 0.6
        sim.setJointTargetPosition(revolute0,math.rad(85))
        sim.setJointTargetPosition(revolute1,math.rad(85)) 
        target_pos[1]=2.5
        target_pos[2]=pos[2]
        if(target_distance < 0.2)then
            target_pos[2]=-7
            ctrl_flag=ctrl_flag+1 
        end
    elseif(ctrl_flag==1) then        
        if(pos[2] > -8.45)then
            target_pos[1]=-0.1
            ctrl_flag=ctrl_flag+1 
        elseif(pos[2] < -9.45 and pos[2] > -10.45)then
            ctrl_angle_x = ctrl_angle_x * 2
        end
    elseif(ctrl_flag==2) then
        target_height = 1
        if(target_distance < 0.2)then
            target_pos[2]=-1.9
            ctrl_flag=ctrl_flag+1 
        end
    elseif(ctrl_flag==3) then
        if(pos[2] > -3.25)then
            target_pos[1]=-3
            ctrl_flag=ctrl_flag+1 
        elseif(pos[2] < -6.25 and pos[2] > -5.25)then
            ctrl_angle_x = ctrl_angle_x * 2
        end
    elseif(ctrl_flag==4) then
        if(target_distance < 0.2)then
            target_pos[2]=2.27
            ctrl_flag=ctrl_flag+1 
        end
    elseif(ctrl_flag==5) then
        
        if(pos[2] > -1)then
            target_pos[1]=-7.2
            ctrl_flag=ctrl_flag+1 
        elseif(pos[2] < -0.9 and pos[2] > -1.9)then
            ctrl_angle_x = ctrl_angle_x * 2
        end
    elseif(ctrl_flag==6) then
        if(target_distance < 0.2)then
            target_pos[2]=8.2
            ctrl_flag=ctrl_flag+1 
        end
    elseif(ctrl_flag==7) then        
        if(pos[2] > 4.27)then
            target_pos[1]=-7.45
            ctrl_flag=ctrl_flag+1 
        elseif(pos[2] < 3.27 and pos[2] > 2.27)then
            ctrl_angle_x = ctrl_angle_x * 2 
        end
    end
        
    
    sim.setObjectPosition(targetObj6, -1,target_pos)

    --  ctrl_angle_x = 0.01
    --  ctrl_angle_y = 0.01
    ------------------ PID Controller -------------------------------------

    -- Vertical control:
    l=sim.getVelocity(heli)
    e_z=math.max(-0.1, math.min(0.1, target_height - pos[3]))
    cumul=cumul+e_z
    thrust=9+pParam*e_z+iParam*cumul+dParam*(e_z-lastE)+l[3]*vParam
    lastE=e_z

    -- Rotational control:
    alphaCumul = alphaCumul + euler[1]-ctrl_angle_y
    alphaCorr=0.00323 + (euler[1]-ctrl_angle_y)*0.225 + 1.4*(euler[1]-pAlphaE) + 0.005 * alphaCumul
    pAlphaE=euler[1]

    betaCumul = betaCumul + euler[2] + ctrl_angle_x
    betaCorr=(euler[2] + ctrl_angle_x)*0.225 + 1.4*(euler[2]-pBetaE) + 0.001 * betaCumul
    pBetaE=euler[2]

    rotCorrCumul = rotCorrCumul + (euler[3])
    rotCorrP = (euler[3])*1
    rotCorrD = 10*(euler[3] - prevEuler)
    rotCorr = rotCorrP + rotCorrD  + 0.001 * rotCorrCumul
    --print('rotCorr value: d,', 8*(euler[3] - prevEuler), 'p,', (euler[3]-math.pi/6)*8)

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

