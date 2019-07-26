
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
    ctrl2_cnt = -3
    ctrl2_fnd = 0

    target_platform_height = 0.2
    ctrl7_cnt = 0
    ctrl7_dir = 0
    target_door = 0

    ctrl3_height = 0
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
    --  target_platform_pos = {9.7, -10.3, 0.2}
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


    doorh = {0,0,0,0,0,0,0}
    doorh[1]=sim.getObjectHandle('GateCounter_55cmX40cm')
    doorh[2]=sim.getObjectHandle('GateCounter_80cmX190cm#0')
    doorh[3]=sim.getObjectHandle('GateCounter_80cmX190cm#1')
    doorh[4]=sim.getObjectHandle('GateCounter_80cmX190cm#2')
    doorh[5]=sim.getObjectHandle('GateCounter_55cmX40cm#0')
    doorh[6]=sim.getObjectHandle('GateCounter_55cmX40cm#1')
    doorh[7]=sim.getObjectHandle('GateCounter_80cmX190cm')
    print(doorh)
    tree = {0,0,0,0,0,0}
    tree[1]=sim.getObjectPosition(sim.getObjectHandle('Tree'),-1)
    tree[2]=sim.getObjectPosition(sim.getObjectHandle('Tree#0'),-1)
    tree[3]=sim.getObjectPosition(sim.getObjectHandle('smoke'),-1)
    tree[4]=sim.getObjectPosition(sim.getObjectHandle('Cylinder'),-1)
    tree[5]=sim.getObjectPosition(sim.getObjectHandle('UR3'),-1)
    tree[6]=sim.getObjectPosition(sim.getObjectHandle('UR3#0'),-1)

    paths = {}
    for i=1,12,1 do
        paths[i]=sim.getObjectHandle('target'..i)
    end
    target_pos = sim.getObjectPosition(paths[path_ctrl_flag + 1], -1)
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

    rapid_ascend = 0


    --sim.setObjectPosition(targetObj0, -1, target_pos)

    --control flag = 0: plan path
    --control flag = 1: accelerate
    --control flag = 2: decelerate 
    --control flag = 3: slower decelerate
    --control flag = 4: stableize
    linearSpeedTotal=math.sqrt(math.pow(linearSpeed_resolute[1],2)+math.pow(linearSpeed_resolute[2],2))
    target_distance=math.sqrt(math.pow(pos[1]-target_pos[1],2)+math.pow(pos[2]-target_pos[2],2))

    ---  print("pos", pos, "tpos", target_pos)
    --  print("v",linearSpeedTotal,"ctrl", ctrl_flag,"dist",target_distance, "euler", euler, "targetheight", target_height)
    temp = {pos[1]-target_pos[1], pos[2]-target_pos[2]}
    target_pushforce=normalization(temp, 1)
    total_force_resolute = {target_pushforce[1],target_pushforce[2],0,0}        
    total_force=sim.multiplyVector(m,total_force_resolute)

    ctrl_angle_x = 0
    ctrl_angle_y = 0

    if(ctrl_flag==0) then
        target_height = 1--debug!
        ctrl_flag_last_time=10
        sim.setJointTargetPosition(revolute0,math.rad(85))
        sim.setJointTargetPosition(revolute1,math.rad(85)) 
        ctrl_flag=1
    elseif(ctrl_flag==1) then

        --------------------------- flight control -------------------------

        angle = math.max(0.1, math.min(1.5,target_distance / 2))
        speed = math.min(5, target_distance)

        --debug: not slow down at first waypoint
        if(target_pos[3] == 0 and target_distance > 2) then

            ctrl_angle_x = (total_force[1] + linearSpeed[1]/5) * ctrl_angle_max * angle / 3
            ctrl_angle_y = (total_force[2] + linearSpeed[2]/5) * ctrl_angle_max * angle / 3
            --debug: not slow down at first     

        else 
            angle = 0.5
            speedx = math.min(2, math.abs(target_pos[1]-pos[1])*1)
            speedy = math.min(2, math.abs(target_pos[2]-pos[2])*1)

            ctrl_angle_x = (total_force[1]*speedx+linearSpeed[1]) * ctrl_angle_max * angle 
            ctrl_angle_x2 = ctrl_angle_x * 7--(-target_pos[1]+pos[1]) * ctrl_angle_max * angle

            ctrl_angle_y = (total_force[2]*speedy+linearSpeed[2]) * ctrl_angle_max * angle

        end
        --  print("x",ctrl_angle_x,"y",ctrl_angle_y)


        if(target_distance < 1) then

            if(target_pos[3] == 1) then
                if(target_distance < 0.2 and linearSpeedTotal < 0.3)then  ctrl_flag = 2 end
                --path_ctrl_flag = path_ctrl_flag + 1
                --target_pos = wayPoint[path_ctrl_flag + 1]
            elseif(target_pos[3] == 2) then
                if(target_distance < 0.2 and linearSpeedTotal < 0.3)then  ctrl_flag = 8 end
            elseif(target_pos[3] == 0) then
                path_ctrl_flag = path_ctrl_flag + 1
                target_pos = sim.getObjectPosition(paths[path_ctrl_flag + 1],-1)
            elseif(target_pos[3] == 3 or target_pos[3] == 4) then
                print("to7")
                ctrl_flag = 7
                target_pos = sim.getObjectPosition(paths[path_ctrl_flag + 1],-1)
                min_dist = 1000
                ctrl_flag_last_time = 0
                for j=1,#doorh,1 do
                    print("door"..j)
                    target_door = doorh[j]
                    door_pos = sim.getObjectPosition(target_door,-1)
                    dist = math.pow(door_pos[1]-target_pos[1],2)+math.pow(door_pos[2]-target_pos[2],2)
                    if(dist < min_dist) then
                        print("min"..j)
                        min_dist=dist
                        ctrl_flag_last_time = j
                        if(target_pos[3] == 3) then ctrl7_dir = 1 else ctrl7_dir = -1 end
                    end
                end
            else
                --              print("dist", target_distance, "req", target_pos[3]) 
            end
        end

    elseif(ctrl_flag==2 or ctrl_flag==3 or ctrl_flag==8 or ctrl_flag==4) then



        target_height = 0.46

        sim.setScriptSimulationParameter(hand_script_handle, 'close_hand', 'true')


        ctrl_flag_last_time = ctrl_flag_last_time - 1
        if(ctrl_flag_last_time <0) then
            target_height = 1
            if(pos[3] > 0.46) then rapid_ascend = 1 end
            if(pos[3] > 0.8) then
                target_height = 0.6
                ctrl_flag=1
                path_ctrl_flag = path_ctrl_flag + 1
                target_pos = sim.getObjectPosition(paths[path_ctrl_flag + 1],-1)
                rapid_ascend = 0
            end
        end

    elseif(ctrl_flag==5)then                

    elseif(ctrl_flag==6)then

    elseif(ctrl_flag==7) then

        angle = 0.5

        -- moving infrontof door:
        -- y pos larger than 1.5
        -- x pos in range of 0.2
        -- xspeed in range of 0.4 
        maxDistX = 0.5
        maxSpeedX = 10

        --  m=sim.getObjectMatrix(d,-1)
        --  pos_to_d = sim.multiplyVector(m,linearSpeed_resolute)

        if(ctrl7_cnt == 0)then
            -- moving to front of door: decelerate when approaching
            speedx = math.min(2, math.abs(target_pos[1]-pos[1])*1)+0.5
            speedy = math.min(2, math.abs(target_pos[2]-pos[2])*1)+0.5
        elseif(ctrl7_cnt == 1)then
            --moving through door: constant speed
            speedx = 1
            speedy = 1
        elseif(ctrl7_cnt == 2)then
            --leaving the door: no deceleration approaching the target
            speedx = 1
            speedy = 1
        end

        ctrl_angle_x = (total_force[1]*speedx+linearSpeed[1]) * ctrl_angle_max * angle 
        ctrl_angle_y = (total_force[2]*speedy+linearSpeed[2]) * ctrl_angle_max * angle

        target_door = doorh[ctrl_flag_last_time]

        m=sim.getObjectMatrix(target_door,-1)
        print('777777777777777777'..ctrl7_dir)

        entry_pos_to_door = {2*ctrl7_dir,0,0}
        --?? -> ??
        entry_pos = sim.multiplyVector(m,entry_pos_to_door)
        leave_pos_to_door = {-2*ctrl7_dir,0,0}
        --?? -> ??
        leave_pos = sim.multiplyVector(m,leave_pos_to_door)
        door_pos = sim.getObjectPosition(target_door,-1)
        pos_to_door = sim.getObjectPosition(d,target_door)

        sim.invertMatrix(m)
        m[4]=0
        m[8]=0
        m[12]=0
        --?? -> ??
        speed_to_door = sim.multiplyVector(m,linearSpeed_resolute)
        --  print(door_pos[3],speed_to_door, pos_to_door, entry_pos_to_door)
        if(ctrl7_cnt == 0)then
            --moving infrontof door
            target_pos[1] = entry_pos[1]
            target_pos[2] = entry_pos[2]

            --if arrived
            if(pos[3]<1.2 and math.abs(pos_to_door[1] - entry_pos_to_door[1]) < 0.5
                and((door_pos[3]>0.9 and math.abs(pos_to_door[2] - entry_pos_to_door[2]) < maxDistX)
                or (door_pos[3]<0.5 and pos[3] < 0.6 and math.abs(pos_to_door[3] - entry_pos_to_door[3]) < maxDistX))
                and math.abs(speed_to_door[1]) < maxSpeedX) then
                ctrl7_cnt = 1
            end
        elseif(ctrl7_cnt == 1)then
            --moving through door

            m=sim.getObjectMatrix(target_door,-1)
            way_pos_to_door = {pos_to_door[1]-1*ctrl7_dir,0,0}
            --?? -> ??
            way_pos = sim.multiplyVector(m,way_pos_to_door)
            way = normalization({pos[1]-way_pos[1], pos[2]-way_pos[2]},2)                

            target_pos[1] = pos[1]-way[1]-linearSpeed[1]/3
            target_pos[2] = pos[2]-way[2]-linearSpeed[2]/3

            --if through
            if(ctrl7_dir == 1 and pos_to_door[1] < -1 or ctrl7_dir == -1 and pos_to_door[1] > 1) then
                ctrl7_cnt = 0
                ctrl_flag = 1
                path_ctrl_flag = path_ctrl_flag + 2
                target_pos = sim.getObjectPosition(paths[path_ctrl_flag + 1],-1)
            end
        end
    end
    print(ctrl_flag, ctrl7_cnt, path_ctrl_flag)
    sim.setObjectPosition(targetObj6, -1,{target_pos[1], target_pos[2], 0.2})

    ------------------ PID Controller -------------------------------------

    -- Vertical control:
    l=sim.getVelocity(heli)
    e_z=target_height - pos[3]
    if(rapid_ascend == 0)then e_z=math.max(-0.1, math.min(0.1, e_z)) end
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
    if(ctrl_flag==100)then thrust = 0 end
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















