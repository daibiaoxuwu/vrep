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
    

    ctrl_speed_x=0
    ctrl_speed_y=0 -- [-2, 2]. speed.

    ctrl_speed_max=0.05

    ctrl_angle_x=0
    ctrl_angle_y=0 -- [-0.1, 0.1]. angle.
    
    ctrl_angle_max=0.5 -- max angle allowed for flight(rad):0.3

    linearSpeed_midway={0,0,0}

    descend_flag_max=0.02 -- max descend speed

    target_height=targetPos[3]
    target_height_change_speed=0 --landing or rising speed
    last_target_height=targetPos[3]

    ------------------ Obstacles position -------------------------------------
    target_platform = sim.getObjectHandle('Target_platform')
    target_platform_pos = sim.getObjectPosition(target_platform, -1)

    tree = sim.getObjectHandle('Tree#0')
    tree_pos = sim.getObjectPosition(tree, -1)


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
    euler=sim.getObjectOrientation(d,targetObj)
    linearSpeed_resolute, angularSpeed=sim.getObjectVelocity(d)
    linearSpeed = {0,0,0}
    --speed in self axis
    linearSpeed[1] = linearSpeed_resolute[1]*math.cos(euler[3])+linearSpeed_resolute[2]*math.sin(euler[3])
    linearSpeed[2] = linearSpeed_resolute[2]*math.cos(euler[3])+linearSpeed_resolute[1]*math.sin(euler[3])

    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end

    ------------------ Path Planning -------------------------------------
    
    --next velocity direction:
    target_distance=math.sqrt(math.pow(pos[1]-target_platform_pos[1],2)+math.pow(pos[2]-target_platform_pos[2],2))
    target_pushforce=normalization({pos[1]-target_platform_pos[1], pos[2]-target_platform_pos[2]}, 5 / target_distance)

    tree_distance=math.sqrt(math.pow(pos[1]-tree_pos[1],2)+math.pow(pos[2]-tree_pos[2],2))
    tree_pushforce=normalization({pos[1]-tree_pos[1], pos[2]-tree_pos[2]}, 2 / tree_distance)
    tree_pushforce={0,0}
    --total direction:
    total_force_resolute = {target_pushforce[1]-tree_pushforce[1],target_pushforce[2]-tree_pushforce[2],0,0}
    print(total_force_resolute)
    sim.setObjectPosition(targetObj, -1, {pos[1]-total_force_resolute[1]*3, pos[2]-total_force_resolute[2]*3, pos[3]})

    --into relative direction:

    m=sim.getObjectMatrix(d,-1)
    sim.invertMatrix(m)
    print(m)
    m[4]=0
    m[8]=0
    m[12]=0
    total_force=sim.multiplyVector(m,total_force_resolute)
    print(euler[3])

    
   
    ------------------ Speed Controller -------------------------------------
    ctrl_angle_x = total_force[1] * ctrl_angle_max
    ctrl_angle_y = total_force[2] * ctrl_angle_max
    print(ctrl_angle_x,ctrl_angle_y)
    sim.setObjectPosition(targetObj, d, {-total_force[1]*3, -total_force[2]*3, 0})


    ------------------ PID Controller -------------------------------------

    -- Vertical control:
    l=sim.getVelocity(heli)
    e_z=(target_height - pos[3])
    cumul=cumul+e_z
    thrust=9+pParam*e_z+iParam*cumul+dParam*(e_z-lastE)+l[3]*vParam
    lastE=e_z
    
    -- Rotational control:
    alphaCumul = alphaCumul + euler[1]
    alphaCorr=0.00323 + (euler[1]-ctrl_angle_x)*0.225 + 1.4*(euler[1]-pAlphaE)-- + 0.005 * alphaCumul
    pAlphaE=euler[1]

    betaCumul = betaCumul + euler[2]
    betaCorr=(euler[2]-ctrl_angle_y)*0.225 + 1.4*(euler[2]-pBetaE)-- + 0.001 * betaCumul
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





