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

    revolute0 = sim.getObjectHandle('Revolute_joint0')
    revolute1 = sim.getObjectHandle('Revolute_joint1')
    
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
    ----- the commented part is the decision logic to grap a 'Sphere'
    --hold_target_handle = sim.getObjectHandle('Sphere')
    --hold_target_position = sim.getObjectPosition(hold_target_handle, -1)

    targetPos=sim.getObjectPosition(targetObj,-1)
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(zed_v0_View)
    sim.floatingViewRemove(zed_v1_View)
end 

function sysCall_actuation() 
    s=sim.getObjectSizeFactor(d)
    pos=sim.getObjectPosition(d,-1)
    sim.setJointTargetPosition(revolute0,math.rad(-90))
    sim.setJointTargetPosition(revolute1,math.rad(-90))
    pos_z_delta = 0

    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end

    


    ------------------ Controller -------------------------------------
    -- Vertical control:
    -- landing down:

    --targetPos[3] = targetPos[3] - 0.01

    pos=sim.getObjectPosition(d,-1)
    l=sim.getVelocity(heli)
    e_z=(targetPos[3]-pos[3])
    cumul=cumul+e_z
    thrust=9+pParam*e_z+iParam*cumul+dParam*(e_z-lastE)+l[3]*vParam
    lastE=e_z
    
    
    -- Rotational control:
    euler=sim.getObjectOrientation(d,targetObj)
    linearSpeed, angularSpeed=sim.getObjectVelocity(d)
    alphaCorr=0
    if(maxCorr<1 and maxCorr > 0.7)then
        deltaSpeed = -0.1
    elseif(maxCorr<0.7 and maxCorr > 0.5)then
        deltaSpeed = 0
    elseif(maxCorr<0.7 and math.abs(euler[1]) < 0.01 and math.abs(euler[2]) < 0.01)then
        deltaSpeed = 0.1
    end

--    deltaSpeed = 0.1*(desSpeed + linearSpeed[2])



 
    print(deltaSpeed)
    alphaCumul = alphaCumul + euler[1] + deltaSpeed
    alphaCorr=0.00323 + euler[1]*0.225 + 1.4*(euler[1]-pAlphaE)-- + 0.005 * alphaCumul
    pAlphaE=euler[1] + deltaSpeed
    maxCorr=maxCorr-0.02
    

    print(maxCorr)
    if(maxCorr<1 and maxCorr > 0.7)then
        deltaSpeed = -0.1
    elseif(maxCorr<0.7 and maxCorr > 0.5)then
        deltaSpeed = 0
    elseif(maxCorr<0.5 and math.abs(euler[1]) < 0.01 and math.abs(euler[2]) < 0.01)then
        deltaSpeed = 0.1
    end

    --deltaSpeed = 0.1*desSpeed-- + linearSpeed[1])


    betaCumul = betaCumul + euler[2] + deltaSpeed
    betaCorr=euler[2]*0.225 + 1.4*(euler[2]-pBetaE)-- + 0.001 * betaCumul
    pBetaE=euler[2] + deltaSpeed



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


