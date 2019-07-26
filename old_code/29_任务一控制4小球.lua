function sysCall_init() 
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v=sim.getInt32Parameter(sim.intparam_program_version)
    if (v<20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    -- Detatch the manipulation sphere:
    targetObj0=sim.getObjectHandle('Quadricopter_target0')
    targetObj1=sim.getObjectHandle('Quadricopter_target1')
    targetObj2=sim.getObjectHandle('Quadricopter_target2')
    targetObj3=sim.getObjectHandle('Quadricopter_target3')
    targetObj4=sim.getObjectHandle('Quadricopter_target4')
    targetObj5=sim.getObjectHandle('Quadricopter_target5')
    targetObj=sim.getObjectHandle('Quadricopter_target')
    sim.setObjectParent(targetObj,-1,true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

    d=sim.getObjectHandle('Quadricopter_base')

    particlesAreVisible=sim.getScriptSimulationParameter(sim.handle_self,'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree,'particlesAreVisible',tostring(particlesAreVisible))
    simulateParticles=sim.getScriptSimulationParameter(sim.handle_self,'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree,'simulateParticles',tostring(simulateParticles))

    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    heli=sim.getObjectAssociatedWithScript(sim.handle_self)

    particlesTargetVelocities={0,0,0,0}

    pParam=2
    iParam=0
    dParam=0
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0

    prevEuler=0


    fakeShadow=sim.getScriptSimulationParameter(sim.handle_self,'fakeShadow')
    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end

    -- Prepare 2 floating views with the camera views:
    zed_vision0 = sim.getObjectHandle('zed_vision0')
    zed_vision1 = sim.getObjectHandle('zed_vision1')
    zed_v0_View=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    zed_v1_View=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(zed_v0_View,zed_vision0,64)
    sim.adjustView(zed_v1_View,zed_vision1,64)
    targetPos=sim.getObjectPosition(targetObj,-1)
    ctrl_flag_last_time=0
    destx=0
    desty=0

    -- landing gear
    revolute0 = sim.getObjectHandle('Revolute_joint0')
    revolute1 = sim.getObjectHandle('Revolute_joint1')
    sim.setJointTargetPosition(revolute0,math.rad(-30))
    sim.setJointTargetPosition(revolute1,math.rad(-30))
   

end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(zed_v0_View)
    sim.floatingViewRemove(zed_v1_View)
end 

function sysCall_actuation() 


    -- Vertical control:
    targetPos=sim.getObjectPosition(targetObj,-1)
    pos=sim.getObjectPosition(d,-1)
    ------------------ Get position -------------------------------------
    s=sim.getObjectSizeFactor(d)

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
    
           ctrl_flag_last_time = ctrl_flag_last_time - 1
    if(math.abs(euler[1]) < 0.03 and math.abs(euler[2]) < 0.03 and ctrl_flag_last_time < 0) then
            ctrl_flag_last_time = 3
        ------------------ Visual -------------------------------------
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
        print(maxx,minx,maxy,miny);
        deltax=0
        deltay=0
        destx1=0
        desty1=0
        destx2=0
        desty2=0
        
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
        
        sim.setObjectPosition(targetObj0, targetObj5, {(destx1+destx2)/2, (desty1+desty2)/2, 0})
        sim.setObjectPosition(targetObj1, targetObj5, {destx1, desty1, 0})
        sim.setObjectPosition(targetObj2, targetObj5, {destx1, desty2, 0})
        sim.setObjectPosition(targetObj3, targetObj5, {destx2, desty1, 0})
        sim.setObjectPosition(targetObj4, targetObj5, {destx2, desty2, 0})

    end




    l=sim.getVelocity(heli)
    e=(targetPos[3]-pos[3])
    cumul=cumul+e
    pv=pParam*e
    thrust=9+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
    lastE=e
    
    -- Horizontal control: 
    sp=sim.getObjectPosition(targetObj,d)
    m=sim.getObjectMatrix(d,-1)
    vx={1,0,0}
    vx=sim.multiplyVector(m,vx)
    vy={0,1,0}
    vy=sim.multiplyVector(m,vy)
    alphaE=(vy[3]-m[12])
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE=(vx[3]-m[12])
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    betaCorr=betaCorr-sp[1]*0.005-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]
    
    -- Rotational control:
    
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3]
    if(pos[3]<0.5) then thrust = 0 end    
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



