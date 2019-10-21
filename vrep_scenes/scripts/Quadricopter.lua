function sysCall_init() 
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    version = sim.getInt32Parameter(sim.intparam_program_version)
    if (version < 20413) then
        sim.displayDialog('Warning',
        'The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!', 
        sim.dlgstyle_ok, false, '', nil, {0.8, 0, 0, 0, 0, 0})
    end

    -- Detatch the manipulation sphere:
    targetObj = sim.getObjectHandle('Quadricopter_target')
    sim.setObjectParent(targetObj, -1, true)

    -- Get the object hangle for the base of the quadricopter
    quadricopterBase = sim.getObjectHandle('Quadricopter_base')

    particlesAreVisible = sim.getScriptSimulationParameter(sim.handle_self, 'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree, 'particlesAreVisible', tostring(particlesAreVisible))
    simulateParticles = sim.getScriptSimulationParameter(sim.handle_self, 'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree, 'simulateParticles', tostring(simulateParticles))

    propellerScripts = {-1, -1, -1, -1}
    for i = 1, 4, 1 do
        propellerScripts[i] = sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    quadricopter = sim.getObjectAssociatedWithScript(sim.handle_self)

    particlesTargetVelocities = {0, 0, 0, 0}

    -- parameters for vertical control
    positionParameter = 2 -- set the thrust wrt to the position of target and quadricopter
    increaseParameter = 0.001 -- increase the thrust depending on the deltaPos, results in overshooting if set to a high value
    dynamicParameter = 0.001 -- similar to positionParameter, increase or decrease the thrust if the deltaPos increases or decreases
    velocityParameter = -2 -- how thrust depends on the current velocity
    cumulativeDistance = 0 -- cumulative distance between target position and current position
    lastDeltaPos = 0 -- previous value of deltaPos, simulation begins with both target and quad at same position so it is initialized 0

    -- parameters for horizontal control
    pAlphaE = 0
    pBetaE = 0
    psp2 = 0
    psp1 = 0

    -- parameters for rotational control
    prevEuler = 0


    fakeShadow = sim.getScriptSimulationParameter(sim.handle_self, 'fakeShadow')
    if (fakeShadow) then
        shadowCont = sim.addDrawingObject(sim.drawing_discpoints + sim.drawing_cyclic + sim.drawing_25percenttransparency + sim.drawing_50percenttransparency + sim.drawing_itemsizes, 0.2, 0, -1, 1)
    end

    -- Prepare 2 floating views with the camera views:
    floorCam = sim.getObjectHandle('Quadricopter_floorCamera')
    frontVisionSensor = sim.getObjectHandle('Quadricopter_frontVisionSensor')
    floorView = sim.floatingViewAdd(0.9, 0.9, 0.2, 0.2, 0)
    frontView = sim.floatingViewAdd(0.75, 0.9, 0.15, 0.2, 0)
    sim.adjustView(floorView, floorCam, 64)
    sim.adjustView(frontView, frontVisionSensor, 64)

    -- Set up
    proximitySensorLeft  =  sim.getObjectHandle("Proximity_sensor_left")
    proximitySensorRight  =  sim.getObjectHandle("Proximity_sensor_right")

    -- Check if RosInterface plugin is avaiable
    if (not pluginNotFound) then
        local sysTime = sim.getSystemTimeInMs(-1) 
        local proximitySensorLeftBoolTopicName = 'proximitySensorLeftBool'
        local proximitySensorRightBoolTopicName = 'proximitySensorRightBool'
        local proximitySensorLeftDistanceTopicName = 'proximitySensorLeftDistance'
        local proximitySensorRightDistanceTopicName = 'proximitySensorRightDistance'
        local targetPositionSendTopicName = 'gpsToROS'
        local targetPositionReceiveTopicName = 'gpsToVREP'

        proximitySensorLeftBoolPub = simROS.advertise('/'..proximitySensorLeftBoolTopicName, 'std_msgs/Bool')
        proximitySensorRightBoolPub = simROS.advertise('/'..proximitySensorRightBoolTopicName, 'std_msgs/Bool')
        proximitySensorLeftDistancePub = simROS.advertise('/'..proximitySensorLeftDistanceTopicName, 'std_msgs/Float64')
        proximitySensorRightDistancePub = simROS.advertise('/'..proximitySensorRightDistanceTopicName, 'std_msgs/Float64')
        targetPositionOrientationPub = simROS.advertise('/'..targetPositionSendTopicName, 'geometry_msgs/Pose')

        targetPositionOrientationSub = simROS.subscribe('/'..targetPositionReceiveTopicName, 'geometry_msgs/Pose', 'moveTarget')
    else
        print("<font color = '#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
end

function moveTarget(msg)
    targetSubscribedData = msg
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(floorView)
    sim.floatingViewRemove(frontView)
    simROS.shutdownPublisher(proximitySensorLeftBoolPub)
    simROS.shutdownPublisher(proximitySensorRightBoolPub)
    simROS.shutdownPublisher(proximitySensorLeftDistancePub)
    simROS.shutdownPublisher(proximitySensorRightDistancePub)
    simROS.shutdownPublisher(targetPositionOrientationPub)
    sim.setObjectParent(targetObj, quadricopterBase, false)
end 

function sysCall_actuation() 
    -- Get position of target
    targetPosition = sim.getObjectPosition(targetObj, -1)
    targetOrientation = sim.getObjectOrientation(targetObj, -1)    
    -- print(targetPosition)
    -- print(targetOrientation)
    --------------------------------PUBLISHING--------------------------------------------------
    -- Publishing left proximity sensor data
    -- sim.readProximitySensor returns 0, 1, or -1 for false, true or invalid
    -- Distance as second parameter
    local resultLeft = {sim.readProximitySensor(proximitySensorLeft)}
    local detectionTriggerLeft = {}
    detectionTriggerLeft['data'] = resultLeft[1]>0
    simROS.publish(proximitySensorLeftBoolPub, detectionTriggerLeft)
    local proximityDistanceLeft = {}
    proximityDistanceLeft['data'] = -1
    if (detectionTriggerLeft['data']) then
        proximityDistanceLeft['data'] = resultLeft[2]
    end
    simROS.publish(proximitySensorLeftDistancePub, proximityDistanceLeft)

    -- Publishing left proximity sensor data
    local resultRight = {sim.readProximitySensor(proximitySensorRight)}
    local detectionTriggerRight = {}
    detectionTriggerRight['data'] = resultRight[1]>0
    simROS.publish(proximitySensorRightBoolPub, detectionTriggerRight)
    local proximityDistanceRight = {}
    proximityDistanceRight['data'] = -1
    if (detectionTriggerRight['data']) then
        proximityDistanceRight['data'] = resultRight[2]
    end
    simROS.publish(proximitySensorRightDistancePub, proximityDistanceRight)

    --publishing target position and orientation (gps)
    targetPublishData = {}
    targetPublishData['position'] = {x = targetPosition[1], y = targetPosition[2], z = targetPosition[3]}
    targetPublishData['orientation'] = {x = targetOrientation[1], y = targetOrientation[2], z = targetOrientation[3], w = 1}
    simROS.publish(targetPositionOrientationPub, targetPublishData)
    --------------------------------------------------------------------------------------------
    

    --------------------------------SUBSCRIBING-------------------------------------------------
    if (targetSubscribedData) then
        newTargetLinear = {targetSubscribedData['position']['x'], targetSubscribedData['position']['y'], targetSubscribedData['position']['z']} 
        newTargetOrientation = {targetSubscribedData['orientation']['x'], targetSubscribedData['orientation']['y'], targetSubscribedData['orientation']['z']} 
        sim.setObjectPosition(targetObj, -1, newTargetLinear)
        sim.setObjectOrientation(targetObj, -1, newTargetOrientation)
    end

    --------------------------------------------------------------------------------------------
    
    s = sim.getObjectSizeFactor(quadricopterBase)
    
    pos = sim.getObjectPosition(quadricopterBase, -1)
    if (fakeShadow) then
        itemData = {pos[1], pos[2], 0.002, 0, 0, 1, 0.2*s}
        sim.addDrawingObjectItem(shadowCont, itemData)
    end
    
    -- Vertical control:
    targetPosition = sim.getObjectPosition(targetObj, -1)
    pos = sim.getObjectPosition(quadricopterBase, -1)
    quadricopterVelocity = sim.getVelocity(quadricopter)
    deltaPos = (targetPosition[3] - pos[3])
    cumulativeDistance = cumulativeDistance + deltaPos
    pv = positionParameter * deltaPos
    thrust = 5.335 + pv + increaseParameter*cumulativeDistance + dynamicParameter*(deltaPos - lastDeltaPos) + quadricopterVelocity[3]*velocityParameter
    lastDeltaPos = deltaPos
    
    -- Horizontal control: 
    sp = sim.getObjectPosition(targetObj, quadricopterBase) -- position of targetObj relative to quadricopterBase
    m = sim.getObjectMatrix(quadricopterBase, -1) -- tranformation matrix of quadricopterBase relative to world 
    vx = {1, 0, 0}
    vx = sim.multiplyVector(m, vx)
    vy = {0, 1, 0}
    vy = sim.multiplyVector(m, vy)
    alphaE = (vy[3] - m[12])
    alphaCorr = 0.25*alphaE + 2.1*(alphaE - pAlphaE)
    betaE = (vx[3] - m[12])
    betaCorr = -0.25*betaE - 2.1*(betaE - pBetaE)
    pAlphaE = alphaE
    pBetaE = betaE
    alphaCorr = alphaCorr + sp[2]*0.005 + 1*(sp[2] - psp2)
    betaCorr = betaCorr - sp[1]*0.005 - 1*(sp[1] - psp1)
    psp2 = sp[2]
    psp1 = sp[1]
    
    -- Rotational control:
    euler = sim.getObjectOrientation(quadricopterBase, targetObj)
    rotCorr = euler[3]*0.1 + 2*(euler[3]-prevEuler)
    prevEuler = euler[3]
    
    -- Decide of the motor velocities:
    particlesTargetVelocities[1] = thrust * (1 - alphaCorr + betaCorr + rotCorr)
    particlesTargetVelocities[2] = thrust * (1 - alphaCorr - betaCorr - rotCorr)
    particlesTargetVelocities[3] = thrust * (1 + alphaCorr - betaCorr + rotCorr)
    particlesTargetVelocities[4] = thrust * (1 + alphaCorr + betaCorr - rotCorr)
    
    -- Send the desired motor velocities to the 4 rotors:
    for i = 1, 4, 1 do
        sim.setScriptSimulationParameter(propellerScripts[i], 'particleVelocity', particlesTargetVelocities[i])
    end
end 
