function sysCall_init() 
    propeller = sim.getObjectHandle('Quadricopter_propeller1')
    propellerRespondable = sim.getObjectHandle('Quadricopter_propeller_respondable1')
    propellerJoint = sim.getObjectHandle('Quadricopter_propeller_joint1')
    type = sim.particle_roughspheres + sim.particle_respondable1to4 + sim.particle_respondable5to8 + 
        sim.particle_cyclic + sim.particle_ignoresgravity
    simulateParticles = sim.getScriptSimulationParameter(sim.handle_self, 'simulateParticles')
    particleVelocity = sim.getScriptSimulationParameter(sim.handle_self, 'particleVelocity')
    particleCountPerSecond = sim.getScriptSimulationParameter(sim.handle_self, 'particleCountPerSecond')
    particleDensity = sim.getScriptSimulationParameter(sim.handle_self, 'particleDensity')
    particleScatteringAngle = sim.getScriptSimulationParameter(sim.handle_self, 'particleScatteringAngle')
    particleLifeTime = sim.getScriptSimulationParameter(sim.handle_self, 'particleLifeTime')
    maxParticleCount = sim.getScriptSimulationParameter(sim.handle_self, 'maxParticleCount')
    if (sim.getScriptSimulationParameter(sim.handle_self, 'particlesAreVisible') == false) then
        type = type + sim.particle_invisible
    end
    maxParticleDeviation = math.tan(particleScatteringAngle * 0.5 * math.pi/180) * particleVelocity
    particleObject = nil
    is = 0 -- previous size factor
    notFullParticles = 0
    params = {2, 1, 0.2, 3, 0.4}
end

function sysCall_cleanup() 
 
end 

function sysCall_actuation()
    local t = sim.getSimulationTime()
    sim.setJointPosition(propellerJoint, t * 10)
    particleVelocity = sim.getScriptSimulationParameter(sim.handle_self, 'particleVelocity')
    s = sim.getObjectSizeFactor(propeller) -- current size factor
    if (s~ = is) then
        if (particleObject) then
            sim.removeParticleObject(particleObject)
            particleObject = nil
        end
        particleSize = sim.getScriptSimulationParameter(sim.handle_self, 'particleSize') * 0.005 * s
        is = s
    end
    ts = sim.getSimulationTimeStep()
    
    if (particleObject == nil)and(simulateParticles) then
        particleObject = sim.addParticleObject(type, particleSize, particleDensity, params, particleLifeTime, maxParticleCount, {0.3, 0.7, 1})
    end
    
    m = sim.getObjectMatrix(propeller, -1)
    particleCnt = 0
    pos = {0, 0, 0}
    dir = {0, 0, 1}
    
    requiredParticleCnt = particleCountPerSecond * ts+notFullParticles
    notFullParticles = requiredParticleCnt % 1
    requiredParticleCnt = math.floor(requiredParticleCnt)
    while (particleCnt<requiredParticleCnt) do
        -- we want a uniform distribution:
        x = (math.random()-0.5) * 2
        y = (math.random()-0.5) * 2
        if (x * x + y * y <= 1) then
            if (simulateParticles) then
                pos[1] = x * 0.08 * s
                pos[2] = y * 0.08 * s
                pos[3] = -particleSize * 0.6
                dir[1] = pos[1]+(math.random()-0.5) * maxParticleDeviation * 2
                dir[2] = pos[2]+(math.random()-0.5) * maxParticleDeviation * 2
                dir[3] = pos[3]-particleVelocity * (1+0.2 * (math.random()-0.5))
                pos = sim.multiplyVector(m, pos)
                dir = sim.multiplyVector(m, dir)
                itemData = {pos[1], pos[2], pos[3], dir[1], dir[2], dir[3]}
                sim.addParticleObjectItem(particleObject, itemData)
            end
            particleCnt = particleCnt+1
        end
    end
    -- Apply a reactive force onto the body:
    totalExertedForce = particleCnt * particleDensity * particleVelocity * math.pi * particleSize * particleSize * particleSize/(6 * ts)
    force = {0, 0, totalExertedForce}
    m[4] = 0
    m[8] = 0
    m[12] = 0
    force = sim.multiplyVector(m, force)
    torque = {0, 0, -0.002 * particleVelocity}
    torque = sim.multiplyVector(m, torque)
    sim.addForceAndTorque(propellerRespondable, force, torque)
    
end 
