-- This function is called once at the beginning of the simulation and initializes the joint handles and PID parameter.
function sysCall_init()
    c=sim.getObjectHandle('BaxterGripper_centerJoint')
    a=sim.getObjectHandle('BaxterGripper_closeJoint')
    PID_P=0.1
end

-- This function is called continuously during simulation for the joints specified in the custom settings of a joint.
-- It implements a simple proportional controller to keep the gripper centered.
function sysCall_jointCallback(inData)
    local cv=sim.getJointPosition(c)
    local av=sim.getJointPosition(a)
    local errorValue=-(av/2)-cv

    -- Calculate the control signal as the error multiplied by the PID gain.
    local ctrl=errorValue*PID_P

    -- Calculate the maximum velocity that can be used for the current control signal.
    local maxVelocity=ctrl/inData.dynStepSize
    if (maxVelocity>inData.velUpperLimit) then
        maxVelocity=inData.velUpperLimit
    end
    if (maxVelocity<-inData.velUpperLimit) then
        maxVelocity=-inData.velUpperLimit
    end

    -- Set the maximum force/torque to be applied to the joint.
    local forceOrTorqueToApply=inData.maxForce

    -- Pack the output data into a table and return it.
    local outData={}
    outData.velocity=maxVelocity
    outData.force=forceOrTorqueToApply
    return outData
end

