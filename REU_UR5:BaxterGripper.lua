function sysCall_init() 
    gripperHandle=sim.getObjectHandle('BaxterGripper')
    objectName=sim.getObjectName(gripperHandle)

    motorHandle=sim.getObjectHandle('BaxterGripper_closeJoint')
    -- the 'BaxterGripper_centerJoint' is handled by a joint callback script (i.e. custom position controller)
    openedGap=0.075
    closedGap=-0.01
    interval={0.075-openedGap,openedGap-closedGap}
    sim.setJointInterval(motorHandle,false,interval)
end
    -- Do following to activate/deactivate the gripper from another location. For example:
    --
    -- gripperName='BaxterGripper#1' -- specify the full name. If the full name is "BaxterGripper", specify "BaxterGripper#"
    -- sim.setInt32Signal(gripperName..'_close',1) -- close
    -- sim.setInt32Signal(gripperName..'_close',0) -- open
    --
    -- See the end of the script for instructions on how to do efficient grasping


function sysCall_cleanup() 
 
end 

function sysCall_actuation()
    --print(sim.getJointForce(motorHandle))
    --[[close=sim.getInt32Signal(objectName..'_close')
    
    if (close==1) then
        sim.setJointTargetVelocity(motorHandle,0.005)
    else
        sim.setJointTargetVelocity(motorHandle,-0.005)
    end]]--
    
    -- You have basically 2 alternatives to grasp an object:
    --
    -- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
    --    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
    --    coefficients, object masses and inertias)
    --
    -- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
    --    much easier and offers very stable results.
end 
