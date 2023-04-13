function sysCall_init()
    -- do some initialization here
    target = sim.getObjectHandle("UR5_target")
    tip = sim.getObjectHandle("UR5_tip")
end

function sysCall_actuation()
    -- put your actuation code here
    --[[target_ori = sim.getObjectOrientation(target, -1)
    sim.setObjectOrientation(target, -1, {0,0,target_ori[3]})
    tip_ori = sim.getObjectOrientation(tip, -1)
    sim.setObjectOrientation(tip, -1, {0,0,tip_ori[3]})]]--
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

