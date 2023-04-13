function sysCall_init()
    math.randomseed(os.time()) -- initialize random number generator with current timee
    math.random()
    math.random()
    math.random()
    origin_pos = {0.0, 0.0}    -- initialize origin position to (0, 0)
    corout=coroutine.create(coroutineMain)   -- create a coroutine to run the main function
end


 

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

 
 

function moveToPoseCallback(q,velocity,accel,auxData)   -- Callback function for moveToPose() function
    sim.setObjectPose(auxData.target,-1,q)
    simIK.applyIkEnvironmentToScene(auxData.ikEnv,auxData.ikGroup)
end

 

function moveToPose_viaIK(maxVelocity,maxAcceleration,maxJerk,targetQ,auxData)   -- Move to a pose using inverse kinematics
    local currentQ=sim.getObjectPose(auxData.tip,-1)
    return sim.moveToPose(-1,currentQ,maxVelocity,maxAcceleration,maxJerk,targetQ,moveToPoseCallback,auxData,nil)
end

 
-- Callback function for moveToConfig() function
function moveToConfigCallback(config,velocity,accel,auxData)
    for i=1,#auxData.joints,1 do
        local jh=auxData.joints[i]
        if sim.getJointMode(jh)==sim.jointmode_force and sim.isDynamicallyEnabled(jh) then
            sim.setJointTargetPosition(jh,config[i])
        else    
            sim.setJointPosition(jh,config[i])
        end
    end
end

 
-- Move to a configuration using forward kinematics
function moveToConfig_viaFK(maxVelocity,maxAcceleration,maxJerk,goalConfig,auxData)
    local startConfig={}
    for i=1,#auxData.joints,1 do
        startConfig[i]=sim.getJointPosition(auxData.joints[i])
    end
    sim.moveToConfig(-1,startConfig,nil,nil,maxVelocity,maxAcceleration,maxJerk,goalConfig,nil,moveToConfigCallback,auxData,nil)
end

 

function coroutineMain()
    
    -- cube_m handle is 2.54cm 1inch cube
    --
    --print(sim.getStringParam(sim.stringparam_scene_path))

    tmp_j = sim.getObjectHandle('BaxterGripper_closeJoint')
    print(tmp_j)
    gripperHandle = sim.getObjectHandle('BaxterGripper')
    simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObjectHandle('UR5_joint'..i)
    end
    
    
    vel=180
    accel=40
    jerk=80
    slow_vel = 90
    slow_accel = 20
    slow_jerk = 40
    maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    slow_maxVel={slow_vel*math.pi/180,slow_vel*math.pi/180,slow_vel*math.pi/180,slow_vel*math.pi/180,slow_vel*math.pi/180,slow_vel*math.pi/180}
    slow_maxAccel={slow_accel*math.pi/180,slow_accel*math.pi/180,slow_accel*math.pi/180,slow_accel*math.pi/180,slow_accel*math.pi/180,slow_accel*math.pi/180}
    slow_maxJerk={slow_jerk*math.pi/180,slow_jerk*math.pi/180,slow_jerk*math.pi/180,slow_jerk*math.pi/180,slow_jerk*math.pi/180,slow_jerk*math.pi/180}
    ikMaxVel={0.4,0.4,0.4,1.8}
    ikMaxAccel={0.8,0.8,0.8,0.9}
    ikMaxJerk={0.6,0.6,0.6,0.8}
    
    ikMaxVel_push={0.1,0.1,0.1,0.1}
    ikMaxAccel_push={0.2,0.2,0.2,0.2}
    ikMaxJerk_push={0.1,0.1,0.1,0.1}
    simTip=sim.getObjectHandle('UR5_tip')
    simTarget=sim.getObjectHandle('UR5_target')
    modelBase=sim.getObjectHandle(sim.handle_self)
    gripperHandle=sim.getObjectHandle('BaxterGripper')
    
    ikEnv=simIK.createEnvironment()
    
    -- Prepare the ik group, using the convenience function 'simIK.addIkElementFromScene':
    ikGroup=simIK.createIkGroup(ikEnv)
    simIK.addIkElementFromScene(ikEnv,ikGroup,modelBase,simTip,simTarget,simIK.constraint_pose)
    data={}
    data.ikEnv=ikEnv
    data.ikGroup=ikGroup
    data.tip=simTip
    data.target=simTarget
    data.joints=simJoints

    

    math.randomseed(os.time()) -- random initialize
    math.random()
    math.random()
    math.random()
        
    origin_coord = {0.0935, 1.025}
    
    cylinder_handle_1 = sim.getObject("/cube_m", {})  -- get object handle
    cylinder_handle_2 = sim.getObject("/cube_m1",{})


    
    random_x1 = origin_coord[1]-0.1+math.random()/5 --  calculate random x, y positions
    random_x2 = origin_coord[1]-0.1+math.random()/5
    random_y1 = origin_coord[2]-0.1+math.random()/5
    random_y2 = origin_coord[2]-0.1+math.random()/5
    random_ori1 = math.random()*math.pi
    random_ori2 = math.random()*math.pi
    
    pose1 = sim.getObjectPosition(cylinder_handle_1, -1)
    pose2 = sim.getObjectPosition(cylinder_handle_2, -1)
    
    sim.setObjectPosition(cylinder_handle_1, -1, {random_x1, random_y1, pose1[3]}) -- reset object's position
    sim.setObjectPosition(cylinder_handle_2, -1, {random_x2, random_y2, pose2[3]})
    sim.setObjectOrientation(cylinder_handle_1, -1, {0,0,random_ori1})
    
    pos1 = sim.getObjectPosition(cylinder_handle_1, -1)
    pos2 = sim.getObjectPosition(cylinder_handle_2, -1)
    
    x_dist = pos1[1]-pos2[1] -- calculate x, y distance
    y_dist = pos1[2]-pos2[2]
    
    direct_distance = math.sqrt((pos1[1]-pos2[1])^2 + (pos1[2]-pos2[2])^2)
    
    print("this is the direct distance", direct_distance)
    
    if direct_distance < 0.06 then
        print("Cylinders inside range to grasp at once,  no push is needed")
    else
        angle = math.atan2(y_dist, x_dist) -- calculate direction
        print(angle)
        
        
        print("this is sin", math.sin(angle))
        y_delta = 0.03*math.sin(angle)
        x_delta = 0.03*math.cos(angle)
        
        print("x_delta", x_delta)
        print("y_delta", y_delta)
        
        pose = sim.getObjectPose(data.tip, -1)
        pose[1] = pos1[1]+x_delta
        pose[2] = pos1[2]+y_delta
        moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, pose, data)
        
        
        
        curr_angle = sim.getJointPosition(simJoints[6])
        print(curr_angle)
        sim.setJointTargetPosition(simJoints[6], -math.pi/2+(curr_angle-angle)) 
        
        
        
        pre = sim.getJointPosition(tmp_j)
        print("this is position", pre)
        while pre > -1 and sim.getJointPosition(tmp_j) < 0.084 do
            sim.setJointTargetPosition(tmp_j, pre+0.0012)
            pre = pre+0.0012
            sim.wait(0.01)
        end
        
        pose = sim.getObjectPose(data.tip, -1)
        pose[3] = pose[3]-0.145
        moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, pose, data)
        
        pose = sim.getObjectPose(data.tip, -1)
        pose[1] = pos2[1]+x_delta*2
        pose[2] = pos2[2]+y_delta*2
        moveToPose_viaIK(ikMaxVel_push, ikMaxAccel_push, ikMaxJerk_push, pose, data)
        
        pose = sim.getObjectPose(data.tip, -1)
        pose[3] = pose[3]+0.145
        moveToPose_viaIK(ikMaxVel, ikMaxAccel, ikMaxJerk, pose, data)
        
    end
    
    
    
    
end