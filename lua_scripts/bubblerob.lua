sim = require 'sim'
simROS2 = require 'simROS2'

function sysCall_init()
    robotHandle = sim.getObject('..')
    leftMotor = sim.getObject("../LeftMotor") -- Handle of the left motor
    rightMotor = sim.getObject("../RightMotor") -- Handle of the right motor
    noseSensor = sim.getObject("../SensingNose") -- Handle of the proximity sensor
    drawingCont = sim.addDrawingObject(sim.drawing_linestrip + sim.drawing_cyclic, 2, 0, -1, 200, {1, 1, 0}, nil, nil, {1, 1, 0})

    -- Launch the ROS2 client application:
    local sysTime = math.floor(sim.getSystemTime() * 1000) 
    local leftMotorTopicName = 'leftMotorSpeed'..sysTime -- we add a random component so that we can have several instances of this robot running
    local rightMotorTopicName = 'rightMotorSpeed'..sysTime -- we add a random component so that we can have several instances of this robot running
    local sensorTopicName = 'sensorTrigger'..sysTime -- we add a random component so that we can have several instances of this robot running
    local simulationTimeTopicName = 'simTime'..sysTime -- we add a random component so that we can have several instances of this robot running

    -- Prepare the sensor publisher and the motor speed subscribers:
    sensorPub = simROS2.createPublisher('/'..sensorTopicName, 'std_msgs/msg/Bool')
    simTimePub = simROS2.createPublisher('/'..simulationTimeTopicName, 'std_msgs/msg/Float32')
    leftMotorSub = simROS2.createSubscription('/'..leftMotorTopicName, 'std_msgs/msg/Float32', 'setLeftMotorVelocity_cb')
    rightMotorSub = simROS2.createSubscription('/'..rightMotorTopicName, 'std_msgs/msg/Float32', 'setRightMotorVelocity_cb')

    -- Create a publisher for the robot's position
    rbtPosPub = simROS2.createPublisher('/rbt_pos', 'geometry_msgs/msg/Point')

    -- Start the client application:
    result = sim.launchExecutable('ros2BubbleRob', leftMotorTopicName.." "..rightMotorTopicName.." "..sensorTopicName.." "..simulationTimeTopicName, 0)
end

function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(leftMotor, msg.data)
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(rightMotor, msg.data)
end

function getTransformStamped(objHandle, name, relTo, relToName)
    t = sim.getSystemTime()
    p = sim.getObjectPosition(objHandle, relTo)
    o = sim.getObjectQuaternion(objHandle, relTo)
    return {
        header = {
            stamp = {sec = math.floor(t), nanosec = (t - math.floor(t)) * 10^9},
            frame_id = relToName
        },
        child_frame_id = name,
        transform = {
            translation = {x = p[1], y = p[2], z = p[3]},
            rotation = {x = o[1], y = o[2], z = o[3], w = o[4]}
        }
    }
end

function sysCall_sensing() 
    local p = sim.getObjectPosition(robotHandle)
    sim.addDrawingObjectItem(drawingCont, p)
end 

function sysCall_actuation()
    -- Send an updated sensor and simulation time message, and send the transform of the robot:
    local result = sim.readProximitySensor(noseSensor)
    local detectionTrigger = {}
    detectionTrigger['data'] = result > 0
    simROS2.publish(sensorPub, detectionTrigger)
    simROS2.publish(simTimePub, {data = sim.getSimulationTime()})

    -- Send the robot's transform:
    simROS2.sendTransform(getTransformStamped(robotHandle, 'ros2InterfaceControlledBubbleRob', -1, 'world'))

    -- Publish the robot's current position on /rbt_pos
    local pos = sim.getObjectPosition(robotHandle, -1)  -- Get the position in the world frame
    local positionMsg = {x = pos[1], y = pos[2], z = pos[3]}
    simROS2.publish(rbtPosPub, positionMsg)
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    simROS2.shutdownPublisher(sensorPub)
    simROS2.shutdownSubscription(leftMotorSub)
    simROS2.shutdownSubscription(rightMotorSub)
    simROS2.shutdownPublisher(rbtPosPub)
end
