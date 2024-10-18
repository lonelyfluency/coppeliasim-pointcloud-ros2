sim = require 'sim'
simVision = require 'simVision'
simROS2 = require 'simROS2'

function sysCall_init()
    visionSensorHandles = {}
    for i = 1, 4, 1 do
        visionSensorHandles[i] = sim.getObject('../sensor', {index = i - 1})
    end
    rotJointHandle = sim.getObject('../rotJoint')
    ptCloudHandle = sim.getObject('../ptCloud')
    
    frequency = 5 -- 5 Hz
    options = 2 + 8 -- Options for point cloud display and data
    pointSize = 2
    coloring_closeAndFarDistance = {1, 4}
    displayScaling = 0.999 -- Adjust to prevent points disappearing in objects
    
    -- Initialize Velodyne HDL64E sensor
    h = simVision.createVelodyneHDL64E(visionSensorHandles, frequency, options, pointSize, coloring_closeAndFarDistance, displayScaling, ptCloudHandle)

    -- ROS2 PointCloud2 publisher for the /pc0 topic
    pc_pub = simROS2.createPublisher('/pc0', 'sensor_msgs/msg/PointCloud2')
    sim.addLog(sim.verbosity_msgs, "PC ready and publishing to /pc0")
end

-- Helper function to convert float32 data into uint8 array (byte stream)
function float32ToUint8Array(floatTable)
    local uint8Array = {}
    local packedFloats = sim.packFloatTable(floatTable) -- Pack the floats into a binary stream
    for i = 1, #packedFloats do
        uint8Array[i] = string.byte(packedFloats, i) -- Convert each byte into uint8
    end
    return uint8Array
end

function sysCall_actuation()
    -- Rotate the joint to simulate Velodyne HDL64E scanning
    p = sim.getJointPosition(rotJointHandle)
    p = p + sim.getSimulationTimeStep() * frequency * math.pi * 2
    sim.setJointPosition(rotJointHandle, p)
end

function sysCall_sensing()
    -- Handle Velodyne HDL64E point cloud data
    data = simVision.handleVelodyneHDL64E(h + sim.handleflag_abscoords, sim.getSimulationTimeStep())

    -- Display detected points in point cloud
    if ptCloud then
        sim.removePointsFromPointCloud(ptCloud, 0, nil, 0)
    else
        ptCloud = sim.createPointCloud(0.02, 20, 0, pointSize)
    end
    sim.insertPointsIntoPointCloud(ptCloud, 0, data)

    -- Prepare PointCloud2 ROS message
    local pointCloud2Msg = {
        header = {
            stamp = simROS2.getTime(),
            frame_id = "sensor_frame"
        },
        height = 1,
        width = #data / 3, -- 3 coordinates per point (x, y, z)
        fields = {
            {name = "x", offset = 0, datatype = 7, count = 1},
            {name = "y", offset = 4, datatype = 7, count = 1},
            {name = "z", offset = 8, datatype = 7, count = 1}
        },
        is_bigendian = false,
        point_step = 12, -- 3 fields of 4 bytes each (float32)
        row_step = 12 * #data / 3,
        data = float32ToUint8Array(data), -- Convert float32 table to uint8 array
        is_dense = true
    }

    -- Publish point cloud data on /pc0
    simROS2.publish(pc_pub, pointCloud2Msg)
end

function sysCall_cleanup()
    -- Clean up Velodyne HDL64E sensor
    simVision.destroyVelodyneHDL64E(h)
end
