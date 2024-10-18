sim=require'sim'
simVision=require'simVision'
simROS2=require'simROS2'

function sysCall_init()
    visionSensorHandles={}
    for i=1,4,1 do
        visionSensorHandles[i]=sim.getObject('../sensor',{index=i-1})
    end
    ptCloudHandle=sim.getObject('../ptCloud')
    frequency=5 -- 5 Hz
    options=2+8 -- bit0 (1)=do not display points, bit1 (2)=display only current points, bit2 (4)=returned data is polar (otherwise Cartesian), bit3 (8)=displayed points are emissive
    pointSize=2
    coloring_closeAndFarDistance={1,4}
    displayScaling=0.999 -- so that points do not appear to disappear in objects
    h=simVision.createVelodyneVPL16(visionSensorHandles,frequency,options,pointSize,coloring_closeAndFarDistance,displayScaling,ptCloudHandle)
    pc_pub = simROS2.createPublisher('/pc1', 'sensor_msgs/msg/PointCloud2')
    sim.addLog(sim.verbosity_msgs,"PC ready.")
end

-- Helper function to convert float32 data into uint8 array (byte stream)
function float32ToUint8Array(floatTable)
    local uint8Array = {}
    local packedFloats = sim.packFloatTable(floatTable)  -- Pack the floats into a binary stream
    for i = 1, #packedFloats do
        uint8Array[i] = string.byte(packedFloats, i)  -- Convert each byte into a uint8
    end
    return uint8Array
end

function sysCall_sensing()
    data = simVision.handleVelodyneVPL16(h+sim.handleflag_abscoords, sim.getSimulationTimeStep())

    -- if we want to display the detected points ourselves:
    if ptCloud then
        sim.removePointsFromPointCloud(ptCloud, 0, nil, 0)
    else
        ptCloud = sim.createPointCloud(0.02, 20, 0, pointSize)
    end
    sim.insertPointsIntoPointCloud(ptCloud, 0, data)

    -- Convert point cloud data into sensor_msgs/PointCloud2 format
    local pointCloud2Msg = {
        header = {
            stamp = simROS2.getTime(),
            frame_id = "sensor_frame"
        },
        height = 1,
        width = #data / 3, -- 3 coordinates per point (x, y, z)
        fields = {
            { name = "x", offset = 0, datatype = 7, count = 1 },
            { name = "y", offset = 4, datatype = 7, count = 1 },
            { name = "z", offset = 8, datatype = 7, count = 1 }
        },
        is_bigendian = false,
        point_step = 12, -- 3 fields of 4 bytes each (float32)
        row_step = 12 * #data / 3,
        data = float32ToUint8Array(data), -- Convert float32 table to uint8 array
        is_dense = true
    }

    simROS2.publish(pc_pub, pointCloud2Msg)
end

function sysCall_cleanup()
    simVision.destroyVelodyneVPL16(h)
end
