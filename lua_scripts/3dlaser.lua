sim=require'sim'
simUI=require'simUI'
simROS2=require'simROS2'

function sysCall_init()
    model=sim.getObject('..')
    laserHandle=sim.getObject("../sensor")
    joint0Handle=sim.getObject("../joint0")
    joint1Handle=sim.getObject("../joint1")
    red={1,0,0}
    points=sim.addDrawingObject(sim.drawing_spherepts,0.01,0,-1,100000,nil,nil,nil,red)
    horizontalScanningAngle=90*math.pi/180
    verticalScanningAngle=90*math.pi/180

    -- Create a ROS2 publisher for PointCloud2 on topic '/pc2'
    pc_pub = simROS2.createPublisher('/pc2', 'sensor_msgs/msg/PointCloud2')
    sim.addLog(sim.verbosity_msgs, "PointCloud2 Publisher ready on /pc2")
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
    local s=sim.getObjectSel()
    local show=(s and #s==1 and s[1]==model)
    if show then
        if not ui then
            local xml =[[<ui title="xxxx" closeable="false" placement="relative" layout="form">
                    <label id="1" text=""/>
                    <hslider id="2" on-change="angleHMoved"/>
                    <label id="3" text=""/>
                    <hslider id="4" on-change="angleVMoved"/>
            </ui>]]
            ui=simUI.create(xml)
            if uiPos then
                simUI.setPosition(ui,uiPos[1],uiPos[2])
            else
                uiPos={}
                uiPos[1],uiPos[2]=simUI.getPosition(ui)
            end
            simUI.setTitle(ui,sim.getObjectAlias(model,1))
            simUI.setLabelText(ui,1,'Horizontal angle ('..math.floor(horizontalScanningAngle*180/math.pi+0.5)..' deg)')
            simUI.setSliderValue(ui,2,(horizontalScanningAngle*180/math.pi-10)/1.7)
            simUI.setLabelText(ui,3,'Vertical angle ('..math.floor(verticalScanningAngle*180/math.pi+0.5)..' deg)')
            simUI.setSliderValue(ui,4,(verticalScanningAngle*180/math.pi-10)/1.7)
        end
    else
        if ui then
            uiPos[1],uiPos[2]=simUI.getPosition(ui)
            simUI.destroy(ui)
            ui=nil
        end
    end

    sim.addDrawingObjectItem(points,nil)
    
    dv=verticalScanningAngle/31
    dh=horizontalScanningAngle/31
    pv=-verticalScanningAngle/2
    local pointCloudData = {}
    
    for y=0,31,1 do
        sim.setJointPosition(joint0Handle,pv)
        ph=-horizontalScanningAngle/2
        for x=0,31,1 do
            sim.setJointPosition(joint1Handle,ph)
            local m=sim.getObjectMatrix(laserHandle)
            local res,dist,pt=sim.handleProximitySensor(laserHandle)
            if (res==1) then
                pt=sim.multiplyVector(m,pt)
                sim.addDrawingObjectItem(points,pt)

                -- Store point cloud data (x, y, z coordinates)
                pointCloudData[#pointCloudData+1] = pt[1]
                pointCloudData[#pointCloudData+1] = pt[2]
                pointCloudData[#pointCloudData+1] = pt[3]
            end
            ph=ph+dh
        end
        pv=pv+dv
    end
    
    -- Prepare PointCloud2 message
    local pointCloud2Msg = {
        header = {
            stamp = simROS2.getTime(),
            frame_id = "sensor_frame"
        },
        height = 1,
        width = #pointCloudData / 3, -- 3 coordinates per point (x, y, z)
        fields = {
            { name = "x", offset = 0, datatype = 7, count = 1 },
            { name = "y", offset = 4, datatype = 7, count = 1 },
            { name = "z", offset = 8, datatype = 7, count = 1 }
        },
        is_bigendian = false,
        point_step = 12, -- 3 fields of 4 bytes each (float32)
        row_step = 12 * #pointCloudData / 3,
        data = float32ToUint8Array(pointCloudData), -- Convert float32 table to uint8 array
        is_dense = true
    }

    -- Publish the PointCloud2 message
    simROS2.publish(pc_pub, pointCloud2Msg)
end 

function angleHMoved(ui,id,v)
    horizontalScanningAngle=math.pi*(10+1.7*v)/180
    simUI.setLabelText(ui,1,'Horizontal angle ('..math.floor(horizontalScanningAngle*180/math.pi+0.5)..')')
end

function angleVMoved(ui,id,v)
    verticalScanningAngle=math.pi*(10+1.7*v)/180
    simUI.setLabelText(ui,3,'Vertical angle ('..math.floor(verticalScanningAngle*180/math.pi+0.5)..')')
end
