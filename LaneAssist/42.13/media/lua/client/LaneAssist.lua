-- =========================================================
-- SHARED STATE (Bridge between Logic and UI)
-- =========================================================
LaneAssistData = {
    isEnabled = true,
    currentAngle = 0,
    targetAngle = 0,
    isActive = false,
    cooldown = 0
}

-- =========================================================
-- CONFIGURATION
-- =========================================================
local KEY_TOGGLE = 56 -- Left Alt
local COOLDOWN_TICKS = 60 -- 1 Second grace period after manual steering

-- == ROAD ANGLES ==
local TARGET_ANGLES = {
    0.0,
    90.0,
    180.0,
    -90.0, -- Cardinals
    26.56,
    -26.56,
    153.43,
    -153.43, -- Standard Iso (2:1)
    45.0,
    -45.0,
    135.0,
    -135.0, -- Perfect Diagonals (1:1)
    63.43,
    -63.43,
    116.57,
    -116.57 -- Steep Iso (1:2)
}

-- =========================================================
-- 1. THE LOGIC (PI CONTROLLER)
-- =========================================================
-- Two vector objects: one for direction, one for movement
local _tempVector = org.joml.Vector3f.new()
local _tempVelocity = org.joml.Vector3f.new()

local fieldCache = {}
local accumulatedError = 0
local lastVehicleId = -1
local lastToggleTime = 0

local function getCachedField(object, fieldName)
    if not object then
        return nil
    end
    local cacheKey = fieldName
    if fieldCache[cacheKey] then
        return fieldCache[cacheKey]
    end
    if getNumClassFields(object) then
        for i = 0, getNumClassFields(object) - 1 do
            local field = getClassField(object, i)
            if field and field:getName() == fieldName then
                field:setAccessible(true)
                fieldCache[cacheKey] = field
                return field
            end
        end
    end
    return nil
end

local function setClientSteering(controller, value)
    if not controller then
        return
    end
    local controlsField = getCachedField(controller, "clientControls")
    if not controlsField then
        return
    end
    local clientControls = getClassFieldVal(controller, controlsField)
    if not clientControls then
        return
    end
    local steeringField = getCachedField(clientControls, "steering")
    if steeringField then
        steeringField:setFloat(clientControls, value)
    end
end

local function getClosestTargetAngle(currentAngle)
    local bestAngle = TARGET_ANGLES[1]
    local minDiff = 360.0
    for _, target in ipairs(TARGET_ANGLES) do
        local diff = math.abs(target - currentAngle)
        if diff > 180 then
            diff = 360 - diff
        end
        if diff < minDiff then
            minDiff = diff
            bestAngle = target
        end
    end
    return bestAngle
end

local function onVehicleUpdate()
    local player = getPlayer()
    if not player then
        return
    end
    local vehicle = player:getVehicle()

    LaneAssistData.isActive = (vehicle and vehicle:getDriver() == player and vehicle:isEngineRunning())

    if not LaneAssistData.isActive then
        accumulatedError = 0
        return
    end

    -- 1. TOGGLE CHECK
    local currentTime = getTimestampMs()
    if isKeyDown(KEY_TOGGLE) and (currentTime - lastToggleTime > 500) then
        LaneAssistData.isEnabled = not LaneAssistData.isEnabled
        lastToggleTime = currentTime
        local status = LaneAssistData.isEnabled and "ON" or "OFF"
        player:Say("Lane Assist: " .. status)
    end

    if not LaneAssistData.isEnabled then
        accumulatedError = 0
        return
    end

    -- 2. RESET & KEY INPUT CHECKS
    local currentVehicleId = vehicle:getId()
    if currentVehicleId ~= lastVehicleId then
        accumulatedError = 0
        lastVehicleId = currentVehicleId
    end

    -- Manual Steering Check (A/D/Left/Right)
    -- This sets the Cooldown (Temporary Disable)
    if
        isKeyDown(Keyboard.KEY_A) or isKeyDown(Keyboard.KEY_D) or isKeyDown(Keyboard.KEY_LEFT) or
            isKeyDown(Keyboard.KEY_RIGHT)
     then
        accumulatedError = 0
        LaneAssistData.cooldown = COOLDOWN_TICKS
        return
    end

    -- Cooldown Timer
    if LaneAssistData.cooldown > 0 then
        LaneAssistData.cooldown = LaneAssistData.cooldown - 1
        return
    end

    -- 3. PHYSICS CHECKS (Speed & Direction)
    local speed = math.abs(vehicle:getCurrentSpeedKmHour())
    if speed < 5 then
        accumulatedError = 0
        return
    end

    -- Direction Check (Forward vs Backward)
    vehicle:getForwardVector(_tempVector) -- The direction the nose is pointing
    vehicle:getLinearVelocity(_tempVelocity) -- The direction the car is moving

    -- Calculate Dot Product of X/Z components (ignoring Y height)
    -- Dot > 0 means moving generally forward
    -- Dot < 0 means moving generally backward
    local dotProduct = (_tempVector:x() * _tempVelocity:x()) + (_tempVector:z() * _tempVelocity:z())

    if dotProduct < 0 then
        -- We are rolling backwards! Disable assist.
        accumulatedError = 0
        return
    end

    -- 4. ANGLE CALCULATIONS
    local currentAngle = math.deg(math.atan2(_tempVector:z(), _tempVector:x()))
    local targetAngle = getClosestTargetAngle(currentAngle)

    LaneAssistData.currentAngle = currentAngle
    LaneAssistData.targetAngle = targetAngle

    local angleError = targetAngle - currentAngle
    angleError = (angleError + 180) % 360 - 180

    -- 5. PID CONTROLLER
    local speedDamping = 1.0
    if speed > 20 then
        speedDamping = 20 / speed
    end

    accumulatedError = accumulatedError + (angleError * speedDamping)

    local MAX_INTEGRAL = 5.0
    if accumulatedError > MAX_INTEGRAL then
        accumulatedError = MAX_INTEGRAL
    end
    if accumulatedError < -MAX_INTEGRAL then
        accumulatedError = -MAX_INTEGRAL
    end

    local Kp = 0.08
    local Ki = 0.005
    local pTerm = angleError * Kp * speedDamping
    local iTerm = accumulatedError * Ki * speedDamping
    local rawOutput = pTerm + iTerm

    local DEADZONE_OFFSET = 0.11
    local finalSteering = 0
    if rawOutput > 0 then
        finalSteering = rawOutput + DEADZONE_OFFSET
    elseif rawOutput < 0 then
        finalSteering = rawOutput - DEADZONE_OFFSET
    end

    local maxSteerAtSpeed = 1.0
    if speed > 30 then
        maxSteerAtSpeed = 35 / speed
    end
    if finalSteering > maxSteerAtSpeed then
        finalSteering = maxSteerAtSpeed
    end
    if finalSteering < -maxSteerAtSpeed then
        finalSteering = -maxSteerAtSpeed
    end

    -- 6. APPLY OUTPUT
    if math.abs(angleError) > 0.05 then
        local controller = vehicle:getController()
        setClientSteering(controller, finalSteering)
    else
        accumulatedError = accumulatedError * 0.9
    end
end

Events.OnPlayerUpdate.Add(onVehicleUpdate)

-- =========================================================
-- 2. THE UI HUD (COMPASS OVERLAY)
-- =========================================================

LaneAssistOverlay = ISUIElement:derive("LaneAssistOverlay")

function LaneAssistOverlay:new()
    local o = {}
    o = ISUIElement:new(0, 0, 0, 0)
    setmetatable(o, self)
    self.__index = self
    o.width = 1
    o.height = 1
    o.anchorLeft = true
    o.anchorTop = true
    return o
end

function LaneAssistOverlay:prerender()
    if not LaneAssistData.isActive then
        return
    end

    local player = getPlayer()
    if not player then
        return
    end

    -- UI Coordinates
    local x = player:getX()
    local y = player:getY()
    local z = player:getZ()
    local sx = IsoUtils.XToScreen(x, y, z + 1.5, 0)
    local sy = IsoUtils.YToScreen(x, y, z + 1.5, 0)

    local core = getCore()
    local zoom = core:getZoom(0)
    sx = (sx - IsoCamera.getOffX()) / zoom
    sy = (sy - IsoCamera.getOffY()) / zoom

    local scale = 1 / zoom
    if scale < 0.5 then
        scale = 0.5
    end

    -- DRAW UI
    local radius = 30 * scale
    local borderThick = 3 * scale

    -- Status Color
    local r, g, b = 1, 0, 0 -- Red (Off)
    if LaneAssistData.isEnabled then
        if LaneAssistData.cooldown > 0 then
            r, g, b = 1, 0.5, 0 -- Orange (Manual Override/Cooldown)
        else
            r, g, b = 0, 1, 0 -- Green (Active)
        end
    end

    -- Border & Background
    self:drawRect(
        sx - radius - borderThick,
        sy - radius - borderThick,
        (radius * 2) + (borderThick * 2),
        (radius * 2) + (borderThick * 2),
        0.8,
        r,
        g,
        b
    )
    self:drawRect(sx - radius, sy - radius, radius * 2, radius * 2, 0.5, 0.05, 0.05, 0.05)

    -- Center Dot & White Car Line
    self:drawRect(sx - 2, sy - 2, 4, 4, 1, 1, 1, 1)
    local lineW = 3 * scale
    self:drawRect(sx - (lineW / 2), sy - radius, lineW, radius, 0.9, 1, 1, 1)

    -- Target Marker
    if LaneAssistData.isEnabled then
        local diff = LaneAssistData.targetAngle - LaneAssistData.currentAngle
        diff = (diff + 180) % 360 - 180

        -- Blue if locked, Yellow if adjusting
        local tr, tg, tb = 1, 1, 0
        if math.abs(diff) < 0.5 then
            tr, tg, tb = 0, 1, 1
        end

        local rad = math.rad(diff - 90)
        local markerSize = 8 * scale
        local markerX = sx + (math.cos(rad) * radius)
        local markerY = sy + (math.sin(rad) * radius)

        self:drawRect(markerX - (markerSize / 2), markerY - (markerSize / 2), markerSize, markerSize, 1, tr, tg, tb)
    end
end

local function createLaneAssistUI()
    local overlay = LaneAssistOverlay:new()
    overlay:addToUIManager()
end

Events.OnGameStart.Add(createLaneAssistUI)
