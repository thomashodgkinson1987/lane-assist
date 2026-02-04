-- =========================================================
-- LANE ASSIST PHYSICS STEERING
-- Physics-based vehicle control without reflection
-- =========================================================

local PhysicsSteering = {}

-- =========================================================
-- CONFIGURATION
-- =========================================================
PhysicsSteering.config = {
    -- Force application parameters
    TURN_FORCE_MULTIPLIER = 20000.0, -- Base force multiplier for turning
    IMPULSE_POSITION_OFFSET = 2.5, -- Distance from vehicle center to apply force
    MIN_FORCE_THRESHOLD = 100.0, -- Minimum force to apply (prevents micro-adjustments)
    MAX_FORCE_LIMIT = 8000.0, -- Maximum safety limit for forces
    -- Speed-based scaling (exponential for better high-speed control)
    SPEED_SCALING_BASE = 0.7, -- Base scaling factor
    SPEED_SCALING_EXPONENT = 0.8, -- Exponent for speed scaling
    MIN_SPEED_FOR_FORCE = 5.0, -- Minimum speed to apply forces
    -- Damping and stability
    DAMPING_FACTOR = 0.92, -- Force reduction over time
    FORCE_INTEGRAL_LIMIT = 2000.0, -- Maximum accumulated force
    UPDATE_FREQUENCY = 1, -- Apply forces every N frames (2 = every other frame)
    -- Vehicle type scaling
    VEHICLE_MULTIPLIERS = {
        ["Car"] = 1.0,
        ["Truck"] = 1.3,
        ["Van"] = 1.2,
        ["SportsCar"] = 0.8,
        ["default"] = 1.0
    }
}

-- =========================================================
-- INTERNAL STATE
-- =========================================================
PhysicsSteering.state = {
    -- Reusable vectors for performance (avoid garbage collection)
    impulseVector = org.joml.Vector3f.new(),
    positionVector = org.joml.Vector3f.new(),
    forwardVector = org.joml.Vector3f.new(),
    rightVector = org.joml.Vector3f.new(),
    -- Force tracking for stability
    lastAppliedForce = 0.0,
    forceIntegral = 0.0,
    frameCounter = 0,
    -- Vehicle detection cache
    vehicleTypeCache = {},
    lastVehicleId = -1,
    -- Performance monitoring
    lastUpdateTime = 0,
    averageForceApplied = 0.0
}

-- =========================================================
-- VEHICLE TYPE DETECTION
-- =========================================================
local function detectVehicleType(vehicle)
    if not vehicle then
        return "default"
    end

    local vehicleId = vehicle:getId()
    local cached = PhysicsSteering.state.vehicleTypeCache[vehicleId]
    if cached then
        return cached
    end

    -- Try to determine vehicle type from script name
    local scriptName = "default"
    local script = vehicle:getScript()
    if script then
        scriptName = script:getName() or "default"
    end

    -- Classify vehicle type
    local vehicleType = "default"
    if scriptName:find("Car") or scriptName:find("Sedan") then
        vehicleType = "Car"
    elseif scriptName:find("Truck") or scriptName:find("Pickup") then
        vehicleType = "Truck"
    elseif scriptName:find("Van") or scriptName:find("Delivery") then
        vehicleType = "Van"
    elseif scriptName:find("Sports") or scriptName:find("Race") then
        vehicleType = "SportsCar"
    end

    -- Cache the result
    PhysicsSteering.state.vehicleTypeCache[vehicleId] = vehicleType
    return vehicleType
end

-- =========================================================
-- SPEED-BASED FORCE CALCULATION
-- =========================================================
local function calculateSpeedScaling(speed)
    if speed < PhysicsSteering.config.MIN_SPEED_FOR_FORCE then
        return 0.0 -- No force at very low speeds
    end

    -- Exponential scaling: better control at high speeds
    local normalizedSpeed = math.min(speed / 50.0, 1.0) -- Normalize to 0-1 (50 km/h as reference)
    local scaling =
        PhysicsSteering.config.SPEED_SCALING_BASE *
        math.pow(normalizedSpeed, PhysicsSteering.config.SPEED_SCALING_EXPONENT)

    return math.max(scaling, 0.1) -- Minimum 10% scaling
end

-- =========================================================
-- OPTIMAL IMPULSE POSITION CALCULATION
-- =========================================================
local function calculateOptimalImpulsePosition(vehicle)
    -- Get vehicle's forward direction and up vector
    vehicle:getForwardVector(PhysicsSteering.state.forwardVector)
    local upVector = org.joml.Vector3f.new()
    vehicle:getUpVector(upVector)

    -- Calculate vehicle's right vector (perpendicular to forward)
    local rightVector = org.joml.Vector3f.new()
    rightVector:set(
        -PhysicsSteering.state.forwardVector:z(),
        0.0,
        PhysicsSteering.state.forwardVector:x()
    )

    -- Apply force at front of vehicle relative to vehicle orientation
    local offset = PhysicsSteering.config.IMPULSE_POSITION_OFFSET
    PhysicsSteering.state.positionVector:set(
        PhysicsSteering.state.forwardVector:x() * offset,
        0.0, -- Keep force at ground level
        PhysicsSteering.state.forwardVector:z() * offset
    )

    -- Store right vector for force direction calculation
    PhysicsSteering.state.rightVector = rightVector

    return PhysicsSteering.state.positionVector
end

-- =========================================================
-- LATERAL FORCE CALCULATION
-- =========================================================
local function calculateLateralForce(steeringOutput, speed, vehicleType)
    -- Get vehicle-specific multiplier
    local vehicleMultiplier =
        PhysicsSteering.config.VEHICLE_MULTIPLIERS[vehicleType] or PhysicsSteering.config.VEHICLE_MULTIPLIERS["default"]

    -- Calculate speed scaling
    local speedScaling = calculateSpeedScaling(speed)

    -- Apply all scaling factors
    local baseForce = steeringOutput * PhysicsSteering.config.TURN_FORCE_MULTIPLIER
    local scaledForce = baseForce * vehicleMultiplier * speedScaling

    -- Apply safety limits
    local clampedForce =
        math.max(-PhysicsSteering.config.MAX_FORCE_LIMIT, math.min(PhysicsSteering.config.MAX_FORCE_LIMIT, scaledForce))

    -- Apply minimum threshold (prevents micro-adjustments that cause wobble)
    if math.abs(clampedForce) < PhysicsSteering.config.MIN_FORCE_THRESHOLD then
        return 0.0
    end

    return clampedForce
end

-- =========================================================
-- FORCE DAMPING AND STABILITY
-- =========================================================
local function applyForceDamping(targetForce)
    -- Apply damping to prevent oscillations
    local dampedForce = targetForce * PhysicsSteering.config.DAMPING_FACTOR

    -- Add integral term for smoother response
    PhysicsSteering.state.forceIntegral = PhysicsSteering.state.forceIntegral + dampedForce

    -- Limit integral to prevent buildup
    local integralLimit = PhysicsSteering.config.FORCE_INTEGRAL_LIMIT
    PhysicsSteering.state.forceIntegral =
        math.max(-integralLimit, math.min(integralLimit, PhysicsSteering.state.forceIntegral))

    -- Combine proportional and integral terms
    local finalForce = dampedForce + (PhysicsSteering.state.forceIntegral * 0.1)

    return finalForce
end

-- =========================================================
-- CORE PHYSICS STEERING APPLICATION
-- =========================================================
local function applyPhysicsSteering(vehicle, steeringOutput, speed)
    -- Update frame counter for frequency control
    PhysicsSteering.state.frameCounter = PhysicsSteering.state.frameCounter + 1

    -- Only apply forces every N frames for stability
    if PhysicsSteering.state.frameCounter % PhysicsSteering.config.UPDATE_FREQUENCY ~= 0 then
        return false
    end

    -- Detect vehicle type for scaling
    local vehicleType = detectVehicleType(vehicle)

    -- Calculate the lateral force needed
    local lateralForce = calculateLateralForce(steeringOutput, speed, vehicleType)

    -- Skip if force is too small
    if math.abs(lateralForce) < PhysicsSteering.config.MIN_FORCE_THRESHOLD then
        -- Decay integral when not applying force
        PhysicsSteering.state.forceIntegral = PhysicsSteering.state.forceIntegral * 0.9
        return false
    end

    -- Apply damping for stability
    local dampedForce = applyForceDamping(lateralForce)

    -- Create impulse vector using vehicle's right direction for consistent turning
    PhysicsSteering.state.impulseVector:set(
        PhysicsSteering.state.rightVector:x() * dampedForce,
        0.0,
        PhysicsSteering.state.rightVector:z() * dampedForce
    )

    -- Calculate optimal position to apply force
    local impulsePosition = calculateOptimalImpulsePosition(vehicle)

    -- Apply the physics impulse
    local success =
        pcall(
        function()
            vehicle:addImpulse(PhysicsSteering.state.impulseVector, impulsePosition)
        end
    )

    if success then
        -- Track applied force for monitoring
        PhysicsSteering.state.lastAppliedForce = dampedForce
        PhysicsSteering.state.averageForceApplied =
            (PhysicsSteering.state.averageForceApplied * 0.9) + (math.abs(dampedForce) * 0.1)
        return true
    else
        -- Reset integral on failure
        PhysicsSteering.state.forceIntegral = 0.0
        return false
    end
end

-- =========================================================
-- MAIN PUBLIC API
-- =========================================================
function setVehicleSteering(vehicle, steeringOutput)
    if not vehicle then
        return false
    end

    -- Get current speed for scaling
    local speed = math.abs(vehicle:getCurrentSpeedKmHour())

    -- Apply physics-based steering
    return applyPhysicsSteering(vehicle, steeringOutput, speed)
end

-- =========================================================
-- CONFIGURATION ACCESS
-- =========================================================
function PhysicsSteering.getConfig()
    return PhysicsSteering.config
end

function PhysicsSteering.setConfig(key, value)
    if PhysicsSteering.config[key] ~= nil then
        PhysicsSteering.config[key] = value
        return true
    end
    return false
end

-- =========================================================
-- DIAGNOSTICS AND MONITORING
-- =========================================================
function PhysicsSteering.getDiagnostics()
    return {
        lastAppliedForce = PhysicsSteering.state.lastAppliedForce,
        forceIntegral = PhysicsSteering.state.forceIntegral,
        averageForceApplied = PhysicsSteering.state.averageForceApplied,
        frameCounter = PhysicsSteering.state.frameCounter,
        cachedVehicleTypes = #PhysicsSteering.state.vehicleTypeCache
    }
end

function PhysicsSteering.resetState()
    PhysicsSteering.state.forceIntegral = 0.0
    PhysicsSteering.state.lastAppliedForce = 0.0
    PhysicsSteering.state.frameCounter = 0
    PhysicsSteering.state.averageForceApplied = 0.0
end

-- =========================================================
-- VEHICLE-SPECIFIC CALIBRATION
-- =========================================================
function calibrateVehicle(vehicle, testForce)
    if not vehicle then
        return false
    end

    local vehicleType = detectVehicleType(vehicle)
    local speed = math.abs(vehicle:getCurrentSpeedKmHour())

    -- Test force application and measure response
    local initialVelocity = org.joml.Vector3f.new()
    vehicle:getLinearVelocity(initialVelocity)

    -- Apply test force
    PhysicsSteering.state.impulseVector:set(testForce, 0.0, 0.0)
    local impulsePosition = calculateOptimalImpulsePosition(vehicle)

    local success =
        pcall(
        function()
            vehicle:addImpulse(PhysicsSteering.state.impulseVector, impulsePosition)
        end
    )

    if success then
        -- Return calibration data
        return {
            vehicleType = vehicleType,
            testForce = testForce,
            speed = speed,
            initialVelocity = {x = initialVelocity:x(), y = initialVelocity:y(), z = initialVelocity:z()}
        }
    end

    return false
end

-- =========================================================
-- CLEANUP AND MAINTENANCE
-- =========================================================
function maintenance()
    -- Clear old vehicle cache entries
    local currentVehicle = getPlayer() and getPlayer():getVehicle()
    if currentVehicle then
        local currentId = currentVehicle:getId()
        -- Keep only current vehicle in cache
        local newCache = {}
        newCache[currentId] = PhysicsSteering.state.vehicleTypeCache[currentId]
        PhysicsSteering.state.vehicleTypeCache = newCache
    end
end

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

local accumulatedError = 0
local lastVehicleId = -1
local lastToggleTime = 0

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
        setVehicleSteering(vehicle, finalSteering)
    else
        accumulatedError = accumulatedError * 0.9
    end
end

Events.OnPlayerUpdate.Add(onVehicleUpdate)

Events.OnTick.Add(
    function()
        if getTimestampMs() % 60000 < 100 then -- Every minute
            maintenance()
        end
    end
)

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
