socket = require 'socket'

local K = 13.54
local PWM_MIN = 130
local PWM_MAX = 255
local vel = 255

function omegaToPWM(omega)
     if math.abs(omega) < 0.05 then 
        return 0 
    end    
    
    local sign = 1
    if omega < 0 then
        sign = -1
        omega = math.abs(omega)
    end
    
    local pwm = PWM_MIN + (K * omega / (K * 18.85)) * (PWM_MAX - PWM_MIN)
    pwm = math.max(0, math.min(PWM_MAX, pwm))
    return sign * math.floor(pwm)
end

function omegaToPWMlaMario(omega)
    if omega == 0 then 
        return 0 
    end  
    
    local sign = 1
    if omega < 0 then
        sign = -1
        omega = math.abs(omega)
    end
    
    
    local pwm = omega * 8.76 + PWM_MIN

    return sign * math.floor(pwm)
end

function connect_to_esp32()
    esp32_socket = socket.tcp()
    esp32_socket:settimeout(1.0)
    local success, err = esp32_socket:connect(esp32_ip, esp32_port)
    if not success then
        print("Error ESP32: " .. tostring(err))
        esp32_socket = nil
        connected = false
    else
        print("ESP32 conectado")
        connected = true
    end
end

function sysCall_init()
    simRemoteApi.start(19999)

    leftFront  = sim.getObjectHandle('Left_front_motor')
    leftRear   = sim.getObjectHandle('Left_rear_motor')
    rightFront = sim.getObjectHandle('Rigth_front_motor')
    rightRear  = sim.getObjectHandle('Rigth_rear_motor')

    esp32_ip   = "192.168.1.88"
    esp32_port = 80
    esp32_socket = nil
    connected    = false

    connect_to_esp32()

    updateInterval = 50
    lastUpdateTime = sim.getSystemTimeInMs(-1)
end

function sysCall_actuation()
    if not connected or esp32_socket == nil then
        local currentTime = sim.getSystemTimeInMs(-1)
        if currentTime - lastUpdateTime >= 1000 then
            print("Reintentando conexion ESP32...")
            connect_to_esp32()
            lastUpdateTime = currentTime
        end
        return
    end

    local currentTime = sim.getSystemTimeInMs(-1)
    if currentTime - lastUpdateTime < updateInterval then return end

    --local leftVel  = (sim.getJointTargetVelocity(leftFront)  + sim.getJointTargetVelocity(leftRear))  / 2
    --local rightVel = (sim.getJointTargetVelocity(rightFront) + sim.getJointTargetVelocity(rightRear)) / 2

    -- Usa sim.getJointVelocity en lugar de TargetVelocity
    
    local leftVel  = (sim.getJointVelocity(leftFront)  + sim.getJointVelocity(leftRear))  / 2
    local rightVel = (sim.getJointVelocity(rightFront) + sim.getJointVelocity(rightRear)) / 2

    local leftPWM  = omegaToPWMlaMario(leftVel)
    local rightPWM = omegaToPWMlaMario(rightVel)

    
    
    local msg = string.format("M:%d,%d,%.3f", leftPWM,rightPWM, sim.getSimulationTime())
    
    local success, err = esp32_socket:send(msg .. "\n")

    if not success then
        print("Error envio: " .. tostring(err))
        esp32_socket:close()
        esp32_socket = nil
        connected = false
    else
        print("Enviado: " .. msg)
    end

    lastUpdateTime = currentTime
end

function sysCall_cleanup()
    if connected and esp32_socket then
        esp32_socket:send("M:0,0\n")
        socket.sleep(0.1)
        esp32_socket:close()
        print("Conexion cerrada")
    end
end