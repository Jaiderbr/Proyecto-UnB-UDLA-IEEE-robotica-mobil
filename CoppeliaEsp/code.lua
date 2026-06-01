socket = require 'socket'

local K = 13.54
local PWM_MIN = 60  
local PWM_MAX = 255
local vel = 255

local DEADZONE = 0.05

function omegaToPWMlaMario(omega)
    
    if math.abs(omega) < DEADZONE then 
        return 20 
    end  
    
    local sign = 1
    if omega < 0 then
        sign = -1
        omega = math.abs(omega)
    end
    
    local pwm = omega * 15    
    pwm = math.min(math.max(pwm, PWM_MIN), PWM_MAX)

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

    leftFront  = sim.getObjectHandle('left_motor_A')
    rightFront = sim.getObjectHandle('rigth_motor_A')

    esp32_ip   = "#"
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

    
    local leftPWM  = omegaToPWMlaMario(sim.getJointVelocity(leftFront))
    local rightPWM = omegaToPWMlaMario(sim.getJointVelocity(rightFront))

    
    local msg = string.format("M:%d,%d,%.3f", leftPWM, rightPWM, sim.getSimulationTime())
    
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