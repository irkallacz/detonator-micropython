from machine import Pin, I2C, Timer
from imu import MPU6050
import tm1637
import utime
import math
import micropython

micropython.alloc_emergency_exception_buf(100)

machine.freq(20*1000000)

MINUTES = 5
FREQ = 2

SYMBOL_OFF = 0b01000000
SYMBOL_ON = 0b00001001

i2c = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)
imu = MPU6050(i2c)

led = Pin(25, Pin.OUT)
buzzer = Pin(22, Pin.IN)
display = tm1637.TM1637(clk=Pin(26), dio=Pin(27))

#debounce = Neotimer(250)

wire_contact = Pin(16, Pin.IN)
wire_trap = Pin(17, Pin.IN)

D_d0 = Pin(18, Pin.IN)
C_d1 = Pin(19, Pin.IN)
B_d2 = Pin(20, Pin.IN)
A_d3 = Pin(21, Pin.IN)

state = 0
end = 0

state_timer = Timer()
countdown_timer = Timer()

def get_imu_values():
    gx=abs(round(imu.accel.x,2))
    gy=abs(round(imu.accel.y,2))
    gz=abs(round(imu.accel.z,2))
    ax=abs(round(imu.gyro.x))
    ay=abs(round(imu.gyro.y))
    az=abs(round(imu.gyro.z))
    
    return ((gx > 0.2) or (gy > 0.2) or ((gz - 1) > 0.2)), ((ax > 20) or (ay > 20) or (az > 20))  

def get_sensor_values():
    gyro, accel = get_imu_values()
    
    return [
        bool(wire_contact.value()),
        not bool(wire_trap.value()),
        gyro,
        accel
    ]

def test_motion():
    gyro, accel = get_imu_values()
    
    if (accel):
        return True
    
    if (gyro):
        return True
    
    return False

def idle_start(pin):
    global state
    
    if (state == 0):
        return None
    
    state = 0;
    state_timer.deinit()
    state_timer.init(freq=1, mode=Timer.PERIODIC, callback=idle_repeat)
    
    buzzer = Pin(22, Pin.IN)
    
    led.off()
    countdown_timer.deinit()
    

def idle_repeat(timer):
    led.toggle()
    #display.show('----')
    
    sensors = list(map(lambda x: SYMBOL_ON if x else SYMBOL_OFF, get_sensor_values()))
    
    display.write(sensors)
    
def countdown_start(pin):
    global state
    global end
    global blink
    
    if (state == 1):
        return None
    
    state = 1
    blink = False
    state_timer.deinit()
    state_timer.init(freq=1, mode=Timer.PERIODIC, callback=countdown_repeat)
    end = utime.time() + MINUTES * 60
    countdown_timer.init(period=MINUTES * 60 * 1000, mode=Timer.ONE_SHOT, callback=countdown_finish)
    
    wire_contact.irq(trigger=Pin.IRQ_RISING, handler=detonation_start)
    wire_trap.irq(trigger=Pin.IRQ_FALLING, handler=detonation_start)

def countdown_repeat(timer):    
    global blink
    led.toggle()
    
    time = end - utime.time()
    minutes = math.floor(time / 60)
    seconds = time - minutes * 60
    
    blink = not(blink)

    #print(time, minutes, seconds)
    
    display.numbers(minutes, seconds, blink)
    
    if (test_motion()):
        state_timer.deinit()
        state_timer.init(freq=FREQ, mode=Timer.PERIODIC, callback=detonation_start)
    
def detonation_start(pin = None):
    global state
    
    if (state == 2):
        return None
    
    state = 2
    state_timer.deinit()
    buzzer.init(Pin.OUT, value=True)
    led.on()
    state_timer.init(freq=FREQ, mode=Timer.PERIODIC, callback=detonation_repeat)
    countdown_timer.deinit()
    
    wire_trap.irq(handler=None)
    wire_contact.irq(handler=None)

def detonation_repeat(timer):
    buzzer.toggle()
    
    toggle = utime.time() % 2
    
    if (toggle):
        display.numbers(0, 0, colon=True)
    else:
        display.write([0, 0, 0, 0])

def countdown_finish(timer):
    detonation_start()

A_d3.irq(trigger=Pin.IRQ_RISING, handler=idle_start)
C_d1.irq(trigger=Pin.IRQ_RISING, handler=countdown_start)
D_d0.irq(trigger=Pin.IRQ_RISING, handler=detonation_start)

wire_contact.irq(trigger=Pin.IRQ_RISING, handler=detonation_start)
wire_trap.irq(trigger=Pin.IRQ_FALLING, handler=detonation_start)

state_timer.init(freq=FREQ, mode=Timer.PERIODIC, callback=idle_repeat)

display.write([32, 32, 32, 32])
machine.lightsleep(100)    
for i in range(6):
    d = pow(2, i)
    display.write([d, d, d, d])
    machine.lightsleep(100)    

