import machine
import time
from machine import Pin, I2C, ADC, PWM, Signal
import uarray as array
import gc
# from pio_rpm import *
from pico_i2c_lcd import I2cLcd
import _thread

machine.freq(133000000)
print("Current Frequency:" + str(machine.freq()))

from micropython import const
import rp2
from rp2 import PIO, asm_pio


def timed_function(f, *args, **kwargs):
    myname = str(f).split(' ')[1]

    def new_func(*args, **kwargs):
        t = time.ticks_us()
        result = f(*args, **kwargs)
        delta = time.ticks_diff(time.ticks_us(), t)
        print('Function {} Time = {:6.3f}ms'.format(myname, delta / 1000))
        return result

    return new_func


@asm_pio(sideset_init=PIO.OUT_HIGH)
def gate():
    """PIO to generate gate signal."""
    mov(x, osr)  # load gate time (in clock pulses) from osr
    wait(0, pin, 0)  # wait for input to go low
    wait(1, pin, 0)  # wait for input to go high - effectively giving us rising edge detection
    label("loopstart")
    jmp(x_dec, "loopstart").side(0)  # keep gate low for time programmed by setting x reg
    wait(0, pin, 0)  # wait for input to go low
    wait(1, pin, 0).side(1)  # set gate to high on rising edge
    irq(block, 0)  # set interrupt 0 flag and wait for system handler to service interrupt
    wait(1, irq, 4)  # wait for irq from clock counting state machine
    wait(1, irq, 5)  # wait for irq from pulse counting state machine


@asm_pio()
def clock_count():
    """PIO for counting clock pulses during gate low."""
    mov(x, osr)  # load x scratch with max value (2^32-1)
    wait(1, pin, 0)  # detect falling edge
    wait(0, pin, 0)  # of gate signal
    label("counter")
    jmp(pin, "output")  # as long as gate is low //
    jmp(x_dec, "counter")  # decrement x reg (counting every other clock cycle - have to multiply output value by 2)
    label("output")
    mov(isr, x)  # move clock count value to isr
    push()  # send data to FIFO
    irq(block, 4)  # set irq and wait for gate PIO to acknowledge


@asm_pio(sideset_init=PIO.OUT_HIGH)
def pulse_count():
    """PIO for counting incoming pulses during gate low."""
    mov(x, osr)  # load x scratch with max value (2^32-1)
    wait(1, pin, 0)
    wait(0, pin, 0).side(0)  # detect falling edge of gate
    label("counter")
    wait(0, pin, 1)  # wait for rising on pin 1
    wait(1, pin, 1)  # edge of input signal
    jmp(pin, "output")  # as long as gate is low //
    jmp(x_dec,
        "counter")  # decrement x req counting incoming pulses (probably will count one pulse less than it should - to be checked later)
    label("output")
    mov(isr, x).side(1)  # move pulse count value to isr and set pin to high to tell clock counting sm to stop counting
    push()  # send data to FIFO
    irq(block, 5)  # set irq and wait for gate PIO to acknowledge


# @asm_pio(sideset_init=PIO.OUT_HIGH)
# def gate():
#     """PIO to generate gate signal."""
#     wait(0, pin, 0)
#     wrap_target()
#     mov(x, osr)
#     wait(1, pin, 0) # Probably not needed, since at this point signal will be high anyway
#     label("loopstart")
#     jmp(x_dec, "loopstart") .side(0)
#     wait(0, pin, 0)
#     wait(1, pin, 0) .side(1)
#     irq(noblock, 0)
#     wrap()
#
# @asm_pio()
# def clock_count():
#     """PIO for counting clock pulses during gate low."""
#     wait(1, pin, 0)
#     wrap_target()
#     mov(x, osr)
#     wait(0, pin, 0)
#     label("counter")
#     jmp(pin, "output")
#     jmp(x_dec, "counter")
#     label("output")
#     mov(isr, x)
#     push()
#
# @asm_pio(sideset_init=PIO.OUT_HIGH)
# def pulse_count():
#     """PIO for counting incoming pulses during gate low."""
#     wait(1, pin, 0)
#     wrap_target()
#     mov(x, osr)
#     wait(0, pin, 0) .side(0)
#     label("counter")
#     wait(0, pin, 1)
#     wait(1, pin, 1)
#     jmp(pin, "output")
#     jmp(x_dec, "counter")
#     label("output")
#     mov(isr, x) .side(1)
#     push()
#     wrap()

def init_sm(freq, input_pin, gate_pin, pulse_fin_pin):
    """Starts state machines."""
    gate_pin.value(1)
    pulse_fin_pin.value(1)
    max_count = const((1 << 32) - 1)

    sm0 = rp2.StateMachine(0, gate, freq=freq, in_base=input_pin, sideset_base=gate_pin)
    sm0.put(freq)
    sm0.exec("pull()")

    sm1 = rp2.StateMachine(1, clock_count, freq=freq, in_base=gate_pin, jmp_pin=pulse_fin_pin)
    sm1.put(max_count)
    sm1.exec("pull()")

    sm2 = rp2.StateMachine(2, pulse_count, freq=freq, in_base=gate_pin, sideset_base=pulse_fin_pin, jmp_pin=gate_pin)
    sm2.put(max_count - 1)
    sm2.exec("pull()")

    sm1.active(1)
    sm2.active(1)
    sm0.active(1)

    return sm0, sm1, sm2


update_flag = False
data = array.array("I", [0, 0])


# @timed_function
def counter_handler(sm):
    print("IRQ")
    global update_flag

    if not update_flag:
        sm0.put(133_000)
        sm0.exec("pull()")
        data[0] = sm1.get()  # clock count
        data[1] = sm2.get()  # pulse count

    update_flag = True


# Initialize the state machines for RPM counting
sm0, sm1, sm2 = init_sm(133_000_000, Pin(15, Pin.IN, Pin.PULL_UP), Pin(14, Pin.OUT), Pin(13, Pin.OUT))
sm0.irq(counter_handler)


class Switch:
    def __init__(self, pin1, pin2):
        self.pin1 = Pin(pin1, Pin.IN, Pin.PULL_UP)
        self.pin2 = Pin(pin2, Pin.IN, Pin.PULL_UP)

    def switch_pos(self):

        if self.pin1.value() == 1 and self.pin2.value() == 1:
            return 2

        elif self.pin1.value() == 1 and self.pin2.value() == 0:
            return 3

        elif self.pin1.value() == 0 and self.pin2.value() == 1:
            return 1

        else:
            return -1


# Initialize LCD
i2c = I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=400000)
lcd = I2cLcd(i2c, 0x27, 2, 16)
lcd.clear()

# Initialize potentiometer
adc = ADC(Pin(28))
print("ADC Value: " + str(adc.read_u16()))

red_led = Pin(2, Pin.OUT)
green_led = Pin(3, Pin.OUT)

# Switch
switch = Switch(27, 26)

# PWM for Motor
pwm = PWM(Pin(4))
pwm.freq(21)
pwm.duty_u16(0)

e_stop_tripped_flag = False


def emergency_stop(pin):
    global e_stop_tripped_flag
    pwm.duty_u16(0)
    red_led.value(1)
    green_led.value(0)
    lcd.clear()
    lcd.putstr("Emergency Stop")
    e_stop_tripped_flag = True


# Emergency stop
emergency_stop_switch = Pin(6, Pin.IN, Pin.PULL_UP)
emergency_stop_switch.irq(trigger=Pin.IRQ_FALLING, handler=emergency_stop)


def main():
    global update_flag, e_stop_tripped_flag

    lcd.print_line_col_in_place(0, 0, "Duty: ")
    lcd.print_line_col_in_place(1, 0, "RPM: ")

    adc_value = 0
    duty_cycle = 0

    # LCD Refresh Rate
    lcd_refresh_time = 500
    lcd_start = time.ticks_ms()

    # Flag to prevent motor from starting when micro is powered on with swich in position 3
    switch_start_in_position_3 = True

    rpm_ramp_up_steps = const(4)
    rpm_ramp_up_step_time = const(1)
    ramp_up_function_enabled = False

    while True:
        # time.sleep(0.25)
        rpm = 0




        # Read RPM and switch.switch_pos() == 3
        if update_flag and not switch_start_in_position_3:
            data[0] = sm1.get()  # clock count
            data[1] = sm2.get()  # pulse count
            clock_count = 2 * (max_count - data[0] + 1)  # 2 * (max count - clock count + 1)
            pulse_count = max_count - data[1]  # Max count - pulse count is the number of pulses counted
            freq = pulse_count * (133000208.6 / clock_count)
            rpm = int(freq / 6)
            print("RPM:   {}".format(rpm))
            update_flag = False

        # Read Potentiometer only if switch is in position 1 or 2
        if switch.switch_pos() == 1 or switch.switch_pos() == 2:
            adc_value = 65535 - adc.read_u16()
            print("ADC Value: " + str(adc_value))



        # If E Stop is not tripped, If tripped only mode 1 can reset it
        if not e_stop_tripped_flag:

            # Running mode.Motor is ON!,
            # Changing duty cycle is NOT! allowed in this mode
            if switch.switch_pos() == 3:
                if not switch_start_in_position_3:
                    print("Switch 3")
                    green_led.value(1)
                    red_led.value(0)

                    # Start Motor if at 0 rpm, ramp up to set point.
                    if rpm == 0 and ramp_up_function_enabled:
                        print("Ramping up")

                        adc_delta = adc_value / (rpm_ramp_up_steps - 1)
                        adc_step_list = [int(adc_delta * i) for i in range(rpm_ramp_up_steps)]
                        print("Adc Step Lists: " + str(adc_step_list))
                        for i in range(rpm_ramp_up_steps):
                            pwm.duty_u16(adc_step_list[i])
                            time.sleep(rpm_ramp_up_step_time)
                            print("Duty Cycle: " + str(adc_step_list[i]))

                        ramp_up_function_enabled = False

                    else:
                        pwm.duty_u16(adc_value)

                else:
                    green_led.value(1)
                    red_led.value(1)

            # Adjusting mode. Motor is kept in last state,
            # Changing duty cycle is allowed in this mode
            elif switch.switch_pos() == 2:
                print("Switch 2")
                switch_start_in_position_3 = False
                green_led.value(0)
                red_led.value(1)
                duty_cycle = int((adc_value / 65535) * 100)
                5
                # pwm.duty_u16(0)

            # Safety mode. Motor is off
            elif switch.switch_pos() == 1:
                print("Switch 1")
                switch_start_in_position_3 = False
                ramp_up_function_enabled = True
                green_led.value(0)
                red_led.value(0)
                duty_cycle = int((adc_value / 65535) * 100)
                pwm.duty_u16(0)

            # Error mode. Motor is off
            else:
                print("Switch Error")
                green_led.value(0)
                red_led.value(0)
                pwm.duty_u16(0)

            # Update LCD
            if time.ticks_diff(time.ticks_ms(), lcd_start) > lcd_refresh_time:
                print("Updating LCD")
                if switch.switch_pos() == 2 or switch.switch_pos() == 1:
                    lcd.print_line_col_in_place(0, 6, "       ")
                    lcd.print_line_col_in_place(0, 6, "%1.0f" % duty_cycle + "%")

                lcd.print_line_col_in_place(1, 5, "        ")
                lcd.print_line_col_in_place(1, 5, str(rpm))
                lcd_start = time.ticks_ms()

        # Reset E Stop
        elif e_stop_tripped_flag:
            if switch.switch_pos() == 1 and emergency_stop_switch.value() == 1:
                e_stop_tripped_flag = False
                print("Switch 1")
                green_led.value(0)
                red_led.value(0)
                lcd.clear()
                lcd.print_line_col_in_place(0, 0, "Duty: ")
                lcd.print_line_col_in_place(1, 0, "RPM: ")

        print("Duty Cycle: " + str(duty_cycle))






if __name__ == "__main__":
    main()
