#
# This file is part of the DS-Pi project, https://github.com/elektroThing/DS-Pi
# code.py - Main entry point for the circuitpyhton bootloader.
# This file tests and configures the peripherals and audio codec communication.
# 
# The MIT License (MIT)
#
# Copyright (c) 2022 elektroThing & gluons.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

# ======= IMPORTS =======
import sys
import time
import array
import math
import board
import busio # i2c peripheral support
import rp2pio # PIO support
import adafruit_pioasm # assembler
import audiocore
import audiobusio # i2s
import digitalio # led

# ======= PIO Clock =======
clock = """
.program clock
	set pins 1		; drive pin high
	set pins 0		; drive pin low
"""

clock_assembled = adafruit_pioasm.assemble(clock)

# The state machine is ran @ 4.096 MHz to generate a clock of half that frequency.
# Each 'set pins' assembly function takes 1 clock cycle, so we need double
# the target frequency (2.048 MHz). This frequency is taken from the TLV320AIC3254
# application reference guide (p. 79) and targets a sampling frequency of 48kHz.
state_machine = rp2pio.StateMachine(
	clock_assembled, # program to run
	frequency=4096000, # run @ 4.096 MHz
	init=adafruit_pioasm.assemble("set pindirs 1"), # set pin direction as output
	first_set_pin=board.GP2 # pin to be controlled, if used for the MCLK change it to GPIO2
)

# prints the actual frequency of the state machine as seen by the RP2040
# we might need to fiddle with the frequency of the state machine.
print("MCLK real frequency:", state_machine.frequency)

# ======= REGISTERS & VARS =======

# this address is for an adxl345 accelerometer, i used it to debug the i2c functions
# comment out this address when testing the aic3254 codec
# dev_address = 0x53

# datasheet says i2c device ID is 0b0011000 (7-bit) so 0b0011000 << 1 = 0b00110000 = 0x30
dev_address = 0x18 # 0b0011000

gain = 0x14 # 10dB gain. see datasheet for this value (p. 141) [0b00010100]

# ======= FUNCTIONS =======

# check for device i2c address on the i2c bus
def ack_peripheral(i2c_address):
	status = False # status of the transaction

	# wait for the i2c bus to be free
	while i2c.try_lock():
		pass

	# perform a scan and print out address if device is found
	for address in i2c.scan():
		if address == i2c_address:
			print("Peripheral found! Address: " + str(hex(address)))
			status = True
	i2c.unlock()
	time.sleep(1)
	return status


# write to a register (aic3254 or aic3204 only)
def set_register(i2c_address, register_address, set_value):
	# limit the registers addresses from 0 to 127. (see datasheet/register_map p.94)
	register_address = register_address & 0x007F
	set_value = set_value & 0xFF # 8-bit registers limit (0 to 255)
	
	while not i2c.try_lock():
		pass
	# this function sends a stop bit when it finishes the transaction, might need to check later if it works well with the codec
	i2c.writeto(i2c_address, bytes([register_address, set_value]))
	i2c.unlock()


# read a specific register and store it into a buffer with given length depending on the data returned by the register
def read_register(i2c_address, register_address, buffer_length):
	register_address = register_address & 0x007F
	
	while not i2c.try_lock():
		pass
	
	i2c.writeto(i2c_address, bytes([register_address]))
	data_buffer = bytearray(buffer_length)
	i2c.readfrom_into(i2c_address, data_buffer)
	i2c.unlock()
	
	return data_buffer

# test a specific register for an expected value. used for debugging purposes
def test_register(i2c_address, register_address, expected_value):
	
	byte_array = read_register(i2c_address, register_address, 1)

	for data in byte_array:
		byte_output = data

	if (byte_output == expected_value):
		print("[PASS] R_%s -> EXPECTED: %s. ACTUAL: %s." % (hex(register_address), hex(expected_value), hex(byte_output)))
	else:
		print("[ERROR] R_%s -> EXPECTED: %s. ACTUAL: %s." % (hex(register_address), hex(expected_value), hex(byte_output)))
		sys.exit(1)


# initialize and configure the AIC3254 codec on DS-Pi startup
# This function is hardcoded to configure the codec to 48 kHz given the 2.048 MHz input from
# the MCLK with a bit depth of 24 bits. See PLL examples configuration (p. 79) for more info
# on how to configure the device for a 44.1 kHz sampling frequency.
#
# FYI values: PLL_CLK is always 86.016 MHz, DAC_CLK is always 12.288 MHz.
#
def init_aic3254(device):
	# issue a soft reset
	set_register(device, 0x00, 0x00) # (P0_R0) go to page 0 on register map
	set_register(device, 0x01, 0x01) # (P0_R1) issue a software reset to the codec
	time.sleep(0.001) # wait for device to initialize registers
	set_register(device, 0x00, 0x01) # (P1_R0) point to page 1 on register map
	set_register(device, 0x01, 0x08) # (P1_R1) disable crude AVDD generation from DVDD [0b00001000]
	set_register(device, 0x02, 0x01) # (P1_R2) enable analog block, AVDD LDO powered up [0b00000001]

	# configure PLL and clocks
	set_register(device, 0x00, 0x00) # (P0_R0) go to page 0 on regiter map
	# TODO: fiddle with the bit depth, maybe tthe rp2040 is generating a fixed 16 bit word length...
	set_register(device, 0x1B, 0x21) # (P0_R27) enable I2S, 24 bit depth, set BCLK and WCLK as inputs to AIC3254 (target) [0b00100001]
	set_register(device, 0x1C, 0x00) # (P0_R28) set data offset to 0 BCLKs
	set_register(device, 0x04, 0x03) # (P0_R4) config PLL settings: Low PLL clock range, MCLK -> PLL_CLKIN, PLL_CLK -> CODEC_CLKIN [0b00000011]
	set_register(device, 0x06, 0x0E) # (P0_R6) set J = 14 [0b00001110]
	set_register(device, 0x07, 0x00) # (P0_R7) set D = 0 (MSB)
	set_register(device, 0x08, 0x00) # (P0_R8) set D = 0 (LSB)
	# for 32 bit clocks per frame in Controller mode ONLY
	set_register(device, 0x1E, 0x08) # (P0_R30) set BCKL N divider to 8. BLCK = DAC_CLK/N = 12.288 MHz/8 = 32*fs = 1.536 MHz
	set_register(device, 0x05, 0x93) # (P0_R5) Power up PLL, set P = 1 and R = 3 [0b10010011]
	set_register(device, 0x0D, 0x00) # (P0_R13) set DOSR = 128 (MSB), hex 0x0080, DAC Oversampling
	set_register(device, 0x0E, 0x80) # (P0_R14) set DOSR = 128 (LSB)
	set_register(device, 0x14, 0x80) # (P0_R20) set AOSR = 128 decimal or 0x0080 hex, for decimation filters 1 to 6, ADC Oversampling
	set_register(device, 0x0B, 0x82) # (P0_R11) power up and set NDAC = 2 [0b10000010]
	set_register(device, 0x0C, 0x87) # (P0_R12) power up and set MDAC = 7 [0b10000111]
	set_register(device, 0x12, 0x87) # (P0_R18) power up and set NADC = 7
	set_register(device, 0x13, 0x82) # (P0_R19) power up and set MADC = 2
	
	# DAC routing and power up
	set_register(device, 0x00, 0x01) # (P1_R0) point to page 1 on register map
	set_register(device, 0x0E, 0x08) # (P1_R14) LDAC AFIR routed to LOL [0b00001000]
	set_register(device, 0x0F, 0x08) # (P1_R15) RDAC AFIR routed to LOR [0b00001000]
	set_register(device, 0x00, 0x00) # (P0_R0) point to page 0 on register map
	set_register(device, 0x40, 0x02) # (P0_R64) DAC left volume = right volumen [0b00000010]
	set_register(device, 0x41, 0x00) # (P0_R65) set left DAC gain to 0dB DIGITAL VOL
	set_register(device, 0x3F, 0xD4) # (P0_R63) Power up left and right DAC data paths and set channel [0b11010100]
	set_register(device, 0x00, 0x01) # (P1_01) point to page 1 on register map
	set_register(device, 0x09, 0x0C) # (P1_R9) power up LOL and LOR [0b00001100]
	set_register(device, 0x0A, 0x08) # (P1_R10) output common mode for LOL and LOR is 1.65 from LDOIN (= Vcc / 2) [0b00001000]
	set_register(device, 0x12, 0x0A) # (P1_R18) unmute LOL, set 10dB gain [0b00001010]
	set_register(device, 0x13, 0x0A) # (P1_R19) unmute LOR, set 10db gain [0b00001010]
	
	# ADC routing and power up
	set_register(device, 0x00, 0x01) # (P1_R0) point to page 1 on register mao
	set_register(device, 0x34, 0x50) # (P1_R52) Route IN1L and IN2L to Left MICPGA with 10kohm resistance [0b01010000]
	set_register(device, 0x37, 0x50) # (P1_R55) Route IN1R and IN2R to RIght MICPGA with 10kohm resistance [0b01010000]
	set_register(device, 0x36, 0x01) # (P1_R54) CM is routed to Left MICPGA via CM2L with 10kohm resistance
	set_register(device, 0x39, 0x40) # (P1_R57) CM is routed to Right MICPGA via CM1R with 10kohm resistance [0b01000000]
	set_register(device, 0x3B, gain) # (P1_R59) Unmute left MICPGA  and set its gain
	set_register(device, 0x3C, gain) # (P1_R60) Unmute right MICPGA and set its gain
	set_register(device, 0x00, 0x00) # (P0_R0) point to page 0 on register map
	set_register(device, 0x51, 0xC0) # (P0_R81) power up left and right ADCs
	set_register(device, 0x52, 0x00) # (P0_R82) unmute left and right ADCs
	set_register(device, 0x00, 0x00) # (P0_R0) point to page 0 on register map
	time.sleep(0.01) # wait 10 ms


# look into the registers to test and double check they hold the right values given the configuration.	
def probe_aic3254(device):
	set_register(device, 0x00, 0x01) # (P1_R0) jump to page 1 on register map
	print("===== PAGE 1 =====")

	test_register(device, 0x01, 0x08) # (P1_R1) disable crude AVDD generation from DVDD [0b00001000]
	test_register(device, 0x02, 0x01) # (P1_R2) enable analog block, AVDD LDO powered up [0b00000001]

	# configure PLL and clocks
	set_register(device, 0x00, 0x00) # (P0_R0) go to page 0 on regiter map
	print("===== PAGE 0 =====")

	# TODO: fiddle with the bit depth, maybe tthe rp2040 is generating a fixed 16 bit word length...
	test_register(device, 0x1B, 0x21) # (P0_R27) enable I2S, 24 bit depth, set BCLK and WCLK as outputs to AIC3254 (target) [0b00100001]
	test_register(device, 0x1C, 0x00) # (P0_R28) set data offset to 0 BCLKs
	test_register(device, 0x04, 0x03) # (P0_R4) config PLL settings: Low PLL clock range, MCLK -> PLL_CLKIN, PLL_CLK -> CODEC_CLKIN [0b00000011]
	test_register(device, 0x06, 0x0E) # (P0_R6) set J = 14 [0b00001110]
	test_register(device, 0x07, 0x00) # (P0_R7) set D = 0 (MSB)
	test_register(device, 0x08, 0x00) # (P0_R8) set D = 0 (LSB)
	# for 32 bit clocks per frame in Controller mode ONLY
	test_register(device, 0x1E, 0x08) # (P0_R30) set BCKL N divider to 8. BLCK = DAC_CLK/N = 12.288 MHz/8 = 32*fs = 1.536 MHz
	test_register(device, 0x05, 0x93) # (P0_R5) Power up PLL, set P = 1 and R = 3 [0b10010011]
	test_register(device, 0x0D, 0x00) # (P0_R13) set DOSR = 128 (MSB), hex 0x0080, DAC Oversampling
	test_register(device, 0x0E, 0x80) # (P0_R14) set DOSR = 128 (LSB)
	test_register(device, 0x14, 0x80) # (P0_R20) set AOSR = 128 decimal or 0x0080 hex, for decimation filters 1 to 6, ADC Oversampling
	test_register(device, 0x0B, 0x82) # (P0_R11) power up and set NDAC = 2 [0b10000010]
	test_register(device, 0x0C, 0x87) # (P0_R12) power up and set MDAC = 7 [0b10000111]
	test_register(device, 0x12, 0x87) # (P0_R18) power up and set NADC = 7
	test_register(device, 0x13, 0x82) # (P0_R19) power up and set MADC = 2
	
	# DAC routing and power up
	set_register(device, 0x00, 0x01) # (P1_R0) point to page 1 on register map
	print("===== PAGE 1 =====")

	test_register(device, 0x0E, 0x08) # (P1_R14) LDAC AFIR routed to LOL [0b00001000]
	test_register(device, 0x0F, 0x08) # (P1_R15) RDAC AFIR routed to LOR [0b00001000]

	set_register(device, 0x00, 0x00) # (P0_R0) point to page 0 on register map
	print("===== PAGE 0 =====")

	test_register(device, 0x40, 0x02) # (P0_R64) DAC left volume = right volumen [0b00000010]
	test_register(device, 0x41, 0x00) # (P0_R65) set left DAC gain to 0dB DIGITAL VOL
	test_register(device, 0x3F, 0xD4) # (P0_R63) Power up left and right DAC data paths and set channel [0b11010100]

	set_register(device, 0x00, 0x01) # (P1_01) point to page 1 on register map
	print("===== PAGE 1 =====")

	test_register(device, 0x09, 0x0C) # (P1_R9) power up LOL and LOR [0b00001100]
	test_register(device, 0x0A, 0x08) # (P1_R10) output common mode for LOL and LOR is 1.65 from LDOIN (= Vcc / 2) [0b00001000]
	test_register(device, 0x12, 0x0A) # (P1_R18) unmute LOL, set 10dB gain [0b00001010]
	test_register(device, 0x13, 0x0A) # (P1_R19) unmute LOR, set 10db gain [0b00001010]
	
	# ADC routing and power up
	set_register(device, 0x00, 0x01) # (P1_R0) point to page 1 on register map
	print("===== PAGE 1 =====")

	test_register(device, 0x34, 0x50) # (P1_R52) Route IN1L and IN2L to Left MICPGA with 10kohm resistance [0b01010000]
	test_register(device, 0x37, 0x50) # (P1_R55) Route IN1R and IN2R to RIght MICPGA with 10kohm resistance [0b01010000]
	test_register(device, 0x36, 0x01) # (P1_R54) CM is routed to Left MICPGA via CM2L with 10kohm resistance
	test_register(device, 0x39, 0x40) # (P1_R57) CM is routed to Right MICPGA via CM1R with 10kohm resistance [0b01000000]
	test_register(device, 0x3B, gain) # (P1_R59) Unmute left MICPGA  and set its gain
	test_register(device, 0x3C, gain) # (P1_R60) Unmute right MICPGA and set its gain

	set_register(device, 0x00, 0x00) # (P0_R0) point to page 0 on register map
	print("===== PAGE 0 =====")

	test_register(device, 0x51, 0xC0) # (P0_R81) power up left and right ADCs
	test_register(device, 0x52, 0x00) # (P0_R82) unmute left and right ADCs
	set_register(device, 0x00, 0x00) # (P0_R0) point to page 0 on register map
	time.sleep(0.01) # wait 10 ms

def reset_aic3254(device):
	# issue a soft reset
	set_register(device, 0x00, 0x00) # (P0_R0) go to page 0 on register map
	set_register(device, 0x01, 0x01) # (P0_R1) issue a software reset to the codec
	time.sleep(0.001) # wait for device to initialize registers

def sine(freq):
	# generate the data for a sine wave @ freq Hz
	tone_volume = 1
	f = freq # tone @ freq Hz (A1 / La)
	length = 8000 // f
	sine_wave = array.array("h", [0] * length)
	for i in range(length):
		sine_wave[i] = int((math.sin(math.pi * 2 * i / length)) * tone_volume * (2 ** 15 - 1))
	return audiocore.RawSample(sine_wave)

# ======= SETUP =======

# setup the i2s bus
i2s = audiobusio.I2SOut(board.GP3, board.GP4, board.GP5) # BCLK, WCLK, DIN respectively

# setup the i2c bus
# YJ: I have jumpered PIN 6 and 7 to PIN 0 and 1 for I2C. Made a mistake with the electronic design. 
i2c = busio.I2C(scl=board.GP1, sda=board.GP0, frequency=400 * 1000) # start i2c port @ 400kHz

time.sleep(0.5) # wait for peripherals to wake up

if (not ack_peripheral(dev_address)):
	while True:
		print("Device not found! Check wiring/address.")
		time.sleep(0.5)

# config codec
reset_aic3254(dev_address)
init_aic3254(dev_address)
probe_aic3254(dev_address)

# setup the led
led = digitalio.DigitalInOut(board.GP25)
led.direction = digitalio.Direction.OUTPUT

# ====== MAIN LOOP ======

while True:
	tone = sine(1000)
	i2s.play(tone, loop=True)
	led.value = True
	time.sleep(0.2)
	
	led.value = False
	time.sleep(0.2)
	
	tone = sine(1200)
	i2s.play(tone, loop=True)
	led.value = True
	time.sleep(0.2)
	
	led.value = False
	time.sleep(0.2)
	
	tone = sine(1400)
	i2s.play(tone, loop=True)
	led.value = True
	time.sleep(0.2)
	
	led.value = False
	time.sleep(0.2)
	
	tone = sine(2000)
	i2s.play(tone, loop=True)
	led.value = True
	time.sleep(0.2)
	
	led.value = False
	time.sleep(0.2)
