# The MIT License (MIT)
# Copyright (c) 2022 Mike Teachman
# https://opensource.org/licenses/MIT

# Purpose:  Play a WAV audio file out of a speaker or headphones
#
# - read audio samples from a WAV file stored on internal flash memory
# - write audio samples to an I2S amplifier or DAC module
# - the WAV fil/play_wav_from_flash_blocking.pye will play continuously in a loop until
#   a keyboard interrupt is detected or the board is reset
#
# Blocking version
# - the write() method blocks until the entire sample buffer is written to I2S
#
# Use a tool such as rshell or ampy to copy the WAV file "side-to-side-8k-16bits-stereo.wav"
# to internal flash memory


import os
from machine import I2S
from machine import Pin
from tempos import pm
from drivers.axp202 import LD04

# ======= I2S CONFIGURATION =======
SCK_PIN = 26
WS_PIN = 25
SD_PIN = 33
I2S_ID = 0
BUFFER_LENGTH_IN_BYTES = 5000
# ======= I2S CONFIGURATION =======


# ======= AUDIO CONFIGURATION =======
WAV_FILE = "recording.wav"
#WAV_FILE = "mic.wav"

WAV_SAMPLE_SIZE_IN_BITS = 16
FORMAT = I2S.MONO
SAMPLE_RATE_IN_HZ = 44100
# ======= AUDIO CONFIGURATION =======

audio_out = I2S(
    I2S_ID,
    sck=Pin(SCK_PIN),
    ws=Pin(WS_PIN),
    sd=Pin(SD_PIN),
    mode=I2S.TX,
    bits=WAV_SAMPLE_SIZE_IN_BITS,
    format=FORMAT,
    rate=SAMPLE_RATE_IN_HZ,
    ibuf=BUFFER_LENGTH_IN_BYTES,
)

wav = open(WAV_FILE, "rb")
pos = wav.seek(44)  # advance to first byte of Data section in WAV file

# allocate sample array
# memoryview used to reduce heap allocation
wav_samples = bytearray(1000)
wav_samples_mv = memoryview(wav_samples)

pm.setPower(LD04, 1)

# continuously read audio samples from the WAV file
# and write them to an I2S DAC
print("==========  START PLAYBACK ==========")
try:
    while True:
        num_read = wav.readinto(wav_samples_mv)
        # end of WAV file?
        if num_read == 0:
            # end-of-file, advance to first byte of Data section
            _ = wav.seek(44)
        else:
            _ = audio_out.write(wav_samples_mv[:num_read])

except (KeyboardInterrupt, Exception) as e:
    print("caught exception {} {}".format(type(e).__name__, e))

# cleanup
wav.close()
audio_out.deinit()
print("Done")
pm.setPower(LD04, 0)

