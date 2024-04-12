# WIP to get PDM on the TWATCH 2020 V3

This repo is to keep track of changes needed to reliably get the microphone working on the twatch 2020 V3.

# Current status
All current files are in `AUDIO_TEST`. There, I have a working script to play wav files on the watch (you can test it with the `recording.wav` file that I found somewhere online), and my plan is to modify the script that is supposed to record until it finally work. Both scripts are derived from [that repo](https://github.com/miketeachman/micropython-i2s-examples). They both read and write `mic.wav` file for easy prototyping.

* **The latest issue is:** the created file is empty of audio: it produces a file called mic.wav but I couldn't play it even on my computer and the hexdump showed it was actually full of 0000s. I won't have time to test some more for a bit so please join in!

* **To help**: Take a look at the folder `AUDIO_TEST`, and tell me why the recordings are empty.

* Here are links that can be relevant by increasing order of importance:
    * https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library/blob/master/docs/watch_2020_v3.md
    * https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library/
    * https://docs.micropython.org/en/latest/library/machine.I2S.html#machine-i2s
    * https://github.com/uraich/twatch2020_firmware/blob/6258eee0021351521da70d63a00d063e7a0acde7/hardware/sound/play-mono-wav-uasyncio.py
    * https://github.com/OPHoperHPO/lilygo-ttgo-twatch-2020-micropython/issues/5
    * https://github.com/miketeachman/micropython-esp32-i2s-examples
        * **Was updated to this repo:**
            * https://github.com/miketeachman/micropython-i2s-examples
    * https://github.com/lunokjod/watch/blob/b71340b486b68d7682a9021e454779557d7a291c/src/app/Mic.cpp#L60-L82
        * **This apparently is a working implementation in C so should contain all the answers?**



See below for the steps I had to take to make the build.

# Roadmap
* Making the microphone work.
* Implement an easy to use API in TempOS
* Build cool stuff


# Steps I had to take to make the build
1. download, install and activate esp-idf
```
git clone -b v5.0.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.0.4 && git submodule update --init --recursive && ./install.sh && source export.sh
```
2. clone micropython a recent version of micropython
* I couldn't cleanly apply the patch from lana_chan to jeffmer's modificaiton of the GPIO wake things so I decided to go the other way around: applying jeffmer's patch after lana_chan's PDM.
* The most recent commit at the time for me was `5114f2c1ea7c05fc7ab920299967595cfc5307de` but I decided to checkout earlier to `01c31ea804c7e2a034e1249b43c30673b4382116` because commits after that modified things to something called berkeley-db that seems broken since and I can't successfuly build otherwwise (something plausibly simple as it's mainly file not founds because the paths changed).
```
cd ..
git clone https://github.com/micropython/micropython/
cd micropython
git branch lanachanthenjeffbranch
git checkout lanachanthenjeffbranch
git checkout 01c31ea804c7e2a034e1249b43c30673b4382116
```
3. Apply @lana-chan patch
```
wget https://github.com/micropython/micropython/pull/14176.diff
git apply 14176.diff
```
4. Apply jeffmer patch
I had to modify a few things. You can see the two commits
5. Build micropython
```
make -C mpy-cross  # in the top level micropython dir
cd ports/esp32
make submodules
make BOARD=ESP32_GENERIC BOARD_VARIANT=SPIRAM
```
6. Erase the watch then flash the new build
```
python -m esptool --port /dev/ttyACM0 erase_flash
python -m esptool --port /dev/ttyACM0 --chip esp32 write_flash -z 0x1000 build-ESP32_GENERIC-SPIRAM/firmware.bin
```
7. install TempOS
```
cd ..
git clone https://github.com/jeffmer/TTGO-T-watch-2020-Micropython-OS/
cd TTGO-T-watch-2020-Micropython-OS/src
```
8. modify the path to mpy-cross in compile.sh to point to the one built in the micropython folder
9. install os
```
./compile.sh
./install.sh
```
