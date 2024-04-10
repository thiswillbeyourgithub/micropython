# WIP to get PDM on the TWATCH 2020 V3

This repo is to keep track of changes needed to reliably get the microphone working on the twatch 2020 V3.

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
# Roadmap
* Testing the microphone: https://github.com/miketeachman/micropython-i2s-examples/blob/master/examples/record_mic_to_sdcard_blocking.py
* Implementing an easy to use API in TempOS
* Profit

