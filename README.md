# Repository Introduction

The following repository contains all of the necessary hardware, software, TTN interface, 3D printing files, and assemble instructions for the CMWX-based LoRaWAN GPS radiotransmitter.

# 1. Hardware 

- datasheets for individual components found in subfolder https://github.com/trollock/solar-lorawan/tree/main/3_hardware

- design files found in subfolder https://github.com/trollock/solar-lorawan/tree/main/5_design_files
- produced in Autodesk Fusion 360 in *.fsch, *.fbrd, and *.f3z formats
- manufactured by https://www.pcbway.com
- production settings:
  + 24.9 x 10.5 mm, 2 layers, 0.4 mm FR4, 0.2 mm min hole size, immersion gold (ENIG) surface finish (1U"), 0.06 mm min track spacing, 1 oz Cu finished copper
    
# 2. Programming 

- programmed using a combination of Arduino, STM32CubeProgrammer, and an ST-Link.
  + *.elf files are first generated in Arduino, then loaded into STM32CubeProgrammer connected via an ST-Link 
- built on the Arduino core found here https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
  + comfirmed using a Mac running macOS Big Sur
- basic arduino example found in subfolder XXXX
  + can be modified and them compiled

- Basic programming example
  + open the basic arduino example
  + click "verify" and then find path to location of *.elf file
  + before opening STM32CubePorgrammer, connect the ST-Link to a suitable port
    + confirm the ST-Link is connected in STM32CubeProgrammer in the upper right under "ST-Link configuration"
  + provide 3.3v power to the device and click "Connect" in STM32CubeProgrammer
  + click "Open File" in STM32CubeProgrammer and find the location of the *.elf file generated earlier by Arduino
  + click "Download" in STM32CubeProgrammer and wait ~ 2-3 seconds until finished 
  + click "Disconnect" in STM32CubeProgrammer

# 3. 3D Printed Housing

- design files for 3D printed housings found in subfolder https://github.com/trollock/solar-lorawan/tree/main/6_housings
- printed on Formlabs Form 3+ / 3B+ at a 0.05 mm micron layer height
- *.stl file sliced in Preform 3.33.1 

# 4. Assembly

# 5. Data compression algorithm

# 6. Data Access
