# Repository Introduction

The following repository contains all of the necessary hardware, software, TTN interface, 3D printing files, and assemble instructions for the CMWX-based LoRaWAN GPS radiotransmitter.

# 1. Hardware 

- datasheets for individual components found in subfolder https://github.com/trollock/solar-lorawan/tree/main/3_hardware

- design files found in subfolder https://github.com/trollock/solar-lorawan/tree/main/5_design_files
- produced in Autodesk Fusion 360 in *.fsch, *.fbrd, and *.f3z formats
- manufactured by https://www.pcbway.com
- production settings:
  + 24.9 x 10.5 mm, 2 layers, 0.4 mm FR4, 0.2 mm min hole size, immersion gold (ENIG) surface finish (1U"), 0.06 mm min track spacing, 1 oz Cu finished copper

<img width="1824" alt="Screen Shot 2023-12-31 at 3 34 29 PM" src="https://github.com/trollock/solar-lorawan/assets/11556670/f0d37c17-0bba-4ecb-ad15-4041fab30a62">

# 2. LoRaWAN Network IDs 

- Example based on connecting to The Things Network (TTN) for Europe EU868 Band
- Create a free TTN User account (not the Things Stack)
  + Become familiar with the basics of LoRa and LoRaWAN https://www.thethingsnetwork.org/docs/lorawan/

- Create an application for your devices

- Registering a new device
  + Click "_Enter the device specifics manually_" radio button
  + Click the frequency plan "_Europe 863-870 MHz (SF9 for RX2 - recommended)_"
  + Click the LoRaWAN version "_LoRaWAN Specification 1.0.2_"
  + Click the Regional Parameters version "_RP001 Regional Parameters 1.0.2 revision B_"
  + Create a unique 64 bit JoinEUI for your device (can be shared amongst devices, can be random)  
  + Click "_Generate_" to create a 64-bit random DevEUI
  + Click "_Generate_" to create a 128-bit random AppKEY (can be shared amongst devices)
    + Keep the JoinEUI, DevEUI, and AppKEY to copy into the Arduino example file  
  + Click "_Register End Device_" to create your new device

<img width="818" alt="Screen Shot 2023-12-31 at 3 21 52 PM" src="https://github.com/trollock/solar-lorawan/assets/11556670/91a278fa-6ada-405c-8e89-6b2317f81165">

# 3. Programming 

- programmed using a combination of Arduino, STM32CubeProgrammer, and an ST-Link.
  + *.elf files are first generated in Arduino, then loaded into STM32CubeProgrammer connected via an ST-Link 
- built on the Arduino core found here https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
  + comfirmed using a Mac running macOS Big Sur
- basic arduino example found in subfolder XXXX
  + can be modified and them compiled

- Open the default example in Arduino IDE found in subfolder XXXX 
- Copy and paste the correct JoinEUI, DevEUI, and AppKEY created earlier under "_Registering a new device_"

<img width="1018" alt="Screen Shot 2023-12-31 at 7 35 50 PM" src="https://github.com/trollock/solar-lorawan/assets/11556670/dd22c578-f315-43bc-a63c-640bd3040f52">

- Basic programming example
  + open the basic arduino example with the correct JoinEUI, DevEUI, and AppKEY created earlier under "_Registering a new device_"
  + click "verify" and then find path to location of *.elf file
  + before opening STM32CubePorgrammer, connect the ST-Link to a suitable port
    + confirm the ST-Link is connected in STM32CubeProgrammer in the upper right under "ST-Link configuration"
  + provide 3.3v power to the device and click "Connect" in STM32CubeProgrammer
  + click "Open File" in STM32CubeProgrammer and find the location of the *.elf file generated earlier by Arduino
  + click "Download" in STM32CubeProgrammer and wait ~ 2-3 seconds until finished 
  + click "Disconnect" in STM32CubeProgrammer
 
  + see image below
 
  ![compile](https://github.com/trollock/solar-lorawan/assets/11556670/dce051d2-2a14-4cbf-ad8e-1987627be524)

# 4. 3D Printed Housing

- design files for 3D printed housings found in subfolder https://github.com/trollock/solar-lorawan/tree/main/6_housings
- printed on Formlabs Form 3+ / 3B+ at a 0.05 mm micron layer height
- *.stl file sliced in Preform 3.33.1 

# 5. Assembly

# 6. Data Access

- log into your new TTN account and click on your application
- 

- files for accessing stored
