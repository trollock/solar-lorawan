# Repository Introduction

The following repository contains all of the necessary hardware, software, TTN interface, 3D printing files, and assembly instructions for the CMWX-based LoRaWAN GPS radiotransmitter.

# 1. Hardware 

- datasheets for individual components found in subfolder https://github.com/trollock/solar-lorawan/tree/main/3_hardware

- design files found in subfolder https://github.com/trollock/solar-lorawan/tree/main/5_design_files
- produced in Autodesk Fusion 360 in *.fsch, *.fbrd, and *.f3z formats
  
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
  
  + see example image below 

<img width="818" alt="Screen Shot 2023-12-31 at 3 21 52 PM" src="https://github.com/trollock/solar-lorawan/assets/11556670/91a278fa-6ada-405c-8e89-6b2317f81165">

# 3. Programming 

- programmed using a combination of Arduino, STM32CubeProgrammer, and an ST-Link.
  + *.elf files are first generated in Arduino, then loaded into STM32CubeProgrammer connected via an ST-Link 
- built on the Arduino core found here https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
  + comfirmed using a Mac running macOS Big Sur
- basic arduino example found in subfolder XXXX
  + can be modified and them compiled

## Basic programming example

- Open the default example in Arduino IDE found in subfolder "sample_software" https://github.com/trollock/solar-lorawan/tree/main/4_software
- Copy and paste the correct JoinEUI, DevEUI, and AppKEY created earlier under "_Registering a new device_"
  + This can be found on lines 42 - 46 of the example *.ino file provided.
 
  + see example image below

<img width="1015" alt="Screen Shot 2024-02-09 at 1 42 21 PM" src="https://github.com/trollock/solar-lorawan/assets/11556670/0e7e3544-53ca-4ae1-9cc9-881eca0b7467">

  + click "verify" and then find path to location of *.elf file
  + before opening STM32CubePorgrammer, connect the ST-Link to a suitable port
    + confirm the ST-Link is connected in STM32CubeProgrammer in the upper right under "ST-Link configuration"
  + provide 3.3v power to the device and click "Connect" in STM32CubeProgrammer
  + click "Open File" in STM32CubeProgrammer and find the location of the *.elf file generated earlier by Arduino
  + click "Download" in STM32CubeProgrammer and wait ~ 2-3 seconds until finished 
  + click "Disconnect" in STM32CubeProgrammer
 
  + see example image below
 
  ![compile](https://github.com/trollock/solar-lorawan/assets/11556670/dce051d2-2a14-4cbf-ad8e-1987627be524)

# 4. 3D Printed Housing

- design files for 3D printed housings found in subfolder https://github.com/trollock/solar-lorawan/tree/main/6_housings
- printed on Formlabs Form 3+ / 3B+ at a 0.05 mm micron layer height
- *.stl file sliced in Preform 3.33.1
  + sample print file with supports provided

- Version A for IXOLAR KXOB based solar cells (with PCB adapter)

![Housing_A](https://github.com/trollock/solar-lorawan/assets/11556670/72a8ed96-9c64-460b-a734-0ed86e181711)

- Version B for raw GaInAs triple junction solar cells (no adapter necessary)


# 5. Assembly

# 6. Data Access

- log into your new TTN account and click on your application

- Payload Formatter
  + copy the data from the file "payload_formatter_uplink.txt" in the subfolder 2_scripts https://github.com/trollock/solar-lorawan/tree/main/2_scripts
  + click "formatter type" and select Custom Javascript formatter.
  + Delete the code in the paylod formatter box
  + Paste the data from the file "payload_formatter_uplink.txt"
  + Click "Save Changes"

  + see example image below
    
<img width="793" alt="Screen Shot 2024-02-09 at 1 27 04 PM" src="https://github.com/trollock/solar-lorawan/assets/11556670/c374ceef-fe46-4c9f-a58a-26451f527281">

- Storage Integration
  + Click on the leftside menu "Integrations" and then click "Storage Integration"
  + Cliack on the radio button "Activate Storage Integration"
  + The status should say "The storage integration is currently activated"

- API Keys
  + API keys are used for accessing the storage API in a secure manner from another process or service (e.g. [R] )
  + Click the radio button "+ Add API Key"
  + Save this information in a secure place for later
 
- 
- files for accessing stored
