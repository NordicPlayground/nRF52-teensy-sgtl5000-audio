
# nRF52-teensy-sgtl5000-audio

This repository contains example source code that can be used together with a nRF52-DK
to interface and use a Teensy SGTL5000 Audio adapter board. A Teensy Arduino shield with 
some modifications is used to connect the audio adapter to the DK.


## Note
I2S MCLK Speed: The nRF52832 is set up to run I2S at 4MHz during normal I2S operation (the maximum of 
nRF52832), while the datasheet for the SGTL5000 specifies 8MHz as the minimum (at least as far as I 
could tell). When configuring the SGTL5000, the I2S MCLK speed is raised to above the nRF52832 defined 
max speed. This has been tested and works for this demo purpose, but for a final produc, you might want 
to stay within the specified limits of the audio device that you are going to use. 

AUDIO VOLUME! The volume output of this demo could be fairly HIGH depending on the microphone/speakers
that are being used. Please be aware of this and don't put your headphones into your ears before you 
have tested the output volume level!


## Running the example
In order to run this example on the nRF52-DK (nRF52832-DK), certain things has to be set up.

Please note the volume level warning as noted earlier. Depending on your microphone and speakers, 
certain dB settings in the header/source driver files should be changed. 

### Prepare the Teensy audio adapter
TODO: How to prepare the Teensy audio adapter.

### Prepare the Teensy Arduino Shield
TODO: How to prepate the Teensy Audio Shield

### Run the example
TODO: Describe how the demo can be run, and what to expect.


## Compiling and Developing
This example is built upon SDK v15.0.0. This can be downloaded here: TODO INSERT LINK. 

The example assumes a location matching 
"...\nRF5_SDK_15.0.0_a53641a\examples\peripheral\nRF52-teensy-sgtl5000-audio\README.md"
for this readme file.

TODO: Fill in more information about header files, source, etc. Explain how things are done and how things
work. 


## nRF52832 Driver Resource Usage
The SGTL5000 driver will use TWI instance 1 (can be modified to use another in the header), 
and also the I2S peripheral. It will also use EGU instance 3 in order to stop the I2S 
peripheral when needed.

## nRF52832 Driver GPIO Usage
All GPIOs are predefined in the drv_sgtl5000.h header file. See list below.

TODO: @LIST OF GPIO usage goes here...


## Audio Conversion
To convert the audio recording used as a sample in this example to a .wav file, the following 
tool was used https://audio.online-convert.com/convert-to-wav.

To convert the .wav file to a sample .c source file, the python script Convert Audio Sample\conv.py
can be used. Follow the instructions as specified within that folder to create the sample .c file. 