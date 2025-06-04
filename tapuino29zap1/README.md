## Tapuino - tweaked version (added support for compressed file loading)

The C64 tape emulator uses an Arduino pro mini (requires ATmega328 2KB hardware).

### Compression Tool
A supporting compression tool can be found here:  
executable - https://github.com/titmouse001/C64TapCompressionTool/releases/tag/v1.0.0.1  
code - https://github.com/titmouse001/C64TapCompressionTool  

----
This project is a fork of the [Original Tapuino].

Here's a list of the extra functionality/changes:
- Added compression support
  - now loads compressed (packed) tap files
  - tap files can be compressed with the [Tap Compression Tool]
    - compressed files are called .zap files
    - compressed files must keep the .zap filename extension
  - tap files are still supported - you can mix both tap and zap files together
  - Example savings: Zybex.tap (881KB) -> Zybex.zap (89KB)
- Replaced 9x9 font with a smaller 5x7
  - includes a dedicated arrow for indicating off-screen text
- Fixed minor scrolling filename bug
- Tweaked display logic

## DIY Tapuino

How do I build my own Tapuino?  Full instructions here: http://sweetlilmre.blogspot.com/2015/03/building-tapuino-r2.html

| Running from a C64 |
 
[Original Tapuino]:https://github.com/sweetlilmre/tapuino
[Tap Compression Tool]:https://github.com/titmouse001/Tapuino-C64TapPackerTool

* V2.51 Menu option for Motor control so it can be used with tzxduino handware to feed a zx-uno
* V2.9zap1 changed files: //**

memstrings.h
memstrings-de.c
memstrings-en.c
memstrings-es.c
memstrings-it.c
memstrings-tr.c
menu.c
tapuino.c
tapuino.h
version.h
--------------

memstrings.h
L35: extern const char S_OPTION_MOTOR_MASK[];		//**

memstrings-de.c
L33: const char S_OPTION_MOTOR_MASK[] PROGMEM = "Motor Mask";		//**

memstrings-en.c
L33: const char S_OPTION_MOTOR_MASK[] PROGMEM = "Motor Mask";		//**

memstrings-es.c
L33: const char S_OPTION_MOTOR_MASK[] PROGMEM = "Motor Mask";		//**

memstrings-it.c
L33: const char S_OPTION_MOTOR_MASK[] PROGMEM = "Motor Mask";		//**

memstrings-tr.c
L32: const char S_OPTION_MOTOR_MASK[] PROGMEM = "Motor Mask";		//**

menu.c
L29: #define OPTION_MOTOR_MASK 8		//**
L372: const char* ppitems[] = {S_OPTION_MACHINE_TYPE, S_OPTION_VIDEO_MODE, S_OPTION_SIGNAL, S_OPTION_KEY_REPEAT, S_OPTION_TICKER_SPEED, S_OPTION_TICKER_HOLD, S_OPTION_REC_FINALIZE, S_OPTION_REC_AUTO_FINALIZE, S_OPTION_MOTOR_MASK}; //**
L378: cur_mode = handle_select_mode(S_MODE_OPTIONS, ppitems, 9, cur_mode); //**
L447: case OPTION_MOTOR_MASK:	//**
L448: value = g_motor_mask;	//**  
L449: if (handle_option_value(S_OPTION_MOTOR_MASK, &value, 0, 1,1)) {	//**
L450: g_motor_mask = value;	//**
L451: save = 1;	//**
L452: }	//**
L453: break;	//**	

tapuino.c
L151: uint8_t g_motor_mask = MOTOR_MASK;	//**
L787: g_motor_mask = eeprom_read_byte((uint8_t *) 9);	//**
L801: eeprom_update_byte((uint8_t *) 9, g_motor_mask);  //**

tapuino.h
L21: extern uint8_t g_motor_mask;	//**

version.h
L6: #define TAPUINO_BUILD_VERSION 1		//**


