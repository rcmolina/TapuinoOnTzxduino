# Tapuino

This project is now in maintanence mode and has been superseeded by the [Tapuino Next](https://github.com/sweetlilmre/TapuinoNext)

However amazing people keep building this version and adding value (see the [3D printable enclosure below](#3d-printable-enclosure))

Limited support can be found in the following Amibay forum [Amibay](https://www.amibay.com/threads/tapuino-the-20-c64-tape-emulator.64874)

## The $20 C64 Tape Emulator

Details can be found on my blog: http://www.sweetlilmre.com

The FAQ and other docs can be found on the wiki: https://github.com/sweetlilmre/tapuino/wiki

A really nice board design can be found here: https://github.com/arananet/tapuinomini1.03

And a write up here: http://arananet-net.kinja.com/tapuino-1-03-mejorando-el-tapuino-mini-1788202676

These schematics are also provided in the "layout" folder as a convenience. 

Portions of this code are copyright by their respective owners:

 * PetitFatfs: Chan: http://elm-chan.org/fsw/ff/00index_p.html
 * I2C code: Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
 * LCD code: conversion of the Arduino code apparently by DFRobot? (converted to C by me)


## 3D printable enclosure
by @fabriziofiorucci

A [3D printable enclosure](CAD) designed with Fusion 360 is available for download as .stl files

* V2.51 Menu option for Motor control so it can be used with tzxduino handware to feed a zx-uno
* V2.11a changed files: //**

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
L362: const char* ppitems[] = {S_OPTION_MACHINE_TYPE, S_OPTION_VIDEO_MODE, S_OPTION_SIGNAL, S_OPTION_KEY_REPEAT, S_OPTION_TICKER_SPEED, S_OPTION_TICKER_HOLD, S_OPTION_REC_FINALIZE, S_OPTION_REC_AUTO_FINALIZE, S_OPTION_MOTOR_MASK}; //**
L368: cur_mode = handle_select_mode(S_MODE_OPTIONS, ppitems, 9, cur_mode); //**
L437: case OPTION_MOTOR_MASK:	//**
L438: value = g_motor_mask;	//**  
L439: if (handle_option_value(S_OPTION_MOTOR_MASK, &value, 0, 1,1)) {	//**
L440: g_motor_mask = value;	//**
L441: save = 1;	//**
L442: }	//**
L443: break;	//**	

tapuino.c
L85: uint8_t g_motor_mask = MOTOR_MASK;	//**
L625: g_motor_mask = eeprom_read_byte((uint8_t *) 9);	//**
L640: eeprom_update_byte((uint8_t *) 9, g_motor_mask);  //**

tapuino.h
L21: extern uint8_t g_motor_mask;	//**

version.h
L6: #define TAPUINO_BUILD_VERSION 1		//**


