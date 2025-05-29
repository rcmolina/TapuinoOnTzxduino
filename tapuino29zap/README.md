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

| ![/Images/DIY-Tapuino.png](/Images/DIY-Tapuino.png) | 
|:--:| 
| DIY Tapuino |

| ![/Images/DIY-Tapuino.png](/Images/DIY-Tapuino-Running.png) | 
|:--:| 
| Running from a C64 |
 
[Original Tapuino]:https://github.com/sweetlilmre/tapuino
[Tap Compression Tool]:https://github.com/titmouse001/Tapuino-C64TapPackerTool
