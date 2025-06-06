/********************************************************************************/
// User selectable configuration settings
//
/********************************************************************************/

/********************************************************************************/
// LCD Definitions
/********************************************************************************/

// uncomment one of these sets for your specific LCD

// #define LCD_USE_1602_LCD_MODULE
//#define LCD_I2C_ADDR        0x3F // I2C address for the LCD
 #define LCD_I2C_ADDR        0x27 // I2C address for the LCD

 #define LCD_USE_SSD1306_OLED_MODULE
 #define LCD_I2C_ADDR        0x3C // I2C address for the OLED
// #define LCD_SSD1306_BIG_FONTS    // define this for ... bigger fonts...
// choose one of these depending on your display
// #define LCD_SSD1306_128x64
 #define LCD_SSD1306_128x32


// #define LCD_USE_SSD131X_OLED_MODULE
// #define LCD_I2C_ADDR        0x3C // I2C address for the OLED

/********************************************************************************/
// Language Definitions
/********************************************************************************/

// uncomment one of these for your language

// #define TAPUINO_LANGUAGE_EN
// #define TAPUINO_LANGUAGE_IT
// #define TAPUINO_LANGUAGE_TR
#define TAPUINO_LANGUAGE_ES
// #define TAPUINO_LANGUAGE_DE
