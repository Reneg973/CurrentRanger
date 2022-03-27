// CurrentRanger(TM) stock firmware
// https://lowpowerlab.com/CurrentRanger
// CurrentRanger is a *high-side* precision current meter featuring:
//   - fast autoranging
//   - uni/bi-directional modes (ie. DC/AC measurements)
//   - ultra low burden voltage
//   - 1mV per nA/uA/mA measurements with DMM/scope
//   - OLED standalone readings
//   - serial data logging option via 3.3v/RX/TX header or USB (must use isolation, read guide!)
//   - full digital control for power/switching
//   - LiPo powered with auto power-off feature (0.6uA quiescent current)
// *************************************************************************************************************
#ifndef CURRENT_RANGER
  #error CurrentRanger target board required, see guide on how to add it to the IDE: lowpowerlab.com/currentranger
#endif
//***********************************************************************************************************
//#include <ATSAMD21_ADC.h>
#include <Adafruit_FreeTouch.h>    //https://github.com/adafruit/Adafruit_FreeTouch
#include <FlashStorage.h>          //for emulated EEPROM - https://github.com/cmaglie/FlashStorage
#include <U8g2lib.h>               //https://github.com/olikraus/u8g2/wiki/u8g2reference fonts:https://github.com/olikraus/u8g2/wiki/fntlistall
#include <Wire.h>                   //i2c scanner: https://playground.arduino.cc/Main/I2cScanner
#include <algorithm>
#include <array>
#include <initializer_list>

// CurrentRanger Firmware Version
#define FW_VERSION "1.1.3"

//***********************************************************************************************************
#define BIAS_LED       11u
#define LPFPIN         4u             //LPF control pin
#define LPFLED         LED_BUILTIN    //STATUS/LPF-LED
#define AUTOFF         PIN_AUTO_OFF
//***********************************************************************************************************
#define MA_PIN PIN_PA13  //#define MA  38
#define UA_PIN PIN_PA14  //#define UA  2
#define NA_PIN PIN_PA15  //#define NA  5
#define MA_GPIO_PIN PIN_PB11
#define UA_GPIO_PIN PIN_PA12
#define NA_GPIO_PIN PIN_PB10
#define PINOP(pin, OP) (PORT->Group[(pin) / 32].OP.reg = (1 << ((pin) % 32)))
#define PIN_OFF(THE_PIN) PINOP(THE_PIN, OUTCLR)
#define PIN_ON(THE_PIN) PINOP(THE_PIN, OUTSET)
#define PIN_TGL(THE_PIN) PINOP(THE_PIN, OUTTGL)
//***********************************************************************************************************
#define SENSE_OUTPUT           A3
#define SENSE_GNDISO           A2  //GND-ISO
#define SENSE_VIN              A5  //VIN > 1MEG > SENSE_VIN > 2MEG > GND
#define ADC_PRESCALER          ADC_CTRLB_PRESCALER_DIV16
//#define ADC_AVGCTRL            ADC_AVGCTRL_SAMPLENUM_128 | ADC_AVGCTRL_ADJRES(0x4ul)
                               //ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x00ul);  // take 1 sample, adjusting result by 0
                               //ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0x4ul); //take 16 samples adjust by 4
                               //ADC_AVGCTRL_SAMPLENUM_256 | ADC_AVGCTRL_ADJRES(0x4ul); //take 256 samples adjust by 4
                               //ADC_AVGCTRL_SAMPLENUM_512 | ADC_AVGCTRL_ADJRES(0x4ul); //take 512 samples adjust by 4
                               //ADC_AVGCTRL_SAMPLENUM_1024 | ADC_AVGCTRL_ADJRES(0x4ul); //take 1024 samples adjust by 4
#define ADC_SAMPCTRL           0b111 //sample timing [fast 0..0b111 slow]
#define ADCFULLRANGE           4095.0
#define VBAT_REFRESH_INTERVAL  5000 //ms
#define LOBAT_THRESHOLD        3.40 //volts
#define DAC_GND_ISO_OFFSET     10
#define DAC_HALF_SUPPLY_OFFSET 512
#define OUTPUT_CALIB_FACTOR    1.00  //calibrate final VOUT value
#define ADC_OVERLOAD           3900  //assuming GNDISO DAC output is very close to 0, this is max value less ground offset (varies from unit to unit, 3900 is a safe value)
//***********************************************************************************************************
//#define ADC_CALIBRATE_FORCED
#define ADC_CALIBRATE_FORCED_OFFSET 0
#define ADC_CALIBRATE_FORCED_GAIN   2048
#define LDO_DEFAULT                 3.300 //volts, change to actual LDO output (measure GND-3V on OLED header)
//***********************************************************************************************************
#define BUZZER    1u   // BUZZER pin
#define NOTE_C5   523
#define NOTE_D5   587
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_G5   784
#define NOTE_B5   988
#define NOTE_C6   1047
#define TONE_BEEP 4200
//***********************************************************************************************************
#define MODE_MANUAL                 0
#define MODE_AUTORANGE              1
#define STARTUP_MODE                MODE_MANUAL //or: MODE_AUTORANGE
#define SWITCHDELAY_UP              8 //ms
#define SWITCHDELAY_DOWN            8 //ms
#define RANGE_SWITCH_THRESHOLD_HIGH ADC_OVERLOAD //ADC's 12bit value
#define RANGE_SWITCH_THRESHOLD_LOW  6 //6*0.4xA ~ 2.4xA - range down below this value
//***********************************************************************************************************
#define OLED_BAUD                   1600000 //fast i2c clock
#define OLED_ADDRESS                0x3C    //i2c address on most small OLEDs
#define OLED_REFRESH_INTERVAL       180     //ms
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//***********************************************************************************************************
#define TOUCH_N        8
#define TOUCH_U        9
#define TOUCH_M        A4
Adafruit_FreeTouch qt[3] = {
  Adafruit_FreeTouch( TOUCH_N, OVERSAMPLE_1, RESISTOR_50K, FREQ_MODE_NONE ),
  Adafruit_FreeTouch( TOUCH_U, OVERSAMPLE_1, RESISTOR_50K, FREQ_MODE_NONE ),
  Adafruit_FreeTouch( TOUCH_M, OVERSAMPLE_1, RESISTOR_50K, FREQ_MODE_NONE ),
};
#define TOUCH_HIGH_THRESHOLD  400 //range is 0..1023
#define TOUCH_SAMPLE_INTERVAL 100 //ms
//***********************************************************************************************************
#define SERIAL_UART_BAUD        230400      //Serial baud for HC-06/bluetooth output
//#define BT_SERIAL_EN
//#define LOGGER_FORMAT_EXPONENT  //ex: 123E-3 = 123mA
//#define LOGGER_FORMAT_NANOS     //ex: 123456 = 123456nA = 123.456uA
//#define LOGGER_FORMAT_ADC       //raw ADC output - note: automatic ADC_REF change
#define BT_REFRESH_INTERVAL     200 //ms
//***********************************************************************************************************
#define AUTOOFF_BUZZ_DELAY     500 //ms
#define AUTOOFF_DEFAULT        600 //seconds, turn unit off after 10min of inactivity
#define AUTOOFF_DISABLED       0xFFFF  // do not turn off
#define AUTOOFF_SMART          0xFFFE  // turn off only if there is no BT or USB data logging
//***********************************************************************************************************
#define LOGGING_FORMAT_EXPONENT 0 //ex: 123E-3 = 123mA
#define LOGGING_FORMAT_NANOS    1 //ex: 1234 = 1.234uA = 0.001234mA
#define LOGGING_FORMAT_MICROS   2 //ex: 1234 = 1.234mA = 1234000nA
#define LOGGING_FORMAT_MILLIS   3 //ex: 1234 = 1.234A = 1234000uA = 1234000000nA
#define LOGGING_FORMAT_ADC      4 //raw output for each range (0..4095)
#define LOGGING_FORMAT_MAXVAL   5
//***********************************************************************************************************
#define ADC_SAMPLING_SPEED_AVG   0
#define ADC_SAMPLING_SPEED_FAST  1
#define ADC_SAMPLING_SPEED_SLOW  2
#define ADC_SAMPLING_SPEED_MAXVAL 3
//***********************************************************************************************************
#define RANGE_SCALE_MA 'm'
#define RANGE_SCALE_UA 181 // prevent multichar by using code for µ
#define RANGE_SCALE_NA 'n'
#define RANGE_MA (rangeUnit==RANGE_SCALE_MA)
#define RANGE_UA (rangeUnit==RANGE_SCALE_UA)
#define RANGE_NA (rangeUnit==RANGE_SCALE_NA)
//***********************************************************************************************************
uint16_t gainCorrectionValue = 0;
uint16_t autooff_interval = 0;
uint16_t ADC_SAMPLING_SPEED = ADC_SAMPLING_SPEED_AVG;
uint32_t ADC_AVGCTRL;
uint32_t oledNextUpdate=0, lpfInterval=0, offsetInterval=0, autorangeInterval=0, btInterval=0,
         lastKeepAlive=0;
int offsetCorrectionValue = 0;
float ldoValue = 0, ldoOptimized=0;
float vbat=0, VOUT=0;
float readDiff=0;
uint8_t LOGGING_FORMAT = LOGGING_FORMAT_EXPONENT;
char rangeUnit = RANGE_SCALE_MA;
byte LPF=0, BIAS=0, AUTORANGE=0;
bool USB_LOGGING_ENABLED = false;
bool TOUCH_DEBUG_ENABLED = false;
bool GPIO_HEADER_RANGING = false;
bool BT_LOGGING_ENABLED = true;
bool calibrationPerformed=false;
bool analog_ref_half=true;
bool OLED_found=false;
bool autoffWarning=false;
bool autoffBuzz=0;
bool rangeSwitched=false;
#ifdef BT_SERIAL_EN
  bool BT_found=false;
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

FlashStorage(eeprom_ADCoffset, int);
FlashStorage(eeprom_ADCgain, uint16_t);
FlashStorage(eeprom_LDO, float);
FlashStorage(eeprom_AUTOFF, uint16_t);
FlashStorage(eeprom_LOGGINGFORMAT, uint8_t);
FlashStorage(eeprom_ADCSAMPLINGSPEED, uint8_t);

inline void Beep(byte theDelay) {
  tone(BUZZER, TONE_BEEP, theDelay);
}
void Beep2(byte theDelay) {
  Beep(theDelay);
  delay(10);
  tone(BUZZER, 4500, theDelay);
}

inline void setupOled() // called just once, so inline it
{
  Serial.print("startup before Oled takes "); Serial.print(millis()); Serial.println("ms");
  //delay(50); //Wire apparently needs this, but assuming this function called at the end of setup already had a delay of > 50
  Wire.begin();
  Wire.beginTransmission(OLED_ADDRESS);
  byte error = Wire.endTransmission();
  if (error != 0)
  {
    Serial.println("NO OLED found...");
    return;
  }

  Serial.print("OLED FOUND at 0x"); Serial.println(OLED_ADDRESS);
  u8g2.begin();
  //u8g2.setDisplayRotation(U8G2_R2); //if required (inside/custom mount?)
  u8g2.setBusClock(OLED_BAUD);
  OLED_found = true;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_8x13B_tf);
  u8g2.drawStr(15,10,"CurrentRanger");  
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0,20,"Offset:");
  u8g2.setCursor(64,20); u8g2.print(offsetCorrectionValue);
  u8g2.drawStr(0,32,"Gain  :");
  u8g2.setCursor(64,32); u8g2.print(gainCorrectionValue);
  u8g2.drawStr(0,44,"LDO   :");
  u8g2.setCursor(64,44); u8g2.print(ldoValue,3);
  u8g2.drawStr(0, 56,"Firmware:");
  u8g2.drawStr(64,56,FW_VERSION);
  u8g2.sendBuffer();
  oledNextUpdate = millis() + 2000;
}

//***********************************************************************************************************
void setup() {
  for(auto pin : {SENSE_OUTPUT, SENSE_GNDISO, SENSE_VIN})
    pinMode(pin, INPUT);
  pinMode(AUTOFF, INPUT_PULLUP);
  for(auto pin : {unsigned(A0), BIAS_LED, LPFLED, LPFPIN, BUZZER})
    pinMode(pin, OUTPUT);
  for(auto pin : {MA_PIN, UA_PIN, NA_PIN, MA_GPIO_PIN, UA_GPIO_PIN, NA_GPIO_PIN})
    PINOP(pin, DIRSET);

  for(auto& q : qt)
    q.begin(); //touch pads
  analogWriteResolution(10);  //DAC resolution
  analogReferenceHalf(true);

  analogWrite(A0, DAC_GND_ISO_OFFSET);  // Initialize Dac to OFFSET

  autooff_interval = eeprom_AUTOFF.read();
  if (autooff_interval==0) {
    autooff_interval = AUTOOFF_DEFAULT;
    eeprom_AUTOFF.write(autooff_interval);
  }

  LOGGING_FORMAT = eeprom_LOGGINGFORMAT.read();

  offsetCorrectionValue = eeprom_ADCoffset.read();
  gainCorrectionValue = eeprom_ADCgain.read();
  ldoValue = eeprom_LDO.read();

  if(ldoValue==0)
    saveLDO(LDO_DEFAULT);
  else ldoOptimizeRefresh();

  ADC_SAMPLING_SPEED = eeprom_ADCSAMPLINGSPEED.read();
  refreshADCSamplingSpeed(); //load correct value into ADC_AVGCTRL

  if (gainCorrectionValue!=0) //check if anything saved in EEPROM (gain changed via SerialUSB +/-)
    analogReadCorrectionForced(offsetCorrectionValue, gainCorrectionValue);
  else {
    analogReadCorrectionForced(ADC_CALIBRATE_FORCED_OFFSET, ADC_CALIBRATE_FORCED_GAIN);
    eeprom_ADCoffset.write(offsetCorrectionValue);
    eeprom_ADCgain.write(gainCorrectionValue);
    //(offset, gain) - gain is 12 bit number (1 bit integer + 11bit fractional, see DS p895)
    //               - offset is 12bit 2s complement format (DS p896)
  }

#ifdef BT_SERIAL_EN
  //BT check
  Serial.print("Bluetooth AT check @");Serial.print(SERIAL_UART_BAUD);Serial.print("baud...");
  delay(600);
  SerialBT.begin(SERIAL_UART_BAUD);
  SerialBT.print("AT"); //assuming HC-06, no line ending required
  uint32_t timer=millis();
  while(millis()-timer<1000) //about 1s to respond
  {
    if (SerialBT.available()==2 && SerialBT.read()=='O' && SerialBT.read()=='K')
    {
      BT_found=true;
      break;
    }
  }

  Serial.print(BT_found ? "OK!" : "No HC-06 response.\r\nChecking for BT v3.0...");

  if (!BT_found)
  {
    SerialBT.print("\r\n"); //assuming HC-06 version 3.0 that requires line ending
    uint32_t timer=millis();
    while(millis()-timer<50) //about 50ms to respond
    {
      if (SerialBT.available()==4 && SerialBT.read()=='O' && SerialBT.read()=='K' && SerialBT.read()=='\r' && SerialBT.read() == '\n')
      {
        BT_found=true;
        break;
      }
    }

    Serial.println(BT_found?"OK!":"No response.");
  }

  BT_LOGGING_ENABLED = BT_found;
#endif

  printSerialMenu();
  WDTset();
  if (STARTUP_MODE == MODE_AUTORANGE) toggleAutoranging();
  setupOled();
}

inline void handleSerialInput() {
  while (Serial.available()) {
    // tickle the AUTOOFF function so it doesn't shut down when there are commands coming over serial
    lastKeepAlive = millis();
    char const inByte = Serial.read();
    switch (inByte) {
      case '+':
      case '-':
        eeprom_ADCgain.write(gainCorrectionValue + (inByte=='+' ? 1 : -1));
        analogReadCorrection(offsetCorrectionValue,gainCorrectionValue);
        Serial.print("new gainCorrectionValue = ");
        Serial.println(gainCorrectionValue);
        break;
      case '*':
      case '/':
        eeprom_ADCoffset.write(offsetCorrectionValue + (inByte == '*' ? 1 : -1));
        analogReadCorrection(offsetCorrectionValue,gainCorrectionValue);
        Serial.print("new offsetCorrectionValue = ");
        Serial.println(offsetCorrectionValue);
        break;
      case '<':
      case '>':
        saveLDO(ldoValue + (inByte == '>' ? 0.001 : -0.001));
        Serial.print("new LDO_Value = ");
        Serial.println(ldoValue, 3);
        break;
      case 'b': //toggle BT/serial logging
#ifdef BT_SERIAL_EN
        if (BT_found) {
          BT_LOGGING_ENABLED =! BT_LOGGING_ENABLED;
          Serial.println(BT_LOGGING_ENABLED ? "BT_LOGGING_ENABLED" : "BT_LOGGING_DISABLED");
        } else {
          BT_LOGGING_ENABLED = false;
          Serial.println("BT Module not found: cannot enable logging");
        }
#else
        Serial.println("BT_LOGGING Not Enabled");
#endif
        break;
      case 'g': //toggle GPIOs indicating ranging
        GPIO_HEADER_RANGING = !GPIO_HEADER_RANGING;
        if (GPIO_HEADER_RANGING) {
          if (RANGE_MA) PIN_ON(MA_GPIO_PIN); else PIN_OFF(MA_GPIO_PIN);
          if (RANGE_UA) PIN_ON(UA_GPIO_PIN); else PIN_OFF(UA_GPIO_PIN);
          if (RANGE_NA) PIN_ON(NA_GPIO_PIN); else PIN_OFF(NA_GPIO_PIN);
        }
        Serial.println(GPIO_HEADER_RANGING ? "GPIO_HEADER_RANGING_ENABLED" : "GPIO_HEADER_RANGING_DISABLED");
        break;
      case 'r': //reboot to bootloader
        Serial.print("\nRebooting to bootloader.");
        for (byte i=30;i;--i) { delay(10); Serial.print('.'); }
        rebootIntoBootloader();
        break;
      case 't': //toggle touchpad serial output debug info
        TOUCH_DEBUG_ENABLED = !TOUCH_DEBUG_ENABLED;
        Serial.println(TOUCH_DEBUG_ENABLED ? "TOUCH_DEBUG_ENABLED" : "TOUCH_DEBUG_DISABLED");
        break;
      case 'u': //toggle USB logging
        USB_LOGGING_ENABLED = !USB_LOGGING_ENABLED;
        Serial.println(USB_LOGGING_ENABLED ? "USB_LOGGING_ENABLED" : "USB_LOGGING_DISABLED");
        break;
      case 'f': //cycle through output logging formats
        LOGGING_FORMAT = (LOGGING_FORMAT+1) % LOGGING_FORMAT_MAXVAL;
        eeprom_LOGGINGFORMAT.write(LOGGING_FORMAT);
        switch(LOGGING_FORMAT)
        {
        case LOGGING_FORMAT_EXPONENT: Serial.println("LOGGING_FORMAT_EXPONENT"); break;
        case LOGGING_FORMAT_NANOS:    Serial.println("LOGGING_FORMAT_NANOS"); break;
        case LOGGING_FORMAT_MICROS:   Serial.println("LOGGING_FORMAT_MICROS");  break;
        case LOGGING_FORMAT_MILLIS:   Serial.println("LOGGING_FORMAT_MILLIS");  break;
        case LOGGING_FORMAT_ADC:      Serial.println("LOGGING_FORMAT_ADC"); break;
        }
        break;
      case 'o': //toggle OLED updates
        if(oledNextUpdate == 0xFFFFFFFF)
        {
          Serial.println("Enable OLED update");
          oledNextUpdate = millis();
          u8g2.sleepOff();
        }
        else
        {
          Serial.println("Disable OLED update");
          oledNextUpdate = 0xFFFFFFFF;
          u8g2.sleepOn();
        }
        break;
      case 's':
        ADC_SAMPLING_SPEED = (ADC_SAMPLING_SPEED+1) % ADC_SAMPLING_SPEED_MAXVAL;
        switch(ADC_SAMPLING_SPEED)
        {
        case ADC_SAMPLING_SPEED_AVG: Serial.println("ADC_SAMPLING_SPEED_AVG"); break;
        case ADC_SAMPLING_SPEED_FAST: Serial.println("ADC_SAMPLING_SPEED_FAST"); break;
        case ADC_SAMPLING_SPEED_SLOW: Serial.println("ADC_SAMPLING_SPEED_SLOW"); break;
        }
        eeprom_ADCSAMPLINGSPEED.write(ADC_SAMPLING_SPEED);
        refreshADCSamplingSpeed();
        break;
      case 'a': //toggle autoOff function
        if (autooff_interval == AUTOOFF_DEFAULT)
        {
          Serial.println("AUTOOFF_DISABLED");
          autooff_interval = AUTOOFF_DISABLED;
        }
        else if (autooff_interval == AUTOOFF_SMART) {
          Serial.println("AUTOOFF_DEFAULT");
          autooff_interval = AUTOOFF_DEFAULT;
          lastKeepAlive = millis();          
        } else {
          // turn off only when there is no serial or BT data logging
          Serial.println("AUTOOFF_SMART");
          autooff_interval = AUTOOFF_SMART;
        }
        eeprom_AUTOFF.write(autooff_interval);
        break;
      case '1': if (AUTORANGE) toggleAutoranging(); rangeMA(); break;
      case '2': if (AUTORANGE) toggleAutoranging(); rangeUA(); break;
      case '3': if (AUTORANGE) toggleAutoranging(); rangeNA(); break;
      case '4': toggleLPF(); break;
      case '5': toggleOffset(); break;
      case '6': toggleAutoranging(); break;
      case '?':
        printSerialMenu();
        break;
    }
  }
}

void loop() {
  //uint32_t timestamp=micros();
  handleSerialInput();
  if (AUTORANGE) {
    readVOUT();
    //assumes we only auto-range in DC mode (no bias)
    if (readDiff <= RANGE_SWITCH_THRESHOLD_LOW)
    {
      if      (RANGE_MA) { rangeUA(); rangeSwitched=true; rangeBeep(SWITCHDELAY_DOWN); }
      else if (RANGE_UA) { rangeNA(); rangeSwitched=true; rangeBeep(SWITCHDELAY_DOWN); }
    }
    else if (readDiff >= RANGE_SWITCH_THRESHOLD_HIGH)
    {
      if      (RANGE_NA) { rangeUA(); rangeSwitched=true; rangeBeep(SWITCHDELAY_UP); }
      else if (RANGE_UA) { rangeMA(); rangeSwitched=true; rangeBeep(SWITCHDELAY_UP); }
    }
    if (rangeSwitched) {
      lastKeepAlive = millis();
      rangeSwitched = false;
      return; //!!!
    }
  }

  bool VOUTCalculated = false;
  auto fnCalcVOut = [&VOUTCalculated]()
  {
    if (VOUTCalculated) return;
    VOUTCalculated = true;
    if (!AUTORANGE) readVOUT();
    VOUT = readDiff * ldoOptimized * (BIAS ? 1 : OUTPUT_CALIB_FACTOR);
  };

  if (USB_LOGGING_ENABLED)
  {
    fnCalcVOut();
    switch(LOGGING_FORMAT)
    {
      case LOGGING_FORMAT_EXPONENT:  Serial.print(VOUT); Serial.print("e"); Serial.println(RANGE_NA ? -9 : RANGE_UA ? -6 : -3); break;
      case LOGGING_FORMAT_NANOS:     Serial.println(VOUT * (RANGE_NA ? 1    : RANGE_UA ? 1e3  : 1e6)); break;
      case LOGGING_FORMAT_MICROS:    Serial.println(VOUT * (RANGE_NA ? 1e-3 : RANGE_UA ? 1    : 1e3)); break;
      case LOGGING_FORMAT_MILLIS:    Serial.println(VOUT * (RANGE_NA ? 1e-6 : RANGE_UA ? 1e-3 : 1)); break;
      case LOGGING_FORMAT_ADC:       Serial.println(readDiff, 0); break;
    }
  }

  
#ifdef BT_SERIAL_EN
  if (BT_LOGGING_ENABLED) {
    if (OLED_found) {
      u8g2.setFont(u8g2_font_siji_t_6x10); //https://github.com/olikraus/u8g2/wiki/fntgrpsiji
      u8g2.drawGlyph(104, 10, 0xE00B); //BT icon
    }

    btInterval = millis();
    fnCalcVOut();
    switch(LOGGING_FORMAT)
    {
      case LOGGING_FORMAT_EXPONENT: SerialBT.print(VOUT); SerialBT.print("e"); SerialBT.println(RANGE_NA ? -9 : RANGE_UA ? -6 : -3); break;
      case LOGGING_FORMAT_NANOS:    SerialBT.println(VOUT * (RANGE_NA ? 1 : RANGE_UA ? 1000 : 1000000)); break;
      case LOGGING_FORMAT_MICROS:   SerialBT.println(VOUT * (RANGE_NA ? 0.001 : RANGE_UA ? 1 : 1000)); break;
      case LOGGING_FORMAT_MILLIS:   SerialBT.println(VOUT * (RANGE_NA ? 0.000001 : RANGE_UA ? 0.001 : 1)); break;
      case LOGGING_FORMAT_ADC:      SerialBT.println(readDiff, 0); break;
    }
  }
#endif

  //OLED refresh: ~22ms (SCK:1.6mhz, ADC:64samples/DIV16/b111)
  uint32_t const m = millis();
  if (OLED_found && m > oledNextUpdate) //refresh rate (ms)
  {
    oledNextUpdate = m + OLED_REFRESH_INTERVAL;
    fnCalcVOut();
    u8g2.clearBuffer(); //175us
    u8g2.setFont(u8g2_font_6x12_tf); //7us

    handleVbatRead();
    //                               ---,              5%,    20%,    40%,    60%,    80%,   100%, Charging%
    float const vbat_voltages[]{       0, LOBAT_THRESHOLD,   3.65,   3.75,   3.85,   3.95,    4.1,      6};
    uint16_t const bat_glyphs[]{  0xE242,          0xE243, 0xE244, 0xE245, 0xE247, 0xE249, 0xE24B,      0xE23A};
    u8g2.setFont(u8g2_font_siji_t_6x10);
    auto glyph = bat_glyphs[std::distance(std::begin(vbat_voltages), std::lower_bound(std::begin(vbat_voltages), std::end(vbat_voltages), vbat))];
    u8g2.drawGlyph(115, 10, glyph); //u8g2.drawStr(88,12,"LoBat!");

    uint16_t xCursor;
    u8g2.setFont(u8g2_font_6x12_tf); //7us
    if (AUTORANGE) {
      u8g2.drawStr(0,12, analog_ref_half ? "AUTO\xb7\xbd" : "AUTO");
      xCursor = 42;
    } else {
      if (analog_ref_half) u8g2.drawStr(0,12,"\xbd");
      xCursor = 12;
    }
    u8g2.setCursor(xCursor,12); u8g2.print(readDiff, 0);

    if (autoffBuzz) u8g2.drawStr(5,26,"* AUTO OFF! *"); //autoffWarning
    u8g2.setFont(u8g2_font_helvB24_te);
    char str[2] = "";
    str[0] = rangeUnit;
    auto w = u8g2.getStrWidth(str);
    u8g2.setFontPosBottom();
    u8g2.setCursor(u8g2.getWidth() - w, 64); u8g2.print(rangeUnit);
    u8g2.setFont(u8g2_font_logisoso32_tr);
    float absVout = abs(VOUT);
    u8g2.setCursor(0,64); u8g2.print((BIAS && absVout>=0.4 || !BIAS && VOUT>=0.4) ? VOUT : 0, absVout >= 1000 ? 0 : 1);
    if (!BIAS && readDiff > ADC_OVERLOAD || BIAS && abs(readDiff) > ADC_OVERLOAD/2)
    {
      u8g2.setFont(u8g2_font_9x15B_tf);
      u8g2.drawStr(0,28, "OVERLOAD!");
    }
    u8g2.sendBuffer(); // ~12 ms
  } // ~17 ms

  WDTclear();
  handleTouchPads(); //~112uS
  handleAutoOff();
  //Serial.println(micros()-timestamp);
} //loop()

void handleVbatRead() {
  //limit how often we read the battery since it's not expected to change a lot
  static uint32_t vbatNextUpdate = 0;
  uint32_t m = millis();
  if (m < vbatNextUpdate) return;
  vbatNextUpdate = m + VBAT_REFRESH_INTERVAL;
  if (analog_ref_half) analogReferenceHalf(false);
  vbat=adcRead(SENSE_VIN)/ADCFULLRANGE * ldoValue * 1.5; //1.5 given by vbat->A5 resistor ratio (1 / (2M * 1/(1M+2M))) 

/*
  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[SENSE_VIN].ulADCChannelNumber;
  ADC->INPUTCTRL.bit.MUXNEG = 0x19;//ioGND
  adcRead(); //discard first reading
  vbat = adcRead();
  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[SENSE_OUTPUT].ulADCChannelNumber;
  ADC->INPUTCTRL.bit.MUXNEG = g_APinDescription[SENSE_GNDISO].ulADCChannelNumber;
  syncADC();
*/
}

uint16_t valM=0, valU=0, valN=0;
void handleTouchPads() {
  static uint32_t touchSampleNextUpdate = 0;
  uint32_t m = millis();
  if (m < touchSampleNextUpdate) return;
  touchSampleNextUpdate = m + TOUCH_SAMPLE_INTERVAL;

  if (TOUCH_DEBUG_ENABLED) {
    for(int i=2; i>=0; --i) {
      Serial.print(qt[i].measure()); Serial.print('\t');
    }
    Serial.println();
  }

  uint8_t code = 0;
  for(int i=2; i>=0; --i)
    code |= (qt[i].measure() > TOUCH_HIGH_THRESHOLD) * (1 << i);

  if (code) lastKeepAlive = m;

  //range switching
  if (!AUTORANGE) {
    if      (code == 0b100) {if (!RANGE_MA) { rangeMA(); rangeBeep(20); }}
    else if (code ==  0b10) {if (!RANGE_UA) { rangeUA(); delay(100); rangeBeep(20); }}
    else if (code ==   0b1) {if (!RANGE_NA) { rangeNA(); rangeBeep(20); }}
  }

  //LPF activation --- [NA+UA]
  if      (code == 0b11 && millis()-lpfInterval>1000) { toggleLPF(); Beep(3); }
  //offset toggling (GNDISO to half supply) --- [MA+UA]
  else if (code == 0b110 && millis()-offsetInterval>1000) { toggleOffset(); Beep(3); }
  //AUTORANGE toggling
  else if (code == 0b101 && millis()-autorangeInterval>1000) { toggleAutoranging(); Beep(20); delay(50); Beep(20); }
}

void rangeMA() {
  rangeUnit = RANGE_SCALE_MA;
  PIN_ON(MA_PIN);
  PIN_OFF(UA_PIN);
  PIN_OFF(NA_PIN);
  if (GPIO_HEADER_RANGING) {
    PIN_ON(MA_GPIO_PIN);
    PIN_OFF(UA_GPIO_PIN);
    PIN_OFF(NA_GPIO_PIN);
  }
  analogReferenceHalf(true);
#ifdef BT_OUTPUT_ADC
  if (BT_found) SerialBT.println("RANGE: MA");
#endif
}

void rangeUA() {
  rangeUnit = RANGE_SCALE_UA;
  PIN_OFF(MA_PIN);
  PIN_ON(UA_PIN);
  PIN_OFF(NA_PIN);
  if (GPIO_HEADER_RANGING) {
    PIN_OFF(MA_GPIO_PIN);
    PIN_ON(UA_GPIO_PIN);
    PIN_OFF(NA_GPIO_PIN);
  }
  analogReferenceHalf(true);
#ifdef BT_OUTPUT_ADC
  if (BT_found) SerialBT.println("RANGE: UA");
#endif
}

void rangeNA() {
  rangeUnit = RANGE_SCALE_NA;
  PIN_OFF(MA_PIN);
  PIN_OFF(UA_PIN);
  PIN_ON(NA_PIN);
  if (GPIO_HEADER_RANGING) {
    PIN_OFF(MA_GPIO_PIN);
    PIN_OFF(UA_GPIO_PIN);
    PIN_ON(NA_GPIO_PIN);
  }
  analogReferenceHalf(true);
#ifdef BT_OUTPUT_ADC
  if (BT_found) SerialBT.println("RANGE: NA");
#endif
}

void handleAutoOff() {
  uint32_t autooff_deadline = uint32_t((autooff_interval == AUTOOFF_SMART && !(USB_LOGGING_ENABLED || BT_LOGGING_ENABLED)) ? AUTOOFF_DEFAULT : autooff_interval) * 1000;
  if (vbat > 4.3)  // increase autooff time when connected to USB
    autooff_deadline *= 10;
  autooff_deadline += lastKeepAlive;
  uint32_t const m = millis();
  if (m > autooff_deadline - 5*1000) {
    autoffWarning = true;

    static uint32_t autoOffBuzzNextUpdate = 0;
    if (m > autoOffBuzzNextUpdate)
    {
      autoOffBuzzNextUpdate = m + AUTOOFF_BUZZ_DELAY;
      autoffBuzz = !autoffBuzz;

      autoffBuzz ? tone(BUZZER, NOTE_B5) : noTone(BUZZER);
    }

    if (m > autooff_deadline) {
      pinMode(AUTOFF, OUTPUT);
      digitalWrite(AUTOFF, LOW);
    }
  }
  else if (autoffWarning) { autoffWarning=autoffBuzz=false; digitalWrite(AUTOFF, HIGH); noTone(BUZZER); }
}

void toggleLPF() {
  LPF = !LPF;
  lpfInterval = millis();
  digitalWrite(LPFPIN, LPF);
  digitalWrite(LPFLED, LPF);
  if (AUTORANGE && !LPF) toggleAutoranging(); //turn off AUTORANGE
}

void toggleOffset() {
  BIAS = !BIAS;
  offsetInterval = millis();
  analogWrite(A0, (BIAS ? DAC_HALF_SUPPLY_OFFSET : DAC_GND_ISO_OFFSET));
  digitalWrite(BIAS_LED, BIAS);
  if (AUTORANGE && BIAS) toggleAutoranging(); //turn off AUTORANGE
  analogReferenceHalf(false);
}

void toggleAutoranging() {
  autorangeInterval = millis();
  AUTORANGE = !AUTORANGE;
  if (AUTORANGE && BIAS) toggleOffset(); //turn off BIAS
  if (AUTORANGE && !LPF) toggleLPF(); //turn on BIAS
}

static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while(ADC->STATUS.bit.SYNCBUSY == 1);
}

inline void setupADC() {
  ADC->CTRLA.bit.ENABLE = 0;              // disable ADC
  syncADC();
  ADC->REFCTRL.bit.REFCOMP = 1;
  ADC->CTRLB.reg = ADC_PRESCALER | ADC_CTRLB_RESSEL_12BIT;
  ADC->AVGCTRL.reg = ADC_AVGCTRL;
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL;
  ADC->CTRLA.bit.ENABLE = 1;  // enable ADC
  syncADC();
//  // ADC Linearity/Bias Calibration from NVM (should already be done done in core)
//  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
//  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
//  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
//  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
}

int adcRead(byte ADCpin) {
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCpin].ulADCChannelNumber;
  syncADC();
  ADC->SWTRIG.bit.START = 1;
  while (ADC->INTFLAG.bit.RESRDY == 0);
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  syncADC();
  return ADC->RESULT.reg;
}

void readVOUT() {
  readDiff = adcRead(SENSE_OUTPUT) - adcRead(SENSE_GNDISO) + offsetCorrectionValue;
  if (!BIAS) {
    if (!analog_ref_half && readDiff > RANGE_SWITCH_THRESHOLD_LOW && readDiff < RANGE_SWITCH_THRESHOLD_HIGH/3)
    {
      analogReferenceHalf(true);
      readVOUT();
    }
    else if (analog_ref_half && readDiff >= RANGE_SWITCH_THRESHOLD_HIGH)
    {
      analogReferenceHalf(false);
      readVOUT();
    }
  }
}

inline void analogReadCorrectionForced(int offset, uint16_t gain) {
  offsetCorrectionValue = offset;
  gainCorrectionValue = gain;
  analogReadCorrection(offset,gain);
}

inline void WDTset() {
  // Generic clock generator 2, divisor = 32 (2^(DIV+1))
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  // Enable clock generator 2 using low-power 32KHz oscillator. With /32 divisor above, this yields 1024Hz(ish) clock.
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
  while(GCLK->STATUS.bit.SYNCBUSY);
  // WDT clock = clock gen 2
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2;

  WDT->CTRL.reg = 0; //disable WDT
  while(WDT->STATUS.bit.SYNCBUSY);
  WDT->INTENCLR.bit.EW   = 1;      //disable early warning
  WDT->CONFIG.bit.PER    = 0xA;    //period ~8s
  WDT->CTRL.bit.WEN      = 0;      //disable window mode
  while(WDT->STATUS.bit.SYNCBUSY);
  WDTclear();
  WDT->CTRL.bit.ENABLE = 1;        //enable WDT
  while(WDT->STATUS.bit.SYNCBUSY);
}

void WDTclear() {
  static uint32_t WDTNextUpdate = 0;
  uint32_t const m = millis();
  if (m > WDTNextUpdate)
  {
    WDTNextUpdate = m + 6999; //pet the dog every 7s
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    //while(WDT->STATUS.bit.SYNCBUSY);
  }
}

inline void ldoOptimizeRefresh() {
  ldoOptimized = (ldoValue*1000)/(ADCFULLRANGE * (1 + analog_ref_half));
}

void saveLDO(float newLdoValue) {
  ldoValue = newLdoValue;
  eeprom_LDO.write(newLdoValue);
  ldoOptimizeRefresh();
}

void refreshADCSamplingSpeed() {
  switch(ADC_SAMPLING_SPEED)
  {
  case ADC_SAMPLING_SPEED_AVG:  ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_128 | ADC_AVGCTRL_ADJRES(0x4ul); break;
  case ADC_SAMPLING_SPEED_FAST: ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0x4ul); break;
  case ADC_SAMPLING_SPEED_SLOW: ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_1024 | ADC_AVGCTRL_ADJRES(0x4ul); break;
  }
  setupADC();
  //other combinations:
  //ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0x00ul);  // take 1 sample, adjusting result by 0
  //ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(0x4ul); //take 16 samples adjust by 4
  //ADC_AVGCTRL_SAMPLENUM_128 | ADC_AVGCTRL_ADJRES(0x4ul)
  //ADC_AVGCTRL_SAMPLENUM_256 | ADC_AVGCTRL_ADJRES(0x4ul); //take 256 samples adjust by 4
  //ADC_AVGCTRL_SAMPLENUM_512 | ADC_AVGCTRL_ADJRES(0x4ul); //take 512 samples adjust by 4
  //ADC_AVGCTRL_SAMPLENUM_1024 | ADC_AVGCTRL_ADJRES(0x4ul); //take 1024 samples adjust by 4
}

void printSerialMenu() {
  // Print device name, firmware version and state for interop on PC side
  Serial.print("\r\nCurrentRanger R3 (");  
  Serial.print("firmware v. "); Serial.print(FW_VERSION); Serial.println(")");

  Serial.println("\r\nADC calibration values:");
  Serial.print("Offset="); Serial.println(offsetCorrectionValue);
  Serial.print("Gain="); Serial.println(gainCorrectionValue);
  Serial.print("LDO="); Serial.println(ldoValue,3);

  Serial.println("\r\nEEPROM Settings:");
  Serial.print("LoggingFormat="); Serial.println(LOGGING_FORMAT); 
  Serial.print("ADCSamplingSpeed="); Serial.println(ADC_SAMPLING_SPEED); 
  Serial.print("AutoOff=");
  switch(autooff_interval)
  {
  case AUTOOFF_DISABLED: Serial.println("DISABLED"); break;
  case AUTOOFF_SMART: Serial.println("SMART"); break;
  default: Serial.println(autooff_interval); break;
  }
  Serial.print("BT Logging: "); Serial.println(BT_LOGGING_ENABLED);
  Serial.print("USB Logging: "); Serial.println(USB_LOGGING_ENABLED); 
  Serial.println();

  Serial.println("a = cycle Auto-Off function");
  Serial.print  ("b = toggle BT/serial logging (");Serial.print(SERIAL_UART_BAUD);Serial.println("baud)");
  static const char* const serStrings[] {
    "f = cycle serial logging formats (exponent,nA,uA,mA/raw-ADC)",
    "g = toggle GPIO range indication (SCK=mA,MISO=uA,MOSI=nA)",
    "o = toggle OLED (update takes ~17ms)",
    "r = reboot into bootloader",
    "s = cycle ADC sampling speeds (0=average,faster,slower)",
    "t = toggle touchpad serial output debug info",
    "u = toggle USB/serial logging",
    "< = Calibrate LDO value (-1mV)",
    "> = Calibrate LDO value (+1mV)",
    "+ = Calibrate GAIN value (+1)",
    "- = Calibrate GAIN value (-1)",
    "* = Calibrate OFFSET value (+1)",
    "/ = Calibrate OFFSET value (-1)",
    "1 = range to MilliAmps (MA)",
    "2 = range to MicroAmps (UA)",
    "3 = range to NanoAmps (NA)",
    "4 = toggle Low Pass Filter (LPF)",
    "5 = toggle BIAS (disables AutoRanging)",
    "6 = toggle AutoRanging (disables BIAS)",
    "? = Print this menu and calib info"
  };
  for(auto p : serStrings)
    Serial.println(p);
  Serial.println();
}

void analogReferenceHalf(uint8_t half) {
  if (half && BIAS) analog_ref_half = false;
  else if (analog_ref_half == half) return;
  else analog_ref_half = half;
  analogReference(analog_ref_half ? AR_INTERNAL1V65 : AR_DEFAULT);
  ldoOptimizeRefresh();
}

void analogReadCorrection(int offset, uint16_t gain) {
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(offset);
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(gain);
  ADC->CTRLB.bit.CORREN = 1;
  while(ADC->STATUS.bit.SYNCBUSY);
}

inline void rangeBeep(uint16_t switch_delay) {
  uint16_t freq = RANGE_UA ? NOTE_D5 : RANGE_MA ? NOTE_E5 : NOTE_C5;
  tone(BUZZER, freq, switch_delay ? switch_delay : 20);
}

#define REBOOT_TOKEN 0xf01669ef //special token in RAM, picked up by the bootloader
void rebootIntoBootloader() {
  *((volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4)) = REBOOT_TOKEN; //Entering bootloader from application: https://github.com/microsoft/uf2-samdx1/issues/41
  NVIC_SystemReset();
}
