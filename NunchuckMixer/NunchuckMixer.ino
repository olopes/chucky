#define _ENABLE_DEBUG_
#define _DISABLE_RADIO_

#include <Wire.h>
#ifdef _ENABLE_DEBUG_
#include <stdio.h>
#endif

/*
 * pinouts
 * 
 * USB to TTL
 * purple -> TX
 * grey   -> RX
 * white  -> VCC
 * black  -> GND
 * 
 * USBASP
 * brown  -> GND
 * red    -> VCC
 * orange -> 13
 * yellow -> RST
 * green  -> 12
 * blue   -> 11
 * 
 * Nunchuck
 * green  -> GND -> GND
 * blue   -> SDA -> A4
 * red    -> VCC -> 3.3V??
 * white  -> SCL -> A5
 * black  -> DET -> ??
 * 
 * 
 * Usbasp
green orange yellow NC blue
NC NC NC brown red

MISO SCK RES NC MOSI
GND  GND GND GND +5V

MISO=green
 SCK=orange
 RES=yellow
MOSI=blue
 GND=brown
 +5V=red
 */

#define interval 300
#define btn_interval 100

// Mixers
#define MIX_PLANE1 0x00
#define MIX_PLANE2 0x01
#define MIX_PLANE3 0x02
#define MIX_PLANE4 0x03
#define MIX_VTAIL1 0x04
#define MIX_VTAIL2 0x05
#define MIX_VTAIL3 0x06
#define MIX_VTAIL4 0x07

#define MIX_RATE_40 0x00
#define MIX_RATE_60 0x01
#define MIX_RATE_80 0x02
#define MIX_RATE_100 0x03

#define DEFAULT_ZERO_JOY_X 124
#define DEFAULT_ZERO_JOY_Y 132

#define STICK_MIN_VAL -127
#define STICK_MAX_VAL 127

// wiichuck function declaration
static uint8_t wiichuck_data[6];
static void wiichuck_init();
static void wiichuck_send(uint8_t,uint8_t);
static void wiichuck_update();
// SPI lowlevel functions
static void _spi_init();
static void _spi_write(uint8_t command);
static void _spi_write_adress(uint8_t address, uint8_t data);
static uint8_t _spi_read(void);
static uint8_t _spi_read_adress(uint8_t address);
static void _spi_strobe(uint8_t address);

// FlySky (A7105) functions
static void A7105_bind();
static void A7105_write_packet(uint8_t init);
static void A7105_reset(void);
static void A7105_WriteID(uint32_t ida);
//static void A7105_ReadID();

// mixer and button handling functions
static void check_buttons();
static void apply_rates();
static void apply_mixes();

#ifdef _ENABLE_DEBUG_
// debug msg buffer
static char cbuf[120];
// debug counter
#endif

static uint8_t i;
static short joyX, joyY, midX, midY, minX, minY, maxX, maxY;
static uint8_t btns, mix_rate, mix_mode;
static short ail, ele;
static uint16_t smin, smax;
// servo data - all centered except ch3 (THROTTLE)
// removed volatile keyword because 
static uint16_t servo_data[10] = {1500,1500,1000,1500,1500,1500,1500,1500};//8 channels

void setup() {
  // put your setup code here, to run once:
#ifdef _ENABLE_DEBUG_
  Serial.begin(19200);           // set up Serial library at 9600 bps  
#endif
  joyX=joyY=0;
  btns=0;
  midX=midY=127;
  minX=minY=-127;
  maxX=maxY=127;

  // setup mixer
  mix_rate=MIX_RATE_60;
  mix_mode=MIX_PLANE1;
  apply_rates();

  i = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  // init wire to comm with nunchuck
  wiichuck_init();

#ifndef _DISABLE_RADIO_
  _spi_init();

  A7105_bind();
#endif

  // read mix rate and mode and such from EEPROM?

#ifdef _ENABLE_DEBUG_
  Serial.println("Ready!");  // prints hello with ending line break 
#endif
}

unsigned long previousMillis = 0; 
void loop() {
  unsigned long currentMillis;
  
  wiichuck_update();

  check_buttons();
  
  apply_mixes();

  // transmit!!  
#ifndef _DISABLE_RADIO_
  A7105_write_packet(0x55);//(data packet)
#endif

#ifdef _ENABLE_DEBUG_
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    sprintf(cbuf,
    "%3u Joy=% 3d,% 3d (% 3d,% 3d) Btn[%1u] Mixer[%1u%1u]=% 3d,% 3d Servo=%4u,%4u (%4u,%4u)",
       i,joyX,joyY,midX,midY,btns,mix_mode,mix_rate,ail,ele,servo_data[0],servo_data[1],smin,smax);
    Serial.println(cbuf);

    if(i==255) i=0;
    else i++;
    digitalWrite(LED_BUILTIN, i&1?HIGH:LOW);  
  }
#endif
}
static long last_press;
static uint8_t last_btns;
static void check_buttons() {
  unsigned long curr_millis;
  // implement - change mode if buttons are pressed
  curr_millis = millis();
  if(btns && btns > last_btns) {
    // begin button press
    last_press = curr_millis;
    last_btns = btns;
  } else if(!btns && btns <= last_btns) {
    // end button press -> check time to decide click type
    // if(curr_millis - last_press >= btn_interval) 
    if(last_btns == 0x01) {
      // C press
      mix_rate++;
      if(mix_rate > MIX_RATE_100) mix_rate=0;
      apply_rates();
    } else if(last_btns == 0x02) {
      // Z press
      mix_mode++;
      if(mix_mode > MIX_VTAIL4) mix_mode=0;
    } else if(last_btns == 0x03) {
      // perform stick calibration
      midX=(short)wiichuck_data[0];
      midY=(short)wiichuck_data[1];
      minX=0-midX;
      minY=0-midY;
      maxX=255-midX;
      maxY=255-midY;
      
    }

    // clear button status
    last_btns=0;
    last_press=0;
  }
}

static void apply_rates() {
  switch(mix_rate) {
  case MIX_RATE_40:
    smin=1300;
    smax=1700;
    break;
  case MIX_RATE_60:
    smin=1200;
    smax=1800;
    break;
  case MIX_RATE_80:
    smin=1100;
    smax=1900;
    break;
  case MIX_RATE_100:
    smin=1000;
    smax=2000;
    break;
  }
}

static void apply_mixes() {
  short ch1, ch2;
  // apply mixes
  switch(mix_mode) {
    case MIX_PLANE1:
      ail=joyX;
      ele=joyY;
      break;
    case MIX_PLANE2:
      ail=-joyX;
      ele=joyY;
      break;
    case MIX_PLANE3:
      ail=joyX;
      ele=-joyY;
      break;
    case MIX_PLANE4:
      ail=-joyX;
      ele=-joyY;
      break;
    case MIX_VTAIL1:
      ail=joyY-joyX;
      ele=joyY+joyX;
      break;
    case MIX_VTAIL2:
      ail=joyY+joyX;
      ele=joyY-joyX;
      break;
    case MIX_VTAIL3:
      ail=-joyY-joyX;
      ele=-joyY+joyX;
      break;
    case MIX_VTAIL4:
      ail=-joyY+joyX;
      ele=-joyY-joyX;
      break;
  }

  ch1=map(ail, STICK_MIN_VAL, STICK_MAX_VAL, smin, smax);
  ch2=map(ele, STICK_MIN_VAL, STICK_MAX_VAL, smin, smax);
  servo_data[0]=ch1>smax?smax:ch1<smin?smin:ch1;
  servo_data[1]=ch2>smax?smax:ch2<smin?smin:ch2;
}

// Just to prevent rewriting every function call
#define TinyWireM Wire

// adapted from:
// http://playground.arduino.cc/Main/WiiChuckClass
// http://todbot.com/blog/2008/02/18/wiichuck-wii-nunchuck-adapter-available/

// nunchuck code
#define NUNCHUCK_ADDR 0x52
// wiichuck_data
static void wiichuck_init()
{ 
  TinyWireM.begin();                // join i2c bus as master
  // instead of the common 0x40 -> 0x00 initialization, we
  // use 0xF0 -> 0x55 followed by 0xFB -> 0x00.
  // this lets us use 3rd party nunchucks (like cheap $4 ebay ones)
  // while still letting us use official oness.
  // only side effect is that we no longer need to decode bytes in _nunchuk_decode_byte
  // see http://forum.arduino.cc/index.php?topic=45924#msg333160
  //  
  wiichuck_send((uint8_t)0xF0, (uint8_t)0x55);
  delay(1);
  wiichuck_send((uint8_t)0xFB, (uint8_t)0x00);
  wiichuck_update();
}

static void wiichuck_send(uint8_t addr, uint8_t data) {
  TinyWireM.beginTransmission(NUNCHUCK_ADDR);// transmit to device 0x52
  TinyWireM.write(addr);// sends memory address
  TinyWireM.write(data);// sends sent data.  
  TinyWireM.endTransmission();// stop transmitting
}

static void wiichuck_update() {
  int cnt=0;
  TinyWireM.requestFrom(NUNCHUCK_ADDR, 6);
  while (TinyWireM.available ()) {
    wiichuck_data[cnt] = TinyWireM.read();
    cnt++;
  }

  if(cnt > 5) {
    joyX = (short)wiichuck_data[0]-midX;
    joyY = (short)wiichuck_data[1]-midY;
    btns = (wiichuck_data[5] & 3)^3;

    // moar data!!
    TinyWireM.beginTransmission(NUNCHUCK_ADDR);// transmit to device 0x52
    TinyWireM.write((uint8_t)0x00);// sends one byte
    TinyWireM.endTransmission();
  }
}


// SPI code adapted from midelic's
// Spi stuff - check ATTiny85 specs for correct instructions
// http://www.rcgroups.com/forums/showthread.php?t=1921870

/* referece ports for attiny:
 *   DDRB - Port B Data Direction Register
 *   PORTB - Port B Data Register
 *   PINB - Port B Input Pins Address
 *   MCUCR - MCU Control Register
 *   GIMSK - General Interrupt Mask Register
 *   PCMSK - Pin Change Mask Register
 *   GIFR - General Interrupt Flag Register
 */

/* reference ports for atmega328p
 *  https://www.arduino.cc/en/Reference/PortManipulation
    PORTD maps to Arduino digital pins 0 to 7
        DDRD  - The Port D Data Direction Register - read/write
        PORTD - The Port D Data Register           - read/write
        PIND  - The Port D Input Pins Register     - read only 
    
    PORTB maps to Arduino digital pins 8 to 13 The two high bits (6 & 7) map to the crystal pins and are not usable
        DDRB  - The Port B Data Direction Register - read/write
        PORTB - The Port B Data Register           - read/write
        PINB  - The Port B Input Pins Register     - read only 
    
    PORTC maps to Arduino analog pins 0 to 5. Pins 6 & 7 are only accessible on the Arduino Mini
        DDRC  - The Port C Data Direction Register - read/write
        PORTC - The Port C Data Register           - read/write
        PINC  - The Port C Input Pins Register     - read only 
 */

// TODO if atmega328 else if attiny85 else end
#define SPI_PORT PORTC
/*
#define PIND 1
#define SDI_1 0
*/

//Spi Comm.pins with A7105/PPM
#define PPM_pin 2//PPM in 
#define SDI_pin 5 //SDIO-D5 
#define SCLK_pin 4 //SCK-D4
#define CS_pin 6//CS-D6
//---------------------------------
#define  CS_on SPI_PORT |= 0x40 //D6
#define  CS_off SPI_PORT &= 0xBF //D6
//
#define  SCK_on SPI_PORT |= 0x10//D4
#define  SCK_off SPI_PORT &= 0xEF//D4
#define  SDI_on SPI_PORT |= 0x20 //D5
#define  SDI_off SPI_PORT &= 0xDF //D5
//
#define  SDI_1 (PIND & 0x20) == 0x20 //D5
#define  SDI_0 (PIND & 0x20) == 0x00 //D5
//
#define RED_LED_pin A3
#define RED_LED_ON  PORTC |= _BV(3)
#define RED_LED_OFF PORTC &= ~_BV(3)
#define NOP() __asm__ __volatile__("nop")


static const uint8_t A7105_regs[] = {
    0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff ,0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
    0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f, 0xff,
};
static const uint8_t tx_channels[16][16] = {
  {0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
  {0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
  {0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
  {0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
  {0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
  {0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
  {0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
  {0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
  {0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
  {0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
  {0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
  {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
  {0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
  {0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
  {0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
  {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46},
};
static uint32_t id;//tx id, don't confuse it with A7105 id
static uint8_t chanrow;
static uint8_t chancol;
static uint8_t chanoffset;
static byte counter=255;
static uint8_t packet[21];//inside code there are 16....so don't bother

static void _spi_init() {
    /* from midelic's code */

  pinMode(RED_LED_pin, OUTPUT); 
  //RF module pins
  pinMode(PPM_pin, INPUT);//PPM input
  pinMode(SDI_pin, OUTPUT);//SDI   SDIO 
  pinMode(SCLK_pin, OUTPUT);//SCLK SCL 
  pinMode(CS_pin, OUTPUT);//CS output
  CS_on;//start CS high
  SDI_on;//start SDIO high
  SCK_off;//start sck low
  
  uint8_t i;
  uint8_t if_calibration1;
  uint8_t vco_calibration0;
  uint8_t vco_calibration1;
  
  //
  //for debug 
  delay(10);//wait 10ms for A7105 wakeup
  //A7105_reset();//reset A7105
  _spi_write_adress(0x00,0x00);//reset A7105
  A7105_WriteID(0x5475c52A);//0x2Ac57554
  //A7105_ReadID();//for debug only
  //Serial.print(aid[0],HEX);
  //Serial.print(aid[1],HEX);
  //Serial.print(aid[2],HEX);
  //Serial.print(aid[3],HEX);
  
  for (i = 0; i < 0x33; i++){
    if(A7105_regs[i] != 0xff)
      _spi_write_adress(i, A7105_regs[i]);
  }
  
  _spi_strobe(0xA0);//stand-by
  _spi_write_adress(0x02,0x01);
  while(_spi_read_adress(0x02)){
    if_calibration1=_spi_read_adress(0x22);
    if(if_calibration1&0x10){//do nothing
    }
  }
  
  _spi_write_adress(0x24,0x13);
  _spi_write_adress(0x26,0x3b);
  _spi_write_adress(0x0F,0x00);//channel 0
  _spi_write_adress(0x02,0x02);
  while(_spi_read_adress(0x02)){
    vco_calibration0=_spi_read_adress(0x25);
    if(vco_calibration0&0x08){//do nothing
    }
  }
  
  _spi_write_adress(0x0F,0xA0);
  _spi_write_adress(0x02,0x02);
  while(_spi_read_adress(0x02)){
    vco_calibration1=_spi_read_adress(0x25);
    if(vco_calibration1&0x08){//do nothing
    }
  }
 
  _spi_write_adress(0x25,0x08);
  _spi_write_adress(0x28,0x1F);//set power to 1db maximum
  _spi_strobe(0xA0);//stand-by strobe command

}

// BIND_TX
static void A7105_bind() {
  // TODO check if this ID can be different
  id=0x30000006;//fixed TX ID(Thierry ID)

  while(counter){ //while counter//counter for 2.5 sec.
    _spi_strobe(0xA0);
    _spi_strobe(0xE0);
    _spi_write_adress(0x0F,0x01);//Tx channel 1
    A7105_write_packet(0xaa);//(bind packet)
    _spi_strobe(0xD0);//strobe Fifo Tx
    delay(10);//wait 10ms
    if (bitRead(counter,3)==1){
      RED_LED_ON;
    } else {
      RED_LED_OFF;
    }
    counter--;
  }

  chanrow=id % 16;
  chanoffset=(id & 0xff) / 16;
  chancol=0;
  RED_LED_ON;
  // ready :-)
}

//-------------------------------
//-------------------------------
//A7105 SPI routines
//-------------------------------
//-------------------------------
static void A7105_WriteID(uint32_t ida) {
  CS_off;
  _spi_write(0x06);
  _spi_write((ida>>24)&0xff); 
  _spi_write((ida>>16)&0xff);
  _spi_write((ida>>8)&0xff);
  _spi_write((ida>>0)&0xff);
  CS_on;
}

/*
void A7105_ReadID(){
  uint8_t i;
  CS_off;
  _spi_write(0x46);
  for(i=0;i<4;i++){
    aid[i]=_spi_read();
  }
  CS_on;
}
*/

//----------------------
static void A7105_write_packet(uint8_t init){ //except adress(0x05)should sent to A7105 21 bytes totally)
  uint8_t i;
  CS_off;
  _spi_write(0x05);//TX/RX FIFO adress
  _spi_write(init);//0xaa or 0x55(depend on bind packet or data packet)
  _spi_write((id >>  0) & 0xff);
  _spi_write((id >>  8) & 0xff);
  _spi_write((id >>  16) & 0xff);
  _spi_write((id >>  24) & 0xff);

  for(i=0;i<8;i++) {
    cli();
    packet[0+2*i]=lowByte(servo_data[i]);//low byte of servo timing(1000-2000us)
    packet[1+2*i]=highByte(servo_data[i]);//high byte of servo timing(1000-2000us)
    sei();
    _spi_write(packet[0+2*i]);
    _spi_write(packet[1+2*i]);
  }

  CS_on;
}

//---------------------------------
//-------------------------------------- 
static void _spi_write(uint8_t command) {  
  uint8_t n=8; 
  SCK_off;
  SDI_off;
  while(n--) {
    if(command&0x80)
      SDI_on;
    else 
      SDI_off;
    SCK_on;
    NOP();
    SCK_off;
    command = command << 1;
  }
  SDI_on;
}  

static void _spi_write_adress(uint8_t address, uint8_t data) {
  CS_off;
  _spi_write(address); 
  NOP();
  _spi_write(data);  
  CS_on;
} 

//-----------------------------------------
static uint8_t _spi_read(void) {
  uint8_t result;
  uint8_t i;
  result=0;
  pinMode(SDI_pin,INPUT);//make SDIO pin input
  //SDI_on;
  for(i=0;i<8;i++) {                    
    if(SDI_1)  ///if SDIO =1 
      result=(result<<1)|0x01;
    else
      result=result<<1;
    SCK_on;
    NOP();
    SCK_off;
    NOP();
  }
  pinMode(SDI_pin,OUTPUT);//make SDIO pin output again
  return result;
}   

//--------------------------------------------
static uint8_t _spi_read_adress(uint8_t address) { 
  uint8_t result;
  CS_off;
  address |=0x40; 
  _spi_write(address);
  result = _spi_read();  
  CS_on;
  return(result); 
} 

//------------------------
static void _spi_strobe(uint8_t address) {
  CS_off;
  _spi_write(address);
  CS_on;
}

//------------------------
static void A7105_reset(void) {
  _spi_write_adress(0x00,0x00); 
}



