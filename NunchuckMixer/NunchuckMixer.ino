#define _ENABLE_DEBUG_

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

#define STICK_MIN_VAL -128
#define STICK_MAX_VAL 127

// function declaration
static uint8_t wiichuck_data[6];
static void wiichuck_init();
static void wiichuck_send(uint8_t,uint8_t);
static void wiichuck_update();
static void check_buttons();
static void apply_rates();
static void apply_mixes();

#ifdef _ENABLE_DEBUG_
// debug msg buffer
static char cbuf[120];
// debug counter
#endif

static uint8_t i;
static short joyX, joyY, midX, midY;
static uint8_t btns, mix_rate, mix_mode;
static short ail, ele;
static uint16_t ch1, ch2, smin, smax;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);           // set up Serial library at 9600 bps  
  joyX=joyY=0;
  btns=0;
  ch1=ch2=1500;
  midX=midY=127;

  // setup mixer
  mix_rate=MIX_RATE_60;
  mix_mode=MIX_PLANE1;
  apply_rates();

  i = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  // init wire to comm with nunchuck
  wiichuck_init();
  Serial.println("Ready!");  // prints hello with ending line break 
}

unsigned long previousMillis = 0; 
void loop() {
  unsigned long currentMillis;
  
  wiichuck_update();

  check_buttons();
  
  apply_mixes();

  currentMillis = millis();

  // transmit!!  
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

#ifdef _ENABLE_DEBUG_
    sprintf(cbuf,
    "%3u Joy=% 3d,% 3d (% 3d,% 3d) Btn[%1u] Mixer[%1u%1u]=% 3d,% 3d Servo=%4u,%4u (%4u,%4u) Raw=[%3u,%3u,%3u,%3u,%3u,%3u]",
       i,joyX,joyY,midX,midY,btns,mix_mode,mix_rate,ail,ele,ch1,ch2,smin,smax,
       wiichuck_data[0],wiichuck_data[1],wiichuck_data[2],wiichuck_data[3],wiichuck_data[4],wiichuck_data[5]);
    Serial.println(cbuf);
#endif

    if(i==255) i=0;
    else i++;
    digitalWrite(LED_BUILTIN, i&1?HIGH:LOW);  
  }
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



