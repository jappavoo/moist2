/*********************
Borrowed code from:
1. Adafruit RGB Character LCD Shield example 
   (with noted fix to process button clicks)
2. Adafruit am2315 example 
   (with care to delay queries by at least 2 seconds based on 
    info on the web regarding getting stable data)
3. remote power code from my cabin/FRANK code

Future: upgrade to replace relays with optoisolators from FRANK
**********************/

// include the library code:
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Adafruit_AM2315.h>
// code for dealing with button directly wired to arduino (debounce logic)
#include <Bounce2.h>


//#define DEBUG 1

// observed that once is several days it looked like
// we missed turning off the remote until we add feedback
// we repeat the toggle to improve reliability
#define REPEAT_REMOTE_TOGGLE 1
    
// global constants 
const char  VERSION[] = "MOIST v0.4";
const int CONFIG_NUM = 3;
const float CONFIG_0_DEFAULT_MIN_HUMIDITY=50.0;
const float CONFIG_0_DEFAULT_MAX_HUMIDITY=60.0;
const float CONFIG_1_DEFAULT_MIN_HUMIDITY=30.0;
const float CONFIG_1_DEFAULT_MAX_HUMIDITY=40.0;
const float CONFIG_2_DEFAULT_MIN_HUMIDITY=0.0;
const float CONFIG_2_DEFAULT_MAX_HUMIDITY=0.0;
const int8_t CONFIG_SYM_COL=15;
const int8_t CONFIG_SYM_ROW=1;

const int8_t LCD_NUM_COLS=16;
const int8_t LCD_NUM_ROWS=2;
const int8_t LCD_OFF=0x0;
const int8_t LCD_ON=0x7;

const int AM2315_DELAY=2000;        // min 2 second delay between queries
                                    // to get stable data
const int AM2315_ERROR_THRESHOLD=1;
const int UPDATE_SENSOR_DATA_PERIOD=(AM2315_DELAY * 15);

const int8_t EXTRA_BUTTON_PIN=5;    // WIRING CONSTRAINT
const int8_t REMOTE_ON_PIN=6;       // WIRING CONSTRAINT
const int8_t REMOTE_OFF_PIN=7;      // WIRING CONSTRAINT
const int    REMOTE_TOGGLE_DELAY=2000;  
const int8_t REMOTE_STATUS_COL=15;
const int8_t REMOTE_STATUS_ROW=0;

const uint8_t BUTTON_EXTRA_0 = 0x80;  // 5 lcd buttons are 0x01, 0x02, 0x4, 0x8 and 0x10  
                                     // Button should be wired between the pin and
                                     // ground
class ArduinoButton : public Bounce {
  int pin_;
  
  public:
  ArduinoButton(int pin): pin_(pin) {}
  void setup() {
     // Setup the button with an internal pull-up :
     // this means that when the button is in the open state the pin
     // internally will be connected to +5 and read 1
     // When the button in pressed the button will connect the pin to ground
     // and we will read a 1
     pinMode(pin_,INPUT_PULLUP);
     // After setting up the button, setup the Bounce instance :
     attach(pin_);
     interval(5); // interval in ms
  }
};

struct GLOBALSTATE {
  // The LCD shield uses the I2C SCL and SDA pins. On classic Arduinos
  // this is Analog 4 and 5 so you can't use those for analogRead() anymore
  // However, you can connect other I2C sensors to the I2C bus and share
  // the I2C bus.
  Adafruit_RGBLCDShield lcd;
  
  // BE CAREFULL must not query AM2315 too quickly as it will act flaky and return
  // NaN's we throttle it to only be queried every 2 seconds (AM2315_DELAY)
  Adafruit_AM2315 am2315;

  ArduinoButton extraButton0;
  
  //  following fields are ordered from biggest to smallest in size to improve
  //  global size by easing alignment constrants (reduce need for padding)
  float temp;               // last read temperature
  float hum;                // last read humidity
  
  struct Configs {
    float humMin;               // min humidity threshold
    float humMax;               // max humdity threshold
    char  symbol;
  } configs[CONFIG_NUM];
  
  int theConfig;
  
  int now;                  // current time
  int lastHum;              // last time humidity was read
  
  int remote_pwr_state;     // state of the remote power switch
  
  int am2315Errors;         // count error 
  bool backLight;           // state of lcd backlight
  uint8_t clicked_buttons;  // state of button clicks (transitions)
  
  GLOBALSTATE() : lcd(), am2315(), extraButton0(EXTRA_BUTTON_PIN), theConfig(0), 
                  am2315Errors(0), clicked_buttons(0x0) {
                    configs[0].humMin = CONFIG_0_DEFAULT_MIN_HUMIDITY;
                    configs[0].humMax = CONFIG_0_DEFAULT_MAX_HUMIDITY;
                    configs[0].symbol = 'A';
                    configs[1].humMin = CONFIG_1_DEFAULT_MIN_HUMIDITY;
                    configs[1].humMax = CONFIG_1_DEFAULT_MAX_HUMIDITY;
                    configs[1].symbol = 'B';
                    configs[2].humMin = CONFIG_2_DEFAULT_MIN_HUMIDITY;
                    configs[2].humMax = CONFIG_2_DEFAULT_MAX_HUMIDITY;
                    configs[2].symbol = 'C';
                  }                   
} State;

void
backLightOn() 
{
  State.lcd.setBacklight(LCD_ON);
  State.backLight=true;
}

void
backLightOff() 
{
  State.lcd.setBacklight(LCD_OFF); 
  State.backLight=false;
}

// readButtonClicks from https://github.com/adafruit/Hunt-The-Wumpus/blob/master/Hunt_The_Wumpus.ino
//! Return a bitmask of clicked buttons.
/*!
  Examine the bitmask of buttons which are currently pressed and compare against
  the bitmask of which buttons were pressed last time the function was called.
  If a button transitions from pressed to released, return it in the bitmask.
  \return the bitmask of clicked buttons
*/
inline void 
readButtonClicks() {
  static uint8_t last_buttons = 0;
  
  uint8_t buttons = State.lcd.readButtons();
  int extraval = State.extraButton0.read();

  if (extraval == LOW) { buttons |= BUTTON_EXTRA_0; }   
  State.clicked_buttons = (last_buttons ^ buttons) & (~buttons);  
  last_buttons = buttons;
}

inline void 
displayPwrState()
{
  State.lcd.setCursor(REMOTE_STATUS_COL,REMOTE_STATUS_ROW);
  if (State.remote_pwr_state == 0) State.lcd.print(" ");
  else State.lcd.print("*");
}

inline void 
remote_toggle_button(int pin)
{
    digitalWrite(pin, HIGH);
    delay(REMOTE_TOGGLE_DELAY);
    digitalWrite(pin, LOW);
    delay(REMOTE_TOGGLE_DELAY);
#ifdef REPEAT_REMOTE_TOGGLE
    // to improve reliability
    delay(500);
    digitalWrite(pin, HIGH);
    delay(REMOTE_TOGGLE_DELAY);
    digitalWrite(pin, LOW);
    delay(REMOTE_TOGGLE_DELAY); 
#endif
}

inline void 
remote_pwr_off()
{
  if (State.remote_pwr_state==1) {
    State.remote_pwr_state = 0;
    displayPwrState();
    remote_toggle_button(REMOTE_OFF_PIN);
  }
}

inline void 
remote_pwr_on()
{
  if (State.remote_pwr_state==0) {
    State.remote_pwr_state = 1;
    displayPwrState();
    remote_toggle_button(REMOTE_ON_PIN);
  }
}

inline void 
remote_pwr_setup()
{
  pinMode(REMOTE_ON_PIN, OUTPUT);
  pinMode(REMOTE_OFF_PIN, OUTPUT);
  remote_toggle_button(REMOTE_OFF_PIN);
  State.remote_pwr_state = 0;
  displayPwrState();
}

inline void 
displayConfigSym()
{
  State.lcd.setCursor(CONFIG_SYM_COL,CONFIG_SYM_ROW);
  State.lcd.print(State.configs[State.theConfig].symbol);
}

inline void 
displayMinMax()
{
  State.lcd.setCursor(0,1);  // COL=0, ROW=1
  if (State.configs[State.theConfig].humMin == 0.0 && 
      State.configs[State.theConfig].humMax == 0.0) {
        State.lcd.print("     OFF        ");
  } else {
    State.lcd.print("min:");
    State.lcd.print(State.configs[State.theConfig].humMin,0);
    State.lcd.print(" max:");
    State.lcd.print(State.configs[State.theConfig].humMax,0);
    displayConfigSym();
  }
}

void 
error(char *msg)
{
#ifdef DEBUG
     Serial.println("Sensor not found, check wiring & pullups!");
#endif
     backLightOn();
     State.lcd.blink();
     State.lcd.setCursor(0,1); // COL=0, ROW=1
     State.lcd.print(msg);
     
     remote_pwr_off();  // polietly try and turn off remote power
     remote_toggle_button(REMOTE_OFF_PIN); // to be safe do it forcefully;
 
     while (1);
}

bool 
updateHumTemp()
{
  if (State.am2315.readTemperatureAndHumidity(State.temp, State.hum)) {    
      State.lcd.print(State.hum); State.lcd.print("% "); 
      State.lcd.print(State.temp); State.lcd.print("C");
      return true;
  } else {
    State.am2315Errors++;
    if (State.am2315Errors == AM2315_ERROR_THRESHOLD) error((char *)" ERROR: AM2315-1");
  }
  return false;
}

void 
setup() 
{
  // Debugging output
#ifdef DEBUG
  Serial.begin(9600);
#endif

  // set up the LCD's number of columns and rows: 
  State.lcd.begin(LCD_NUM_COLS, LCD_NUM_ROWS);

  State.lcd.setCursor(0,0); // COL=0, ROW=0
  State.lcd.print(VERSION);
  backLightOn();
 
  State.extraButton0.setup();   
  
  State.lcd.setCursor(0,1); // COL=0, ROW=1
  State.lcd.print("SETUP REMOTE PWR");
  remote_pwr_setup();
  
  State.lcd.setCursor(0,1); // COL=0, ROW=1
  State.lcd.print("SETUP HUM & TEMP");
  
  if (! State.am2315.begin()) { error((char *)" ERROR: AM2315-0"); }
 
  State.lastHum = millis(); // setup times so first read will be 
  delay(AM2315_DELAY);      // at least after AM2315 delay   
  
  State.lcd.clear();        // clear lcd
  displayMinMax();          // display min max
  updateHumTemp();          // get and display current humdity and temp
}

inline void 
processButtons()
{
  State.extraButton0.update();
  
  readButtonClicks();
  
  if (State.clicked_buttons) {
    switch(State.clicked_buttons) {
      case BUTTON_UP:
           State.configs[State.theConfig].humMax++;
           displayMinMax();
           break;
      case BUTTON_DOWN:
           State.configs[State.theConfig].humMax--;
           displayMinMax();
           break;
      case BUTTON_RIGHT:
           State.configs[State.theConfig].humMin--;
           displayMinMax();
           break;
      case BUTTON_LEFT:
           State.configs[State.theConfig].humMin++;
           displayMinMax();
           break;
      case BUTTON_SELECT:
        if (State.backLight==true) {
           backLightOff();
         } else { 
           backLightOn();
         }
        break;
      case BUTTON_EXTRA_0:
        State.theConfig++;
        State.theConfig %= CONFIG_NUM;
        displayMinMax(); 
        break;
      case (BUTTON_SELECT | BUTTON_RIGHT):
        remote_pwr_on();
        break;
      case (BUTTON_SELECT | BUTTON_LEFT): 
        remote_pwr_off();
        break;      
    }
  }
}

inline void 
processSensors()
{
  State.now = millis();
  if ((State.now-State.lastHum) >= UPDATE_SENSOR_DATA_PERIOD) {
    State.lastHum = State.now;
    State.lcd.setCursor(0, 0); // COL=0, ROW=0
   if (updateHumTemp()) { 
      if (State.hum <= State.configs[State.theConfig].humMin) remote_pwr_on();
      else if (State.hum > State.configs[State.theConfig].humMax) remote_pwr_off();
    }   
  }
}

void 
loop() {
  processButtons();
  processSensors();  
}
