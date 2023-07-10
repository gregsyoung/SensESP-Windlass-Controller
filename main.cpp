// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include "sensesp/sensors/digital_output.h"
#include "sensesp/controllers/smart_switch_controller.h"
#include "sensesp/signalk/signalk_listener.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/signalk/signalk_put_request_listener.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp_app.h"
#include "sensesp_app_builder.h"

#include <Preferences.h>

#include <math.h>

using namespace sensesp;


#define WINDLASS_GO_DOWN +1
#define WINDLASS_GO_UP -1


reactesp::ReactESP app;

// Define the Pins for chain counter & chain speed
const uint8_t chainCounterPin = 23;
const uint8_t chainSpeedPin = 22;

// Define the PIN for Relay goUP (output)
const uint8_t goUpPin = 19;

// Define the PIN for Relay goDown (output)
const uint8_t goDownPin = 4;

// Define the PIN for detecting windlass direction goingUp
const uint8_t goingUpPin = 16;

// Define the PIN for detecting windlass direction goingDown
const uint8_t goingDownPin = 17;     
 
 //Define the PIN for chain counter reset button
const uint8_t resetPin = 21;

// upDown is chain direction:  1 =  Chain down / count up, -1 = Chain up / count backwards
// initialise to zero 
int upDown = 0;       

//  initialise chainCounter 
  int chainCounter ;

// initialise chainIdleCounter to 60 seconds
int chainIdleCounter = 60;

// Define windlass (anchor winch)  gypsy details
// example used: Muir 4200 Thor horizontal windlass 10mm chain
// Chain wheel =  1 rotation = 0.420m
// Translates one sensor  impulse to meters ( 0.420 m per pulse)
// this windlass outputs a 50mS positive pulse every rotation (approx 1 to 1.3 second) 

float chainCalibrationValue = 0.420f; 
float chainCalibrationOffset = 0.00f;

// initiate an instance of the Preferences library. Here its called preferences

  Preferences preferences;


// The setup function performs one-time application initialization.
void setup() {

// Delays
 int goingUpSensorDebounceDelay = 20;
 int goingDownSensorDebounceDelay = 20;
 int chainCounterSensorDebounceDelay = 20;
 int chainSpeedSensorDebounceDelay = 20;
 int resetbtnDebounceDelay = 20;

  // Configuration paths
  // const char* goingUpDownSensorReadDelayConfigPath = "/sensor_going_up_down/read_delay";
  const char* windlassStatusSKPathConfigPath = "/windlass_status/sk";
  const char* goingUpSensorDebounceDelayConfigPath = "/sensor_going_up/debounce_delay";
  const char* goingDownSensorDebounceDelayConfigPath = "/sensor_going_down/debounce_delay";
  const char* chainCounterSKPathConfigPath = "/rodeDeployed/sk";
  const char* chainCounterSensorDebounceDelayConfigPath = "/chain_counter_sensor/debounce_delay";
  const char* chainSpeedSensorDebounceDelayConfigPath = "/chain_speed_sensor/debounce_delay";
  const char* chainSpeedSKPathConfigPath = "/chainSpeed/sk";
  const char* chainCalibrationSKPathConfigPath = "/chain_counter_sensor/calibration_value";
  const char* resetbtnDebounceDelaySKPathConfigPath = "/chain_counter_resetbtn_debounce/delay";
  const char* chainCounterResetSKPath = "navigation.anchor.chainCounterReset";

  // Signal K paths
  const char* windlassStatusSKPath = "navigation.anchor.windlass.status";
  const char* chainCounterSKPath = "navigation.anchor.rodeDeployed";
  const char* chainSpeedSKPath = "navigation.anchor.windlass.speed";

  // Chain counter metadata
  SKMetadata* chainCounterMetadata = new SKMetadata();
  chainCounterMetadata->units_ = "m";
  chainCounterMetadata->description_ = "Anchor Chain Deployed";
  chainCounterMetadata->display_name_ = "Chain Deployed";
  chainCounterMetadata->short_name_ = "Chain Out";

  // Chain counter speed metadata
  SKMetadata* chainSpeedMetadata = new SKMetadata();
  chainSpeedMetadata->units_ = "m/s";
  chainSpeedMetadata->description_ = "Windlass chain speed";
  chainSpeedMetadata->display_name_ = "Windlass speed";
  chainSpeedMetadata->short_name_ = "Chain speed";
  
 //

 #ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
 #endif


  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("sensESP_windlass_chain_counter")
                   ->set_wifi("ssid", "password")
                    ->set_sk_server("192.168.1.77", 80)
                    ->get_app();

  // -----  retrieve Non Volatile (flash memory) stored value for chainCounter -----

  // Open Preferences with "my_app" namespace. Each application module, library, etc
  // has to use a namespace name to prevent key name collisions. 
  // We will open storage in RW-mode (second parameter has to be false).
  // Note: Namespace name is limited to 15 chars.

  preferences.begin("my_app", false);

  // Retrieve stored_ChainCounter value from nonvolatile storage
  //  if the key does not exist, it will return a default value of 0.
  // Note: Key name is limited to 15 chars.
  // "counter" is the name of the key used to store chainCounter in ESP32 flash (aka Non Vol)

  int  stored_chainCounter = preferences.getInt("counter", 0);

  Serial.printf(" ******  stored_chainCounter value retrieved from flash = : %d\n", stored_chainCounter);

  // retrieve the stored counter and update chainCounter 
  chainCounter=stored_chainCounter;

 
   //  -----  DigitalInputChange monitors a physical button/solenoid connected to BUTTON_PIN ------
   //  The input PIN is set to PULLDOWN (LOW) ; thus the button or solenoid must take the input voltage HIGH when operated.
   //  Because its interrupt type is CHANGE, it will emit a value when the button
   //  is pressed, and again when it's released. 
   // The LambdaConsumer function determines the action based on HIGH (pressed) or LOW (released) state of input
   
  
  auto* goingUpSensor = new DigitalInputChange(goingUpPin, INPUT_PULLDOWN, CHANGE);

  auto* goingDownSensor = new DigitalInputChange(goingDownPin, INPUT_PULLDOWN, CHANGE);

  auto* windlassStatusSKOutput = new SKOutputString(windlassStatusSKPath, windlassStatusSKPathConfigPath);

  // initialise SKPath with "off" status
  
    windlassStatusSKOutput->emit("off");

  /**
   * Create a Debounce for each direction sensor""" to get a clean signal from the
   * button/solenoid. The debounce delay period (setup initially under // Delays above) 
   *  can also be configured by debounce_config_path in the Config UI.
   */
    auto* goingUpSensorDebounce = new DebounceInt(goingUpSensorDebounceDelay, goingUpSensorDebounceDelayConfigPath); 

    auto* goingDownSensorDebounce = new DebounceInt(goingDownSensorDebounceDelay, goingDownSensorDebounceDelayConfigPath); 


  //  ----- Manage the going down sensor  ----

  goingDownSensor
    // Debounce the signal
    ->connect_to(goingDownSensorDebounce)

    // Update the windlass status according to the going down sensor
    ->connect_to(new LambdaConsumer<int>([windlassStatusSKOutput](int input) {

      // Check if the windlass is going down (HIGH = solenoid/button positive volts)
      if (input == HIGH) {

        // The windlass is going down, update the upDown variable
        upDown = WINDLASS_GO_DOWN;
        
         // Update the windlass status
        windlassStatusSKOutput->emit("down");
      } else {

        // The windlass is not going down anymore (LOW=solenoid/button zero volts) 
        //  update the windlass status to off
        windlassStatusSKOutput->emit("off");
      }
    }));


  // ----  Manage the going up sensor -----

  goingUpSensor
    // Debounce the signal
    ->connect_to(goingUpSensorDebounce)

    // Update the windlass status according to the going up sensor
    ->connect_to(new LambdaConsumer<int>([windlassStatusSKOutput](int input) {

      // Check if the windlass is going up
      if (input == HIGH) {

        // The windlass is going up, update the upDown variable
        upDown = WINDLASS_GO_UP;

        // Update the windlass status
        windlassStatusSKOutput->emit("up");
      } else {


        // The windlass is not going up anymore, update the windlass status
        windlassStatusSKOutput->emit("off");
      }
    }));


  // ----   Define a digital input sensor for detecting windlass rotations ---
  //  from a hall effect sensor or reed relay.
  // you need to check if you have a positive or negative pulse on each rotation of windlass
  //  PULLUP input type, assumes its a NEGATIVE pulse.
  //  NOTE: when connecting the physical sensor to BOTH chainCounterPIN  AND chainSensorPIN .. 
  //  then BOTH DigitalInputChange parameters eg(INPUT_PULLUP and CHANGE) MUST be set the same!!
  //  Below is setup for a NEGATIVE pulse
  // the DigitalInputChange will detect on BOTH the leading and trailing edges of the pulse (CHANGE parameter)
  // but the transform will check for a "LOW" pulse

  auto* chainCounterSensor = new DigitalInputChange(chainCounterPin,INPUT_PULLUP,CHANGE);

  /**
   * Create a DebounceInt to ensure a clean signal from the button. 
   * Set the debounce delay period initially (from //Delays above), 
   * which can also be configured via debounce_config_path in the Config UI.
   * Note the debounce period is important to ensure a clean trigger; if its too long, it will not reliably trigger
   */
  auto* chainCounterSensorDebounce = new DebounceInt(chainCounterSensorDebounceDelay, chainCounterSensorDebounceDelayConfigPath); 
  
  //  Manage the chain counter sensor 
  
  chainCounterSensor
    // Debounce the signal
    ->connect_to(chainCounterSensorDebounce)

    // Transform the signal in deployed rode in meters
    ->connect_to(

      // Create a lambda transform function
      new LambdaTransform<int,int>(

        // Catch the counter and the status output instances, input is HIGH when the pin status changes
        [windlassStatusSKOutput](int input) {
        
          // Check if it is the rising front of the pin status change
          if (input == LOW) {

            // Increase or decrease the couter
            chainCounter = chainCounter + upDown;

            // reset chain idle counter to 60 seconds
            // chainIdleCounter =60;
            
          }

          // Returm the chain counter value
          return chainCounter;
        }
        )
      )
    ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
    ->connect_to(new SKOutputFloat(chainCounterSKPath, chainCounterSKPathConfigPath, chainCounterMetadata));
    
// above working OK .. but check that chainIdleCounter =60; is correctly executing



  // -------  Manage chainIdleCounter and store chainCounter to Non Vol memory ----
  //  after an elapsed period of 60 seconds from last chain movement. 
  // Its Important to let chain  activity settle & thus reduce frequency/number of writes to flash memory)
                
 // every second decrement chainIdleCounter;
    
/**

  app.onRepeat(1000,[]() {

  chainIdleCounter = (chainIdleCounter -1);

  Serial.printf(" ******  chainIdleCounter = : %d\n", chainIdleCounter);

            if (chainIdleCounter=0) 

          {
            // Open Preferences with "my_app" namespace
              preferences.begin("my_app", false);

          // store  chainCounter into non vol           
              preferences.putInt("counter", chainCounter);

              Serial.println("stored chainCounter to Flash");

          // Close the Preferences

              preferences.end();;
  
          }
          else {




          }
       }
  );

 */


/**
 // temp disable during testing 
 
  // -----  Define a counter sensor with debouncer  for measuring the chain speed -------
  // in the above  windlass example, the wheel typically rotates between 1 to 2 seconds per revolution ..
  //  so the read timer is set to cover a 2000mS window
  // PULLUP input type, assumes its a NEGATIVE pulse FALLING edge trigger


  auto* chainSpeedSensor = new DigitalInputDebounceCounter(chainSpeedPin, INPUT_PULLUP, FALLING, 2000,
                                     chainSpeedSensorDebounceDelay, chainSpeedSensorDebounceDelayConfigPath);

  // ------- Manage the chain speed sensor -----
  // Counter result is put thru linear transform to get meters for every result period (2000mS)
  // Then thru a MOving average calculation over a 4 sec window, 
  // & with above 2000mS counter period, the moving average output needs to be scaled by 0.5 to get counts per second 
 
  chainSpeedSensor
      ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
      ->connect_to(new MovingAverage(4, 0.5F))                                          
      ->connect_to(new SKOutputFloat(chainSpeedSKPath, chainSpeedSKPathConfigPath, chainSpeedMetadata));  

  */


   // ----- Connect a physical button that will reset chainCounter = 0  ------
  // a momentary push button pulls the PIN low when operated 
  // there is probably a more efficient way to code this?
  
  auto* resetbtn = new DigitalInputChange(resetPin, INPUT_PULLUP, FALLING);
  
  auto* resetbtnDebounce = new DebounceInt(resetbtnDebounceDelay, resetbtnDebounceDelaySKPathConfigPath); 
  
  //   reset the chain counter 
  resetbtn
    // Debounce the signal
    ->connect_to(resetbtnDebounce)

    // Transform 
    ->connect_to(

      // Create a lambda transform function
      new LambdaTransform<int,int>(

        // Catch the counter and the resetbtn state, input is LOW when button pressed 
        [](int input) {
        
          // Check if button pressed (input =LOW )
          if (input == LOW) {

            // reset couter =0
            chainCounter = 0;
            
          }

          // Returm the updated chain counter value
          return chainCounter;
        }
        )
      )
  ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
  ->connect_to(new SKOutputFloat(chainCounterSKPath, chainCounterSKPathConfigPath, chainCounterMetadata));

/** disable for testing 

// ----- Reset chain counter from NodeRed or similar external program/application ------
// monitor an SKPath that is toggled  "true" by external NodeRed flow 
// there is probably a better way to achive this using a Put? style message

auto* windlassResetListener = new BoolSKListener(chainCounterResetSKPath);

     windlassResetListener 
    ->connect_to(

      // Create a lambda transform function
      new LambdaTransform<int,int>(

        // Catch the counter and the windlassResetListener state, input is LOW to reset chainCounter 
        [windlassStatusSKOutput](int input) {
        
          // Check if remote skpath set = true )
          if (input == true) {

            // reset couter =0
            chainCounter = 0;
            
          }

          // Returm the updated chain counter value
          return chainCounter;
        }
        )
      )
  ->connect_to(new Linear(chainCalibrationValue, chainCalibrationOffset, chainCalibrationSKPathConfigPath))
  ->connect_to(new SKOutputFloat(chainCounterSKPath, chainCounterSKPathConfigPath, chainCounterMetadata));

*/

//  

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
