// Includes for the display.
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Define the pins for SCL and SDA
#define SCL_PIN 19
#define SDA_PIN 18
#define buzzerPin 7
//----------------------------
#include <ADC.h>
#include <ADC_util.h>
#include <vector>
#include <iostream>

//---------------------------- ADC---------------------------------
const char adcResolution = 12;
const int adcPin = 14;
const int adcPin2 = 22;
const int numSamples = 2000;  // Number of samples to take
int adcValues[numSamples];
int adcValues2[numSamples];

//--------------------------Menu defination------------------------------------
#include "U8g2lib.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0); // [full framebuffer, size = 1024 bytes]


// all the arrays below are generated from images using Image Magick
// scroll down to see the actual code

const unsigned char upir_logo [] PROGMEM = {  
  0xEA, 0x3A, 0xAA, 0x28, 0x6A, 0x1A, 0x26, 0x2A, };


// 'icon_3dcube', 16x16px
const unsigned char bitmap_icon_3dcube [] PROGMEM = {
  0x00, 0x00, 0x80, 0x01, 0xE0, 0x06, 0x98, 0x18, 0x86, 0x60, 0x8A, 0x50, 
  0xA2, 0x45, 0x82, 0x40, 0xA2, 0x44, 0x82, 0x40, 0xA2, 0x45, 0x8A, 0x50, 
  0x86, 0x60, 0x98, 0x18, 0xE0, 0x06, 0x80, 0x01, };  
// 'icon_battery', 16x16px
const unsigned char bitmap_icon_battery [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x1F, 0x02, 0x20, 
  0xDA, 0x66, 0xDA, 0x66, 0xDA, 0x66, 0x02, 0x20, 0xFC, 0x1F, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
// 'icon_dashboard', 16x16px
const unsigned char bitmap_icon_dashboard [] PROGMEM = {
  0xE0, 0x07, 0x18, 0x18, 0x84, 0x24, 0x0A, 0x40, 0x12, 0x50, 0x21, 0x80, 
  0xC1, 0x81, 0x45, 0xA2, 0x41, 0x82, 0x81, 0x81, 0x05, 0xA0, 0x02, 0x40, 
  0xD2, 0x4B, 0xC4, 0x23, 0x18, 0x18, 0xE0, 0x07, };
// 'icon_fireworks', 16x16px
const unsigned char bitmap_icon_fireworks [] PROGMEM = {
  0x00, 0x00, 0x00, 0x10, 0x00, 0x29, 0x08, 0x10, 0x08, 0x00, 0x36, 0x00, 
  0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x63, 0x00, 0x00, 0x00, 0x08, 
  0x20, 0x08, 0x50, 0x00, 0x20, 0x00, 0x00, 0x00, };
// 'icon_gps_speed', 16x16px
const unsigned char bitmap_icon_gps_speed [] PROGMEM = {
  0x00, 0x00, 0xC0, 0x0F, 0x00, 0x10, 0x80, 0x27, 0x00, 0x48, 0x00, 0x53, 
  0x60, 0x54, 0xE0, 0x54, 0xE0, 0x51, 0xE0, 0x43, 0xE0, 0x03, 0x50, 0x00, 
  0xF8, 0x00, 0x04, 0x01, 0xFE, 0x03, 0x00, 0x00, };
// 'icon_knob_over_oled', 16x16px
const unsigned char bitmap_icon_knob_over_oled [] PROGMEM = {
  0x00, 0x00, 0xF8, 0x0F, 0xC8, 0x0A, 0xD8, 0x0D, 0x88, 0x0A, 0xF8, 0x0F, 
  0xC0, 0x01, 0x80, 0x00, 0x00, 0x00, 0x90, 0x04, 0x92, 0x24, 0x04, 0x10, 
  0x00, 0x80, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, };
// 'icon_parksensor', 16x16px
const unsigned char bitmap_icon_parksensor [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x44, 0x00, 0xA4, 0x00, 
  0x9F, 0x00, 0x00, 0x81, 0x30, 0xA1, 0x48, 0xA9, 0x4B, 0xA9, 0x30, 0xA0, 
  0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };
// 'icon_turbo', 16x16px
const unsigned char bitmap_icon_turbo [] PROGMEM = {
  0x00, 0x70, 0xE0, 0x8F, 0x18, 0x80, 0x04, 0x80, 0x02, 0x80, 0xC2, 0x8F, 
  0x21, 0x72, 0x51, 0x05, 0x91, 0x44, 0x51, 0x45, 0x21, 0x42, 0xC2, 0x21, 
  0x02, 0x20, 0x04, 0x10, 0x18, 0x0C, 0xE0, 0x03, };

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 384)
const unsigned char* bitmap_icons[8] = {
  bitmap_icon_3dcube,
  bitmap_icon_battery,
  bitmap_icon_dashboard,
  bitmap_icon_fireworks,
  bitmap_icon_gps_speed,
  bitmap_icon_knob_over_oled,
  bitmap_icon_parksensor,
  bitmap_icon_turbo
};



// 'screenshot_3dcube', 128x64px

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 8320)


// 'qr_3dcube', 128x64px

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 8320)


// 'scrollbar_background', 8x64px
const unsigned char bitmap_scrollbar_background [] PROGMEM = {
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 
  0x00, 0x40, 0x00, 0x00, };


// 'item_sel_outline', 128x21px
const unsigned char bitmap_item_sel_outline [] PROGMEM = {
  0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x02, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x0C, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0xF8, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 
  };
// ------------------ end generated bitmaps from image2cpp ---------------------------------
const int NUM_ITEMS = 5; // number of items in the list and also the number of screenshots and screenshots with QR codes (other screens)
const int MAX_ITEM_LENGTH = 20; // maximum characters for the item name
char menu_items [NUM_ITEMS] [MAX_ITEM_LENGTH] = {  // array with item names
  { "Calibrate" }, 
  { "Measure & Detect" }, 
  { "Volume" }, 
  { "Discriminate" },
  {"Send 2 Julia"} 
 };
// note - when changing the order of items above, make sure the other arrays referencing bitmaps
#define BUTTON_UP_PIN 11 // pin for UP button 
#define BUTTON_SELECT_PIN 10 // pin for SELECT button
#define BUTTON_DOWN_PIN 12 // pin for DOWN button



int button_up_clicked = 0; // only perform action when button is clicked, and wait until another press
int button_select_clicked = 0; // same as above
int button_down_clicked = 0; // same as above

int item_selected = 0; // which item in the menu is selected

int item_sel_previous; // previous item - used in the menu screen to draw the item before the selected one
int item_sel_next; // next item - used in the menu screen to draw next item after the selected one

int current_screen = 0;   // 0 = menu, 1 = screenshot, 2 = qr
//--------------------------------------------------------------------
int mean1;
int mean2;
int buzzerFrequency = 0;
int buzzerDutyCycle = 50; //full volume
float transmitter_coil[numSamples];
float receiver_coil[numSamples];
double sample_rate;

ADC* adc = new ADC();

std::vector<double> refPhasors = { 0, 0, 0, 0 };
//-----------------------------OLED--------------------------------
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//---------------------------- Square wave ------------------------
const int squareWavePin = 15;   // Define the pin number
const int frequencyHz = 15200;  // Frequency of the square wave in Hz
const float dutyCycle = 0.5;    // Duty cycle (0.0 to 1.0, 0.5 for a 50% duty cycle)
//-----------------------------------------------------------------
//--------------declaring functions--------------------------------
void sendData(int inputNumber);
void sendArray(int array[], int arraysize);
std::vector<double> phaseShift(float* transmitter_coil, float* receiver_coil, int N, int sample_rate, float waveFrequency);
void testdrawstyles(void);
std::vector<double> getPhasors();
std::vector<double> calibrate(int averageNum);
bool isMetal(double amplitude);
std::vector<double> vectorSubtract(std::vector<double> V2, std::vector<double> V1);
void sampleADC(void);


void setup() {
  Serial.begin(9600);
  Wire.begin();
  // Setting up the adc for source sampling
  adc->adc0->setResolution(adcResolution);                               // Set ADC resolution to 12 bits (0-4095)
  adc->adc0->setAveraging(4);                                            // Set averaging (optional)
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // Set conversion speed (optional)
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);      // Set sampling speed (optional)

  // Setting up the adc for reciever sampling
  adc->adc1->setResolution(adcResolution);                               // Set ADC resolution to 12 bits (0-4095)
  adc->adc1->setAveraging(4);                                            // Set averaging (optional)
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);  // Set conversion speed (optional)
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);      // Set sampling speed (optional)

  // Code for generating square wave for transmitter coil
  pinMode(squareWavePin, OUTPUT);                    // Set the pin as an output
  analogWriteFrequency(squareWavePin, frequencyHz);  // Set the PWM frequency
  analogWrite(squareWavePin, 255 * dutyCycle);       // Set the PWM duty cycle
  delay(3000); // to allow sufficient time for wave to reach steady state

  // Code for generating square wave for buzzer
  pinMode(buzzerPin, OUTPUT);                    // Set the pin as an output

  //take the reference phasor
      std::vector<double> result = calibrate(50);

      refPhasors[0] = result[0];
      refPhasors[1] = result[1];

      refPhasors[2] = result[2];
      refPhasors[3] = result[3];
  //------------------menu setup--------------------------
  u8g2.setColorIndex(1);  // set the color to white
  u8g2.begin();
  u8g2.setBitmapMode(1);

  // define pins for buttons
  // INPUT_PULLUP means the button is HIGH when not pressed, and LOW when pressed
  // since it´s connected between some pin and GND
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP); // up button
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP); // select button
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP); // down button
  
  //Making sure that the screen address is correct

}
void loop() {
   if (current_screen == 0) { // MENU SCREEN

      // up and down buttons only work for the menu screen
      if ((digitalRead(BUTTON_UP_PIN) == LOW) && (button_up_clicked == 0)) { // up button clicked - jump to previous menu item
        item_selected = item_selected - 1; // select previous item
        button_up_clicked = 1; // set button to clicked to only perform the action once
        if (item_selected < 0) { // if first item was selected, jump to last item
          item_selected = NUM_ITEMS-1;
        }
      }
      else if ((digitalRead(BUTTON_DOWN_PIN) == LOW) && (button_down_clicked == 0)) { // down button clicked - jump to next menu item
        item_selected = item_selected + 1; // select next item
        button_down_clicked = 1; // set button to clicked to only perform the action once
        if (item_selected >= NUM_ITEMS) { // last item was selected, jump to first menu item
          item_selected = 0;
          }
      } 

      if ((digitalRead(BUTTON_UP_PIN) == HIGH) && (button_up_clicked == 1)) { // unclick 
        button_up_clicked = 0;
      }
      if ((digitalRead(BUTTON_DOWN_PIN) == HIGH) && (button_down_clicked == 1)) { // unclick
        button_down_clicked = 0;
      }

  }


  if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_select_clicked == 0)) { // select button clicked, jump between screens
     button_select_clicked = 1; // set button to clicked to only perform the action once
     if (current_screen == 0) {current_screen = 1;} // menu items screen --> screenshots screen
     else if (current_screen == 1) {current_screen = 0;} // screenshots screen --> qr codes screen
  }
  if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_select_clicked == 1)) { // unclick 
    button_select_clicked = 0;
  }

  // set correct values for the previous and next items
  item_sel_previous = item_selected - 1;
  if (item_sel_previous < 0) {item_sel_previous = NUM_ITEMS - 1;} // previous item would be below first = make it the last
  item_sel_next = item_selected + 1;  
  if (item_sel_next >= NUM_ITEMS) {item_sel_next = 0;} // next item would be after last = make it the first



  

    if (current_screen == 0) { // MENU SCREEN
    u8g2.clearBuffer();  // clear buffer for storing display content in RAM
      // selected item background
      u8g2.drawXBMP(0, 22, 128, 21, bitmap_item_sel_outline);

      // draw previous item as icon + label
      u8g2.setFont(u8g_font_7x14);
      u8g2.drawStr(25, 15, menu_items[item_sel_previous]); 
      u8g2.drawXBMP( 4, 2, 16, 16, bitmap_icons[item_sel_previous]);          

      // draw selected item as icon + label in bold font
      u8g2.setFont(u8g_font_7x14B);    
      u8g2.drawStr(25, 15+20+2, menu_items[item_selected]);   
      u8g2.drawXBMP( 4, 24, 16, 16, bitmap_icons[item_selected]);     

      // draw next item as icon + label
      u8g2.setFont(u8g_font_7x14);     
      u8g2.drawStr(25, 15+20+20+2+2, menu_items[item_sel_next]);   
      u8g2.drawXBMP( 4, 46, 16, 16, bitmap_icons[item_sel_next]);  

      // draw scrollbar background
      u8g2.drawXBMP(128-8, 0, 8, 64, bitmap_scrollbar_background);

      // draw scrollbar handle
      u8g2.drawBox(125, 64/NUM_ITEMS * item_selected, 3, 64/NUM_ITEMS); 
          
    } 
    else if (current_screen == 1) { // 
        
        if (item_selected == 0)
        {
          std::vector<double> result = calibrate(50);

          refPhasors[0] = result[0];
          refPhasors[1] = result[1];

          refPhasors[2] = result[2];
          refPhasors[3] = result[3];
        }
        if (item_selected == 1)
        {
        measureMode();//do something

      
        }
        if(item_selected == 2)
        {   
            displayVolume();
          // Check if volume up button is pressed
            if (digitalRead(BUTTON_UP_PIN) == HIGH && buzzerDutyCycle < 100) 
            { 
              buzzerDutyCycle++;
              analogWrite(buzzerPin, 255 * buzzerDutyCycle/100);
              displayVolume();
              delay(200); // Debounce delay
            }
            // Check if volume down button is pressed
            if (digitalRead(BUTTON_DOWN_PIN) == HIGH && buzzerDutyCycle > 0) 
            { 
              buzzerDutyCycle--;
              analogWrite(buzzerPin, 255 * buzzerDutyCycle/100);
              displayVolume();
              delay(200); // Debounce delay
            }
        }
        if (item_selected == 4)
        {
          u8g2.clearBuffer();                  // clear the internal memory
          u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font

          char buf[30];
  
          sprintf(buf, "sending...");
          u8g2.drawStr(0,30,buf);

          if (Serial.available()) {
    char data = Serial.read();

    if (data == 65) {
      int time_taken = 0;
      long start_time;
      long end_time;
      start_time = micros();
      for (int i = 0; i < numSamples; i++) {
      adcValues[i] = adc->adc0->analogRead(adcPin);
      adcValues2[i] = adc->adc1->analogRead(adcPin2);
      }
      end_time = micros();
      time_taken = (int)(end_time - start_time);
      sample_rate = numSamples / (time_taken * pow(10, -6));
      sendHeader(2*2*sizeof(adcValues)/sizeof(adcValues[0]), adcResolution, 12, time_taken);
      sendArray(adcValues, sizeof(adcValues)/sizeof(adcValues[0]));
      sendArray(adcValues2, sizeof(adcValues2)/sizeof(adcValues2[0]));


      u8g2.clearBuffer();                  // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font

      char buf[30];

      sprintf(buf, "Data sent.");
      u8g2.drawStr(0,30,buf);
      u8g2.sendBuffer();
      delay(1000);
    }
  }
        }
    }
    else if (current_screen == 2) { // QR SCREEN
    }   


  u8g2.sendBuffer(); // send buffer from RAM to display controller
      
}

std::vector<double> phaseShift(float* transmitter_coil, float* receiver_coil, int N, int sample_rate, float waveFrequency) {
  double delta_t = 1.0 / sample_rate;  // Time step
  double t[N];

  for (int i = 0; i < N; i++) {
    t[i] = i * delta_t;
  }

  float omega0 = 2 * PI * waveFrequency;  // Angular frequency in rad/s
  float A = 3300;

  float a[N];
  float b[N];

  for (int i = 0; i < N; i++) {
    a[i] = A * cos(omega0 * t[i]);
    b[i] = -A * sin(omega0 * t[i]);
  }


  float I = 0.0;
  float R = 0.0;

  float I2 = 0.0;
  float R2 = 0.0;

  for (int i = 0; i < N; i++) {
    I += transmitter_coil[i] * b[i];
    R += transmitter_coil[i] * a[i];

    I2 += receiver_coil[i] * b[i];
    R2 += receiver_coil[i] * a[i];
  }

  float phase = atan2(I, R) * (180.0 / PI);
  float phase2 = atan2(I2, R2) * (180.0 / PI);
  double shift = phase - phase2;
  double amplitude = sqrt(pow(I2, 2) + pow(R2, 2)) / N;

  if (shift < 0) {
   shift += 360;
  }

  std::vector<double> myArray = {amplitude, shift};
  return myArray;
}

void testdrawstyles(std::vector<double> toDisplay) {
  double amplitude1 = toDisplay[0];
  double phase1     = toDisplay[1];
  double amplitude2 = toDisplay[2];
  double phase2     = toDisplay[3];

  u8g2.clearBuffer();                  // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font

  char buf[30];
  
  sprintf(buf, "A at f0: %.2f", amplitude1);
  u8g2.drawStr(0,10,buf);

  // Assuming vectorSubtract and refPhasors are defined elsewhere
  std::vector<double> measured = {amplitude1, phase1};
  std::vector<double> ref = {refPhasors[0], refPhasors[1]};
  std::vector<double> resultantVector = vectorSubtract(measured,ref);

  std::vector<double> measured2 = {amplitude2, phase2};
  std::vector<double> ref2 = {refPhasors[2], refPhasors[3]};
  std::vector<double> resultantVector2 = vectorSubtract(measured2,ref2);

  sprintf(buf, "0.A resultant: %.2f", resultantVector[0]);
  u8g2.drawStr(0,20,buf);
  sprintf(buf, "0.Phase: %.2f", resultantVector[1]);
  u8g2.drawStr(0,30,buf);

  sprintf(buf, "1.A resultant: %.2f", resultantVector2[0]);
  u8g2.drawStr(0,40,buf);
  sprintf(buf, "1.Phase: %.2f", resultantVector2[1]);
  u8g2.drawStr(0,50,buf);

  Serial.print("0A: ");
  Serial.println(resultantVector[0]);
  Serial.print("0P: ");
  Serial.println(resultantVector[1]);

  Serial.print("1A: ");
  Serial.println(resultantVector2[0]);
  Serial.print("1P: ");
  Serial.println(resultantVector2[1]);

  if (isMetal(resultantVector[0])) {
    u8g2.setFontMode(1); // transparent
    u8g2.setDrawColor(2); // xor (draw in inverse)
    u8g2.drawStr(50,60, "Metal");
    if(isFerrous(resultantVector[1]))
    {
      u8g2.setDrawColor(2); // xor (draw in inverse)
      u8g2.drawStr(50,70, "Ferrous Metal");

    }
    else
    {
      u8g2.setDrawColor(2); // xor (draw in inverse)
      u8g2.drawStr(50,70, "Non Ferrous Metal");
    }
  } else {
    u8g2.setDrawColor(1); // normal
    u8g2.drawStr(50,60, "No Metal");
  }
  
  u8g2.sendBuffer();                  // transfer internal memory to the display
}

void sampleADC(void)
{
  mean1 = 0;
  mean2 = 0;
  for (int i = 0; i < numSamples; i++) {
      adcValues[i] = adc->adc0->analogRead(adcPin);
      adcValues2[i] = adc->adc1->analogRead(adcPin2);
      mean1 += adcValues[i];
      mean2 += adcValues2[i];
      }
  mean1 = mean1/numSamples;
  mean2 = mean2/numSamples;
}
void adc2Voltage(void)
{
  for (int j = 0; j < numSamples; j++) {
        transmitter_coil[j] = (adcValues[j] - mean1) * (3.3 / pow(2, adcResolution));
        receiver_coil[j] = (adcValues2[j] - mean2) * (3.3 / pow(2, adcResolution));
      }
}

std::vector<double> getPhasors()
{
      double sample_rate;
      int time_taken = 0;
      long start_time;
      long end_time;
      //sample the adc both transmitter and reciever
      start_time = micros();
      sampleADC();
      end_time = micros();

      time_taken = (double)(end_time - start_time);
      sample_rate = numSamples / (time_taken * pow(10, -6));
    
      adc2Voltage();

      std::vector<double> freq1 = phaseShift(transmitter_coil, receiver_coil, numSamples, sample_rate, frequencyHz);
      std::vector<double> freq2 = phaseShift(transmitter_coil, receiver_coil, numSamples, sample_rate, 3*frequencyHz);
      std::vector<double> result = {freq1[0], freq1[1], freq2[0], freq2[1]};
      return result;
}

std::vector<double> calibrate(int averageNum)
{
  std::vector<double> result = {0, 0, 0, 0};
  std::vector<double> samples = getPhasors();
    for(int i = 0; i < averageNum; i++)
    {
      samples = getPhasors();
      result[0] += samples[0];
      result[1] += samples[1];

      result[2] += samples[2];
      result[3] += samples[3];
    }
      result[0] = result[0]/averageNum;
      result[1] = result[1]/averageNum;

      result[2] = result[2]/averageNum;
      result[3] = result[3]/averageNum;

      return result;
}

std::vector<double> vectorSubtract(std::vector<double> V2, std::vector<double> V1) //takes vectors in phasor form
{
  double I = V2[0]*sin(V2[1]*(PI/180.0)) - V1[0]*sin(V1[1]*(PI/180.0));
  double R = V2[0]*cos(V2[1]*(PI/180.0)) - V1[0]*cos(V1[1]*(PI/180.0));
  double angle = atan2(I,R)*(180.0/PI);
  double amplitude = sqrt(I*I + R*R);
  std::vector<double> result = {amplitude, angle}; 
  return result;
}

bool isMetal(double amplitude)
{
  if (amplitude > 25)
  {
    return true;
    
  }
  analogWrite(buzzerPin, 0); 
  return false;

}

bool isFerrous(double phase)
{
  if(phase > 0)
  { 
    analogWrite(buzzerPin, buzzerDutyCycle/100);
    analogWrite(buzzerPin, 2000);
    return false;
  }
  analogWrite(buzzerPin, buzzerDutyCycle/100);
  analogWrite(buzzerPin, 50);
  return true;
}

void measureMode(void)
{   
      std::vector<double> toDisplay = calibrate(20);
      testdrawstyles(toDisplay);
}




void displayVolume() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr); // Choose a suitable font
  char buf[20];
  sprintf(buf, "Volume: %d%%", buzzerDutyCycle);
  u8g2.drawStr(0, 30, buf); // Draw volume string
  u8g2.sendBuffer(); // Send to display
}

void sendData(int inputNumber) {
  byte highByte = (inputNumber >> 8) & 0x00FF;  // Get the high byte
  byte lowByte = inputNumber & 0x00FF;
  Serial.write(highByte);
  Serial.write(lowByte);
}

void sendArray(int array[], int arraysize) {
  for (int i = 0; i < arraysize; i++) {
    sendData(array[i]);
  }
}

void sendHeader(int numberOfBytes, uint8_t typeOfData, uint8_t resolution, int time_taken) {
  sendData(numberOfBytes);
  Serial.write(typeOfData);
  Serial.write(resolution);
  sendData(time_taken);
}