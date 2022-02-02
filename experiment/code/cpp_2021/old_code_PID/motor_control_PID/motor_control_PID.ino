// Library inclusion
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v2.h>

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MOTOR_DRIVE 3   // RELAY_OUTPUT in example
#define TRIG_PIN 9
#define ECHO_PIN 10

#define CAL_COUNT 10
#define LOOP_TIME 5
#define PWM_STEADY 160
#define WAIT_TIME 3000

byte count = 0;
double baseline_height = 0, height, duration;

double K_crit = 6.5, T_crit = 2.287;
//double Kp = 0.5*K_crit, Ki = 0, Kd = 0;  // P Controller
//double Kp = 0.45*K_crit, Ki = 0.54*K_crit/ T_crit, Kd = 0;  // PI Controller
//double Kp = 0.8*K_crit, Ki = 0, Kd = 0.1 * K_crit * T_crit;  // PD Controller
//double Kp = 0.6*K_crit, Ki = 1.2 * K_crit / T_crit, Kd = 0.075 * K_crit * T_crit;  // Classic PID Controller
double Kp = 0.75 * K_crit, Ki = 1.75 * K_crit / T_crit, Kd = 0.075 * K_crit * T_crit; // Pessen Integral Rule
//double Kp = 2 / 3 * K_crit, Ki = 2 / 3 * K_crit / T_crit, Kd = 1 / 9 * K_crit * T_crit; // Some Overshoot
//double Kp = 0.2*K_crit, Ki = 0.4 * K_crit / T_crit, Kd = 2/30 * K_crit * T_crit;  // No Overshoot

unsigned long starting_time;

PID_v2 myPID(Kp, Ki, Kd, PID::Direct);

/*********************************************************

                  INITIALIZATION LOOP

*********************************************************/

void setup() {
  Serial.begin(9600);

  // Initialization of PINS
  pinMode(ECHO_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(MOTOR_DRIVE, OUTPUT);

  analogWrite(MOTOR_DRIVE, PWM_STEADY);  // Balance ball at bottom

  // OLED Initialiazation
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  delay(5000);  // For data logging

  do {
    baseline_height = init_baseline();
  } while (abs(baseline_height - readSensor()) > 1 );

  // Initialization of PID controller
  myPID.SetOutputLimits(0, 255);
  myPID.Start((baseline_height - readSensor()), PWM_STEADY, 20);
  delay(LOOP_TIME);
  starting_time = millis();
}

/*********************************************************

                      VOID LOOP

*********************************************************/

void loop() {
  height = (baseline_height - readSensor());
  const double output = myPID.Run(height);
  analogWrite(MOTOR_DRIVE, (int)output);
  if (count == 9) {
    display_height(height);
    count = 0;
  }
  else {
    count++;
  }
  Serial.println((String) (millis() - starting_time) + "," + (20 - height) + "," + height); // Used for data_logging
  delayMicroseconds(LOOP_TIME);
}

double init_baseline() {
  double cal_height = 0;
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(10, 18);
  display.clearDisplay();
  display.println("Wait..");
  display.display();
  delay(WAIT_TIME);

  // Calibration of the baseline height
  display.setCursor(10, 18);
  display.clearDisplay();
  display.println("Cal...");
  display.display();
  for (byte cal = 0; cal < CAL_COUNT; cal++) {
    cal_height += readSensor();
    delay(LOOP_TIME * 5);
  }

  display.clearDisplay();
  display.setCursor(15, 18);
  display.println("Done!");
  display.display();
  delay(1000);

  return (cal_height / CAL_COUNT);
}

double readSensor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  return ((duration * .0343) / 2);
}

void display_height(const double height_disp) {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.print(String(height_disp));
  display.display();
}
