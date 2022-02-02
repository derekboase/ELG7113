// Library inclusion
#include <Fuzzy.h>

#define MOTOR_DRIVE 3
#define TRIG_PIN 9
#define ECHO_PIN 10

#define CAL_COUNT 10
#define LOOP_TIME 20
#define PWM_STEADY 160
#define WAIT_TIME 5000

Fuzzy* control = new Fuzzy();

byte count = 0;
float error_in, motor_pwm;
float baseline_height = 0, height, duration;
const float setpoint = 20;
unsigned long starting_time;

void setup() {
  Serial.begin(9600);

  // Initialization of PINS
  pinMode(ECHO_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(MOTOR_DRIVE, OUTPUT);

  analogWrite(MOTOR_DRIVE, PWM_STEADY);
  delay(10000);

  // Takes baseline height for true height calculation later
  delay(WAIT_TIME);
  do {
    baseline_height = init_baseline();
  } while (abs(baseline_height - readSensor()) > 1 );
  //  } while (false);

  /*
    DEFINITION AND INSTANTIATION OF FUZZY INPUT MEMBERSHIP FUNCTIONS
  */

  // Instantiating a fuzzy input object
  FuzzyInput *error = new FuzzyInput(1);

  // Error is negative
  FuzzySet *neg = new FuzzySet(-30, -30, -10, -5);
  error->addFuzzySet(neg);

  // Error is approximately zero
  FuzzySet *zero = new FuzzySet(-7.5, -2.5, 2.5, 7.5);
  error->addFuzzySet(zero);

  // Error is positive
  FuzzySet *pos = new FuzzySet(5, 10, 30, 30);
  error->addFuzzySet(pos);

  // point the control object to the fuzzy input membership functions
  control->addFuzzyInput(error);

  /*
    DEFINITION AND INSTANTIATION OF FUZZY OUTPUT MEMBERSHIP FUNCTIONS
  */

  // Instantiating a fuzzy output object
  FuzzyOutput *pwm = new FuzzyOutput(1);

  // Set fan to slow
  FuzzySet *slow = new FuzzySet(0, 0, 205, 215);
  pwm->addFuzzySet(slow);

  // Set fan to average
  FuzzySet *average = new FuzzySet(213, 215, 217, 221);
  pwm->addFuzzySet(average);

  // Set fan to fast
  FuzzySet *fast = new FuzzySet(217, 237, 255, 255);
  pwm->addFuzzySet(fast);

  // point the control object to the fuzzy output membership functions
  control->addFuzzyOutput(pwm);

  /*
        DEFINITION AND INSTANTIATION OF FUZZY RULESETS
  */

  // Rule 1

  // Antecedent - if error is negative
  FuzzyRuleAntecedent *ifErrorNeg = new FuzzyRuleAntecedent();
  ifErrorNeg->joinSingle(neg);

  // Consequent - set fan fast
  FuzzyRuleConsequent *thenPWMFast = new FuzzyRuleConsequent();
  thenPWMFast->addOutput(fast);

  // Rule - if error is negative set fan fast
  FuzzyRule *rule_1 = new FuzzyRule(1, ifErrorNeg, thenPWMFast);
  control->addFuzzyRule(rule_1);

  // Rule 2

  // Antecedent - if error is approximately zero
  FuzzyRuleAntecedent *ifErrorZero = new FuzzyRuleAntecedent();
  ifErrorZero->joinSingle(zero);
  // Consequent - set fan average
  FuzzyRuleConsequent *thenPWMAverage = new FuzzyRuleConsequent();
  thenPWMAverage->addOutput(average);
  // Rule - if error is approximately zero set fan average
  FuzzyRule *rule_2 = new FuzzyRule(2, ifErrorZero, thenPWMAverage);
  control->addFuzzyRule(rule_2);

  // Rule 3

  // Antecedent - if error is positive
  FuzzyRuleAntecedent *ifErrorPos = new FuzzyRuleAntecedent();
  ifErrorPos->joinSingle(pos);
  // Consequent - set fan slow
  FuzzyRuleConsequent *thenPWMSlow = new FuzzyRuleConsequent();
  thenPWMSlow->addOutput(slow);
  // Rule - if error is approximately zero set fan average
  FuzzyRule *rule_3 = new FuzzyRule(3, ifErrorPos, thenPWMSlow);
  control->addFuzzyRule(rule_3);
  

  starting_time = millis();
}

void loop() {
  height = (baseline_height - readSensor());
  error_in = height - setpoint;
  control->setInput(1, error_in);
  control->fuzzify();
  motor_pwm = control->defuzzify(1);
  analogWrite(MOTOR_DRIVE, motor_pwm);
  Serial.println((String)(millis() - starting_time) + "," + error_in + "," + height); // Used for data_logging
  //    if (count == 0) {
  //      Serial.println((String)"HEIGHT:\t" + height); // Used for data_logging
  //      count = 0;
  //    }
  //    else{
  //      count++;
  //    }
  delayMicroseconds(LOOP_TIME);
}

double init_baseline() {
  double cal_height = 0;
  Serial.println("Wait...");
  Serial.println("Calibrating...");
  for (byte cal = 0; cal < CAL_COUNT; cal++) {
    cal_height += readSensor();
    delay(LOOP_TIME);
  }
  Serial.println("Done!");

  return (cal_height / CAL_COUNT);
}

double readSensor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  return ((duration *  .0343) / 2);
}
