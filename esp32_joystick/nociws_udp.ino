#include <WiFi.h>
#include <WiFiAP.h>
#include <AsyncUDP.h>

const char *ssid = "nociws_bot";
const char *password = "mercurylives";

const uint8_t  L_FORWARD_LED = 1;
const uint8_t  L_REVERSE_LED = 18;
const uint8_t  UD_PIN = 10;
const uint8_t  R_FORWARD_LED = 2;
const uint8_t  R_REVERSE_LED = 17;
const uint8_t  LR_PIN = 12;

uint16_t udCenter;
uint16_t lrCenter;
uint16_t udMin;
uint16_t udMax;
uint16_t lrMin;
uint16_t lrMax;

uint8_t leftSpeed;
uint8_t rightSpeed;
uint8_t leftForward;
uint8_t rightForward;

const double EPSILON = 0.01;

AsyncUDP udp;

void setup() {
  pinMode(UD_PIN, INPUT);
  pinMode(LR_PIN, INPUT);
  pinMode(L_FORWARD_LED, OUTPUT);
  pinMode(R_FORWARD_LED, OUTPUT);
  pinMode(L_REVERSE_LED, OUTPUT);
  pinMode(R_REVERSE_LED, OUTPUT);
  udCenter = udMin = udMax = analogRead(UD_PIN);
  lrCenter = lrMin = lrMax = analogRead(LR_PIN);

  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while(1);
  }
}

void loop() {
  uint16_t rawUdVal, rawLrVal;
  int16_t udVal, lrVal;
  double magnitude, angle, stickScale;

  rawUdVal = analogRead(UD_PIN);
  rawLrVal = analogRead(LR_PIN);

  udMin = min(rawUdVal, udMin);
  lrMin = min(rawLrVal, lrMin);
  udMax = max(rawUdVal, udMax);
  lrMax = max(rawLrVal, lrMax);

  if(rawUdVal >= udCenter) {
    udVal = map(rawUdVal, udCenter, udMax, 0, 255);
  } else {
    udVal = map(rawUdVal, udMin, udCenter, -255, 0);
  }
  if(rawLrVal >= lrCenter) {
    lrVal = map(rawLrVal, lrCenter, lrMax, 0, 255);
  } else {
    lrVal = map(rawLrVal, lrMin, lrCenter, -255, 0);
  }

  getSingleStickBytes(double(lrVal)/255.0, double(udVal)/255.0);

  analogWrite(L_FORWARD_LED, leftForward   ? leftSpeed  : 0);
  analogWrite(L_REVERSE_LED, !leftForward  ? leftSpeed  : 0);
  analogWrite(R_FORWARD_LED, rightForward  ? rightSpeed : 0);
  analogWrite(R_REVERSE_LED, !rightForward ? rightSpeed : 0);

  processClient(rawUdVal, rawLrVal);
}

/**
 * Send motor speeds (0-255) and polarities (0 or 1) over UDP.
 */
void processClient(uint16_t ud, uint16_t lr) {
  if(udp.listen(1234)) {
    udp.onPacket([ud, lr](AsyncUDPPacket packet) {
        //reply to the client
        packet.printf("%d %d %d %d",
          leftSpeed,
          rightSpeed,
          leftForward,
          rightForward
        );
    });
  }
}

/**
 * Returns the left and right motor bytes in single-stick
 * control mode. The most significant byte is the left
 * motor byte, and the least significant byte is the right
 * motor byte. This is mostly pirated from Matthew Spinks's
 * code.
 *
 * Adapted from Nick Overacker's 2014 Mercury Robotics code.
 *
 * @param xInput The horizontal axis data for the analog stick.
 * @param yInput The vertical axis data for the analog stick.
 * @return The left and right motor bytes concatenated into
 *           one short. The left motor byte is the most
 *           significant byte.
 */
void getSingleStickBytes(double xInput, double yInput) {
  if(abs(yInput) < EPSILON && abs(xInput) < EPSILON) {
    // The stick is probably not even being moved.
    leftSpeed = rightSpeed = 0;
    return;
  }

  double lMotorFloat = 0;
  double rMotorFloat = 0;

  // Trig functions use RADIANS by default.
  double angle = atan2(yInput, xInput);
  double magnitude = sqrt(xInput*xInput + yInput*yInput);
  double stickScale = cos(2*angle);

  // remap the magnitude eliminating any slack in the joystick
  // think about where the function should start mapping joystick values.
  // It begins at the point (tolerance, tolerance). Now find the magnitude
  // of that.
  // mag = sqrt((tolerance^2) + (tolerance^2))
  // which simplifies to:
  // mag = sqrt(2*(tolerance^2))
  // That's our starting point. i.i. our new zero point.
  double inMin = sqrt(2*EPSILON*EPSILON);
  double inMax = 1;
  double outMin = 0;
  double outMax = 1;
  magnitude = (
    (magnitude - inMin) *
    (outMax - outMin) /
    (inMax - inMin) +
    outMin
  );

  // joystick is not a perfect circle, therefore ignore anything
  // greater than 1.0 or less than zero
  if(magnitude > 1.0) magnitude = 1.0;
  if(magnitude < 0.0)   magnitude = 0;

  if(angle >= 0 && angle < PI/2) { // Quadrant I
    lMotorFloat = magnitude;
    rMotorFloat = -1 * stickScale * magnitude;
  }
  else if(angle >= PI/2 && angle < PI) { // Quadrant II
    rMotorFloat = magnitude;
    lMotorFloat = -1 * stickScale * magnitude;
  }
  else if(angle >= -PI && angle < -PI/2) { // Quadrant III
    lMotorFloat = -1 * magnitude;
    rMotorFloat = stickScale * magnitude;
  }
  else if(angle >= -PI/2 && angle < 0) { // Quadrant IV
    rMotorFloat = -1 * magnitude;
    lMotorFloat = stickScale * magnitude;
  }

  // Get the sign bits.
  leftForward = lMotorFloat > 0;
  rightForward = rMotorFloat > 0;

  // Get the absolute values of the motor floats.
  lMotorFloat = abs(lMotorFloat);
  rMotorFloat = abs(rMotorFloat);

  // Get the left byte.
  leftSpeed  = callFscale(lMotorFloat);
  rightSpeed = callFscale(rMotorFloat);
}

/**
* Calls fscale with appropriate parameters.
* 
* I don't want to modify fscale, since it is a good general-purpose
* function. So instead, I am using this to set the parameters.
*
* Adapted from Nick Overacker's 2014 Mercury Robotics code.
*
* @param input The input value to scale.
* @return The scaled value.
*/
uint8_t callFscale(double input) {
  uint16_t retVal = uint16_t(255 * fscale(input, 0, 1, 0, 1, -4));
  return uint8_t(min(uint16_t(255), retVal));
}

/**
* A remapping function with logarithmic scaling.
*
* Based on a function written by Gred Shakar, and subsequently
* modified by Paul Badger (2007) and Matthew Spinks (2012).
*
* Comments in the method are carried over directly from Matthew's
* version.
*
* Adapted in 2014 by Nick Overacker for Team Atomic (Mercury Robotics).
* Re-adapted in 2024 by Nick Overacker for Nociws.
*
* @param inputValue  The value to remap.
* @param originalMin The original minimum value.
* @param originalMax The original maximum value.
* @param newBegin    The new minimum value.
* @param newEnd      The new maximum value.
* @param curveValue  The "sharpness" of the remapping curve.
* @return A curved remapping of the input value.
*/
double fscale(double inputValue, double originalMin, double originalMax,
              double newBegin, double newEnd, double curveValue) {
  double originalRange = 0;
  double newRange = 0;
  double zeroRefCurVal = 0;
  double normalizedCurVal = 0;
  double rangedValue = 0;
  bool   invFlag = false;

  // Condition curve parameter
  // Limit range
  if(curveValue > 10)       curveValue = 10;
  else if(curveValue < -10) curveValue = -10;

  // Invert and scale - this seems more intuitive.
  // Positive numbers give more weight to high end on output.
  curveValue *= -.1;
  // Convert linear scale into logarithmic exponent for other pow function.
  curveValue = pow(10, curveValue);

  // Zero reference the values.
  originalRange = originalMax - originalMin;

  if(newEnd > newBegin) {
    newRange = newEnd - newBegin;
  }
  else {
    newRange = newBegin - newEnd;
    invFlag = true;
  }

  zeroRefCurVal = inputValue - originalMin;
  // Normalize to 0 - 1 double.
  normalizedCurVal = zeroRefCurVal / originalRange; 

  if(invFlag == false) {
    rangedValue = pow(normalizedCurVal, curveValue) * newRange + newBegin;
  }
  else { // invert the ranges
    rangedValue = newBegin - pow(normalizedCurVal, curveValue) * newRange;
  }

  return rangedValue;
}