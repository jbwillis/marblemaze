/*
 * Read two integer servo commands and send them to the servos.
 */
#include <Servo.h>

Servo bot;
Servo top;

#define RANGE_MAX 1700
#define RANGE_MIN 1300

#define TOP_RANGE_OFFSET -25
#define BOT_RANGE_OFFSET -125

void setup() {
  bot.attach(9);
  top.attach(10);
  Serial.begin(115200);

  // initialize servos to be centered
  top.writeMicroseconds(1500 + TOP_RANGE_OFFSET);
  bot.writeMicroseconds(1500 + BOT_RANGE_OFFSET);
}

void loop() {
  while(Serial.available() > 0) 
  {
    // get commands
    int top_cmd = Serial.parseInt();
    int bot_cmd = Serial.parseInt();

    // write to servos, making sure the values are within a range
    if(bot_cmd >= RANGE_MIN + BOT_RANGE_OFFSET && bot_cmd <= RANGE_MAX + BOT_RANGE_OFFSET)
    {
      bot.writeMicroseconds(bot_cmd);
    }
    
    if(top_cmd >= RANGE_MIN + TOP_RANGE_OFFSET && top_cmd <= RANGE_MAX + TOP_RANGE_OFFSET)
    {
      top.writeMicroseconds(top_cmd);
    }

    // wait for a newline
    while(Serial.read() != '\n');
  }
}
