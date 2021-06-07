#include <Bounce.h>
#include <TeensyThreads.h>

const int buttonPin = 12;
Bounce pushbutton = Bounce(buttonPin, 10);  // 10 ms debounce

byte previousState = HIGH;         // what state was the button last time
unsigned int intCount = 0;            // how many times has it changed to low
unsigned long countAt = 0;         // when count changed
unsigned int countPrinted = 0;     // last count printed

void hallEffectThread() {
  while (1) {
    if (pushbutton.update()) {
    if (pushbutton.fallingEdge()) {
      intCount = intCount + 1;
      countAt = millis();
    }
  } else {
    if (intCount != countPrinted) {
      unsigned long nowMillis = millis();
      if (nowMillis - countAt > 100) {
        Serial.print("Count: ");
        Serial.println(intCount);
        countPrinted = intCount;
      }
    }
  }
    threads.yield();
  }
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(57600);
  Serial.println("Pushbutton Bounce library test:");

  threads.addThread(hallEffectThread);
}


void loop() {
  //Serial.println("lol");
  //delay(1000);
}
