#define TOUCH_PIN 7  // TTP223B OUT pin connected to digital pin 7
#define LED_PIN 13   // On-board LED on Arduino Uno

void setup() {
    pinMode(TOUCH_PIN, INPUT);  // Set touch sensor as input
    pinMode(LED_PIN, OUTPUT);   // Set LED as output
    Serial.begin(9600);         // Start serial communication
}

void loop() {
    int touchState = digitalRead(TOUCH_PIN);  // Read sensor state

    if (touchState == HIGH) {  // If touched
        digitalWrite(LED_PIN, HIGH);  // Turn on LED
        Serial.println("CLICK");  // Send signal to PC
        delay(100);  // Small debounce delay
    } else {
        digitalWrite(LED_PIN, LOW);   // Turn off LED
    }
}
