// delaywfi  like DUE and teennsy and pyboard, use wfi in delay
//  maple delay() uses spin loop, we'll  use millis

#include <stdint.h>

void delaywfi(uint32_t ms){
    uint32_t start = micros();

    if (ms > 0) {
        while (1) {
            if ((micros() - start) >= 1000) {
                ms--;
                if (ms == 0) return;
                start += 1000;
            }
            asm("wfi");
        }
    }
}

void setup() {
	pinMode(BOARD_LED_PIN, OUTPUT);
}

void loop() {
	toggleLED();
	delay(5000);
}
