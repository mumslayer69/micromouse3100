// Display class manages an SSD1306 object and provides interfacing to it.
#include <Adafruit_SSD1306.h>

#define SCREEN_ADDRESS 0x3C
#define TEXT_SIZE 2

namespace mtrn3100 {
    class Display {
    public:
        Display(Adafruit_SSD1306 &display) : display(display) {
        }

        initialise() {
            // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
            if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
                Serial.println(F("SSD1306 allocation failed"));
                for(;;); // Don't proceed, loop forever
            }
            display.clearDisplay();
        }

        print(char* string) {
            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(TEXT_SIZE);
            display.setCursor(0,0);
            display.print(string);
            display.display();
        }

        print(long int value) {
            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(TEXT_SIZE);
            display.setCursor(0,0);
            display.print(value);
            display.display();
        }

        // to print two integers, easiest way to implement is function overloading
        print(long int value1, long int value2) {
            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(TEXT_SIZE);
            display.setCursor(0,0);
            display.print(value1);
            display.print(" ");
            display.print(value2);
            display.display();
        }

        // same for 3
        print(long int value1, long int value2, long int value3) {
            display.clearDisplay();
            display.setTextColor(WHITE);
            display.setTextSize(TEXT_SIZE);
            display.setCursor(0,0);
            display.print(value1);
            display.print(" ");
            display.print(value2);
            display.print(" ");
            display.print(value3);
            display.display();
        }
    
    private:
        Adafruit_SSD1306 &display;
    };
}