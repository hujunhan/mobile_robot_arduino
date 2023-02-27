
#include <IRremote.h>

/*
 *  IR read codes
 *  by Hanie kiani
 *  https://electropeak.com/learn/
 */
// #include <IRremote.h>  //including infrared remote header file
int RECV_PIN = 12; // the pin where you connect the output pin of IR sensor
IRrecv irrecv(RECV_PIN);
decode_results results;
void setup()
{
    Serial.begin(115200);
    irrecv.enableIRIn();
}
void loop()
{
    if (irrecv.decode(&results)) // Returns 0 if no data ready, 1 if data ready.
    {
        Serial.println(results.value);
        switch (results.value)
        {
        case 0xFF18E7:
            Serial.println("up");
            break;
        case 0xFF5AA5:
            Serial.println("right");
            break;
        case 0xFF4AB5:
            Serial.println("down");
            break;
        case 0xFF10EF:
            Serial.println("left");
            break;
        case 0xFF38C7:
            Serial.println("ok");
            break;
        }
        // int value = results.value; ;// Results of decoding are stored in result.value
        irrecv.resume(); // Restart the ISR state machine and Receive the next value
    }
}
