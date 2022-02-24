// ----------------------------------------------------------------------------
// READ RC CHANNEL DATA FROM SPEKTRUM DSMX REMOTE RECEIVER
// 
// Code By: Michael Wrona | mwrona.com
// ----------------------------------------------------------------------------
/**
 * Sample Packet:
 *  00 a2 0b fe 2b fe 13 fe 21 32 1b fe 30 00 01 30
 *  ChID: 1     5     2     4     3     6     0
 */


#include <string.h>
#include <SoftwareSerial.h>

void processByte(uint8_t inByte);
void processFrame();

//delay between data bytes 22ms

uint8_t byteIndex {0};
uint8_t frameCount {0};  // Number of evaluated frames
uint8_t dataBytes[16];  // Array to store the entire 16 byte packet
uint8_t prevDataBytes[16];  // Store prev. ver. in case of data read error
uint16_t servo[7];  // Array to hold channel data (7 in total)
uint16_t numBadReadings {0};  // Number of missed frames (counter)
uint16_t prevServo[7];  // Store prev. correct readings
unsigned long prevUpdateTime {0};

SoftwareSerial MySerial(10, 11);  // Spektrum serial port


void setup() {
   // while (!Serial);  // Wait for serial console to open
    Serial.begin(115200);  // Arduino's port
    MySerial.begin(115200);  // Spektrum's port
}


void loop() {
    if (MySerial.available()) {
        uint8_t incomingByte = MySerial.read();  // Get data!
        //Serial.print(incomingByte);
        processByte(incomingByte);
    }
}


void processByte(uint8_t inByte) {
    unsigned long currTime {millis()};
    if (currTime - prevUpdateTime > 10) {
        // 22ms delay between data frames has passed, reset index
        byteIndex = 0;
        //Serial.print(1);
    }

    dataBytes[byteIndex] = inByte;  // Add byte to array
    byteIndex++;

    if (byteIndex == sizeof(dataBytes)) {  // If the index is at 16
        // dataBytes is full, time to parse and extract channel
        // data/servo positions
        ////Serial.print(1);
        processFrame();
        byteIndex = 0;
        frameCount++;
    }

    // currTime and prevUpdateTime will be very close when receiving bytes in
    // quick succession When the frame transmission is done, the time will
    // hold constant. Then, when a new frame comes along, the time difference
    // will be large
    prevUpdateTime = currTime;
}


void processFrame() {
    // Expects second byte (protocol ID) to be 0xA2.
    // If not, use previous valid data
    if (dataBytes[1] == 0xA2) {
        for (uint8_t ii = 2; ii < sizeof(dataBytes); ii += 2) {
            uint8_t hiByte {dataBytes[ii]};
            uint8_t loByte {dataBytes[ii+1]};
            uint16_t servoVal{};
            uint8_t chanID {(hiByte >> 3) & 0xf}; //0x7800 // Extract channel ID

            // Make sure channel ID is less than 6 (range from 0 to 6)
            //Serial.print(chanID+' ');
            if (chanID <= 6) {
                // Convert two bytes in big-endian to int
                // https://stackoverflow.com/a/2660326
                servoVal = ((hiByte << 8) | loByte) & 0x07FF;

                // Constrain values to a range, just in case an error occurs
                if (servoVal < 300)
                    servoVal = 300;

                if (servoVal > 1730)
                    servoVal = 1730;

                // Use equation in DSMX datasheet to convert [0 2048] values
                // to standard PWM range [~1000ms ~2000ms] with center at 1500ms
                servo[chanID] = (0.583f * servoVal) + 903;
                //Serial.print(servo[chanID]);
                //Serial.print(chanID);
               // Serial.print("\n");
            }
        }
        // memcpy(dest, src, sizeof);
        memcpy(prevDataBytes, dataBytes, sizeof(dataBytes));
        memcpy(prevServo, servo, sizeof(servo));
    }
    else {
        // Bad reading, use previous (valid) data
        memcpy(dataBytes, prevDataBytes, sizeof(prevDataBytes));
        memcpy(servo, prevServo, sizeof(prevServo));
        numBadReadings++;
    }
}
