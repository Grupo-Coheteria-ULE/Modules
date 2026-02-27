// AMF - BASIC APOGEE DETECTION & DATA LOGGING
// OBJECTIVE: LAUNCH
// FIRST VERSION
//
// Description:
//   - Reads pressure/temperature/altitude from MS5837 barometric sensor
//   - Streams values over Serial and logs to an SD card in delimited format
//   - Performs a simple apogee detection based on a decreasing altitude trend
//
// Log format (semicolon separated):
//   time_us;pressure_mbar;temperature_c;altitude_m

// Libraries
#include <Wire.h>
#include "SdFat.h"                // SD card handling library (SdFat)
#include "MS5837.h"

// Communication speed definitions
#define SPI_CLOCK 4000000         // SPI clock frequency at 4 MHz
#define BAUD_RATE 1000000         // Serial baud rate (bps)

// Pin definitions
#define LED_PIN 13                // Status LED pin (unused in this sketch)
#define CS_SD A4                  // SD card Chip Select pin

// Object instances and constants
#define DELIMITER ";"            // CSV-like delimiter for output/logging
MS5837 sensor;                    // Barometric sensor instance
SdFat SD;                         // SD card stack instance
SdFile dataFile;                  // SD file handle for data logging

// State variables
int apogeed;                      // Apogee flag (0: not reached, 1: reached)
uint32_t pressure_, temperature_, altitude_;   // Sensor readings (MS5837)
unsigned long currentTime;        // Timestamp in microseconds (micros())
uint8_t Sd_flush = 0;             // Counter to throttle SD flush calls

void setup() {

  pinMode(PC13,OUTPUT);
    delay(500);
    digitalWrite(PC13,1);
    

    // Initialize serial for debugging/telemetry
    Serial.begin(115200);
    while(!Serial) { }                        //Esto porque la blackpill a veces empieza a leer el codigo en dfu y tarda en cargar el cdc para ver el serial
    Serial.println("Starting");

digitalWrite(PC13,0);

    // Initialize I2C bus for the barometer
    Wire.begin();

    // Initialize peripherals
    initializeBarometer();
    initializeSD();
    printHeader();
    

}

void loop() {


    // Capture timestamp at the start of the loop (microseconds)
    currentTime = micros();

    // Read latest sensor values (pressure/temperature are updated internally)
    sensor.read();

    // Retrieve computed readings from the driver
    pressure_ = sensor.pressure();
    temperature_ = sensor.temperature();
    altitude_ = sensor.altitude();

    // Output to Serial and log to SD
    printValues();
    logData();

    // Evaluate apogee condition
    apogeed = apogeeDetection();
    if (apogeed == 1) {
        trigger();
    }

    // Loop pacing (10 ms)
    delay(10);
}

void initializeSD() {

    // Try to initialize the SD card at CS pin with 4 MHz clock
    if (!SD.begin(CS_SD, SD_SCK_MHZ(SPI_CLOCK/1000000))) {
        Serial.println("Error initializing SD. Check connection and card.");
        while (true) { /* Halt if SD is not detected */ }
    } else {
        Serial.println("SD card initialized successfully.");
    }

    // Open (or create) the data log file for appending
    if (!dataFile.open("data_bar.txt", O_RDWR | O_CREAT | O_AT_END)) {
        Serial.println("Error opening SD file.");
        while (true) { /* Halt if file can't be opened */ }
    } else {
        Serial.println("File data.txt opened successfully on SD.");
    }
}

void initializeBarometer() {

    // Initialize the pressure sensor. Block until successful.
    // We can't continue if the sensor cannot be initialized.
    while (!sensor.init()) {
        Serial.println("MS5837 init failed, retrying...");
        delay(1000);
    }

    // Configure sensor model and medium density (air).
    sensor.setModel(MS5837::MS5837_02BA); 
    sensor.setFluidDensity(1.25); // kg/m^3 (1.2 for air, ~1029 for seawater)
    Serial.println("MS5837 initialized.");
}

void printHeader() {
    Serial.println("############################");
    Serial.println("##   BAROMETER DATA LOG   ##");
    Serial.println("############################");
    Serial.println("##   TIME (us)   ##   PRESSURE (mbar)   ##   TEMPERATURE (C)   ##   ALTITUDE (m)   ##");
    Serial.println("############################");

    dataFile.println("############################");
    dataFile.println("##   BAROMETER DATA LOG   ##");
    dataFile.println("############################");
    dataFile.println("##   TIME (us)   ##   PRESSURE (mbar)   ##   TEMPERATURE (C)   ##   ALTITUDE (m)   ##");
    dataFile.println("############################");
}

void printValues() {

    // Print a single line to Serial using the configured delimiter
    Serial.print(currentTime);
    Serial.print(DELIMITER);
    Serial.print(pressure_);
    Serial.print(DELIMITER);
    Serial.print(temperature_);
    Serial.print(DELIMITER);
    Serial.println(altitude_);
}

void logData(){

    // Write a data record to the SD file
    dataFile.print(currentTime);
    dataFile.print(DELIMITER);
    dataFile.print(pressure_);
    dataFile.print(DELIMITER);
    dataFile.print(temperature_);
    dataFile.print(DELIMITER);
    dataFile.println(altitude_);
    // Note: This does not append a newline. Consider dataFile.println(...) if each record should be on its own line.

    // Flush periodically to reduce data loss risk while limiting write overhead
    Sd_flush++; // Increment the flush counter
    if (Sd_flush > 50) // Flush the SD card every 50 writes; tune as needed
    {
        dataFile.flush();
    }
    
}

void trigger(){

    // Notification when apogee is detected (placeholder for pyro/actuation)
    Serial.println("############################ TRIGGER ############################");
    dataFile.print("############################ TRIGGER ############################");
    // TODO: Light up a LED or perform another action
}

int apogeeDetection(){ // Example apogee detection algorithm; can be replaced by other logic

    // Simple apogee detection using a short decreasing trend of altitude samples.
    // Keep a circular buffer of the last 101 altitude samples and consider apogee
    // detected if most of the last 5 comparisons show a decrease.
    static float altBuffer[101] = {0};
    static int idx = 0;

    // Store newest altitude sample into the ring buffer
    altBuffer[idx] = (float) altitude_;
    idx = (idx + 1) % 101;

    // Check the last 5 consecutive pairs for a decreasing trend
    int decreasingCount = 0;
    for (int i = 0; i < 5; i++) {
        int a = (idx - 1 - i + 101) % 101;
        int b = (idx - 2 - i + 101) % 101;
        if (altBuffer[a] < altBuffer[b]) {
            decreasingCount++;
        }
    }

    // If most of the recent comparisons show a decrease, treat as apogee
    if (decreasingCount >= 4) {
        return 1;
    } else {
        return 0;
    }
}