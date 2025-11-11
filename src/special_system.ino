// ============================================
// LIBRARY INCLUDES
// ============================================
#include "Particle.h"
#include "HTS221.h"

// ============================================
// SYSTEM CONFIGURATION
// ============================================
const int NUM_PLANTS = 2;

// Timing intervals (milliseconds)
const unsigned long NORMAL_READ_INTERVAL_MS = 30000;      // 30 seconds
const unsigned long POST_WATERING_READ_INTERVAL_MS = 5000; // 5 seconds
const unsigned long POST_WATERING_DURATION_MS = 60000;     // 1 minute
const unsigned long VALVE_OPEN_DURATION_MS = 1000;         // 1 second
const unsigned long MAX_VALVE_OPEN_TIME_MS = 2000;         // 2 seconds (safety)

// Error detection
const int SENSOR_ERROR_THRESHOLD = 5;  // consecutive failed reads

// ============================================
// HARDWARE PIN MAPPINGS
// ============================================
// Soil moisture sensor pins (analog)
const int soilMoisturePins[NUM_PLANTS] = {A0, A1};

// Calibration values for soil moisture sensors
// These will be populated during calibration in Phase 1
int soilMoistureDry[NUM_PLANTS] = {50, 50};   // Value when completely dry
int soilMoistureWet[NUM_PLANTS] = {3000, 3000};   // Value when fully saturated

// ============================================
// SENSOR OBJECTS
// ============================================
// HTS221 humidity/temperature sensors
HTS221 hts221_sensors[NUM_PLANTS];

// ============================================
// DATA STRUCTURES
// ============================================
struct PlantState {
    // Sensor readings
    int soilMoisture;           // Raw analog value
    float humidity;             // Percentage (0-100)
    float temperature;          // Celsius
    
    // Sensor health tracking
    int soilMoistureErrorCount;
    int humidityErrorCount;
    int temperatureErrorCount;
    String soilMoistureStatus;  // "ok", "degraded", "error"
    String humidityStatus;
    String temperatureStatus;
    
    // Watering state
    bool isWatering;
    bool inPostWateringMode;
    unsigned long lastWateringEnd_ms;
    unsigned long postWateringStart_ms;
    bool wateringCommandPending;
};

// ============================================
// GLOBAL STATE VARIABLES
// ============================================
// Array holding state for all plants
PlantState plants[NUM_PLANTS];

// Global timing
unsigned long lastSensorRead_ms = 0;
unsigned int currentReadInterval_ms = NORMAL_READ_INTERVAL_MS;

// ============================================
// SENSOR READING FUNCTIONS
// ============================================

/**
 * Read soil moisture sensor for a specific plant
 * @param plantId The plant index (0 to NUM_PLANTS-1)
 * @return Raw analog value (0-4095 for Photon 2), or -1 if invalid plantId
 */
int readSoilMoisture(int plantId) {
    // Validate plant ID
    if (plantId < 0 || plantId >= NUM_PLANTS) {
        Serial.println("ERROR: Invalid plant ID " + String(plantId));
        return -1;
    }
    
    // Read analog value from the sensor pin
    int rawValue = analogRead(soilMoisturePins[plantId]);
    
    // Debug output
    Serial.println("Plant " + String(plantId) + " - Soil Moisture Raw: " + String(rawValue));
    
    return rawValue;
}

/**
 * Read humidity sensor for a specific plant
 * @param plantId The plant index (0 to NUM_PLANTS-1)
 * @return Humidity percentage (0-100), or -999 if error
 */
float readHumidity(int plantId) {
    // Validate plant ID
    if (plantId < 0 || plantId >= NUM_PLANTS) {
        Serial.println("ERROR: Invalid plant ID " + String(plantId));
        return -999;
    }
    
    // Use sensor 0 for all plants (shared sensor)
    float humidity = hts221_sensors[0].readHumidity();
    
    // Debug output
    Serial.println("Plant " + String(plantId) + " - Humidity: " + String(humidity) + " %");
    
    return humidity;
}

/**
 * Read temperature sensor for a specific plant
 * @param plantId The plant index (0 to NUM_PLANTS-1)
 * @return Temperature in Celsius, or -999 if error
 */
float readTemperature(int plantId) {
    // Validate plant ID
    if (plantId < 0 || plantId >= NUM_PLANTS) {
        Serial.println("ERROR: Invalid plant ID " + String(plantId));
        return -999;
    }
    
    // Use sensor 0 for all plants (shared sensor)
    float temperature = hts221_sensors[0].readTemperature();
    
    // Debug output
    Serial.println("Plant " + String(plantId) + " - Temperature: " + String(temperature) + " °C");
    
    return temperature;
}

/**
 * Read all sensors for all plants and update the plants array
 */
/**
 * Read all sensors for all plants and update the plants array
 * Includes error detection and status tracking
 */
void readAllSensors() {
    for (int i = 0; i < NUM_PLANTS; i++) {
        // Read soil moisture
        int moisture = readSoilMoisture(i);
        if (moisture >= 0) {
            // Validate reading (0-4095 is full ADC range for Photon 2)
            if (validateSensorReading(moisture, 0, 4095)) {
                plants[i].soilMoisture = moisture;
                plants[i].soilMoistureErrorCount = 0;  // Reset error counter
                plants[i].soilMoistureStatus = "ok";
            } else {
                plants[i].soilMoistureErrorCount++;
                if (plants[i].soilMoistureErrorCount >= SENSOR_ERROR_THRESHOLD) {
                    plants[i].soilMoistureStatus = "error";
                } else {
                    plants[i].soilMoistureStatus = "degraded";
                }
            }
        } else {
            // Read failed
            plants[i].soilMoistureErrorCount++;
            if (plants[i].soilMoistureErrorCount >= SENSOR_ERROR_THRESHOLD) {
                plants[i].soilMoistureStatus = "error";
            } else {
                plants[i].soilMoistureStatus = "degraded";
            }
        }
        
        // Read humidity
        float humidity = readHumidity(i);
        if (humidity > -999) {
            // Validate reading (0-100% is valid range)
            if (validateSensorReading(humidity, 0, 100)) {
                plants[i].humidity = humidity;
                plants[i].humidityErrorCount = 0;
                plants[i].humidityStatus = "ok";
            } else {
                plants[i].humidityErrorCount++;
                if (plants[i].humidityErrorCount >= SENSOR_ERROR_THRESHOLD) {
                    plants[i].humidityStatus = "error";
                } else {
                    plants[i].humidityStatus = "degraded";
                }
            }
        } else {
            plants[i].humidityErrorCount++;
            if (plants[i].humidityErrorCount >= SENSOR_ERROR_THRESHOLD) {
                plants[i].humidityStatus = "error";
            } else {
                plants[i].humidityStatus = "degraded";
            }
        }
        
        // Read temperature
        float temperature = readTemperature(i);
        if (temperature > -999) {
            // Validate reading (-40 to 120°C is sensor range)
            if (validateSensorReading(temperature, -40, 120)) {
                plants[i].temperature = temperature;
                plants[i].temperatureErrorCount = 0;
                plants[i].temperatureStatus = "ok";
            } else {
                plants[i].temperatureErrorCount++;
                if (plants[i].temperatureErrorCount >= SENSOR_ERROR_THRESHOLD) {
                    plants[i].temperatureStatus = "error";
                } else {
                    plants[i].temperatureStatus = "degraded";
                }
            }
        } else {
            plants[i].temperatureErrorCount++;
            if (plants[i].temperatureErrorCount >= SENSOR_ERROR_THRESHOLD) {
                plants[i].temperatureStatus = "error";
            } else {
                plants[i].temperatureStatus = "degraded";
            }
        }
    }
}

/**
 * Display all sensor data in a clean, simple format
 */
void printSensorData() {
    Serial.println("\n========================================");
    Serial.println("PLANT WATERING SYSTEM STATUS");
    Serial.println("========================================\n");
    
    for (int i = 0; i < NUM_PLANTS; i++) {
        Serial.println("PLANT " + String(i));
        Serial.println("----------------------------------------");
        
        // Soil Moisture
        int moisturePercent = moistureToPercent(i, plants[i].soilMoisture);
        Serial.println("Soil Moisture:  " + String(moisturePercent) + "% (raw: " + String(plants[i].soilMoisture) + ") [" + plants[i].soilMoistureStatus + "]");
        
        // Humidity
        Serial.println("Air Humidity:   " + String(plants[i].humidity, 1) + "% [" + plants[i].humidityStatus + "]");
        
        // Temperature
        Serial.println("Temperature:    " + String(plants[i].temperature, 1) + " C [" + plants[i].temperatureStatus + "]");
        
        // Error counts (only show if > 0)
        if (plants[i].soilMoistureErrorCount > 0 || 
            plants[i].humidityErrorCount > 0 || 
            plants[i].temperatureErrorCount > 0) {
            Serial.println("WARNING: Errors detected - M:" + String(plants[i].soilMoistureErrorCount) + 
                         " H:" + String(plants[i].humidityErrorCount) + 
                         " T:" + String(plants[i].temperatureErrorCount));
        }
        
        Serial.println();
    }
    
    Serial.println("========================================\n");
}

/**
 * Validate if a sensor reading is within acceptable range
 * @param value The sensor reading to validate
 * @param minValue Minimum acceptable value
 * @param maxValue Maximum acceptable value
 * @return true if valid, false if out of range
 */
bool validateSensorReading(float value, float minValue, float maxValue) {
    return (value >= minValue && value <= maxValue);
}

/**
 * Convert raw soil moisture reading to percentage
 * @param plantId The plant index for calibration lookup
 * @param rawValue The raw analog reading
 * @return Moisture percentage (0-100)
 */
int moistureToPercent(int plantId, int rawValue) {
    // Map raw value to percentage using calibration
    int percent = map(rawValue, soilMoistureDry[plantId], soilMoistureWet[plantId], 0, 100);
    
    // Constrain to 0-100 range
    percent = constrain(percent, 0, 100);
    
    return percent;
}

// ============================================
// SETUP FUNCTION
// ============================================
void setup() {
    // Initialize serial communication for debugging
    Serial.begin(9600);
    
    // Wait for serial connection (useful for debugging)
    waitFor(Serial.isConnected, 10000);
    
    Serial.println("=================================");
    Serial.println("Plant Watering System Starting...");
    Serial.println("=================================");
    
    // Initialize all plant states
    for (int i = 0; i < NUM_PLANTS; i++) {
        plants[i].soilMoisture = 0;
        plants[i].humidity = 0.0;
        plants[i].temperature = 0.0;
        
        plants[i].soilMoistureErrorCount = 0;
        plants[i].humidityErrorCount = 0;
        plants[i].temperatureErrorCount = 0;
        plants[i].soilMoistureStatus = "ok";
        plants[i].humidityStatus = "ok";
        plants[i].temperatureStatus = "ok";
        
        plants[i].isWatering = false;
        plants[i].inPostWateringMode = false;
        plants[i].lastWateringEnd_ms = 0;
        plants[i].postWateringStart_ms = 0;
        plants[i].wateringCommandPending = false;
    }
    
    Serial.println("System initialized successfully!");
    Serial.println("Number of plants: " + String(NUM_PLANTS));
    Serial.println("=================================\n");
    
    // Initialize I2C
    Wire.begin();
    Serial.println("I2C initialized");
    delay(100);
    
// Initialize and activate HTS221 sensors
    Serial.println("\nInitializing HTS221 sensors...");
    for (int i = 0; i < NUM_PLANTS; i++) {
        Serial.print("Initializing sensor " + String(i) + "...");
        
        if (hts221_sensors[i].begin()) {
            Serial.println(" SUCCESS!");
        } else {
            Serial.println(" FAILED!");
        }
    }
    Serial.println("HTS221 initialization complete\n");
    Serial.println("=================================\n");
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    // Read all sensors
    readAllSensors();
    
    // Display formatted data
    printSensorData();
    
    // Wait 3 seconds before next read
    delay(3000);
}