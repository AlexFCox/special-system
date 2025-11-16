// ============================================
// LIBRARY INCLUDES
// ============================================
#include "Particle.h"
#include "HTS221.h"

// ============================================
// SYSTEM CONFIGURATION
// ============================================
const int NUM_PLANTS = 2;
const int NUM_VALVES = 2;

// Simulation mode - set to false when real relay hardware is connected
const bool SIMULATION_MODE = true;

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

// ============================================
// VALVE/RELAY CONFIGURATION
// ============================================
// M5Stack 4-Relay I2C configuration
const uint8_t RELAY_I2C_ADDRESS = 0x26;
const uint8_t RELAY_CONTROL_REGISTER = 0x11;

// Plant to valve mapping (which valve waters which plant)
const int PLANT_TO_VALVE_MAP[NUM_PLANTS] = {0, 1};  // plant 0->valve 0, plant 1->valve 1

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

// Valve state structure
struct ValveState {
    bool isOpen;
    unsigned long openStartTime_ms;
    String status;  // "ok", "error"
};

// ============================================
// GLOBAL STATE VARIABLES
// ============================================
// Array holding state for all plants
PlantState plants[NUM_PLANTS];
// Array holding state for all valves
ValveState valves[NUM_VALVES];

// Global timing
unsigned long lastSensorRead_ms = 0;
unsigned int currentReadInterval_ms = NORMAL_READ_INTERVAL_MS;

// Connection tracking
String connectionStatus = "ok";

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
    //Serial.println("Plant " + String(plantId) + " - Soil Moisture Raw: " + String(rawValue));
    
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
    //Serial.println("Plant " + String(plantId) + " - Humidity: " + String(humidity) + " %");
    
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
    //Serial.println("Plant " + String(plantId) + " - Temperature: " + String(temperature) + " °C");
    
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

// ============================================
// VALVE CONTROL FUNCTIONS
// ============================================

/**
 * Open a specific valve
 * @param valveId The valve index (0 to NUM_VALVES-1)
 */
void openValve(int valveId) {
    // Validate valve ID
    if (valveId < 0 || valveId >= NUM_VALVES) {
        Serial.println("ERROR: Invalid valve ID " + String(valveId));
        return;
    }
    
    // Check if valve is already open
    if (valves[valveId].isOpen) {
        Serial.println("WARNING: Valve " + String(valveId) + " is already open");
        return;
    }
    
    if (SIMULATION_MODE) {
        // Simulation: just print
        Serial.println("🚰 [SIM] Opening valve " + String(valveId));
    } else {
        // Real hardware: send I2C command to relay
        // Calculate bit mask for this valve (bit 0 = valve 0, bit 1 = valve 1, etc.)
        uint8_t relayState = (1 << valveId);
        
        Wire.beginTransmission(RELAY_I2C_ADDRESS);
        Wire.write(RELAY_CONTROL_REGISTER);
        Wire.write(relayState);
        Wire.endTransmission();
        
        Serial.println("🚰 Opening valve " + String(valveId));
    }
    
    // Update valve state
    valves[valveId].isOpen = true;
    valves[valveId].openStartTime_ms = millis();
}

/**
 * Close a specific valve
 * @param valveId The valve index (0 to NUM_VALVES-1)
 */
void closeValve(int valveId) {
    // Validate valve ID
    if (valveId < 0 || valveId >= NUM_VALVES) {
        Serial.println("ERROR: Invalid valve ID " + String(valveId));
        return;
    }
    
    // Check if valve is already closed
    if (!valves[valveId].isOpen) {
        Serial.println("INFO: Valve " + String(valveId) + " is already closed");
        return;
    }
    
    if (SIMULATION_MODE) {
        // Simulation: just print
        Serial.println("🛑 [SIM] Closing valve " + String(valveId));
    } else {
        // Real hardware: send I2C command to relay (all bits 0 = all valves off)
        Wire.beginTransmission(RELAY_I2C_ADDRESS);
        Wire.write(RELAY_CONTROL_REGISTER);
        Wire.write(0x00);  // Turn off all relays
        Wire.endTransmission();
        
        Serial.println("🛑 Closing valve " + String(valveId));
    }
    
    // Update valve state
    valves[valveId].isOpen = false;
}

/**
 * Check all valves and close any that have exceeded maximum open time
 * This is a safety feature to prevent stuck-open valves
 */
void checkValveSafety() {
    unsigned long currentTime = millis();
    
    for (int i = 0; i < NUM_VALVES; i++) {
        if (valves[i].isOpen) {
            unsigned long openDuration = currentTime - valves[i].openStartTime_ms;
            
            // Check if valve has been open too long (safety timeout)
            if (openDuration > MAX_VALVE_OPEN_TIME_MS) {
                Serial.println("⚠ SAFETY: Valve " + String(i) + " exceeded max open time! Force closing.");
                closeValve(i);
                valves[i].status = "error";  // Mark valve as having an error
            }
        }
    }
}

/**
 * Update valve states - close valves after normal duration and update plant states
 * Called from main loop every iteration
 */
void updateValveStates() {
    unsigned long currentTime = millis();
    
    for (int i = 0; i < NUM_VALVES; i++) {
        // Only process valves that are currently open
        if (valves[i].isOpen) {
            unsigned long openDuration = currentTime - valves[i].openStartTime_ms;
            
            // Check if valve should close (normal duration reached)
            if (openDuration >= VALVE_OPEN_DURATION_MS) {
                Serial.println("✓ Valve " + String(i) + " reached target duration, closing");
                closeValve(i);
                
                // Find which plant uses this valve and update its state
                for (int plantId = 0; plantId < NUM_PLANTS; plantId++) {
                    if (PLANT_TO_VALVE_MAP[plantId] == i) {
                        // Update plant watering state
                        plants[plantId].isWatering = false;
                        plants[plantId].lastWateringEnd_ms = currentTime;
                        plants[plantId].inPostWateringMode = true;
                        plants[plantId].postWateringStart_ms = currentTime;
                        
                        Serial.println("  → Plant " + String(plantId) + " watering complete");
                        Serial.println("  → Entering post-watering monitoring (5s intervals for 1 min)");
                        
                        // Publish watering end event
                        String eventData = "{\"plantId\":" + String(plantId) + 
                                         ",\"valveId\":" + String(i) + 
                                         ",\"timestamp\":" + String(currentTime) + 
                                         ",\"duration_ms\":" + String(openDuration) + "}";
                        Particle.publish("event/watering_end", eventData, PRIVATE);
                        
                        break;  // Found the plant, no need to continue loop
                    }
                }
            }
        }
    }
}

/**
 * Water a specific plant - called via Particle.function() from cloud
 * @param command String containing plant ID (e.g., "0" or "1")
 * @return Status code: 0=success, -1=invalid ID, -2=already watering, -3=in cooldown, -4=valve error
 */
int waterPlant(String command) {
    // Parse plant ID from command
    int plantId = command.toInt();
    
    // Validate plant ID
    if (plantId < 0 || plantId >= NUM_PLANTS) {
        Serial.println("❌ Water command rejected: Invalid plant ID " + String(plantId));
        return -1;
    }
    
    Serial.println("\n💧 Water command received for Plant " + String(plantId));
    
    // Check if plant is already being watered
    if (plants[plantId].isWatering) {
        Serial.println("❌ Water command rejected: Plant " + String(plantId) + " is already being watered");
        return -2;
    }
    
    // Check if plant is in cooldown period
    unsigned long currentTime = millis();
    unsigned long timeSinceWatering = currentTime - plants[plantId].lastWateringEnd_ms;
    if (plants[plantId].lastWateringEnd_ms > 0 && timeSinceWatering < POST_WATERING_DURATION_MS) {
        unsigned long cooldownRemaining = POST_WATERING_DURATION_MS - timeSinceWatering;
        Serial.println("❌ Water command rejected: Plant " + String(plantId) + " in cooldown");
        Serial.println("   Cooldown remaining: " + String(cooldownRemaining / 1000) + " seconds");
        return -3;
    }
    
    // Get the valve ID for this plant
    int valveId = PLANT_TO_VALVE_MAP[plantId];
    
    // Check if valve is healthy
    if (valves[valveId].status != "ok") {
        Serial.println("❌ Water command rejected: Valve " + String(valveId) + " has error status");
        return -4;
    }
    
    // Check if any other valve is currently open (power supply limitation)
    for (int i = 0; i < NUM_VALVES; i++) {
        if (valves[i].isOpen) {
            Serial.println("❌ Water command rejected: Another valve (valve " + String(i) + ") is currently open");
            Serial.println("   Only one valve can operate at a time due to power supply limitation");
            return -5;
        }
    }
    
    // All checks passed - start watering!
    Serial.println("✓ Starting watering for Plant " + String(plantId) + " using Valve " + String(valveId));
    
    // Open the valve
    openValve(valveId);
    
    // Update plant state
    plants[plantId].isWatering = true;
    plants[plantId].wateringCommandPending = false;
    
    // Publish watering start event
    String eventData = "{\"plantId\":" + String(plantId) + 
                     ",\"valveId\":" + String(valveId) + 
                     ",\"timestamp\":" + String(currentTime) + "}";
    Particle.publish("event/watering_start", eventData, PRIVATE);
    
    return 0;  // Success!
}

/**
 * Check if enough time has elapsed to read sensors again
 * @return true if sensors should be read, false otherwise
 */
bool shouldReadSensors() {
    unsigned long currentTime = millis();
    unsigned long timeSinceLastRead = currentTime - lastSensorRead_ms;
    
    // Check if enough time has passed based on current interval
    if (timeSinceLastRead >= currentReadInterval_ms) {
        return true;
    }
    
    return false;
}

/**
 * Update the current sensor read interval based on plant states
 * If any plant is in post-watering mode, use 5s interval
 * Otherwise use normal 30s interval
 */
void updateReadInterval() {
    bool anyPlantInPostWatering = false;
    
    // Check if any plant is in post-watering monitoring mode
    for (int i = 0; i < NUM_PLANTS; i++) {
        if (plants[i].inPostWateringMode) {
            anyPlantInPostWatering = true;
            break;
        }
    }
    
    // Set appropriate interval
    if (anyPlantInPostWatering) {
        currentReadInterval_ms = POST_WATERING_READ_INTERVAL_MS;  // 5 seconds
    } else {
        currentReadInterval_ms = NORMAL_READ_INTERVAL_MS;  // 30 seconds
    }
}

/**
 * Check if plants should exit post-watering monitoring mode
 * Exit after POST_WATERING_DURATION_MS (1 minute) has elapsed
 */
void updatePlantPostWateringStates() {
    unsigned long currentTime = millis();
    
    for (int i = 0; i < NUM_PLANTS; i++) {
        if (plants[i].inPostWateringMode) {
            unsigned long timeSinceWateringStart = currentTime - plants[i].postWateringStart_ms;
            
            // Check if post-watering monitoring period is over
            if (timeSinceWateringStart >= POST_WATERING_DURATION_MS) {
                Serial.println("✓ Plant " + String(i) + " exiting post-watering mode (back to 30s intervals)");
                plants[i].inPostWateringMode = false;
            }
        }
    }
}

/**
 * Build JSON string with all sensor data for publishing
 * @return JSON formatted string
 */
String buildSensorJSON() {
    String json = "{";
    
    // Add timestamp
    json += "\"timestamp\":" + String(millis()) + ",";
    
    // Add connection status
    json += "\"connection\":\"" + connectionStatus + "\",";
    
    // Add plants array
    json += "\"plants\":[";
    
    for (int i = 0; i < NUM_PLANTS; i++) {
        json += "{";
        json += "\"id\":" + String(i) + ",";
        
        // Soil moisture
        json += "\"soilMoisture\":{";
        json += "\"value\":" + String(plants[i].soilMoisture) + ",";
        json += "\"percent\":" + String(moistureToPercent(i, plants[i].soilMoisture)) + ",";
        json += "\"status\":\"" + plants[i].soilMoistureStatus + "\",";
        json += "\"errorCount\":" + String(plants[i].soilMoistureErrorCount);
        json += "},";
        
        // Humidity
        json += "\"humidity\":{";
        json += "\"value\":" + String(plants[i].humidity, 1) + ",";
        json += "\"status\":\"" + plants[i].humidityStatus + "\",";
        json += "\"errorCount\":" + String(plants[i].humidityErrorCount);
        json += "},";
        
        // Temperature
        json += "\"temperature\":{";
        json += "\"value\":" + String(plants[i].temperature, 1) + ",";
        json += "\"status\":\"" + plants[i].temperatureStatus + "\",";
        json += "\"errorCount\":" + String(plants[i].temperatureErrorCount);
        json += "}";
        
        json += "}";
        
        // Add comma if not last plant
        if (i < NUM_PLANTS - 1) {
            json += ",";
        }
    }
    
    json += "]";  // Close plants array
    json += "}";  // Close main object
    
    return json;
}

/**
 * Publish sensor data to Particle Cloud
 */
void publishSensorData() {
    if (!Particle.connected()) {
        Serial.println("⚠ Not connected - skipping publish");
        return;
    }
    
    String jsonData = buildSensorJSON();
    bool success = Particle.publish("sensors/all", jsonData, PRIVATE);
    
    if (success) {
        Serial.println("✓ Published to cloud");
    } else {
        Serial.println("✗ Publish failed");
    }
}

/**
 * Update connection status based on Particle Cloud connectivity
 */
void updateConnectionStatus() {
    if (Particle.connected()) {
        connectionStatus = "ok";
    } else {
        connectionStatus = "lost";
    }
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

    // Initialize valve states
    for (int i = 0; i < NUM_VALVES; i++) {
        valves[i].isOpen = false;
        valves[i].openStartTime_ms = 0;
        valves[i].status = "ok";
    }
    
    Serial.println("Number of valves: " + String(NUM_VALVES));

    Particle.function("water", waterPlant);
    Serial.println("Cloud function 'water' registered");
    
    if (SIMULATION_MODE) {
        Serial.println("⚠ SIMULATION MODE - No physical relay connected");
    }
    
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
    updateConnectionStatus();
    
    // Update plant post-watering states (exit mode after 1 minute)
    updatePlantPostWateringStates();
    
    // Update read interval based on plant states
    updateReadInterval();
    
    // Update valve states (close valves at right time, update plant states)
    updateValveStates();
    
    // Check valve safety (force close if timeout exceeded)
    checkValveSafety();
    
    if (shouldReadSensors()) {
        readAllSensors();
        lastSensorRead_ms = millis();
        publishSensorData();
        printSensorData();
    }
    
    delay(100);
}