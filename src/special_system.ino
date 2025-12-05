#include "Particle.h"
#include "HTS221.h"

// ============================================================================
// CONSTANTS
// ============================================================================
const int NUM_PLANTS = 2;
const int NUM_VALVES = 2;
const bool SIMULATION_MODE = false;

const unsigned long NORMAL_READ_INTERVAL_MS = 30000;
const unsigned long POST_WATERING_READ_INTERVAL_MS = 5000;
const unsigned long POST_WATERING_DURATION_MS = 60000;
const unsigned long VALVE_OPEN_DURATION_MS = 2000;
const unsigned long MAX_VALVE_OPEN_TIME_MS = 3000;

const int SENSOR_ERROR_THRESHOLD = 5;

const int soilMoisturePins[NUM_PLANTS] = {A0, A1};
const int VALVE_PINS[] = {D10, D7};
const int PLANT_TO_VALVE_MAP[NUM_PLANTS] = {0, 1};

int soilMoistureDry[NUM_PLANTS] = {50, 50};
int soilMoistureWet[NUM_PLANTS] = {3000, 3000};

// ============================================================================
// STATE MACHINE ENUMS
// ============================================================================
enum PlantState_t {
    PLANT_IDLE,
    PLANT_WATERING_ACTIVE,
    PLANT_WATERING_REQUESTED,
    PLANT_POST_WATERING,
    PLANT_COOLDOWN,
    PLANT_ERROR
};

enum ValveState_t {
    VALVE_CLOSED,
    VALVE_OPEN,
    VALVE_ERROR,
    VALVE_SAFETY_SHUTDOWN
};

enum SystemState_t {
    SYS_INITIALIZING,
    SYS_NORMAL_OPERATION,
    SYS_DEGRADED_OPERATION,
    SYS_ERROR_STATE,
    SYS_SAFE_MODE
};

enum SensorStatus_t {
    SENSOR_OK,
    SENSOR_DEGRADED,
    SENSOR_ERROR
};

// ============================================================================
// STRING CONVERSION HELPERS
// ============================================================================
const char* sensorStatusToStr(SensorStatus_t status) {
    switch(status) {
        case SENSOR_OK: return "ok";
        case SENSOR_DEGRADED: return "degraded";
        case SENSOR_ERROR: return "error";
        default: return "unknown";
    }
}

const char* plantStateToStr(PlantState_t state) {
    switch(state) {
        case PLANT_IDLE: return "idle";
        case PLANT_WATERING_REQUESTED: return "watering_requested";
        case PLANT_WATERING_ACTIVE: return "watering_active";
        case PLANT_POST_WATERING: return "post_watering";
        case PLANT_COOLDOWN: return "cooldown";
        case PLANT_ERROR: return "error";
        default: return "unknown";
    }
}

const char* valveStateToStr(ValveState_t state) {
    switch(state) {
        case VALVE_CLOSED: return "closed";
        case VALVE_OPEN: return "open";
        case VALVE_ERROR: return "error";
        case VALVE_SAFETY_SHUTDOWN: return "safety_shutdown";
        default: return "unknown";
    }
}

const char* systemStateToStr(SystemState_t state) {
    switch(state) {
        case SYS_INITIALIZING: return "initializing";
        case SYS_NORMAL_OPERATION: return "normal";
        case SYS_DEGRADED_OPERATION: return "degraded";
        case SYS_ERROR_STATE: return "error";
        case SYS_SAFE_MODE: return "safe_mode";
        default: return "unknown";
    }
}

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct SensorReading {
    int rawValue;
    float value;
    uint8_t errorCount;
    SensorStatus_t status;
    unsigned long lastValidReading_ms;
};

struct PlantState {
    SensorReading soilMoisture;
    SensorReading humidity;
    SensorReading temperature;
    PlantState_t state;
    PlantState_t previousState;
    unsigned long stateEntryTime_ms;
    int plantId;
    int assignedValveId;
    unsigned long lastWateringEnd_ms;
    unsigned long postWateringStart_ms;
};

struct ValveState {
    ValveState_t state;
    ValveState_t previousState;
    unsigned long stateEntryTime_ms;
    int valveId;
    int pin;
    unsigned long openStartTime_ms;
    int requestingPlantId;
    unsigned long targetOpenDuration_ms;
};

struct SystemStateInfo {
    SystemState_t state;
    unsigned long stateEntryTime_ms;
    bool cloudConnected;
    uint8_t totalSensorErrors;
    uint8_t totalValveErrors;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
HTS221 hts221_sensors[NUM_PLANTS];
PlantState plants[NUM_PLANTS];
ValveState valves[NUM_VALVES];
SystemStateInfo systemState;

unsigned long lastSensorRead_ms = 0;
unsigned int currentReadInterval_ms = NORMAL_READ_INTERVAL_MS;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
inline int getSoilMoisturePin(int plantId) {
    return soilMoisturePins[plantId];
}

inline int getValvePin(int valveId) {
    return VALVE_PINS[valveId];
}

bool validateSensorReading(float value, float minValue, float maxValue) {
    return (value >= minValue && value <= maxValue);
}

int moistureToPercent(int plantId, int rawValue) {
    int percent = map(rawValue, soilMoistureDry[plantId], soilMoistureWet[plantId], 0, 100);
    percent = constrain(percent, 0, 100);
    return percent;
}

// ============================================================================
// SENSOR READING FUNCTIONS
// ============================================================================
int readSoilMoisture(int plantId) {
    if (plantId < 0 || plantId >= NUM_PLANTS) {
        Serial.println("ERROR: Invalid plant ID " + String(plantId));
        return -1;
    }
    return analogRead(getSoilMoisturePin(plantId));
}

float readHumidity(int plantId) {
    if (plantId < 0 || plantId >= NUM_PLANTS) {
        Serial.println("ERROR: Invalid plant ID " + String(plantId));
        return -999;
    }
    return hts221_sensors[0].readHumidity();
}

float readTemperature(int plantId) {
    if (plantId < 0 || plantId >= NUM_PLANTS) {
        Serial.println("ERROR: Invalid plant ID " + String(plantId));
        return -999;
    }
    return hts221_sensors[0].readTemperature();
}

void updateSensorStatus(SensorReading& sensor, bool isValid) {
    if (isValid) {
        sensor.errorCount = 0;
        sensor.status = SENSOR_OK;
        sensor.lastValidReading_ms = millis();
    } else {
        sensor.errorCount++;
        if (sensor.errorCount >= SENSOR_ERROR_THRESHOLD) {
            sensor.status = SENSOR_ERROR;
        } else {
            sensor.status = SENSOR_DEGRADED;
        }
    }
}

bool processSoilMoistureReading(int plantId) {
    int rawValue = readSoilMoisture(plantId);
    
    if (rawValue < 0) {
        updateSensorStatus(plants[plantId].soilMoisture, false);
        return false;
    }
    
    bool isValid = validateSensorReading(rawValue, 0, 4095);
    
    if (isValid) {
        plants[plantId].soilMoisture.rawValue = rawValue;
    }
    
    updateSensorStatus(plants[plantId].soilMoisture, isValid);
    return isValid;
}

bool processHumidityReading(int plantId) {
    float value = readHumidity(plantId);
    
    if (value <= -999) {
        updateSensorStatus(plants[plantId].humidity, false);
        return false;
    }
    
    bool isValid = validateSensorReading(value, 0, 100);
    
    if (isValid) {
        plants[plantId].humidity.value = value;
    }
    
    updateSensorStatus(plants[plantId].humidity, isValid);
    return isValid;
}

bool processTemperatureReading(int plantId) {
    float value = readTemperature(plantId);
    
    if (value <= -999) {
        updateSensorStatus(plants[plantId].temperature, false);
        return false;
    }
    
    bool isValid = validateSensorReading(value, -40, 120);
    
    if (isValid) {
        plants[plantId].temperature.value = value;
    }
    
    updateSensorStatus(plants[plantId].temperature, isValid);
    return isValid;
}

void readAllSensors() {
    for (int i = 0; i < NUM_PLANTS; i++) {
        processSoilMoistureReading(i);
        processHumidityReading(i);
        processTemperatureReading(i);
        
        if (plants[i].soilMoisture.status == SENSOR_ERROR &&
            plants[i].humidity.status == SENSOR_ERROR &&
            plants[i].temperature.status == SENSOR_ERROR) {
            if (plants[i].state != PLANT_ERROR) {
                Serial.println("Plant " + String(i) + " entering ERROR state - all sensors failed");
                plants[i].previousState = plants[i].state;
                plants[i].state = PLANT_ERROR;
                plants[i].stateEntryTime_ms = millis();
            }
        }
        else if (plants[i].state == PLANT_ERROR) {
            if (plants[i].previousState == PLANT_IDLE || 
                plants[i].previousState == PLANT_COOLDOWN) {
                    // If state is plant watering active - we mess with watering schedule... Thats why we check for permissable states here.
                Serial.println("Plant " + String(i) + " recovering from ERROR state");
                plants[i].state = PLANT_IDLE;
                plants[i].stateEntryTime_ms = millis();
            }
        }
    }
}

// ============================================================================
// VALVE CONTROL FUNCTIONS
// ============================================================================
void openValve(int valveId) {
    if (valveId < 0 || valveId >= NUM_VALVES) {
        Serial.println("ERROR: Invalid valve ID " + String(valveId));
        return;
    }

    if (valves[valveId].state == VALVE_OPEN) {
        Serial.println("WARNING: Valve " + String(valveId) + " is already open");
        return;
    }

    digitalWrite(VALVE_PINS[valveId], HIGH);
    Serial.println("Opening valve " + String(valveId) + " (D" + String(VALVE_PINS[valveId]) + " -> HIGH)");

    valves[valveId].previousState = valves[valveId].state;
    valves[valveId].state = VALVE_OPEN;
    valves[valveId].stateEntryTime_ms = millis();
    valves[valveId].openStartTime_ms = millis();
}

void closeValve(int valveId) {
    if (valveId < 0 || valveId >= NUM_VALVES) {
        Serial.println("ERROR: Invalid valve ID " + String(valveId));
        return;
    }

    if (valves[valveId].state == VALVE_CLOSED) {
        Serial.println("INFO: Valve " + String(valveId) + " is already closed");
        return;
    }

    digitalWrite(VALVE_PINS[valveId], LOW);
    Serial.println("Closing valve " + String(valveId) + " (D" + String(VALVE_PINS[valveId]) + " -> LOW)");

    valves[valveId].previousState = valves[valveId].state;
    valves[valveId].state = VALVE_CLOSED;
    valves[valveId].stateEntryTime_ms = millis();
}

void checkValveSafety() {
    unsigned long currentTime = millis();

    for (int i = 0; i < NUM_VALVES; i++) {
        if (valves[i].state == VALVE_OPEN) {
            unsigned long openDuration = currentTime - valves[i].openStartTime_ms;

            if (openDuration > MAX_VALVE_OPEN_TIME_MS) {
                Serial.println("SAFETY: Valve " + String(i) + " exceeded max open time! Force closing.");
                closeValve(i);
                valves[i].previousState = valves[i].state;
                valves[i].state = VALVE_SAFETY_SHUTDOWN;
                valves[i].stateEntryTime_ms = millis();
            }
        }
    }
}

void updateValveStates() {
    unsigned long currentTime = millis();

    for (int i = 0; i < NUM_VALVES; i++) {
        if (valves[i].state == VALVE_OPEN) {
            unsigned long openDuration = currentTime - valves[i].openStartTime_ms;

            if (openDuration >= VALVE_OPEN_DURATION_MS) {
                Serial.print("Valve ");
                Serial.print(i);
                Serial.println(" reached target duration, closing");
                
                closeValve(i);

                int plantId = valves[i].requestingPlantId;

                if (plantId >= 0 && plantId < NUM_PLANTS) {
                    plants[plantId].previousState = plants[plantId].state;
                    plants[plantId].state = PLANT_POST_WATERING;
                    plants[plantId].stateEntryTime_ms = currentTime;
                    plants[plantId].lastWateringEnd_ms = currentTime;
                    plants[plantId].postWateringStart_ms = currentTime;

                    Serial.print("  → Plant ");
                    Serial.print(plantId);
                    Serial.println(" watering complete");
                    Serial.println("  → Entering post-watering monitoring (5s intervals for 1 min)");

                    String eventData = "{\"plantId\":" + String(plantId) +
                                    ",\"valveId\":" + String(i) +
                                    ",\"timestamp\":" + String(currentTime) +
                                    ",\"duration_ms\":" + String(openDuration) + "}";
                    Particle.publish("event/watering_end", eventData, PRIVATE);
                    
                    valves[i].requestingPlantId = -1;
                }
            }
        }
    }
}

// ============================================================================
// WATERING CONTROL
// ============================================================================
int waterPlant(String command) {
    int plantId = command.toInt();

    if (plantId < 0 || plantId >= NUM_PLANTS) {
        Serial.println("Water command rejected: Invalid plant ID " + String(plantId));
        return -1;
    }

    Serial.println("\nWater command received for Plant " + String(plantId));

    if (plants[plantId].state == PLANT_ERROR) {
        Serial.println("Water command rejected: Plant " + String(plantId) + " is in ERROR state");
        Serial.println("   Check sensor status before attempting to water");
        return -6;
    }

    if (plants[plantId].state == PLANT_WATERING_ACTIVE || 
        plants[plantId].state == PLANT_WATERING_REQUESTED) {
        Serial.println("Water command rejected: Plant " + String(plantId) + " is already being watered");
        return -2;
    }

    unsigned long currentTime = millis();
    if (plants[plantId].state == PLANT_POST_WATERING || 
        plants[plantId].state == PLANT_COOLDOWN) {
        unsigned long timeSinceWatering = currentTime - plants[plantId].lastWateringEnd_ms;
        if (timeSinceWatering < POST_WATERING_DURATION_MS) {
            unsigned long cooldownRemaining = POST_WATERING_DURATION_MS - timeSinceWatering;
            Serial.println("Water command rejected: Plant " + String(plantId) + " in cooldown");
            Serial.println("   Cooldown remaining: " + String(cooldownRemaining / 1000) + " seconds");
            return -3;
        }
    }

    int valveId = PLANT_TO_VALVE_MAP[plantId];

    if (valves[valveId].state == VALVE_ERROR || 
        valves[valveId].state == VALVE_SAFETY_SHUTDOWN) {
        Serial.println("Water command rejected: Valve " + String(valveId) + " has error status");
        return -4;
    }

    for (int i = 0; i < NUM_VALVES; i++) {
        if (valves[i].state == VALVE_OPEN) {
            Serial.println("Water command rejected: Another valve (valve " + String(i) + ") is currently open");
            Serial.println("   Only one valve can operate at a time due to power supply limitation");
            return -5;
        }
    }

    Serial.println("Starting watering for Plant " + String(plantId) + " using Valve " + String(valveId));

    plants[plantId].previousState = plants[plantId].state;
    plants[plantId].state = PLANT_WATERING_ACTIVE;
    plants[plantId].stateEntryTime_ms = currentTime;

    openValve(valveId);
    valves[valveId].requestingPlantId = plantId;

    String eventData = "{\"plantId\":" + String(plantId) +
                       ",\"valveId\":" + String(valveId) +
                       ",\"timestamp\":" + String(currentTime) + "}";
    Particle.publish("event/watering_start", eventData, PRIVATE);

    return 0;
}

// ============================================================================
// STATE UPDATE FUNCTIONS
// ============================================================================
bool shouldReadSensors() {
    unsigned long currentTime = millis();
    unsigned long timeSinceLastRead = currentTime - lastSensorRead_ms;
    return (timeSinceLastRead >= currentReadInterval_ms);
}

void updateReadInterval() {
    bool anyPlantInPostWatering = false;

    for (int i = 0; i < NUM_PLANTS; i++) {
        if (plants[i].state == PLANT_POST_WATERING) {
            anyPlantInPostWatering = true;
            break;
        }
    }

    unsigned int newInterval = anyPlantInPostWatering ? 
                               POST_WATERING_READ_INTERVAL_MS : 
                               NORMAL_READ_INTERVAL_MS;
    
    if (newInterval != currentReadInterval_ms) {
        Serial.print("Read interval changed: ");
        Serial.print(currentReadInterval_ms / 1000);
        Serial.print("s -> ");
        Serial.print(newInterval / 1000);
        Serial.println("s");
        
        currentReadInterval_ms = newInterval;
    }
}

void updatePlantPostWateringStates() {
    unsigned long currentTime = millis();
    
    for (int i = 0; i < NUM_PLANTS; i++) {
        if (plants[i].state == PLANT_POST_WATERING) {
            unsigned long timeSinceStart = currentTime - plants[i].postWateringStart_ms;
            
            if (timeSinceStart >= POST_WATERING_DURATION_MS) {
                Serial.print("Plant ");
                Serial.print(i);
                Serial.println(" post-watering period ended, entering cooldown");
                
                plants[i].previousState = plants[i].state;
                plants[i].state = PLANT_COOLDOWN;
                plants[i].stateEntryTime_ms = currentTime;
            }
        }
        
        if (plants[i].state == PLANT_COOLDOWN) {
            unsigned long timeSinceEnd = currentTime - plants[i].lastWateringEnd_ms;
            
            if (timeSinceEnd >= POST_WATERING_DURATION_MS) {
                Serial.print("Plant ");
                Serial.print(i);
                Serial.println(" cooldown ended, returning to IDLE");
                
                plants[i].previousState = plants[i].state;
                plants[i].state = PLANT_IDLE;
                plants[i].stateEntryTime_ms = currentTime;
            }
        }
    }
}

void updateConnectionStatus() {
    systemState.cloudConnected = Particle.connected();
}

void evaluateSystemState() {
    int sensorErrors = 0;
    int sensorDegraded = 0;
    int valveErrors = 0;
    
    for (int i = 0; i < NUM_PLANTS; i++) {
        if (plants[i].state == PLANT_ERROR) {
            sensorErrors++;
        }
        
        if (plants[i].soilMoisture.status == SENSOR_ERROR ||
            plants[i].humidity.status == SENSOR_ERROR ||
            plants[i].temperature.status == SENSOR_ERROR) {
            sensorErrors++;
        }
        
        if (plants[i].soilMoisture.status == SENSOR_DEGRADED ||
            plants[i].humidity.status == SENSOR_DEGRADED ||
            plants[i].temperature.status == SENSOR_DEGRADED) {
            sensorDegraded++;
        }
    }
    
    for (int i = 0; i < NUM_VALVES; i++) {
        if (valves[i].state == VALVE_ERROR || 
            valves[i].state == VALVE_SAFETY_SHUTDOWN) {
            valveErrors++;
        }
    }
    
    systemState.totalSensorErrors = sensorErrors;
    systemState.totalValveErrors = valveErrors;
    
    SystemState_t newState = systemState.state;
    
    if (sensorErrors >= NUM_PLANTS || valveErrors >= NUM_VALVES) {
        newState = SYS_ERROR_STATE;
    }
    else if (sensorErrors > 0 || valveErrors > 0 || sensorDegraded > 0) {
        newState = SYS_DEGRADED_OPERATION;
    }
    else {
        newState = SYS_NORMAL_OPERATION;
    }
    
    if (newState != systemState.state) {
        SystemState_t oldState = systemState.state;
        systemState.state = newState;
        systemState.stateEntryTime_ms = millis();
        
        Serial.print("SYSTEM STATE CHANGE: ");
        Serial.print(systemStateToStr(oldState));
        Serial.print(" -> ");
        Serial.println(systemStateToStr(newState));
    }
}

// ============================================================================
// CLOUD COMMUNICATION
// ============================================================================
String buildSensorJSON() {
    static char buffer[1024];
    int pos = 0;
    
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "{\"timestamp\":%lu,", millis());
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "\"connection\":\"%s\",", 
                    systemState.cloudConnected ? "ok" : "lost");
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "\"plants\":[");
    
    for (int i = 0; i < NUM_PLANTS; i++) {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "{\"id\":%d,", i);
        
        int moisturePercent = moistureToPercent(i, plants[i].soilMoisture.rawValue);
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, 
                        "\"soilMoisture\":{\"value\":%d,\"percent\":%d,\"status\":\"%s\",\"errorCount\":%d},",
                        plants[i].soilMoisture.rawValue,
                        moisturePercent,
                        sensorStatusToStr(plants[i].soilMoisture.status),
                        plants[i].soilMoisture.errorCount);
        
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "\"humidity\":{\"value\":%.1f,\"status\":\"%s\",\"errorCount\":%d},",
                        plants[i].humidity.value,
                        sensorStatusToStr(plants[i].humidity.status),
                        plants[i].humidity.errorCount);
        
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "\"temperature\":{\"value\":%.1f,\"status\":\"%s\",\"errorCount\":%d},",
                        plants[i].temperature.value,
                        sensorStatusToStr(plants[i].temperature.status),
                        plants[i].temperature.errorCount);
        
        unsigned long currentTime = millis();
        bool inCooldown = (plants[i].state == PLANT_COOLDOWN || plants[i].state == PLANT_POST_WATERING);
        bool isWatering = (plants[i].state == PLANT_WATERING_ACTIVE);
        
        unsigned long cooldownRemaining = 0;
        if (inCooldown && plants[i].lastWateringEnd_ms > 0) {
            unsigned long timeSince = currentTime - plants[i].lastWateringEnd_ms;
            if (timeSince < POST_WATERING_DURATION_MS) {
                cooldownRemaining = POST_WATERING_DURATION_MS - timeSince;
            }
        }
        
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "\"state\":{\"current\":\"%s\",\"isWatering\":%s,\"inCooldown\":%s,\"cooldownRemaining_ms\":%lu}",
                        plantStateToStr(plants[i].state),
                        isWatering ? "true" : "false",
                        inCooldown ? "true" : "false",
                        cooldownRemaining);
        
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "}");
        
        if (i < NUM_PLANTS - 1) {
            pos += snprintf(buffer + pos, sizeof(buffer) - pos, ",");
        }
    }
    
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "]}");
    
    return String(buffer);
}

void checkJSONBufferSize() {
    String json = buildSensorJSON();
    int jsonLength = json.length();
    
    if (jsonLength > 800) {
        Serial.print("WARNING: JSON buffer usage high: ");
        Serial.print(jsonLength);
        Serial.println(" bytes (1024 allocated)");
    }
}

void publishSensorData() {
    if (!Particle.connected()) {
        Serial.println("Not connected - skipping publish");
        return;
    }

    String jsonData = buildSensorJSON();
    bool success = Particle.publish("sensors/all", jsonData, PRIVATE);

    if (success) {
        Serial.println("Published to cloud");
    } else {
        Serial.println("Publish failed");
    }
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================
void printSensorData() {
    Serial.println("\n========================================");
    Serial.println("PLANT WATERING SYSTEM STATUS");
    Serial.println("========================================");
    Serial.print("System State: ");
    Serial.println(systemStateToStr(systemState.state));
    Serial.print("Cloud: ");
    Serial.println(systemState.cloudConnected ? "Connected" : "Disconnected");
    Serial.println();

    for (int i = 0; i < NUM_PLANTS; i++) {
        Serial.print("PLANT ");
        Serial.println(i);
        Serial.println("----------------------------------------");
        
        Serial.print("State:          ");
        Serial.println(plantStateToStr(plants[i].state));

        int moisturePercent = moistureToPercent(i, plants[i].soilMoisture.rawValue);
        Serial.print("Soil Moisture:  ");
        Serial.print(moisturePercent);
        Serial.print("% (raw: ");
        Serial.print(plants[i].soilMoisture.rawValue);
        Serial.print(") [");
        Serial.print(sensorStatusToStr(plants[i].soilMoisture.status));
        Serial.println("]");
        
        Serial.print("Air Humidity:   ");
        Serial.print(plants[i].humidity.value, 1);
        Serial.print("% [");
        Serial.print(sensorStatusToStr(plants[i].humidity.status));
        Serial.println("]");
        
        Serial.print("Temperature:    ");
        Serial.print(plants[i].temperature.value, 1);
        Serial.print(" C [");
        Serial.print(sensorStatusToStr(plants[i].temperature.status));
        Serial.println("]");

        if (plants[i].soilMoisture.errorCount > 0 ||
            plants[i].humidity.errorCount > 0 ||
            plants[i].temperature.errorCount > 0) {
            Serial.print("WARNING: Errors detected - M:");
            Serial.print(plants[i].soilMoisture.errorCount);
            Serial.print(" H:");
            Serial.print(plants[i].humidity.errorCount);
            Serial.print(" T:");
            Serial.println(plants[i].temperature.errorCount);
        }

        Serial.println();
    }

    Serial.println("========================================\n");
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);

    Serial.println("=================================");
    Serial.println("Plant Watering System Starting...");
    Serial.println("=================================");

    for (int i = 0; i < NUM_PLANTS; i++) {
        plants[i].soilMoisture.rawValue = 0;
        plants[i].soilMoisture.value = 0.0;
        plants[i].soilMoisture.errorCount = 0;
        plants[i].soilMoisture.status = SENSOR_OK;
        plants[i].soilMoisture.lastValidReading_ms = 0;

        plants[i].humidity.rawValue = 0;
        plants[i].humidity.value = 0.0;
        plants[i].humidity.errorCount = 0;
        plants[i].humidity.status = SENSOR_OK;
        plants[i].humidity.lastValidReading_ms = 0;

        plants[i].temperature.rawValue = 0;
        plants[i].temperature.value = 0.0;
        plants[i].temperature.errorCount = 0;
        plants[i].temperature.status = SENSOR_OK;
        plants[i].temperature.lastValidReading_ms = 0;

        plants[i].state = PLANT_IDLE;
        plants[i].previousState = PLANT_IDLE;
        plants[i].stateEntryTime_ms = 0;

        plants[i].plantId = i;
        plants[i].assignedValveId = PLANT_TO_VALVE_MAP[i];

        plants[i].lastWateringEnd_ms = 0;
        plants[i].postWateringStart_ms = 0;
    }

    Serial.println("System initialized successfully!");
    Serial.println("Number of plants: " + String(NUM_PLANTS));
    Serial.println("=================================\n");

    for (int i = 0; i < NUM_VALVES; i++) {
        pinMode(VALVE_PINS[i], OUTPUT);
        digitalWrite(VALVE_PINS[i], LOW);
    }
    Serial.println("Relay pins initialized");

    for (int i = 0; i < NUM_VALVES; i++) {
        valves[i].state = VALVE_CLOSED;
        valves[i].previousState = VALVE_CLOSED;
        valves[i].stateEntryTime_ms = 0;
        valves[i].valveId = i;
        valves[i].pin = VALVE_PINS[i];
        valves[i].openStartTime_ms = 0;
        valves[i].requestingPlantId = -1;
        valves[i].targetOpenDuration_ms = VALVE_OPEN_DURATION_MS;
    }

    systemState.state = SYS_INITIALIZING;
    systemState.stateEntryTime_ms = millis();
    systemState.cloudConnected = false;
    systemState.totalSensorErrors = 0;
    systemState.totalValveErrors = 0;

    Serial.println("Number of valves: " + String(NUM_VALVES));

    Particle.function("water", waterPlant);
    Serial.println("Cloud function 'water' registered");

    Serial.println("\nWaiting for cloud connection...");
    waitFor(Particle.connected, 30000);

    if (Particle.connected()) {
        Serial.println("Cloud connected!");
        Serial.println("Device ready to receive function calls");
        systemState.cloudConnected = true;
    } else {
        Serial.println("Cloud NOT connected after 30 seconds");
        Serial.println("Function calls will not work");
        systemState.cloudConnected = false;
    }
    
    systemState.state = SYS_NORMAL_OPERATION;
    systemState.stateEntryTime_ms = millis();
    Serial.println("\nSystem transitioned to NORMAL_OPERATION state");
    
    Serial.println("\n=================================");
    Serial.println("SYSTEM READY");
    Serial.println("=================================\n");

    if (SIMULATION_MODE) {
        Serial.println("SIMULATION MODE - No physical relay connected");
    }

    Wire.begin();
    Serial.println("I2C initialized");
    delay(100);

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
    
    Serial.println("Running JSON buffer size check...");
    checkJSONBufferSize();
    
    Serial.println("=================================\n");
}

void loop() {
    updateConnectionStatus();
    updatePlantPostWateringStates();
    updateReadInterval();
    updateValveStates();
    checkValveSafety();
    
    if (shouldReadSensors()) {
        readAllSensors();
        evaluateSystemState();
        lastSensorRead_ms = millis();
        publishSensorData();
        printSensorData();
    }
    
    delay(100);
}