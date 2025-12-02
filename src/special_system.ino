#include "Particle.h"
#include "HTS221.h"

// ============================================================================
// STATE MACHINE DEFINITIONS
// ============================================================================
enum PlantState_t {
    PLANT_IDLE,
    PLANT_WATERING_REQUESTED,
    PLANT_WATERING_ACTIVE,
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

const int NUM_PLANTS = 2;
const int NUM_VALVES = 2;

const bool SIMULATION_MODE = false;

const unsigned long NORMAL_READ_INTERVAL_MS = 30000;
const unsigned long POST_WATERING_READ_INTERVAL_MS = 5000;
const unsigned long POST_WATERING_DURATION_MS = 60000;
const unsigned long VALVE_OPEN_DURATION_MS = 2000;
const unsigned long MAX_VALVE_OPEN_TIME_MS = 3000;

const int SENSOR_ERROR_THRESHOLD = 5;

// ============================================================================
// STRING CONVERSION HELPERS (Stack-based, no heap allocation)
// ============================================================================

// Convert sensor status enum to string
const char* sensorStatusToStr(SensorStatus_t status) {
    switch(status) {
        case SENSOR_OK: return "ok";
        case SENSOR_DEGRADED: return "degraded";
        case SENSOR_ERROR: return "error";
        default: return "unknown";
    }
}

// Convert plant state enum to string
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

// Convert valve state enum to string
const char* valveStateToStr(ValveState_t state) {
    switch(state) {
        case VALVE_CLOSED: return "closed";
        case VALVE_OPEN: return "open";
        case VALVE_ERROR: return "error";
        case VALVE_SAFETY_SHUTDOWN: return "safety_shutdown";
        default: return "unknown";
    }
}

// Convert system state enum to string
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

const int soilMoisturePins[NUM_PLANTS] = {A0, A1};
const int VALVE_PINS[] = {D10, D7};

// Fast inline pin lookup (compiler will optimize to direct access)
inline int getSoilMoisturePin(int plantId) {
    return soilMoisturePins[plantId];
}

inline int getValvePin(int valveId) {
    return VALVE_PINS[valveId];
}

const int PLANT_TO_VALVE_MAP[NUM_PLANTS] = {0, 1};

int soilMoistureDry[NUM_PLANTS] = {50, 50};
int soilMoistureWet[NUM_PLANTS] = {3000, 3000};

HTS221 hts221_sensors[NUM_PLANTS];

struct SensorReading {
    int rawValue;           // For soil moisture (int)
    float value;            // For humidity/temp (float)
    uint8_t errorCount;
    SensorStatus_t status;
    unsigned long lastValidReading_ms;
};

struct PlantState
{
    // Sensor readings (combines value + errorCount + status)
    SensorReading soilMoisture;
    SensorReading humidity;
    SensorReading temperature;

    // State machine (replaces isWatering, inPostWateringMode, wateringCommandPending)
    PlantState_t state;
    PlantState_t previousState;
    unsigned long stateEntryTime_ms;

    // Configuration
    int plantId;
    int assignedValveId;

    // Watering tracking
    unsigned long lastWateringEnd_ms;
    unsigned long postWateringStart_ms;
};

struct ValveState
{
    // State machine (replaces bool isOpen)
    ValveState_t state;
    ValveState_t previousState;
    unsigned long stateEntryTime_ms;
    
    // Hardware control
    int valveId;
    int pin;
    unsigned long openStartTime_ms;
    
    // Safety & control
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

PlantState plants[NUM_PLANTS];
ValveState valves[NUM_VALVES];
SystemStateInfo systemState;

unsigned long lastSensorRead_ms = 0;
unsigned int currentReadInterval_ms = NORMAL_READ_INTERVAL_MS;

int readSoilMoisture(int plantId)
{
    if (plantId < 0 || plantId >= NUM_PLANTS)
    {
        Serial.println("ERROR: Invalid plant ID " + String(plantId));
        return -1;
    }

    return analogRead(getSoilMoisturePin(plantId));
}

float readHumidity(int plantId)
{
    if (plantId < 0 || plantId >= NUM_PLANTS)
    {
        Serial.println("ERROR: Invalid plant ID " + String(plantId));
        return -999;
    }

    return hts221_sensors[0].readHumidity();
}

float readTemperature(int plantId)
{
    if (plantId < 0 || plantId >= NUM_PLANTS)
    {
        Serial.println("ERROR: Invalid plant ID " + String(plantId));
        return -999;
    }

    return hts221_sensors[0].readTemperature();
}

// ============================================================================
// SENSOR ERROR HANDLING HELPERS
// ============================================================================

// Update sensor reading status based on validity
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

// Process a soil moisture reading (returns true if updated)
bool processSoilMoistureReading(int plantId) {
    int rawValue = readSoilMoisture(plantId);
    
    if (rawValue < 0) {
        // Sensor read failed
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

// Process a humidity reading (returns true if updated)
bool processHumidityReading(int plantId) {
    float value = readHumidity(plantId);
    
    if (value <= -999) {
        // Sensor read failed
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

// Process a temperature reading (returns true if updated)
bool processTemperatureReading(int plantId) {
    float value = readTemperature(plantId);
    
    if (value <= -999) {
        // Sensor read failed
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

void readAllSensors()
{
    for (int i = 0; i < NUM_PLANTS; i++)
    {
        // Process all sensor readings
        processSoilMoistureReading(i);
        processHumidityReading(i);
        processTemperatureReading(i);
        
        // Check if plant should enter ERROR state
        if (plants[i].soilMoisture.status == SENSOR_ERROR &&
            plants[i].humidity.status == SENSOR_ERROR &&
            plants[i].temperature.status == SENSOR_ERROR)
        {
            // All sensors in error - transition plant to error state
            if (plants[i].state != PLANT_ERROR) {
                Serial.println("Plant " + String(i) + " entering ERROR state - all sensors failed");
                plants[i].previousState = plants[i].state;
                plants[i].state = PLANT_ERROR;
                plants[i].stateEntryTime_ms = millis();
            }
        }
        else if (plants[i].state == PLANT_ERROR)
        {
            // At least one sensor recovered - exit error state if appropriate
            if (plants[i].previousState == PLANT_IDLE || 
                plants[i].previousState == PLANT_COOLDOWN) {
                Serial.println("Plant " + String(i) + " recovering from ERROR state");
                plants[i].state = PLANT_IDLE;
                plants[i].stateEntryTime_ms = millis();
            }
        }
    }
}

// ============================================================================
// SYSTEM STATE EVALUATION
// ============================================================================

// Evaluate and update overall system state based on plant/valve conditions
void evaluateSystemState() {
    int sensorErrors = 0;
    int sensorDegraded = 0;
    int valveErrors = 0;
    
    // Count plant sensor issues
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
    
    // Count valve issues
    for (int i = 0; i < NUM_VALVES; i++) {
        if (valves[i].state == VALVE_ERROR || 
            valves[i].state == VALVE_SAFETY_SHUTDOWN) {
            valveErrors++;
        }
    }
    
    // Update system state counters
    systemState.totalSensorErrors = sensorErrors;
    systemState.totalValveErrors = valveErrors;
    
    // Determine new system state
    SystemState_t newState = systemState.state;
    
    if (sensorErrors >= NUM_PLANTS || valveErrors >= NUM_VALVES) {
        // Critical: all plants or all valves in error
        newState = SYS_ERROR_STATE;
    }
    else if (sensorErrors > 0 || valveErrors > 0 || sensorDegraded > 0) {
        // Some issues but still operational
        newState = SYS_DEGRADED_OPERATION;
    }
    else {
        // Everything nominal
        newState = SYS_NORMAL_OPERATION;
    }
    
    // Transition if state changed
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

void printSensorData()
{
    Serial.println("\n========================================");
    Serial.println("PLANT WATERING SYSTEM STATUS");
    Serial.println("========================================");
    Serial.print("System State: ");
    Serial.println(systemStateToStr(systemState.state));
    Serial.print("Cloud: ");
    Serial.println(systemState.cloudConnected ? "Connected" : "Disconnected");
    Serial.println();

    for (int i = 0; i < NUM_PLANTS; i++)
    {
        Serial.print("PLANT ");
        Serial.println(i);
        Serial.println("----------------------------------------");
        
        // Display current state
        Serial.print("State:          ");
        Serial.println(plantStateToStr(plants[i].state));

        // Display sensor readings
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

        // Display error counts if any errors exist
        if (plants[i].soilMoisture.errorCount > 0 ||
            plants[i].humidity.errorCount > 0 ||
            plants[i].temperature.errorCount > 0)
        {
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

void checkValveSafety()
{
    unsigned long currentTime = millis();

    for (int i = 0; i < NUM_VALVES; i++)
    {
        if (valves[i].state == VALVE_OPEN)
        {
            unsigned long openDuration = currentTime - valves[i].openStartTime_ms;

            if (openDuration > MAX_VALVE_OPEN_TIME_MS)
            {
                Serial.println("SAFETY: Valve " + String(i) + " exceeded max open time! Force closing.");
                closeValve(i);
                valves[i].previousState = valves[i].state;
                valves[i].state = VALVE_SAFETY_SHUTDOWN;
                valves[i].stateEntryTime_ms = millis();
            }
        }
    }
}

void updateValveStates()
{
    unsigned long currentTime = millis();

    for (int i = 0; i < NUM_VALVES; i++)
    {
        if (valves[i].state == VALVE_OPEN)
        {
            unsigned long openDuration = currentTime - valves[i].openStartTime_ms;

            if (openDuration >= VALVE_OPEN_DURATION_MS)
            {
                Serial.println("Valve " + String(i) + " reached target duration, closing");
                closeValve(i);

                int plantId = valves[i].requestingPlantId;

                if (plantId >= 0 && plantId < NUM_PLANTS) {
                    plants[plantId].previousState = plants[plantId].state;
                    plants[plantId].state = PLANT_POST_WATERING;
                    plants[plantId].stateEntryTime_ms = currentTime;
                    plants[plantId].lastWateringEnd_ms = currentTime;
                    plants[plantId].postWateringStart_ms = currentTime;

                    Serial.println("  → Plant " + String(plantId) + " watering complete");
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

int waterPlant(String command)
{
    int plantId = command.toInt();

    if (plantId < 0 || plantId >= NUM_PLANTS)
    {
        Serial.println("Water command rejected: Invalid plant ID " + String(plantId));
        return -1;
    }

    Serial.println("\nWater command received for Plant " + String(plantId));

    // Check if plant is in error state
    if (plants[plantId].state == PLANT_ERROR)
    {
        Serial.println("Water command rejected: Plant " + String(plantId) + " is in ERROR state");
        Serial.println("   Check sensor status before attempting to water");
        return -6;  // New error code for plant in error state
    }

    // Check if plant is already watering
    if (plants[plantId].state == PLANT_WATERING_ACTIVE || 
        plants[plantId].state == PLANT_WATERING_REQUESTED)
    {
        Serial.println("Water command rejected: Plant " + String(plantId) + " is already being watered");
        return -2;
    }

    // Check cooldown period (if in POST_WATERING or COOLDOWN state)
    unsigned long currentTime = millis();
    if (plants[plantId].state == PLANT_POST_WATERING || 
        plants[plantId].state == PLANT_COOLDOWN)
    {
        unsigned long timeSinceWatering = currentTime - plants[plantId].lastWateringEnd_ms;
        if (timeSinceWatering < POST_WATERING_DURATION_MS)
        {
            unsigned long cooldownRemaining = POST_WATERING_DURATION_MS - timeSinceWatering;
            Serial.println("Water command rejected: Plant " + String(plantId) + " in cooldown");
            Serial.println("   Cooldown remaining: " + String(cooldownRemaining / 1000) + " seconds");
            return -3;
        }
    }

    int valveId = PLANT_TO_VALVE_MAP[plantId];

    // Check valve status
    if (valves[valveId].state == VALVE_ERROR || 
        valves[valveId].state == VALVE_SAFETY_SHUTDOWN)
    {
        Serial.println("Water command rejected: Valve " + String(valveId) + " has error status");
        return -4;
    }

    // Check if any valve is currently open
    for (int i = 0; i < NUM_VALVES; i++)
    {
        if (valves[i].state == VALVE_OPEN)
        {
            Serial.println("Water command rejected: Another valve (valve " + String(i) + ") is currently open");
            Serial.println("   Only one valve can operate at a time due to power supply limitation");
            return -5;
        }
    }

    Serial.println("Starting watering for Plant " + String(plantId) + " using Valve " + String(valveId));

    // Transition plant to WATERING_ACTIVE state
    plants[plantId].previousState = plants[plantId].state;
    plants[plantId].state = PLANT_WATERING_ACTIVE;
    plants[plantId].stateEntryTime_ms = currentTime;

    // Open valve
    openValve(valveId);
    valves[valveId].requestingPlantId = plantId;

    String eventData = "{\"plantId\":" + String(plantId) +
                       ",\"valveId\":" + String(valveId) +
                       ",\"timestamp\":" + String(currentTime) + "}";
    Particle.publish("event/watering_start", eventData, PRIVATE);

    return 0;
}

bool shouldReadSensors()
{
    unsigned long currentTime = millis();
    unsigned long timeSinceLastRead = currentTime - lastSensorRead_ms;

    if (timeSinceLastRead >= currentReadInterval_ms)
    {
        return true;
    }

    return false;
}

void updateReadInterval()
{
    bool anyPlantInPostWatering = false;

    for (int i = 0; i < NUM_PLANTS; i++)
    {
        if (plants[i].state == PLANT_POST_WATERING)
        {
            anyPlantInPostWatering = true;
            break;
        }
    }

    if (anyPlantInPostWatering)
    {
        currentReadInterval_ms = POST_WATERING_READ_INTERVAL_MS;
    }
    else
    {
        currentReadInterval_ms = NORMAL_READ_INTERVAL_MS;
    }
}

void updatePlantPostWateringStates()
{
    unsigned long currentTime = millis();

    for (int i = 0; i < NUM_PLANTS; i++)
    {
        if (plants[i].state == PLANT_POST_WATERING)
        {
            unsigned long timeSinceWateringStart = currentTime - plants[i].postWateringStart_ms;

            if (timeSinceWateringStart >= POST_WATERING_DURATION_MS)
            {
                Serial.println("Plant " + String(i) + " exiting post-watering mode (back to 30s intervals)");
                
                // Transition to COOLDOWN state
                plants[i].previousState = plants[i].state;
                plants[i].state = PLANT_COOLDOWN;
                plants[i].stateEntryTime_ms = currentTime;
            }
        }
        
        // Check if cooldown period is complete
        if (plants[i].state == PLANT_COOLDOWN)
        {
            unsigned long timeSinceWateringEnd = currentTime - plants[i].lastWateringEnd_ms;
            
            if (timeSinceWateringEnd >= POST_WATERING_DURATION_MS)
            {
                Serial.println("Plant " + String(i) + " cooldown complete, returning to IDLE");
                
                // Transition back to IDLE state
                plants[i].previousState = plants[i].state;
                plants[i].state = PLANT_IDLE;
                plants[i].stateEntryTime_ms = currentTime;
            }
        }
    }
}

String buildSensorJSON()
{
    // Use a fixed buffer to avoid heap fragmentation
    // Estimated size: ~400 bytes for 2 plants with full data
    static char buffer[512];
    int pos = 0;
    
    // Start JSON
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "{\"timestamp\":%lu,", millis());
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "\"connection\":\"%s\",", 
                    systemState.cloudConnected ? "ok" : "lost");
    pos += snprintf(buffer + pos, sizeof(buffer) - pos, "\"plants\":[");
    
    for (int i = 0; i < NUM_PLANTS; i++)
    {
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, "{\"id\":%d,", i);
        
        // Soil moisture
        int moisturePercent = moistureToPercent(i, plants[i].soilMoisture.rawValue);
        pos += snprintf(buffer + pos, sizeof(buffer) - pos, 
                        "\"soilMoisture\":{\"value\":%d,\"percent\":%d,\"status\":\"%s\",\"errorCount\":%d},",
                        plants[i].soilMoisture.rawValue,
                        moisturePercent,
                        sensorStatusToStr(plants[i].soilMoisture.status),
                        plants[i].soilMoisture.errorCount);
        
        // Humidity
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "\"humidity\":{\"value\":%.1f,\"status\":\"%s\",\"errorCount\":%d},",
                        plants[i].humidity.value,
                        sensorStatusToStr(plants[i].humidity.status),
                        plants[i].humidity.errorCount);
        
        // Temperature
        pos += snprintf(buffer + pos, sizeof(buffer) - pos,
                        "\"temperature\":{\"value\":%.1f,\"status\":\"%s\",\"errorCount\":%d},",
                        plants[i].temperature.value,
                        sensorStatusToStr(plants[i].temperature.status),
                        plants[i].temperature.errorCount);
        
        // State
        unsigned long currentTime = millis();
        bool inCooldown = (plants[i].state == PLANT_COOLDOWN || plants[i].state == PLANT_POST_WATERING);
        bool isWatering = (plants[i].state == PLANT_WATERING_ACTIVE);
        
        unsigned long cooldownRemaining = 0;
        if (inCooldown && plants[i].lastWateringEnd_ms > 0)
        {
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
    
    // Convert to String for Particle.publish compatibility
    return String(buffer);
}

// ============================================================================
// MEMORY DIAGNOSTICS
// ============================================================================

// Check if JSON buffer size is adequate
void checkJSONBufferSize() {
    String json = buildSensorJSON();
    int jsonLength = json.length();
    
    // We allocated 512 bytes, warn if we're using >80%
    if (jsonLength > 410) {
        Serial.print("WARNING: JSON buffer usage high: ");
        Serial.print(jsonLength);
        Serial.println(" bytes (512 allocated)");
    }
}

void publishSensorData()
{
    if (!Particle.connected())
    {
        Serial.println("Not connected - skipping publish");
        return;
    }

    String jsonData = buildSensorJSON();
    bool success = Particle.publish("sensors/all", jsonData, PRIVATE);

    if (success)
    {
        Serial.println("Published to cloud");
    }
    else
    {
        Serial.println("Publish failed");
    }
}

void updateConnectionStatus()
{
    systemState.cloudConnected = Particle.connected();
}

bool validateSensorReading(float value, float minValue, float maxValue)
{
    return (value >= minValue && value <= maxValue);
}

int moistureToPercent(int plantId, int rawValue)
{
    int percent = map(rawValue, soilMoistureDry[plantId], soilMoistureWet[plantId], 0, 100);
    percent = constrain(percent, 0, 100);
    return percent;
}

void setup()
{
    Serial.begin(9600);
    waitFor(Serial.isConnected, 10000);

    Serial.println("=================================");
    Serial.println("Plant Watering System Starting...");
    Serial.println("=================================");

    for (int i = 0; i < NUM_PLANTS; i++)
    {
        // Initialize sensor readings
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

        // Initialize state machine
        plants[i].state = PLANT_IDLE;
        plants[i].previousState = PLANT_IDLE;
        plants[i].stateEntryTime_ms = 0;

        // Initialize configuration
        plants[i].plantId = i;
        plants[i].assignedValveId = PLANT_TO_VALVE_MAP[i];

        // Initialize watering tracking
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
        // Initialize state machine
        valves[i].state = VALVE_CLOSED;
        valves[i].previousState = VALVE_CLOSED;
        valves[i].stateEntryTime_ms = 0;

        // Initialize hardware
        valves[i].valveId = i;
        valves[i].pin = VALVE_PINS[i];
        valves[i].openStartTime_ms = 0;

        // Initialize control
        valves[i].requestingPlantId = -1;
        valves[i].targetOpenDuration_ms = VALVE_OPEN_DURATION_MS;
        
        // Initialize system state
    systemState.state = SYS_INITIALIZING;
    systemState.stateEntryTime_ms = millis();
    systemState.cloudConnected = false;
    systemState.totalSensorErrors = 0;
    systemState.totalValveErrors = 0;
    }

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
    
    // Transition system to normal operation
    systemState.state = SYS_NORMAL_OPERATION;
    systemState.stateEntryTime_ms = millis();
    Serial.println("\nSystem transitioned to NORMAL_OPERATION state");

    if (SIMULATION_MODE)
    {
        Serial.println("SIMULATION MODE - No physical relay connected");
    }

    Wire.begin();
    Serial.println("I2C initialized");
    delay(100);

    Serial.println("\nInitializing HTS221 sensors...");
    for (int i = 0; i < NUM_PLANTS; i++)
    {
        Serial.print("Initializing sensor " + String(i) + "...");

        if (hts221_sensors[i].begin())
        {
            Serial.println(" SUCCESS!");
        }
        else
        {
            Serial.println(" FAILED!");
        }
    }
    Serial.println("HTS221 initialization complete\n");
    
    // Diagnostic: Check JSON buffer sizing
    Serial.println("Running JSON buffer size check...");
    checkJSONBufferSize();
    
    Serial.println("=================================\n");
}

void loop()
{
    updateConnectionStatus();
    updatePlantPostWateringStates();
    updateReadInterval();
    updateValveStates();
    checkValveSafety();

    if (shouldReadSensors())
    {
        readAllSensors();
        evaluateSystemState();
        lastSensorRead_ms = millis();
        publishSensorData();
        printSensorData();
    }

    delay(100);
}
