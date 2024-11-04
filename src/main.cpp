#include "QuadrupedRobot.h"

QuadrupedRobot robot;
bool isLearning = true;
unsigned long lastDebugOutput = 0;
unsigned long lastTerrainCheck = 0;
unsigned long lastBalanceUpdate = 0;
unsigned long lastBatteryCheck = 0;

// Configuration constants
const float LEARNING_DURATION = 300000;  // 5 minutes
const float INITIAL_SPEED = 0.5f;
const float INITIAL_STEP_HEIGHT = 30.0f;
const float INITIAL_BODY_HEIGHT = 100.0f;

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing quadruped robot...");
    
    // Initialize robot with default configuration
    robot.init();
    robot.setMaxSpeed(INITIAL_SPEED);
    robot.setStepHeight(INITIAL_STEP_HEIGHT);
    robot.setBodyHeight(INITIAL_BODY_HEIGHT);
    robot.setPIDGains(0.8f, 0.2f, 0.1f);
    
    // Initial calibration
    robot.calibrateSensors();
    
    // Wait for stability
    delay(1000);
    
    Serial.println("Robot initialized and ready.");
}

void handleLearningPhase() {
    static unsigned long learningStartTime = millis();
    static float velocities[] = {0.3f, 0.4f, 0.5f};
    static int currentVelocityIndex = 0;
    
    // Check if learning phase should end
    if (millis() - learningStartTime > LEARNING_DURATION) {
        isLearning = false;
        robot.saveModel("/model.dat");
        Serial.println("Learning phase completed");
        return;
    }
    
    // Cycle through different velocities
    float currentVelocity = velocities[currentVelocityIndex];
    robot.setMaxSpeed(currentVelocity);
    
    // Walk and learn
    robot.walk(5, currentVelocity, FORWARD);
    robot.learn();
    
    // Rotate through velocities
    currentVelocityIndex = (currentVelocityIndex + 1) % 3;
    
    // Periodic model saving
    static unsigned long lastSave = 0;
    if (millis() - lastSave > 300000) { // Save every 5 minutes
        robot.saveModel("/model.dat");
        lastSave = millis();
    }
}

void handleNormalOperation() {
    static float velocities[] = {0.3f, 0.4f, 0.5f};
    static int currentVelocityIndex = 0;
    static GaitType gaits[] = {TROT, WALK, GALLOP};
    static int currentGaitIndex = 0;
    
    // Cycle through gaits and velocities
    robot.setGait(gaits[currentGaitIndex]);
    robot.setMaxSpeed(velocities[currentVelocityIndex]);
    
    // Execute movement
    robot.walk(5, velocities[currentVelocityIndex], FORWARD);
    
    // Rotate through patterns
    currentVelocityIndex = (currentVelocityIndex + 1) % 3;
    if (currentVelocityIndex == 0) {
        currentGaitIndex = (currentGaitIndex + 1) % 3;
    }
}

void performPeriodicChecks() {
    unsigned long currentTime = millis();
    
    // Balance control (every 20ms)
    if (currentTime - lastBalanceUpdate > 20) {
        robot.updateBalanceControl();
        lastBalanceUpdate = currentTime;
    }
    
    // Terrain adaptation (every 1 second)
    if (currentTime - lastTerrainCheck > 1000) {
        robot.adaptToTerrain();
        lastTerrainCheck = currentTime;
    }
    
    // Battery monitoring (every 5 seconds)
    if (currentTime - lastBatteryCheck > 5000) {
        robot.monitorBatteryLevel();
        robot.optimizeEnergyUsage();
        lastBatteryCheck = currentTime;
    }
    
    // Debug output (every 500ms)
    if (currentTime - lastDebugOutput > 500) {
        robot.debugOutput();
        lastDebugOutput = currentTime;
    }
}

void handleSafetyChecks() {
    // Continuous safety monitoring
    robot.validateMovement();
    
    // Emergency handling
    if (!robot.isBalanced()) {
        robot.stabilize();
    }
    
    if (robot.getBatteryLevel() < 0.1f) {
        robot.emergencyStop();
        Serial.println("Emergency stop: Low battery");
    }
}

void loop() {
    // Perform periodic checks and safety monitoring
    performPeriodicChecks();
    handleSafetyChecks();
    
    // Main operation logic
    if (isLearning) {
        handleLearningPhase();
    } else {
        handleNormalOperation();
    }
    
    // Basic stabilization
    robot.balance();
    robot.stabilize();
    
    // Small delay to prevent CPU overload
    delay(10);
}