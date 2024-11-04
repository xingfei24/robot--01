#include "QuadrupedRobot.h"

// ... [Previous implementation remains unchanged until the end]

void QuadrupedRobot::walk(int cycles, float velocity, Direction direction) {
    moving = true;
    float angles[TOTAL_SERVOS];
    
    for (int cycle = 0; cycle < cycles; cycle++) {
        generateWalkingGait(angles, cycle, velocity, direction);
        applyBalanceCorrection(angles);
        smoothMove(angles, 100);  // 100ms per cycle
        
        if (!isBalanced()) {
            stabilize();
            break;
        }
    }
    
    moving = false;
}

void QuadrupedRobot::learn() {
    // Collect current state data
    float input[14];  // 6 MPU + 8 leg angles
    
    // MPU data
    input[0] = mpuData.accel_x;
    input[1] = mpuData.accel_y;
    input[2] = mpuData.accel_z;
    input[3] = mpuData.gyro_x;
    input[4] = mpuData.gyro_y;
    input[5] = mpuData.gyro_z;
    
    // Current leg angles
    for (int i = 0; i < 8; i++) {
        input[i + 6] = currentLegAngles[i];
    }
    
    // Generate target angles for stable walking
    float target[8];
    for (int i = 0; i < 8; i++) {
        target[i] = currentLegAngles[i];  // Use current angles as initial targets
    }
    
    // Train neural network
    nn->train(input, target, 0.01f);  // Learning rate = 0.01
}

void QuadrupedRobot::stabilize() {
    updateMpuData();
    
    float pitchError = mpuData.Sta_Pitch;
    float rollError = mpuData.Sta_Roll;
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float x = 0;
        float z = -bodyHeight;
        
        // Adjust for pitch
        if (leg < 2) z += pitchError * 2;  // Front legs
        else z -= pitchError * 2;          // Back legs
        
        // Adjust for roll
        if (leg % 2) z += rollError * 2;   // Right legs
        else z -= rollError * 2;           // Left legs
        
        float angles[2];
        calculateInverseKinematics(x, 0, z, angles);
        setServoAngle(leg * 2, angles[0]);
        setServoAngle(leg * 2 + 1, angles[1]);
    }
}

void QuadrupedRobot::balance() {
    updateMpuData();
    
    // PID control for balance
    static float lastPitchError = 0;
    static float lastRollError = 0;
    static float pitchErrorSum = 0;
    static float rollErrorSum = 0;
    
    float pitchError = mpuData.pitch;
    float rollError = mpuData.roll;
    
    pitchErrorSum += pitchError;
    rollErrorSum += rollError;
    
    float pitchCorrection = 
        pidGains[0] * pitchError +                    // Proportional
        pidGains[1] * pitchErrorSum +                 // Integral
        pidGains[2] * (pitchError - lastPitchError);  // Derivative
        
    float rollCorrection = 
        pidGains[0] * rollError +
        pidGains[1] * rollErrorSum +
        pidGains[2] * (rollError - lastRollError);
    
    lastPitchError = pitchError;
    lastRollError = rollError;
    
    // Apply corrections
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float z = -bodyHeight;
        
        if (leg < 2) z += pitchCorrection;
        else z -= pitchCorrection;
        
        if (leg % 2) z += rollCorrection;
        else z -= rollCorrection;
        
        float angles[2];
        calculateInverseKinematics(0, 0, z, angles);
        setServoAngle(leg * 2, angles[0]);
        setServoAngle(leg * 2 + 1, angles[1]);
    }
}

void QuadrupedRobot::stand() {
    moving = false;
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float angles[2];
        calculateInverseKinematics(0, 0, -bodyHeight, angles);
        targetAngles[leg * 2] = angles[0];
        targetAngles[leg * 2 + 1] = angles[1];
    }
    
    smoothMove(targetAngles, 500);  // 500ms transition
}

void QuadrupedRobot::updateMpuData() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    mpuData.accel_x = a.acceleration.x;
    mpuData.accel_y = a.acceleration.y;
    mpuData.accel_z = a.acceleration.z;
    
    mpuData.gyro_x = g.gyro.x;
    mpuData.gyro_y = g.gyro.y;
    mpuData.gyro_z = g.gyro.z;
    
    mpuData.temp = temp.temperature;
    
    mpuData.calculateAngles();
}

void QuadrupedRobot::initServoConfigs() {
    // Initialize servo configurations for each leg
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // Ham servo (upper leg)
        servoConfigs[leg * 2] = {
            .pin = leg * 2,
            .minPulse = 150,
            .maxPulse = 600,
            .minAngle = 0,
            .maxAngle = 180,
            .offset = 90,
            .reversed = false
        };
        
        // Shank servo (lower leg)
        servoConfigs[leg * 2 + 1] = {
            .pin = leg * 2 + 1,
            .minPulse = 150,
            .maxPulse = 600,
            .minAngle = 0,
            .maxAngle = 180,
            .offset = 90,
            .reversed = true
        };
    }
}

void QuadrupedRobot::setServoAngle(int servoIndex, float angle) {
    if (servoIndex >= TOTAL_SERVOS) return;
    
    const ServoConfig& config = servoConfigs[servoIndex];
    
    // Apply offset and reverse if needed
    float adjustedAngle = angle + config.offset;
    if (config.reversed) adjustedAngle = 180 - adjustedAngle;
    
    // Constrain angle
    adjustedAngle = constrain(adjustedAngle, config.minAngle, config.maxAngle);
    
    // Convert angle to pulse width
    float pulse = map(adjustedAngle, 0, 180, config.minPulse, config.maxPulse);
    
    // Update servo position
    pwm.setPWM(config.pin, 0, (int)pulse);
    
    // Store current angle
    currentLegAngles[servoIndex] = angle;
}

void QuadrupedRobot::saveModel(const char* filename) {
    nn->save(filename);
}