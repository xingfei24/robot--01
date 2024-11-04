#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <math.h>
#include "NeuralNetwork.h"
#include "DataCollector.h"
#include "MpuData.h"

enum Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

enum GaitType {
    TROT,
    WALK,
    GALLOP,
    BOUND
};

class QuadrupedRobot {
public:
    QuadrupedRobot();
    ~QuadrupedRobot();
    
    // Basic initialization and control
    void init();
    void update();
    void walk(int cycles, float velocity, Direction direction = FORWARD);
    void stand();
    void balance();
    void stabilize();
    
    // Advanced movement controls
    void setGait(GaitType gait);
    void jump(float height, float duration);
    void turn(float angle, float duration);
    void crouch(float height);
    void stretch();
    
    // Learning and adaptation
    void learn();
    void saveModel(const char* filename);
    void loadModel(const char* filename);
    void adaptToTerrain();
    void calibrateSensors();
    
    // State management
    bool isBalanced() const;
    bool isMoving() const;
    float getBatteryLevel() const;
    const MpuData& getSensorData() const;
    
    // Configuration
    void setMaxSpeed(float speed);
    void setStepHeight(float height);
    void setBodyHeight(float height);
    void setPIDGains(float kp, float ki, float kd);
    
private:
    static const int NUM_LEGS = 4;
    static const int SERVOS_PER_LEG = 2;
    static const int TOTAL_SERVOS = NUM_LEGS * SERVOS_PER_LEG;
    
    // Hardware components
    Adafruit_PWMServoDriver pwm;
    Adafruit_MPU6050 mpu;
    NeuralNetwork* nn;
    DataCollector dataCollector;
    
    // State variables
    float currentLegAngles[TOTAL_SERVOS];
    float defaultStandAngles[TOTAL_SERVOS];
    float targetAngles[TOTAL_SERVOS];
    bool isInitialized;
    bool moving;
    GaitType currentGait;
    
    // Configuration parameters
    float maxSpeed;
    float stepHeight;
    float bodyHeight;
    float pidGains[3];  // Kp, Ki, Kd
    
    // Servo configuration
    struct ServoConfig {
        int pin;
        int minPulse;
        int maxPulse;
        float minAngle;
        float maxAngle;
        float offset;
        bool reversed;
    };
    ServoConfig servoConfigs[TOTAL_SERVOS];
    
    // MPU data and processing
    MpuData mpuData;
    void updateMpuData();
    MpuData getMpuData() const { return mpuData; }
    
    // Neural network functions
    void initNeuralNetwork();
    void collectTrainingData();
    void trainModel();
    void applyLearnedBehavior();
    
    // Servo control functions
    void initServoConfigs();
    int getServoIndex(int legIndex, bool isHam) const;
    void setServoAngle(int servoIndex, float angle);
    void moveLegs(const float* targetAngles, float speed);
    void smoothMove(const float* targetAngles, float duration);
    
    // Gait generation and control
    void generateWalkingGait(float* angles, int cycle, float velocity, Direction direction);
    void calculateLegTrajectory(float* x, float* y, float phase, float velocity);
    void applyBalanceCorrection(float* angles);
    void calculateInverseKinematics(float x, float y, float z, float* angles);
    
    // Balance and stability
    void updateBalanceControl();
    void filterSensorData();
    void calculateCenterOfMass();
    void adjustPosture();
    
    // Terrain adaptation
    void detectTerrainType();
    void adjustGaitParameters();
    void compensateForSlope();
    void detectObstacles();
    
    // Energy management
    void monitorBatteryLevel();
    void optimizeEnergyUsage();
    void enterLowPowerMode();
    
    // Safety features
    void checkJointLimits();
    void detectFalls();
    void emergencyStop();
    void validateMovement();
    
    // Utility functions
    float normalizeAngle(float angle);
    float interpolateAngle(float start, float end, float t);
    void debugOutput();
};

#endif // QUADRUPED_ROBOT_H