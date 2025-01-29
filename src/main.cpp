#include <TMCStepper.h>

#define DIR_PIN 25
#define STEP_PIN 26
#define STALL_PIN 27
#define DRIVER_ADDRESS 2
#define R_SENSE 0.11f

constexpr uint16_t fullPathStepCount = 12500;
constexpr uint16_t offsetSteps = 500;

enum Direction
{
    BACKWARD = 1,
    FORWARD = 0,
};


volatile bool stalled = false;

void stallInterrupt()
{
    stalled = true;
}

enum ResultCode
{
    MOVE_OK, MOVE_STALL, MOVE_TOO_LONG
};

struct Result
{
    size_t steps;
    ResultCode resultCode;
};

TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDRESS);

void configureDriver()
{
    driver.begin();
    driver.toff(5);
    driver.rms_current(600);
    driver.microsteps(64);
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);
    driver.TCOOLTHRS(0xFFFFF);
    driver.SGTHRS(1);
}

void configurePins()
{
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STALL_PIN, INPUT);
}

Result moveUntilStall(const size_t maxSteps, const Direction dir)
{
    size_t doneSteps = 0;
    stalled = false;
    driver.shaft(dir);

    while (true) {
        if (stalled) {
            return Result{doneSteps, MOVE_STALL};
        }
        if (doneSteps > maxSteps) {
            return Result{doneSteps, MOVE_TOO_LONG};
        }
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(160);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(160);
        ++doneSteps;
    }
}

Result moveSteps(size_t stepsLeft, const Direction dir)
{
    size_t doneSteps = 0;
    stalled = false;
    driver.shaft(dir);

    for (; stepsLeft > 0; --stepsLeft) {
        if (stalled) {
            return Result{doneSteps, MOVE_STALL};
        }
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(160);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(160);
        ++doneSteps;
    }
    return Result{doneSteps, MOVE_OK};
}

void home()
{
    uint8_t counter = 0;
    Result res = moveUntilStall(fullPathStepCount + offsetSteps, FORWARD);
    while (res.resultCode != MOVE_STALL) {
        if (counter == 3) {
            // TODO sens message lock bricked
            Serial.println("Lock bricked");
        }
        res = moveUntilStall(fullPathStepCount + offsetSteps - res.steps, FORWARD);
        ++counter;
    }
}

void open()
{
    uint8_t counter = 0;
    Result res = moveSteps(fullPathStepCount, BACKWARD);
    while (res.resultCode != MOVE_OK) {
        if (counter == 3) {
            Serial.println("Lock bricked");
            // TODO send message lock bricked
        }
        res = moveSteps(fullPathStepCount - res.steps, BACKWARD);
        ++counter;
    }
}

void close()
{
    home();
}

void setup()
{
    Serial.begin(9600);
    Serial2.begin(115200);

    configurePins();
    attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallInterrupt, RISING);
    configureDriver();
    home();
}

void loop()
{
    delay(4000);
    open();
    delay(4000);
    close();
}
