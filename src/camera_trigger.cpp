#include "camera_trigger.h"

const int CAMERA_TRIGGER_PIN = 0;
const unsigned long CAMERA_TRIGGER_DELAY_MS = 5000; // user configurable
const unsigned long CAMERA_PULSE_DURATION_MS = 100;

static bool cameraTriggered = false;
static bool cameraPulseActive = false;
static unsigned long cameraPulseStart = 0;
static unsigned long cameraCountdownStart = 0;

void initCameraTrigger() {
    pinMode(CAMERA_TRIGGER_PIN, OUTPUT);
    analogWrite(CAMERA_TRIGGER_PIN, 0);
    cameraCountdownStart = millis();
}

void handleCameraTrigger() {
    unsigned long now = millis();
    if (!cameraTriggered) {
        if (!cameraPulseActive) {
            if (now - cameraCountdownStart >= CAMERA_TRIGGER_DELAY_MS) {
                analogWrite(CAMERA_TRIGGER_PIN, 255);
                cameraPulseActive = true;
                cameraPulseStart = now;
            }
        } else {
            if (now - cameraPulseStart >= CAMERA_PULSE_DURATION_MS) {
                analogWrite(CAMERA_TRIGGER_PIN, 0);
                cameraPulseActive = false;
                cameraTriggered = true;
            }
        }
    }
}
