#pragma once

enum eSensor {
    MONOCULAR = 0,
    STEREO = 1,
    RGBD = 2
};

enum InputType {
    DATASET,
    MONO_CAMERA,
    D435i_CAMERA
};

// Tracking states
enum eTrackingState {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3
};
