#include "HandTracking.h"
#include "XrApp.h"
#include <openxr/openxr.h>

void HandTracking::Update(XrSpace &space, XrTime time)
{
    scale = {XR_TYPE_HAND_TRACKING_SCALE_FB};
    scale.next = nullptr;
    scale.sensorOutput = 1.0f;
    scale.currentOutput = 1.0f;
    scale.overrideValueInput = 1.00f;
    scale.overrideHandScale = XR_FALSE; // XR_TRUE;
    capsuleState = {
        XR_TYPE_HAND_TRACKING_CAPSULES_STATE_FB};
    capsuleState.next = &scale;
    aimState = {XR_TYPE_HAND_TRACKING_AIM_STATE_FB};
    aimState.next = &capsuleState;
    velocities = {XR_TYPE_HAND_JOINT_VELOCITIES_EXT};
    velocities.next = &aimState;
    velocities.jointCount = XR_HAND_JOINT_COUNT_EXT;
    velocities.jointVelocities = jointVelocities;
    locations = {XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
    locations.next = &velocities;
    locations.jointCount = XR_HAND_JOINT_COUNT_EXT;
    locations.jointLocations = jointLocations;

    locateInfo = {
        XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
    locateInfo.baseSpace = space;
    locateInfo.time = ToXrTime(time);
}
