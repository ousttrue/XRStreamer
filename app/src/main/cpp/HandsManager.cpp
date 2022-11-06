#include "HandsManager.h"
#include "XrApp.h"
#include <openxr/openxr.h>

void HandTracker::Update(XrSpace &space, XrTime time)
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

HandsManager::HandsManager(XrInstance &Instance)
{
    /// Hook up extensions for hand tracking
    OXR(xrGetInstanceProcAddr(
        Instance, "xrCreateHandTrackerEXT",
        (PFN_xrVoidFunction *)(&xrCreateHandTrackerEXT_)));
    OXR(xrGetInstanceProcAddr(
        Instance, "xrDestroyHandTrackerEXT",
        (PFN_xrVoidFunction *)(&xrDestroyHandTrackerEXT_)));
    OXR(xrGetInstanceProcAddr(Instance, "xrLocateHandJointsEXT",
                              (PFN_xrVoidFunction *)(&xrLocateHandJointsEXT_)));
}

HandsManager::~HandsManager()
{
}

HandsManager *HandsManager::Create(XrInstance &Instance, XrSystemId &systemId)
{
    // Inspect hand tracking system properties
    XrSystemHandTrackingPropertiesEXT handTrackingSystemProperties{
        XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT};
    XrSystemProperties systemProperties{XR_TYPE_SYSTEM_PROPERTIES,
                                        &handTrackingSystemProperties};
    OXR(xrGetSystemProperties(Instance, systemId, &systemProperties));
    if (!handTrackingSystemProperties.supportsHandTracking)
    {
        // The system does not support hand tracking
        ALOG("xrGetSystemProperties XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT "
             "FAILED.");
        return nullptr;
    }
    else
    {
        ALOG("xrGetSystemProperties XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT "
             "OK - initiallizing hand tracking...");

        return new HandsManager(Instance);
    }
}

void HandsManager::OnSessionInit(XrInstance &Instance, XrSession &Session)
{
    OXR(xrCreateHandTrackerEXT_(Session, left_.CreateInfo(XR_HAND_LEFT_EXT), &left_.handTracker));
    ALOG("xrCreateHandTrackerEXT handTrackerL_=%llx",
         (long long)left_.handTracker);

    OXR(xrCreateHandTrackerEXT_(Session, right_.CreateInfo(XR_HAND_RIGHT_EXT), &right_.handTracker));
    ALOG("xrCreateHandTrackerEXT handTrackerR_=%llx",
         (long long)right_.handTracker);
}

void HandsManager::Shutdown(XrInstance &Instance)
{
    if (xrDestroyHandTrackerEXT_)
    {
        OXR(xrDestroyHandTrackerEXT_(left_.handTracker));
        OXR(xrDestroyHandTrackerEXT_(right_.handTracker));
    }
}

void HandsManager::Update(XrInstance &Instance, XrSpace &space, XrTime time)
{
    /// L
    left_.Update(space, time);
    OXR(xrLocateHandJointsEXT_(left_.handTracker, &left_.locateInfo, &left_.locations));

    /// R
    right_.Update(space, time);
    OXR(xrLocateHandJointsEXT_(right_.handTracker, &right_.locateInfo, &right_.locations));
}
