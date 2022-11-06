#include "XrHands.h"
#include "XrApp.h"
#include <openxr/openxr.h>

XrHands::XrHands(XrInstance &Instance)
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

XrHands::~XrHands()
{
}

XrHands *XrHands::Create(XrInstance &Instance, XrSystemId &systemId)
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

        return new XrHands(Instance);
    }
}

void XrHands::OnSessionInit(XrInstance &Instance, XrSession &Session)
{
    XrHandTrackerCreateInfoEXT createInfo{
        XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
    createInfo.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
    createInfo.hand = XR_HAND_LEFT_EXT;
    OXR(xrCreateHandTrackerEXT_(Session, &createInfo, &handTrackerL_));
    createInfo.hand = XR_HAND_RIGHT_EXT;
    OXR(xrCreateHandTrackerEXT_(Session, &createInfo, &handTrackerR_));

    ALOG("xrCreateHandTrackerEXT handTrackerL_=%llx",
         (long long)handTrackerL_);
    ALOG("xrCreateHandTrackerEXT handTrackerR_=%llx",
         (long long)handTrackerR_);
}

void XrHands::Shutdown(XrInstance &Instance)
{
    if (xrDestroyHandTrackerEXT_)
    {
        OXR(xrDestroyHandTrackerEXT_(handTrackerL_));
        OXR(xrDestroyHandTrackerEXT_(handTrackerR_));
    }
}

void XrHands::Update(XrInstance &Instance, XrSpace &space, XrTime time)
{
    /// L
    scaleL = {XR_TYPE_HAND_TRACKING_SCALE_FB};
    scaleL.next = nullptr;
    scaleL.sensorOutput = 1.0f;
    scaleL.currentOutput = 1.0f;
    scaleL.overrideValueInput = 1.00f;
    scaleL.overrideHandScale = XR_FALSE; // XR_TRUE;
    capsuleStateL = {
        XR_TYPE_HAND_TRACKING_CAPSULES_STATE_FB};
    capsuleStateL.next = &scaleL;
    aimStateL = {XR_TYPE_HAND_TRACKING_AIM_STATE_FB};
    aimStateL.next = &capsuleStateL;
    XrHandJointVelocitiesEXT velocitiesL{XR_TYPE_HAND_JOINT_VELOCITIES_EXT};
    velocitiesL.next = &aimStateL;
    velocitiesL.jointCount = XR_HAND_JOINT_COUNT_EXT;
    velocitiesL.jointVelocities = jointVelocitiesL_;
    locationsL = {XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
    locationsL.next = &velocitiesL;
    locationsL.jointCount = XR_HAND_JOINT_COUNT_EXT;
    locationsL.jointLocations = jointLocationsL_;

    XrHandJointsLocateInfoEXT locateInfoL{
        XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
    locateInfoL.baseSpace = space;
    locateInfoL.time = ToXrTime(time);
    OXR(xrLocateHandJointsEXT_(handTrackerL_, &locateInfoL, &locationsL));

    /// R
    scaleR = {XR_TYPE_HAND_TRACKING_SCALE_FB};
    scaleR.next = nullptr;
    scaleR.sensorOutput = 1.0f;
    scaleR.currentOutput = 1.0f;
    scaleR.overrideValueInput = 1.00f;
    scaleR.overrideHandScale = XR_FALSE; // XR_TRUE;
    capsuleStateR = {
        XR_TYPE_HAND_TRACKING_CAPSULES_STATE_FB};
    capsuleStateR.next = &scaleR;
    aimStateR = {XR_TYPE_HAND_TRACKING_AIM_STATE_FB};
    aimStateR.next = &capsuleStateR;
    XrHandJointVelocitiesEXT velocitiesR{XR_TYPE_HAND_JOINT_VELOCITIES_EXT};
    velocitiesR.next = &aimStateR;
    velocitiesR.jointCount = XR_HAND_JOINT_COUNT_EXT;
    velocitiesR.jointVelocities = jointVelocitiesR_;
    locationsR = {XR_TYPE_HAND_JOINT_LOCATIONS_EXT};
    locationsR.next = &velocitiesR;
    locationsR.jointCount = XR_HAND_JOINT_COUNT_EXT;
    locationsR.jointLocations = jointLocationsR_;
    XrHandJointsLocateInfoEXT locateInfoR{
        XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT};
    locateInfoR.baseSpace = space;
    locateInfoR.time = ToXrTime(time);
    OXR(xrLocateHandJointsEXT_(handTrackerR_, &locateInfoR, &locationsR));
}
