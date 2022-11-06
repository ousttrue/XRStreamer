#include "HandRenderer.h"
#include "HandsManager.h"
#include "XrApp.h"

bool HandRenderer::OnSessionInit(bool isLeft)
{
    if (!controllerRender.Init(isLeft))
    {
        return false;
    }
    /// Hand rendering
    axisRenderer.Init();

    /// mesh
    mesh = {XR_TYPE_HAND_TRACKING_MESH_FB};
    mesh.next = nullptr;
    /// mesh - skeleton
    mesh.jointCapacityInput = 0;
    mesh.jointCountOutput = 0;
    mesh.jointBindPoses = nullptr;
    mesh.jointRadii = nullptr;
    mesh.jointParents = nullptr;
    /// mesh - vertex
    mesh.vertexCapacityInput = 0;
    mesh.vertexCountOutput = 0;
    mesh.vertexPositions = nullptr;
    mesh.vertexNormals = nullptr;
    mesh.vertexUVs = nullptr;
    mesh.vertexBlendIndices = nullptr;
    mesh.vertexBlendWeights = nullptr;
    /// mesh - index
    mesh.indexCapacityInput = 0;
    mesh.indexCountOutput = 0;
    mesh.indices = nullptr;

    return true;
}

void HandRenderer::OnMeshSize()
{
    /// mesh storage - update sizes
    mesh.jointCapacityInput = mesh.jointCountOutput;
    mesh.vertexCapacityInput = mesh.vertexCountOutput;
    mesh.indexCapacityInput = mesh.indexCountOutput;
    /// skeleton
    jointBindLocations.resize(mesh.jointCountOutput);
    parentData.resize(mesh.jointCountOutput);
    jointRadii.resize(mesh.jointCountOutput);
    mesh.jointBindPoses = jointBindLocations.data();
    mesh.jointParents = parentData.data();
    mesh.jointRadii = jointRadii.data();
    /// vertex
    vertexPositions.resize(mesh.vertexCountOutput);
    vertexNormals.resize(mesh.vertexCountOutput);
    vertexUVs.resize(mesh.vertexCountOutput);
    vertexBlendIndices.resize(mesh.vertexCountOutput);
    vertexBlendWeights.resize(mesh.vertexCountOutput);
    mesh.vertexPositions = vertexPositions.data();
    mesh.vertexNormals = vertexNormals.data();
    mesh.vertexUVs = vertexUVs.data();
    mesh.vertexBlendIndices = vertexBlendIndices.data();
    mesh.vertexBlendWeights = vertexBlendWeights.data();
    /// index
    indices.resize(mesh.indexCountOutput);
    mesh.indices = indices.data();
    /// chain capsules
    capsuleState = {
        XR_TYPE_HAND_TRACKING_CAPSULES_STATE_FB};
    capsuleState.next = nullptr;
    mesh.next = &capsuleState;
}

void HandRenderer::OnMeshData(HandTracker *hand, const OVR::Vector4f &jointColor_,
                              const OVR::Vector4f &capsuleColor_

)
{
    /// init renderer
    handRenderer.Init(&mesh, true);
    /// Render jointRadius for all left hand joints
    {
        handJointRenderers.resize(XR_HAND_JOINT_COUNT_EXT);
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
        {
            const OVR::Posef pose = FromXrPosef(hand->jointLocations[i].pose);
            OVRFW::GeometryRenderer &gr = handJointRenderers[i];
            gr.Init(OVRFW::BuildTesselatedCapsuleDescriptor(
                mesh.jointRadii[i], 0.0f, 7, 7));
            gr.SetPose(pose);
            gr.DiffuseColor = jointColor_;
        }
    }
    /// One time init for capsules
    {
        handCapsuleRenderers.resize(XR_FB_HAND_TRACKING_CAPSULE_COUNT);
        for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
        {
            const OVR::Vector3f p0 =
                FromXrVector3f(hand->capsuleState.capsules[i].points[0]);
            const OVR::Vector3f p1 =
                FromXrVector3f(hand->capsuleState.capsules[i].points[1]);
            const OVR::Vector3f d = (p1 - p0);
            const float h = d.Length();
            const float r = hand->capsuleState.capsules[i].radius;
            const OVR::Quatf look = OVR::Quatf::LookRotation(d, {0, 1, 0});
            OVRFW::GeometryRenderer &gr = handCapsuleRenderers[i];
            gr.Init(OVRFW::BuildTesselatedCapsuleDescriptor(r, h, 7, 7));
            gr.SetPose(OVR::Posef(look, p0));
            gr.DiffuseColor = capsuleColor_;
        }
    }
    /// Print hierarchy
    {
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
        {
            const OVR::Posef pose = FromXrPosef(hand->jointLocations[i].pose);
            ALOG(" { {%.6f, %.6f, %.6f},  {%.6f, %.6f, %.6f, %.6f} } // "
                 "joint = %d, parent = %d",
                 pose.Translation.x, pose.Translation.y, pose.Translation.z,
                 pose.Rotation.x, pose.Rotation.y, pose.Rotation.z,
                 pose.Rotation.w, i, (int)parentData[i]);
        }
    }
}
