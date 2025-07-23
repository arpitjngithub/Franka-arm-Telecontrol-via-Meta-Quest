using UnityEngine;
using UnityEngine.XR;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;               // for HeaderMsg
using RosMessageTypes.Geometry;          // for PoseStampedMsg, PoseMsg, PointMsg, QuaternionMsg
using RosMessageTypes.BuiltinInterfaces; // for TimeMsg

public class XRControllerPublisher : MonoBehaviour
{
    [Header("XR Settings")]
    public XRNode node;       // set to LeftHand or RightHand in the Inspector

    [Header("ROS Settings")]
    public string topicName;  // e.g. "/vr/controller_left/pose"

    ROSConnection ros;

    void Start()
    {
        // Initialize ROS connection & register the publisher
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);
    }

    void Update()
    {
        // 1) Read the controller's current pose
        Vector3 pos = InputTracking.GetLocalPosition(node);
        Quaternion rot = InputTracking.GetLocalRotation(node);

        // 2) (Optional) visualize in the Unity scene
        transform.localPosition = pos;
        transform.localRotation = rot;

        // 3) Build the ROS header with correct timestamp types
        System.DateTime now   = System.DateTime.UtcNow;
        System.DateTime epoch = new System.DateTime(1970,1,1,0,0,0,System.DateTimeKind.Utc);
        System.TimeSpan delta = now - epoch;

        int  secs  = (int)delta.TotalSeconds;                // signed seconds
        uint nsecs = (uint)(delta.Milliseconds * 1_000_000); // unsigned nanoseconds

        var header = new HeaderMsg
        {
            stamp    = new TimeMsg(secs, nsecs),
            frame_id = "vr"
        };

        // 4) Build the Pose message
        var poseMsg = new PoseMsg(
            new PointMsg(pos.x, pos.y, pos.z),
            new QuaternionMsg(rot.x, rot.y, rot.z, rot.w)
        );

        // 5) Assemble and publish the PoseStamped
        var stamped = new PoseStampedMsg
        {
            header = header,
            pose   = poseMsg
        };

        ros.Publish(topicName, stamped);
    }
}

