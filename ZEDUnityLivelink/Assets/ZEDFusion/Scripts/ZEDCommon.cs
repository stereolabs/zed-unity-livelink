using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace sl
{
    public class ZEDCommon
    {
        public static bool IsVector3NaN(Vector3 input)
        {
            if (float.IsNaN(input.x) || float.IsNaN(input.y) || float.IsNaN(input.z))
                return true;
            else
                return false;
        }
    }

    /// <summary>
    /// ssemantic of human body parts and order keypoints for BODY_FORMAT.BODY_34.
    /// </summary>
    public enum BODY_PARTS_POSE_34
    {
        PELVIS = 0,
        NAVAL_SPINE = 1,
        CHEST_SPINE = 2,
        NECK = 3,
        LEFT_CLAVICLE = 4,
        LEFT_SHOULDER = 5,
        LEFT_ELBOW = 6,
        LEFT_WRIST = 7,
        LEFT_HAND = 8,
        LEFT_HANDTIP = 9,
        LEFT_THUMB = 10,
        RIGHT_CLAVICLE = 11,
        RIGHT_SHOULDER = 12,
        RIGHT_ELBOW = 13,
        RIGHT_WRIST = 14,
        RIGHT_HAND = 15,
        RIGHT_HANDTIP = 16,
        RIGHT_THUMB = 17,
        LEFT_HIP = 18,
        LEFT_KNEE = 19,
        LEFT_ANKLE = 20,
        LEFT_FOOT = 21,
        RIGHT_HIP = 22,
        RIGHT_KNEE = 23,
        RIGHT_ANKLE = 24,
        RIGHT_FOOT = 25,
        HEAD = 26,
        NOSE = 27,
        LEFT_EYE = 28,
        LEFT_EAR = 29,
        RIGHT_EYE = 30,
        RIGHT_EAR = 31,
        LEFT_HEEL = 32,
        RIGHT_HEEL = 33,
        LAST = 34
    };

    /// <summary>
    /// ssemantic of human body parts and order keypoints for BODY_FORMAT.BODY_38.
    /// </summary>
    public enum BODY_PARTS_POSE_38
    {
        PELVIS = 0,
        SPINE_1 = 1,
        SPINE_2 = 2,
        SPINE_3 = 3,
        NECK = 4,
        NOSE = 5,
        LEFT_EYE = 6,
        RIGHT_EYE = 7,
        LEFT_EAR = 8,
        RIGHT_EAR = 9,
        LEFT_CLAVICLE = 10,
        RIGHT_CLAVICLE = 11,
        LEFT_SHOULDER = 12,
        RIGHT_SHOULDER = 13,
        LEFT_ELBOW = 14,
        RIGHT_ELBOW = 15,
        LEFT_WRIST = 16,
        RIGHT_WRIST = 17,
        LEFT_HIP = 18,
        RIGHT_HIP = 19,
        LEFT_KNEE = 20,
        RIGHT_KNEE = 21,
        LEFT_ANKLE = 22,
        RIGHT_ANKLE = 23,
        LEFT_BIG_TOE = 24,
        RIGHT_BIG_TOE = 25,
        LEFT_SMALL_TOE = 26,
        RIGHT_SMALL_TOE = 27,
        LEFT_HEEL = 28,
        RIGHT_HEEL = 29,
        // Hands
        LEFT_HAND_THUMB_4 = 30, // tip
        RIGHT_HAND_THUMB_4 = 31,
        LEFT_HAND_INDEX_1 = 32, // knuckle
        RIGHT_HAND_INDEX_1 = 33,
        LEFT_HAND_MIDDLE_4 = 34, // tip
        RIGHT_HAND_MIDDLE_4 = 35,
        LEFT_HAND_PINKY_1 = 36, // knuckle
        RIGHT_HAND_PINK_1 = 37,
        LAST = 38
    };

    public enum OBJECT_TRACK_STATE
    {
        OFF, /**< The tracking is not yet initialized, the object ID is not usable */
        OK, /**< The object is tracked */
        SEARCHING,/**< The object couldn't be detected in the image and is potentially occluded, the trajectory is estimated */
        TERMINATE/**< This is the last searching state of the track, the track will be deleted in the next retreiveObject */
    };

    public enum OBJECT_ACTION_STATE
    {
        IDLE = 0, /**< The object is staying static. */
        MOVING = 1, /**< The object is moving. */
        LAST = 2
    };

    public struct CameraIdentifier
    {
        public ulong sn;
    }

    public class CameraMetrics
    {
        public ulong sn;

        public float received_fps;

        public float received_latency;

        public float synced_latency;

        public bool is_present;

        public float ratio_detection;

        public float delta_ts;

        public static CameraMetrics CreateFromJSON(byte[] data)
        {
            return JsonConvert.DeserializeObject<CameraMetrics>(Encoding.ASCII.GetString(data));
        }
    }

    public class SingleCameraMetric
    {
        public ulong sn;

        public CameraMetrics cameraMetrics;
    }

    public class FusionMetrics
    {
        public float mean_camera_fused;

        public float mean_stdev_between_camera;

        public List<CameraMetrics> camera_individual_stats;
    }

    /// <summary>
    /// Lists of supported skeleton body model
    /// </summary>
    public enum BODY_FORMAT
    {
        BODY_18,
        BODY_34,
        BODY_38
    };

    public class BodyData
    {
        /// <summary>
        /// Object identification number, used as a reference when tracking the object through the frames.
        /// </summary>
        public int id;
        /// <summary>
        ///  Object label, forwarded from \ref CustomBoxObjects when using DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        /// </summary>
        public OBJECT_TRACK_STATE tracking_state;
        public OBJECT_ACTION_STATE action_state;
        public float confidence;
        /// <summary>
        /// 3D space data (Camera Frame since this is what we used in Unity)
        /// </summary>
        public Vector3 position; //object root position
        //public Vector3 head_position; //object head position (only for HUMAN detectionModel)
        public Vector3 velocity; //object root velocity
        /// <summary>
        /// 3D object dimensions: width, height, length. Defined in InitParameters.UNIT, expressed in RuntimeParameters.measure3DReferenceFrame.
        /// </summary>
        public Vector3 dimensions;

        /// <summary>
        /// The 3D space bounding box. given as array of vertices
        /// </summary>
        ///   1 ---------2  
        ///  /|         /|
        /// 0 |--------3 |
        /// | |        | |
        /// | 5--------|-6
        /// |/         |/
        /// 4 ---------7
        /// 
        //public Vector3[] bounding_box; // 3D Bounding Box of object
                /// <summary>
        /// The 3D position of skeleton joints
        /// </summary>
        public Vector3[] keypoint;// 3D position of the joints of the skeleton

        // Full covariance matrix for position (3x3). Only 6 values are necessary
        // [p0, p1, p2]
        // [p1, p3, p4]
        // [p2, p4, p5]
        //public float[] position_covariance;// covariance matrix of the 3d position, represented by its upper triangular matrix value

        /// <summary>
        ///  Per keypoint detection confidence, can not be lower than the ObjectDetectionRuntimeParameters.detection_confidence_threshold.
        ///  Not available with DETECTION_MODEL.MULTI_CLASS_BOX.
        ///  in some cases, eg. body partially out of the image or missing depth data, some keypoint can not be detected, they will have non finite values.
        /// </summary>
        public float[] keypoint_confidence;

        /// <summary>
        /// Global position per joint in the coordinate frame of the requested skeleton format.
        /// </summary>
        public Vector3[] local_position_per_joint;
        /// <summary>
        /// Local orientation per joint in the coordinate frame of the requested skeleton format.
        /// The orientation is represented by a quaternion.
        /// </summary>
        public Quaternion[] local_orientation_per_joint;
        /// <summary>
        /// Global root rotation.
        /// </summary>
        public Quaternion global_root_orientation;
    };

    public class Bodies
    {
        public sl.BODY_FORMAT body_format;
        /// <summary>
        /// How many objects were detected this frame. Use this to iterate through the top of objectData; objects with indexes greater than nb_object are empty. 
        /// </summary>
        public int nb_object;
        /// <summary>
        /// Timestamp of the image where these objects were detected.
        /// </summary>
        public ulong timestamp;
        /// <summary>
        /// Defines if the object frame is new (new timestamp)
        /// </summary>
        public int is_new;
        /// <summary>
        /// Defines if the object is tracked
        /// </summary>
        public int is_tracked;
        /// <summary>
        /// Array of objects 
        /// </summary>
        public BodyData[] body_list;

        public static Bodies CreateFromJSON(byte[] data)
        {
            return JsonConvert.DeserializeObject<Bodies>(Encoding.ASCII.GetString(data));
        }
    };

    public class DetectionData
    {
        public FusionMetrics fusionMetrics;

        public Bodies bodies;

        public static DetectionData CreateFromJSON(byte[] data)
        {
            return JsonConvert.DeserializeObject<DetectionData>(Encoding.ASCII.GetString(data));
        }
    }
}

