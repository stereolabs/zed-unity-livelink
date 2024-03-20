using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.Playables;

namespace sl
{
    public class ZEDCommon
    {
        public static bool IsNaN(Vector3 input)
        {
            if (float.IsNaN(input.x) || float.IsNaN(input.y) || float.IsNaN(input.z))
                return true;
            else
                return false;
        }

        public static bool IsNull(Quaternion input)
        {
            if (input.x == 0 && input.y == 0 && input.z == 0 && input.w == 0)
                return true;
            else
                return false;
        }

        //Convert vector to Unity coordinate unit (centimeter)
        static float GetUnitFactor(ZED_COORDINATE_UNIT InUnit)
        {
            float factor = 1;

            switch (InUnit)
            {
                case ZED_COORDINATE_UNIT.METER:
                    factor = 1;
                    break;
                case ZED_COORDINATE_UNIT.CENTIMETER:
                    factor = 0.01f;
                    break;
                case ZED_COORDINATE_UNIT.MILLIMETER:
                    factor = 0.001f;
                    break;
                case ZED_COORDINATE_UNIT.INCH:
                    factor = 0.0254f;
                    break;
                case ZED_COORDINATE_UNIT.FOOT:
                    factor = 0.3048f;
                    break;
                default:
                    factor = 0.1f;
                    break;
            }
            return factor;
        }

        public static void GetCoordinateTransform(ZED_COORDINATE_SYSTEM coord_system, ref Matrix4x4 coordinateMatrix)
        {
            coordinateMatrix = Matrix4x4.identity;

            // set desired coordinate system
            switch (coord_system)
            {
                case ZED_COORDINATE_SYSTEM.IMAGE:
                    coordinateMatrix = Matrix4x4.identity;
                    break;
                case ZED_COORDINATE_SYSTEM.LEFT_HANDED_Y_UP:
                    coordinateMatrix.m11 = -1;
                    break;
                case ZED_COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP:
                    coordinateMatrix.m11 = -1;
                    coordinateMatrix.m22 = -1;
                    break;
                case ZED_COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP:
                    coordinateMatrix.m11 = 0;
                    coordinateMatrix.m12 = -1;
                    coordinateMatrix.m21 = 1;
                    coordinateMatrix.m22 = 0;
                    break;
                case ZED_COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD:
                    coordinateMatrix.m00 = 0;
                    coordinateMatrix.m11 = 0;
                    coordinateMatrix.m22 = 0;
                    coordinateMatrix.m12 = -1;
                    coordinateMatrix.m01 = -1;
                    coordinateMatrix.m20 = 1;

                    break;
                case ZED_COORDINATE_SYSTEM.LEFT_HANDED_Z_UP:
                    coordinateMatrix.m00 = 0;
                    coordinateMatrix.m01 = 1;
                    coordinateMatrix.m10 = 0;
                    coordinateMatrix.m11 = 0;
                    coordinateMatrix.m12 = -1;
                    coordinateMatrix.m20 = 1;
                    coordinateMatrix.m21 = 0;
                    coordinateMatrix.m22 = 0;
                    break;
                default:
                    break;
            }
        }

        //Convert Transform to Unity coordinate frame (Left Y up)
        public static Vector3 ConvertToUnityCoordinateSystem(ZED_COORDINATE_SYSTEM InFrame, ZED_COORDINATE_UNIT InUnit, Vector3 InVector)
        {
            float UnitFactor = GetUnitFactor(InUnit);

            Matrix4x4 tmp = Matrix4x4.identity; 
            Matrix4x4 coordTransf = Matrix4x4.identity;

            GetCoordinateTransform(InFrame, ref tmp); //src 
            GetCoordinateTransform(ZED_COORDINATE_SYSTEM.LEFT_HANDED_Y_UP, ref coordTransf); // dst is unity coordinate system

            Matrix4x4 mat = (coordTransf.inverse * tmp);
            Vector3 res = mat.MultiplyPoint(InVector) * UnitFactor;

            return res;
        }

        public static Quaternion ConvertToUnityCoordinateSystem(ZED_COORDINATE_SYSTEM InFrame, Quaternion InQuat)
        {
            Matrix4x4 tmp = Matrix4x4.identity;
            Matrix4x4 coordTransf = Matrix4x4.identity;

            GetCoordinateTransform(InFrame, ref tmp); //src 
            GetCoordinateTransform(ZED_COORDINATE_SYSTEM.LEFT_HANDED_Y_UP, ref coordTransf); // dst is unity coordinate system

            Matrix4x4 mat = (coordTransf.inverse * tmp);

            if (!ZEDCommon.IsNull(InQuat))
            {
                Matrix4x4 transformationMatrix = Matrix4x4.TRS(Vector3.zero, InQuat, Vector3.one);
                Matrix4x4 resultMatrix = mat * transformationMatrix * mat.inverse;
                return resultMatrix.rotation;
            }
            else
            {
                return Quaternion.identity;
            }

        }
    }

    public enum CONNECTION_TYPE
    {
        UNICAST = 0,
        MULTICAST
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

    public enum ZED_COORDINATE_SYSTEM
    {
        IMAGE, /**< Standard coordinates system in computer vision.\n Used in OpenCV: see <a href="http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html">here</a>. */
        LEFT_HANDED_Y_UP, /**< Left-handed with Y up and Z forward.\n Used in Unity with DirectX. */
        RIGHT_HANDED_Y_UP, /**< Right-handed with Y pointing up and Z backward.\n Used in OpenGL. */
        RIGHT_HANDED_Z_UP, /**< Right-handed with Z pointing up and Y forward.\n Used in 3DSMax. */
        LEFT_HANDED_Z_UP, /**< Left-handed with Z axis pointing up and X forward.\n Used in Unreal Engine. */
        RIGHT_HANDED_Z_UP_X_FWD /**< Right-handed with Z pointing up and X forward.\n Used in ROS (REP 103). */
    };

    public enum ZED_COORDINATE_UNIT
    {
        MILLIMETER, /**< International System (1/1000 meters) */
        CENTIMETER, /**< International System (1/100 meters) */
        METER, /**< International System (1 meter)*/
        INCH, /**< Imperial Unit (1/12 feet) */
        FOOT, /**< Imperial Unit (1 foot)*/
    };

    public enum TRACKING_STATE
    {
        OFF, /**< The tracking is not yet initialized, the object ID is not usable */
        OK, /**< The object is tracked */
        SEARCHING,/**< The object couldn't be detected in the image and is potentially occluded, the trajectory is estimated */
        TERMINATE/**< This is the last searching state of the track, the track will be deleted in the next retreiveObject */
    };

    public enum ACTION_STATE
    {
        IDLE = 0, /**< The object is staying static. */
        MOVING = 1, /**< The object is moving. */
    };

    /// <summary>
    /// type of data : camera data, animation data, etc
    /// </summary>
    public enum LIVELINK_ROLE
    {
        TRANSFORM,
        CAMERA,
        ANIMATION
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

    public class LiveLinkCameraData
    {
        public int SerialNumber = -1;

        public int FrameID = -1;

        public ulong Timestamp = 0;

        public LIVELINK_ROLE Role = LIVELINK_ROLE.CAMERA;

        public ZED_COORDINATE_SYSTEM CoordinateSystem = ZED_COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP;

        public ZED_COORDINATE_UNIT CoordinateUnit = ZED_COORDINATE_UNIT.MILLIMETER;

        public Vector3 CameraPosition = Vector3.zero;

        public Quaternion CameraRotation = Quaternion.identity;
        public static LiveLinkCameraData CreateFromJSON(byte[] data)
        {
            return JsonConvert.DeserializeObject<LiveLinkCameraData>(Encoding.ASCII.GetString(data));
        }
    }

    public class LiveLinkBodyData
    {
        public int frame_id = -1;

        public ulong timestamp = 0;

        public LIVELINK_ROLE role = LIVELINK_ROLE.CAMERA;

        public BODY_FORMAT body_format;

        public bool is_new = false;

        public ZED_COORDINATE_SYSTEM coordinate_system = ZED_COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP;

        public ZED_COORDINATE_UNIT coordinate_unit = ZED_COORDINATE_UNIT.MILLIMETER;

        public int nb_bodies = 0;

        public TRACKING_STATE tracking_state = TRACKING_STATE.OFF;

        public ACTION_STATE action_state = ACTION_STATE.IDLE;

        public int id = -1;

        public Vector3 position = Vector3.zero;

        public float confidence = 0.0f;

        public float[] keypoint_confidence;

        public Vector3[] keypoint;

        public Vector3[] local_position_per_joint;

        public Quaternion[] local_orientation_per_joint;

        public Quaternion global_root_orientation;

        public Vector3 global_root_position;

        public static LiveLinkBodyData CreateFromJSON(byte[] data)
        {
            return JsonConvert.DeserializeObject<LiveLinkBodyData>(Encoding.ASCII.GetString(data));
        }
    }

    public class BodyData
    {
        /// <summary>
        /// Object identification number, used as a reference when tracking the object through the frames.
        /// </summary>
        public int id;
        /// <summary>
        ///  Object label, forwarded from \ref CustomBoxObjects when using DETECTION_MODEL.CUSTOM_BOX_OBJECTS
        /// </summary>
        public TRACKING_STATE tracking_state;
        public ACTION_STATE action_state;
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

        public static BodyData FromLiveLinkBodyData(LiveLinkBodyData In)
        {
            BodyData bodyData= new BodyData();

            bodyData.id = In.id;
            bodyData.tracking_state = In.tracking_state;
            bodyData.action_state = In.action_state;
            bodyData.confidence = In.confidence;
            bodyData.position = ZEDCommon.ConvertToUnityCoordinateSystem(In.coordinate_system, In.coordinate_unit, In.position);

            bodyData.keypoint = new Vector3[In.keypoint.Length];
            bodyData.local_position_per_joint = new Vector3[In.local_position_per_joint.Length];
            bodyData.local_orientation_per_joint = new Quaternion[In.local_orientation_per_joint.Length];
            for (int i = 0; i < In.keypoint.Length; i++)
            {
                bodyData.keypoint[i] = ZEDCommon.ConvertToUnityCoordinateSystem(In.coordinate_system, In.coordinate_unit, In.keypoint[i]);
                bodyData.local_position_per_joint[i] = ZEDCommon.ConvertToUnityCoordinateSystem(In.coordinate_system, In.coordinate_unit, In.local_position_per_joint[i]);

                bodyData.local_orientation_per_joint[i] = ZEDCommon.ConvertToUnityCoordinateSystem(In.coordinate_system, In.local_orientation_per_joint[i]);
            }
            bodyData.global_root_orientation = ZEDCommon.ConvertToUnityCoordinateSystem(In.coordinate_system, In.global_root_orientation);

            bodyData.keypoint_confidence = In.keypoint_confidence;

            return bodyData;
        }
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
        public bool is_new;
        /// <summary>
        /// Defines if the object is tracked
        /// </summary>
        public bool is_tracked;
        /// <summary>
        /// Array of objects 
        /// </summary>
        public BodyData[] body_list;
    };
}

