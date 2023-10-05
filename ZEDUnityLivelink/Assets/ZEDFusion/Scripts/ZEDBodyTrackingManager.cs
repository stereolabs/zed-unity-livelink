//======= Copyright (c) Stereolabs Corporation, All rights reserved. ===============

using UnityEngine;
using System.Collections.Generic;
using sl;
using System.Collections;
using System;

/// <summary>
/// 
/// </summary>
[DisallowMultipleComponent]
public class ZEDBodyTrackingManager : MonoBehaviour
{
    public enum BODY_MODE
    {
        FULL_BODY = 0,
        UPPER_BODY = 1
    }

    #region vars
    /// <summary>
    /// Vizualisation mode. Use a 3D model or only display the skeleton
    /// </summary>
    [Header("Vizualisation Mode")]
    /// <summary>
    /// Display 3D avatar. If set to false, only display bones and joint
    /// </summary>
    [Tooltip("Display 3D avatar or not.")]
    public bool enableAvatar = true;

    /// <summary>
    /// Maximum number of detection displayed in the scene.
    /// </summary>
    [Tooltip("Maximum number of detections spawnable in the scene")]
    public int maximumNumberOfDetections = 75;

    [Space(5)]
    [Header("------ Avatar Controls ------")]
    /// <summary>
    /// Avatar game objects
    /// </summary>
    [Tooltip("3D Rigged model.")]
    public GameObject[] avatars;
    public Material skeletonBaseMaterial;
    [Tooltip("Display bones and joints along 3D avatar")]
    [SerializeField]
    private bool displaySDKSkeleton = false;
    public static bool DisplaySDKSkeleton = false;
    [SerializeField]
    private Vector3 offsetSDKSkeleton = new Vector3(0f, 0f, 0f);
    public static Vector3 OffsetSDKSkeleton = new Vector3(0f, 0f, 0f);
    [Tooltip("Mirror the animation.")]
    public bool mirrorMode;
    [Tooltip("Which body mode to use: \nFULL_BODY uses the root position to move the avatar and the local rotations to animate all the limbs." +
        "\nUPPER_BODY uses the navigation system and animates the legs to match the movement in space, and animates the body from the hips and above with the local rotations from the ZED SDK.")]
    public BODY_MODE bodyMode = BODY_MODE.FULL_BODY;

    [Space(10)]
    [Header("------ Heigh Offset ------")]
    [Tooltip("Height offset applied to transform each frame.")]
    public Vector3 manualOffset = Vector3.zero;
    [Tooltip("Automatic offset adjustment: Finds an automatic offset that sets both feet above ground, and at least one foot on the ground.")]
    public bool automaticOffset = false;
    [Tooltip("Step in manual increase/decrease of offset.")]
    public float offsetStep = 0.1f;

    [Space(5)]
    [Header("------ Animation Smoothing ------")]
    [Tooltip("Animation smoothing setting. 0 = No latency, no smoothing. 1 = Maximum latency.\n Tweak this value depending on your framerate, and the fps of the camera."), Range(0f, 1f)]
    public float smoothingValue = .2f;
    [SerializeField]
    [Tooltip("Enable animation smoothing or not (induces latency).")]
    private bool enableSmoothing = true;

    [Space(5)]
    [Header("------ Experimental - IK Settings ------")]
    [Tooltip("Enable foot IK (feet on ground when near it)")]
    [SerializeField]
    private bool enableFootIK = false;
    [SerializeField]
    [Tooltip("Enable animation smoothing or not (induces latency).")]
    private bool enableFootLocking = true;
    [Tooltip("Foot locking smoothing setting. 0 = No latency, no smoothing. 1 = \"Full latency\" so no movement.\n Tweak this value depending on your framerate, and the fps of the camera.\nValues closer to 1 induce more latency, but improve fluidity."), Range(0f, 1f)]
    public float footLockingSmoothingValue = .8f;

    [Tooltip("Delay before spawning an avatar. At the end, the tracking state is checked, and if the filters pass then the avatar is spawned.\n" +
    "Use this to reduce popups of avatars in case of partial occlusion/tracked people at the edges of the ZED's frustum.")]
    public float delayBeforeSpawn = 0.5f;

    [Space(5)]
    [Header("------ Keyboard mapping ------")]
    public KeyCode toggleFootIK = KeyCode.I;
    public KeyCode toggleFootLock = KeyCode.F;
    public KeyCode toggleMirrorMode = KeyCode.M;
    public KeyCode toggleAutomaticHeightOffset = KeyCode.O;
    public KeyCode increaseOffsetKey = KeyCode.UpArrow;
    public KeyCode decreaseOffsetKey = KeyCode.DownArrow;
    public KeyCode increaseSkeOffsetXKey = KeyCode.LeftArrow;
    public KeyCode decreaseSkeOffsetXKey = KeyCode.RightArrow;

    public Dictionary<int,SkeletonHandler> avatarControlList;
    public ZEDStreamingClient zedStreamingClient;
    public bool EnableSmoothing { get => enableSmoothing; set => enableSmoothing = value; }
    public bool EnableFootIK { get => enableFootIK; set => enableFootIK = value; }
    public bool EnableFootLocking { get => enableFootLocking; set => enableFootLocking = value; }
    #endregion

    /// <summary>
    /// Start this instance.
    /// </summary>
    private void Start()
    {
        QualitySettings.vSyncCount = 1; // Activate vsync

        avatarControlList = new Dictionary<int,SkeletonHandler> ();
        if (!zedStreamingClient)
        {
            zedStreamingClient = FindObjectOfType<ZEDStreamingClient>();
        }

        zedStreamingClient.OnNewDetection += UpdateSkeletonData;
    }

	private void OnDestroy()
    {
        if (zedStreamingClient)
        {
            zedStreamingClient.OnNewDetection -= UpdateSkeletonData;
        }
    }

    /// <summary>
    /// Coroutine to spawn an avatar only after it's been detected for a set duration.
    /// </summary>
    /// <returns></returns>
    IEnumerator InstanciateAvatarWithDelay(float delay, BodyData dbody, BODY_FORMAT bodyFormat)
    {
        SkeletonHandler handler = ScriptableObject.CreateInstance<SkeletonHandler>();
        handler.Create(avatars[UnityEngine.Random.Range(0, avatars.Length)], bodyFormat);
        handler.InitSkeleton(dbody.id, new Material(skeletonBaseMaterial));

        avatarControlList.Add(dbody.id, handler);
        float unixTime = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        UpdateAvatarControl(handler, dbody);
        handler.GetAnimator().gameObject.SetActive(false);
        yield return new WaitForSeconds(delay);

        if (avatarControlList.ContainsKey(dbody.id)) handler.zedSkeletonAnimator.canSpawn = true;
    }


    /// <summary>
    /// Updates the skeleton data from ZEDCamera call and send it to Skeleton Handler script.
    /// </summary>
    private void UpdateSkeletonData(sl.Bodies bodies)
    {
        List<int> remainingKeyList = new List<int>(avatarControlList.Keys);
        List<sl.BodyData> newBodies = new List<sl.BodyData>(bodies.body_list);

        DateTimeOffset now = DateTimeOffset.UtcNow;
        float unixTime = now.ToUnixTimeMilliseconds();

        foreach (sl.BodyData bodyData in newBodies)
        {
            int person_id = bodyData.id;

            if (bodyData.tracking_state == sl.OBJECT_TRACK_STATE.OK)
            {
                //Avatar controller already exist --> update position
                if (avatarControlList.ContainsKey(person_id))
                {
                    // remove keys from list
                    remainingKeyList.Remove(person_id);
                    SkeletonHandler handler = avatarControlList[person_id];
                    UpdateAvatarControl(handler, bodyData);
                }
                else
                {
                    if (avatarControlList.Count < maximumNumberOfDetections)
                    {
                        StartCoroutine(InstanciateAvatarWithDelay(delayBeforeSpawn, bodyData, bodies.body_format));
                    }
                }
            }
            else if (bodyData.tracking_state == OBJECT_TRACK_STATE.TERMINATE) // detection lost, destroy gameobject.
            {
                if (avatarControlList.ContainsKey(person_id))
                {
                    SkeletonHandler handler = avatarControlList[person_id];
                    handler.Destroy();
                    avatarControlList.Remove(person_id);
                }
            }
        }

        /*foreach (int index in remainingKeyList)
		{
			SkeletonHandler handler = avatarControlList[index];
			handler.Destroy();
			avatarControlList.Remove(index);
            avatarLastSeenTime.Remove(index);

        }*/
    }

	public void Update()
	{
        DisplaySDKSkeleton = displaySDKSkeleton;
        OffsetSDKSkeleton = offsetSDKSkeleton;

            //if (Input.GetKeyDown(KeyCode.Space))
            //{
            //    useAvatar = !useAvatar;
            //}

            if (Input.GetKeyDown(KeyCode.Keypad0))
            {
                displaySDKSkeleton = !displaySDKSkeleton;
            }

            if (Input.GetKeyDown(toggleFootIK))
            {
                enableFootIK = !enableFootIK;
            }

            if (Input.GetKeyDown(toggleFootLock))
            {
                enableFootLocking = !enableFootLocking;
            }

            if (Input.GetKeyDown(toggleMirrorMode))
            {
                mirrorMode = !mirrorMode;
            }

            if (Input.GetKeyDown(increaseOffsetKey))
            {
                manualOffset.y += offsetStep;
            }
            else if (Input.GetKeyDown(decreaseOffsetKey))
            {
                manualOffset.y -= offsetStep;
            }
            else if (Input.GetKeyDown(increaseSkeOffsetXKey))
            {
                offsetSDKSkeleton.x += offsetStep;
            }
            else if (Input.GetKeyDown(decreaseSkeOffsetXKey))
            {
                offsetSDKSkeleton.x -= offsetStep;
            }
            if (Input.GetKeyDown(toggleAutomaticHeightOffset))
            {
                automaticOffset = !automaticOffset;
            }


        // Display avatars or not depending on useAvatar setting.
        foreach (var skelet in avatarControlList)
        {
            skelet.Value.zedSkeletonAnimator.TryShowAvatar(enableAvatar);
        }
    }

    /// <summary>
    /// Function to update avatar control with data from ZED SDK.
    /// </summary>
    /// <param name="handler">Handler.</param>
    /// <param name="data">Body tracking data.</param>
    private void UpdateAvatarControl(SkeletonHandler handler, sl.BodyData data)
    {
        Vector3[] worldJointsPos = new Vector3[handler.currentKeypointsCount];
        Quaternion[] normalizedLocalJointsRot = new Quaternion[handler.currentKeypointsCount];

        for (int i = 0; i < worldJointsPos.Length; i++)
        {
            worldJointsPos[i] = data.keypoint[i];
            normalizedLocalJointsRot[i] = data.local_orientation_per_joint[i].normalized;
        }
        Quaternion worldGlobalRotation = data.global_root_orientation;

        if (data.local_orientation_per_joint.Length > 0 && data.keypoint.Length > 0 && data.keypoint_confidence.Length > 0)
        {
            handler.SetConfidences(data.keypoint_confidence);
            handler.SetControlWithJointPosition(
                worldJointsPos,
                normalizedLocalJointsRot, worldGlobalRotation,
                enableAvatar, mirrorMode);
            if (enableFootLocking)
            {
                handler.CheckFootLockAnimator();
            }
            // For Upper-body mode
            handler.rootVelocity = data.velocity;
        }
    }
}
