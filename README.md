# Stereolabs ZED - Live Link for Unity
ZED LiveLink Plugin for Unity

## ZED Live Link for Unity

This tool is an interface between the SDK Fusion module and Unity. It sends fused skeleton data to the engine so that 3D avatars can be animated in Unity using one or several ZED cameras working together.

The data is broadcast via UDP on a specified port, and can be received using the assets of the paired Unity project.

## Compatibility

> **Current version**: 4.2.0.
>
> **Compatible ZED SDK Versions**: 4.0.5 - 4.2.0

### Dependencies

- **ZED SDK v4.x**, available on [stereolabs.com](https://www.stereolabs.com/developers/)
- **Unity 2021.3** and more recent versions, available on [unity.com](https://unity.com/download)
    - It will probably work well with older versions, as its core is an UDP receiver which animates an Humanoid avatar, but has not been tested with them.
    - You may encounter an error about the package **Newtonsoft** not being installed at first launch. With versions 2022.x and more recent of Unity, it should not happen, but in any case please refer to [the Newtonsoft repo](https://github.com/jilleJr/Newtonsoft.Json-for-Unity/wiki/Install-official-via-UPM) for installation instructions with Unity Package Manager.
- **Universal Render Pipeline (URP)**: The Unity project is built on URP, however this is not blocking if you want to use another render pipeline as there is no deep shader compatibility issues, just some materials to update.

### Difference with the ZED Unity Plugin

The ZED Unity plugin, available [here](https://github.com/stereolabs/zed-unity), is a full integration of the SDK features in Unity, except for the multi-camera data fusion. It comes as an Unity Package, allowing dialogue between your project and all the ZED SDK capabilities. 

This sample comes as an alternative and a complement, allowing your Unity project to receive only **Body Tracking data**, from one or several cameras. The Unity project provided does not implement any dialogue with the SDK, only being able to receive data from the sender tools (not the SDK directly).

## Content

### Sender tools

The `ZED Unity Live Link` and `ZED Unity Live Link Fusion` projects are the C++ samples responsible for sending the data from the SDK to Unity via UDP.

- The `ZED Unity Live Link Mono` sample directly calls the SDK and sends only the Body Tracking data to Unity.

- The `ZED Unity Live Link Fusion` sample uses the Fusion module of the SDK to combine the Body Tracking data from several cameras, via a direct USB connection or ZEDHub.

### Unity project

The `Live Link for Unity` project comes with some scripts to receive skeleton data, a sample scene to discover it and a 3D avatar to populate said scene.

Scenes :
- *Skeleton Fusion* : main scene, simple space showcasing the necessary elements to receive skeleton data from the sender and animate some 3D avatars.
- *Upper Body Tracking* : Implementation of a way to use only the upper part of the body (from the spine and above) to animate the avatar.

Main scripts :
- *ZEDStreamingClient* : Manage the reception of data from the sender.
- *ZEDCommon* : Defines the structure of the body data received and used in the sample.
- *ZEDSkeletonTrackingManager* : Manages the display of the 3D avatars by processing the data received by the `ZEDStreamingClient` at each reception.
- *SkeletonHandler* : Main script to manage the "data" part of each body detected.
- *ZEDSkeletonAnimator* : Script to manage the animation of each avatar, using the `SkeletonHandler` data.

Main prefabs :
- *FusionManager* : The main manager, with the `ZEDStreamingClient` and `ZEDSkeletonTrackingManager` components attached.
- *Unity_Avatar* : Sample avatar using Unity's [Starter Assets](https://assetstore.unity.com/packages/essentials/starter-assets-third-person-character-controller-196526) 3D humanoid model.

## Using the Live Link for Unity tools

### Quick Start

- First, download and install the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/) (**Minimum requirement: ZED SDK v4.0**).
    - For more information, read the ZED [Documentation](https://www.stereolabs.com/docs) and [API documentation](https://www.stereolabs.com/docs/api/)
- Download the C++ samples and the Unity project (or unitypackage) on the Releases page on this repo. You can also directly pull the `main` branch of the repo for an up-to-date version.
- Generate either or both of the projects (`fusion` or `mono`) using CMake.
- Open the `main.cpp` file and set the `servAddress` and `servPort` variables with your desired address and port.
    - The default values are `230.0.0.1` for the IP and `20001` for the port.

```c++
    // ----------------------------------
    // UDP to Unity----------------------
    // ----------------------------------
    std::string servAddress;
    unsigned short servPort;
    UDPSocket sock;

    sock.setMulticastTTL(1);

    servAddress = "230.0.0.1";
    servPort = 20001;

    std::cout << "Sending fused data at " << servAddress << ":" << servPort << std::endl;
    // ----------------------------------
    // UDP to Unity----------------------
    // ----------------------------------
```

- Build the sample and execute it, passing your calibration file generated by ZED360 as argument, if you're using the Fusion sender, or either nothing or your SVO file for the mono-camera sender.

> Note: The sender needs to be built in **Release** configuration. Visual Studio may open it in **Debug** mode by default. Please check the Solution Configuration in Visual Studio before building.

```
> path/to/the/ZED_Sender_Fusion.exe path/to/the/calib_file.json
```
```
> path/to/the/ZED_Sender_Monocam.exe [path/to/the/file.svo]
```
- Import the Unity package in your project or just open the project provided on this repo.
- Check that the `Port` and `Multicast IP Address` variables of the `ZED Streaming Client` script on the `Fusion Manager` prefab are set to the same values as in the `main.cpp` set previously.

![zedstreamclient](https://user-images.githubusercontent.com/113181784/228796267-3901e7aa-842b-4453-bda6-0461e0b27552.jpg)

- Run the scene and you should see avatars moving in Unity !

### Main settings in Unity

These are located on the `ZED Skeleton Tracking Manager` script in the `Fusion Manager` prefab.

![sedskmanager](https://user-images.githubusercontent.com/113181784/228796295-655becda-8b87-47a0-be5a-3e9f5ed69f55.jpg)

- `Enable Avatar` / `Enable SDK Skeleton` : controls the visibility of the 3D avatar and of the skeleton directly derived from the keypoints of the SDK.
- `Avatars` : Array of 3D avatars randomly chosen when detecting a new person.
    - All of Unity **Humanoid** avatars should be compatible, provided they are made into a prefab with a `ZEDSkeletonAnimator` component attached.
- `Enable SDK Skeleton` : controls the visibility of the stickman view of the SDK keypoints. Whereas the 3D avatar is animated using local rotations derived from the keypoints, this show the actual positions of the keypoints detected by the SDK.
- `Log Fusion Metrics` : enables logging the metrics sent by the Fusion module in the console.

### Troubleshooting

If no skeleton data is received in Unity, you can either try to :

- Disable your firewall.
- Change the port used to send the data, it might already be used by another process.

## Support
You will find guidance and assistance :
- In the [documentation of the sample](https://www.stereolabs.com/docs/livelink/livelink-unity/)
- On our [Community forums](https://community.stereolabs.com/)

## Bugs and fixes
You found a bug / a flaw in our plugin ? Please check that it is not already reported, and open an issue if necessary. You can also reach out to us on the community forums for any question or feedback !

*By the way, we also have a special place in our hearts for PR senders.*
