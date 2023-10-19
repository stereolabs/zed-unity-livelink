using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Linq;
using System;

public class ZEDStreamingClient : MonoBehaviour
{
    UdpClient clientData;

    IPEndPoint ipEndPointData;

    public bool useMulticast = true;

    public int port = 20000;
    public string multicastIpAddress = "230.0.0.1";

    public bool showZEDFusionMetrics = false;

    private object obj = null;
    private System.AsyncCallback AC;
    byte[] receivedBytes;

    bool newDataAvailable = false;
    sl.DetectionData data;

    public delegate void onNewDetectionTriggerDelegate(sl.Bodies bodies);
    public event onNewDetectionTriggerDelegate OnNewDetection;

    object mutex_buffer = new object();
    SortedDictionary<ulong, List<sl.DetectionData>> detectionDataDict = new SortedDictionary<ulong, List<sl.DetectionData>>();

    void Start()
    {
        InitializeUDPListener();
    }
    public void InitializeUDPListener()
    {
        ipEndPointData = new IPEndPoint(IPAddress.Any, port);
        clientData = new UdpClient();
        clientData.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, optionValue: true);
        clientData.ExclusiveAddressUse = false;
        clientData.EnableBroadcast = false;

        if (useMulticast)
        {
            clientData.JoinMulticastGroup(IPAddress.Parse(multicastIpAddress));

        }
        clientData.Client.Bind(ipEndPointData);

        clientData.DontFragment = true;
        AC = new System.AsyncCallback(ReceivedUDPPacket);
        clientData.BeginReceive(AC, obj);
        Debug.Log("UDP - Start Receiving..");
    }

    void ReceivedUDPPacket(System.IAsyncResult result)
    {
        receivedBytes = clientData.EndReceive(result, ref ipEndPointData);
        ParsePacket();
        clientData.BeginReceive(AC, obj);
    } // ReceiveCallBack

    // fill detectionDataDict with received data
    void ParsePacket()
    {
        sl.DetectionData d = sl.DetectionData.CreateFromJSON(receivedBytes);

        // initialize data if necessary
        if (data==null) { data = sl.DetectionData.CreateFromJSON(receivedBytes); }

        lock (mutex_buffer) // lock dictionary.
        {
            // add to data dictionary
            if (detectionDataDict.ContainsKey(d.bodies.timestamp)) // if key (timestamp) exists
            {
                detectionDataDict.GetValueOrDefault(d.bodies.timestamp).Add(d);
            }
            else // key (timestamp) does not exist yet
            {
                List<sl.DetectionData> tmpL = new List<sl.DetectionData> { d };
                detectionDataDict.Add(d.bodies.timestamp, tmpL);
            }
        }
    }

    // merge bodies data from dictionary into 1 bodies data
    public void PrepareData()
    {
        if(data!=null)
        {
            lock (mutex_buffer) // lock dictionary.
            {
                if (detectionDataDict.Count > 0)
                {
                    sl.Bodies bodies = new sl.Bodies();

                    // get oldest timestamp from dict
                    bodies.timestamp = detectionDataDict.First().Key;

                    // get a ref bodies
                    sl.Bodies refBodies = detectionDataDict.First().Value.First().bodies;

                    //initialize body_list
                    bodies.body_list = new sl.BodyData[detectionDataDict.First().Value.Count];

                    // fill bodies.body_list with list of detection data for said timestamp
                    for (int i = 0; i < bodies.body_list.Length; ++i)
                    {
                        bodies.body_list[i] = detectionDataDict.First().Value[i].bodies.body_list[0];
                    }

                    // fill additional data
                    data.fusionMetrics = detectionDataDict.First().Value.First().fusionMetrics;
                    bodies.body_format = refBodies.body_format;
                    bodies.is_tracked = refBodies.is_tracked;
                    bodies.nb_object = refBodies.nb_object;
                    bodies.is_new = refBodies.is_new;

                    data.bodies = bodies;

                    // remove from dictionary
                    detectionDataDict.Remove(bodies.timestamp);


                    newDataAvailable = true;
                }
                else
                {
                    data.bodies.is_new = 0;
                }
            }
        }        
    }

    public bool IsNewDataAvailable()
    {
        return newDataAvailable;
    }

    public sl.FusionMetrics GetLastFusionMetrics()
    {
        lock (mutex_buffer)
        {
            return data.fusionMetrics;
        }
    }
    public sl.Bodies GetLastBodies()
    {
        lock (mutex_buffer)
        {
            return data.bodies;
        }
    }

    public bool ShowFusionMetrics()
    {
        return showZEDFusionMetrics && data.fusionMetrics != null;
    }


    private void Update()
    {
        PrepareData();
        if (IsNewDataAvailable())
        {
            OnNewDetection(GetLastBodies());

            newDataAvailable = false;

            if (ShowFusionMetrics())
            {
                sl.FusionMetrics metrics = GetLastFusionMetrics();
                string tmpdbg = "";
                foreach (var camera in metrics.camera_individual_stats)
                {
                    tmpdbg += "SN : " + camera.sn + " Synced Latency: " + camera.synced_latency + "FPS : " + camera.received_fps + "\n";
                }
                Debug.Log(tmpdbg);
            }
        }
    }

    void OnDestroy()
    {
        if (clientData != null)
        {
            Debug.Log("Stop receiving ..");
            clientData.Close();
        }
    }
}