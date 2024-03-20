using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using UnityEngine;
using System.Linq;
using Newtonsoft.Json.Linq;
using System.Text;
using sl;

public class ZEDStreamingClient : MonoBehaviour
{
    UdpClient clientData;

    IPEndPoint ipEndPointData;

    public CONNECTION_TYPE ConnectionType = CONNECTION_TYPE.UNICAST;
    public string multicastIpAddress = "230.0.0.1";
    public int port = 20000;

    private object obj = null;
    private System.AsyncCallback AC;
    byte[] receivedBytes;

    bool newDataAvailable = false;
    sl.Bodies LastBodies;
    sl.LIVELINK_ROLE CurrentDataRole = sl.LIVELINK_ROLE.CAMERA;

    public delegate void onNewDetectionTriggerDelegate(sl.Bodies bodies);
    public event onNewDetectionTriggerDelegate OnNewDetection;

    object mutex_buffer = new object();
    SortedDictionary<ulong, List<sl.LiveLinkBodyData>> DataDict = new SortedDictionary<ulong, List<sl.LiveLinkBodyData>>();

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
        clientData.EnableBroadcast = true;

        if (ConnectionType == CONNECTION_TYPE.MULTICAST)
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
        sl.LIVELINK_ROLE Role = GetLiveLinkRole(receivedBytes);
        // initialize data if necessary
        if (Role == sl.LIVELINK_ROLE.ANIMATION) 
        { 
            sl.LiveLinkBodyData LiveLinkData = sl.LiveLinkBodyData.CreateFromJSON(receivedBytes);
            lock (mutex_buffer) // lock dictionary.
            {
                // add to data dictionary
                if (DataDict.ContainsKey(LiveLinkData.timestamp)) // if key (timestamp) exists
                {
                    DataDict.GetValueOrDefault(LiveLinkData.timestamp).Add(LiveLinkData);
                }
                else // key (timestamp) does not exist yet
                {
                    List<sl.LiveLinkBodyData> tmpL = new List<sl.LiveLinkBodyData> { LiveLinkData };
                    DataDict.Add(LiveLinkData.timestamp, tmpL);
                }
            }
        }


    }

    // merge bodies data from dictionary into 1 bodies data
    public void PrepareData()
    {
            lock (mutex_buffer) // lock dictionary.
            {
                if (DataDict.Count > 0)
                {
                    if (CurrentDataRole == sl.LIVELINK_ROLE.ANIMATION) 
                    {
                        sl.Bodies bodies = new sl.Bodies();
                        // get oldest timestamp from dict
                        bodies.timestamp = DataDict.First().Key;
                        // get a ref bodies
                        sl.LiveLinkBodyData refBodies = DataDict.First().Value.First();

                        //initialize body_list
                        bodies.body_list = new sl.BodyData[DataDict.First().Value.Count];

                        // fill bodies.body_list with list of detection data for said timestamp
                        for (int i = 0; i < bodies.body_list.Length; ++i)
                        {
                           bodies.body_list[i] = sl.BodyData.FromLiveLinkBodyData(DataDict.First().Value[i]);
                        }

                        // fill additional data
                        bodies.body_format = refBodies.body_format;
                        bodies.is_new = true;
                        bodies.is_tracked = true;
                        bodies.nb_object = bodies.body_list.Length;
                            
                        // remove from dictionary
                        DataDict.Remove(bodies.timestamp);
                        LastBodies = bodies;
                        newDataAvailable = true;
                    }
                }
                else
                {
                }
        }        
    }

    public bool IsNewDataAvailable()
    {
        return newDataAvailable;
    }

    public sl.Bodies GetLastBodies()
    {
        lock (mutex_buffer)
        {
            return LastBodies;
        }
    }

    public sl.LIVELINK_ROLE GetLiveLinkRole(byte[] receivedBytes)
    {
        sl.LIVELINK_ROLE Role = sl.LIVELINK_ROLE.CAMERA;

        JObject ZedData = JObject.Parse(Encoding.ASCII.GetString(receivedBytes));

        // get JSON result objects into a list
        Role = ZedData["role"].ToObject<sl.LIVELINK_ROLE>();

        CurrentDataRole = Role;
        return Role;
    }

    private void Update()
    {
        PrepareData();
        if (IsNewDataAvailable())
        {
            if (CurrentDataRole == sl.LIVELINK_ROLE.ANIMATION)
            {
                OnNewDetection(GetLastBodies());

                newDataAvailable = false;
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