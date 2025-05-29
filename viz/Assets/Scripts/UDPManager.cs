using System;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using Unity.VisualScripting;
using UnityEditor.AssetImporters;
using UnityEngine;
using System.Text;
using System.Collections.Generic;

public class UDPManager : MonoBehaviour
{
    UdpClient udp;
    Thread thread;

    public GameObject executorObj;

    static readonly object lockObject = new object();
    List<string> returnData = new();

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        thread = new Thread(new ThreadStart(UDPThreadMethod));
        thread.Start();
        print("UDP thread started");
    }

    private void OnDestroy()
    {
        if (udp != null) 
        {
            udp.Close();
            udp = null;
        }
        if (thread != null) 
        {
            if (thread.Join(100))
            {
                print("UDP thread succesful termination");
            }
            else 
            {
                print("UDP thread did not terminate within 100ms, aborting");
                thread.Abort();
            }
        }


    }


    // Update is called once per frame
    void Update()
    {
        if (returnData.Count != 0) 
        {
            lock (lockObject) 
            {
                foreach (string s in returnData)
                {
                    //print("Main thread has recieved: " + s.ToString());
                    executorObj.GetComponent<CommandExecutor>().addCommandToParse(s);
                    
                }
                returnData.Clear();
            }
        }
    }

    void UDPThreadMethod() 
    {
        udp = new UdpClient(35891);
        Byte[] recieveBytes = new byte[0];
        string recievedText = "";
        IPEndPoint RemoteIPEndPoint = new IPEndPoint(IPAddress.Any, 0);
        while (udp != null) 
        {
            try 
            {
                recievedText = "";
                recieveBytes = udp.Receive(ref RemoteIPEndPoint);
                recievedText = Encoding.UTF8.GetString(recieveBytes);
                lock (lockObject) 
                {
                    returnData.Add(recievedText);
                    //print("recieved text: " + recievedText.ToString());
                }

            }
            catch (Exception ex) 
            {
                if (udp != null) 
                {
                    Debug.Log("UDP Client Socket Exception Error: " + ex);
                }
                else 
                {
                    Debug.Log("Thread terminating");
                }
            }

        }

    }

}
