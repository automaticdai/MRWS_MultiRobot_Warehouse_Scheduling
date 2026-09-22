using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Listens for the simulator and hands its messages to the CommandExecutor on
/// the main thread, which is the only place Unity objects may be touched.
///
/// This was UDP, and UDP lost messages: the simulator emits thousands of small
/// datagrams with no flow control, so the receive buffer overflowed and the
/// kernel silently discarded the excess. The viewer had no way to know it had
/// missed a MOVEROBOT or an ORDERCOMPLETE, so it quietly desynced. TCP gives
/// the simulator backpressure instead -- it waits for us rather than losing
/// what we could not take.
/// </summary>
public class TcpManager : MonoBehaviour
{
    public const int ListenPort = 35891;

    public GameObject executorObj;

    TcpListener listener;
    Thread thread;
    volatile bool listening;

    static readonly object lockObject = new object();
    readonly List<string> returnData = new();

    void Start()
    {
        listening = true;
        thread = new Thread(ListenLoop)
        {
            // A background thread cannot keep the process alive, so leaving play
            // mode can never hang on it. The previous code relied on
            // Thread.Abort, which throws PlatformNotSupportedException on the
            // .NET runtime Unity 6 uses.
            IsBackground = true,
            Name = "MRWS simulator listener"
        };
        thread.Start();
    }

    void OnDestroy()
    {
        listening = false;

        // Stopping the listener makes the blocking Accept throw, which is how
        // the loop is woken up to notice that it should stop.
        if (listener != null)
        {
            try
            {
                listener.Stop();
            }
            catch (SocketException)
            {
                // Already torn down.
            }
            listener = null;
        }

        if (thread != null && !thread.Join(200))
        {
            Debug.LogWarning("Simulator listener did not stop within 200ms; it is a background thread and will be reclaimed.");
        }
        thread = null;
    }

    void Update()
    {
        if (returnData.Count == 0)
        {
            return;
        }

        CommandExecutor executor = executorObj.GetComponent<CommandExecutor>();
        lock (lockObject)
        {
            foreach (string message in returnData)
            {
                executor.addCommandToParse(message);
            }
            returnData.Clear();
        }
    }

    void ListenLoop()
    {
        try
        {
            listener = new TcpListener(IPAddress.Loopback, ListenPort);
            listener.Start();
        }
        catch (SocketException ex)
        {
            Debug.LogError("Could not listen on TCP port " + ListenPort + ": " + ex.Message);
            return;
        }

        Debug.Log("Waiting for simulator on TCP port " + ListenPort);

        while (listening)
        {
            try
            {
                // One simulator at a time; when a run ends and disconnects, go
                // back to waiting so the next run is picked up without a
                // restart.
                using (TcpClient client = listener.AcceptTcpClient())
                {
                    client.NoDelay = true;
                    ReadMessages(client);
                }
            }
            catch (Exception ex)
            {
                // Expected once the listener is stopped on shutdown.
                if (listening)
                {
                    Debug.LogError("Simulator listener failed: " + ex.Message);
                }
                return;
            }
        }
    }

    /// <summary>
    /// Read newline-framed JSON until the simulator disconnects.
    ///
    /// TCP is a stream, not a sequence of messages: one read can return half a
    /// message, or three and a half. Anything after the last newline is kept
    /// and completed by the following read.
    /// </summary>
    void ReadMessages(TcpClient client)
    {
        NetworkStream stream = client.GetStream();
        byte[] buffer = new byte[8192];
        StringBuilder pending = new StringBuilder();

        while (listening)
        {
            int read = stream.Read(buffer, 0, buffer.Length);
            if (read <= 0)
            {
                break; // simulator closed the connection
            }

            pending.Append(Encoding.UTF8.GetString(buffer, 0, read));

            string content = pending.ToString();
            int newline;
            int consumed = 0;
            List<string> complete = new List<string>();

            while ((newline = content.IndexOf('\n', consumed)) >= 0)
            {
                string message = content.Substring(consumed, newline - consumed).Trim();
                if (message.Length > 0)
                {
                    complete.Add(message);
                }
                consumed = newline + 1;
            }

            pending.Remove(0, consumed);

            if (complete.Count > 0)
            {
                lock (lockObject)
                {
                    returnData.AddRange(complete);
                }
            }
        }
    }
}
