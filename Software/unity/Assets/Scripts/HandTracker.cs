using UnityEngine;
using System.IO.Ports;
using System.Threading;

public class HandTracker : MonoBehaviour
{
    [Header("Serial Settings")]
    public string portName = "COM3";
    public int baudRate = 115200;

    [Header("받은 6축 값 (deg)")]
    public float baseDeg, axis2Deg, axis3Deg;
    public float pitchDeg, yawDeg, rollDeg;
    public bool hasData = false;

    [Header("C 패킷: 팔 3축 절대각 (deg)")]
    public float baseAbs, axis2Abs, axis3Abs;
    public bool hasCalib = false;     // C 도착 (ArmKinematics가 읽고 false로 리셋)

    [Header("상태")]
    public bool isSystemOn = false;   // q로 ON, w로 OFF

    private SerialPort serial;
    private Thread readThread;
    private volatile bool running = false;
    private volatile string latestLine = null;

    void Start()
    {
        try
        {
            serial = new SerialPort(portName, baudRate);
            serial.ReadTimeout = 50;
            serial.WriteTimeout = 50;
            serial.NewLine = "\n";
            serial.Open();
            running = true;
            readThread = new Thread(ReadLoop);
            readThread.Start();
            Debug.Log("[HandTracker] Serial opened: " + portName);
        }
        catch (System.Exception e)
        {
            Debug.LogError("[HandTracker] Serial open failed: " + e.Message);
        }
    }

    void ReadLoop()
    {
        while (running)
        {
            try { latestLine = serial.ReadLine(); }
            catch (System.TimeoutException) { }
            catch { }
        }
    }

    void Update()
    {
        // --- 수신 ---
        if (latestLine != null)
        {
            if (latestLine.StartsWith("C,"))      ParseCalib(latestLine);
            else if (isSystemOn)                  ParseLine(latestLine);
            latestLine = null;
        }

        // --- 송신 (키 명령) ---
        if (Input.GetKeyDown(KeyCode.H)) { isSystemOn = false; SendCommand("h"); }
        if (Input.GetKeyDown(KeyCode.J)) { isSystemOn = false; SendCommand("j"); }
        if (Input.GetKeyDown(KeyCode.K)) { SendCommand("k"); }
        if (Input.GetKeyDown(KeyCode.Q)) { isSystemOn = true;  SendCommand("q"); }
        if (Input.GetKeyDown(KeyCode.W))
        {
            isSystemOn = false;
            hasData = false;
            baseDeg = axis2Deg = axis3Deg = pitchDeg = yawDeg = rollDeg = 0f;
            SendCommand("w");
        }
    }

    // 키 명령 전송 (로그 있음)
    public void SendCommand(string cmd)
    {
        if (serial == null || !serial.IsOpen) return;
        try
        {
            serial.WriteLine(cmd);
            Debug.Log("[HandTracker] Sent: " + cmd);
        }
        catch (System.Exception e) { Debug.LogError("[HandTracker] Send failed: " + e.Message); }
    }

    // F 패킷 전송 (로그 없음, 50Hz라 도배 방지)
    public void SendForce(string cmd)
    {
        if (serial == null || !serial.IsOpen) return;
        try { serial.WriteLine(cmd); }
        catch { }
    }

    // P,base,axis2,axis3,pitch,yaw,roll (상대각)
    void ParseLine(string line)
    {
        if (!line.StartsWith("P,")) return;
        string[] p = line.Split(',');
        if (p.Length < 7) return;

        if (float.TryParse(p[1], out float b)  && float.TryParse(p[2], out float a2) &&
            float.TryParse(p[3], out float a3) && float.TryParse(p[4], out float pi) &&
            float.TryParse(p[5], out float y)  && float.TryParse(p[6], out float r))
        {
            baseDeg = b; axis2Deg = a2; axis3Deg = a3;
            pitchDeg = pi; yawDeg = y; rollDeg = r;
            hasData = true;
        }
    }

    // C,base,axis2,axis3 (팔 절대각, k 순간 1회)
    void ParseCalib(string line)
    {
        string[] p = line.Split(',');
        if (p.Length < 4) return;

        if (float.TryParse(p[1], out float b) &&
            float.TryParse(p[2], out float a2) &&
            float.TryParse(p[3], out float a3))
        {
            baseAbs = b; axis2Abs = a2; axis3Abs = a3;
            hasCalib = true;
            Debug.Log($"[HandTracker] CALIB: base={b:F1} a2={a2:F1} a3={a3:F1}");
        }
    }

    void OnDestroy()
    {
        running = false;
        if (readThread != null && readThread.IsAlive) readThread.Join(100);
        if (serial != null && serial.IsOpen) serial.Close();
    }
}