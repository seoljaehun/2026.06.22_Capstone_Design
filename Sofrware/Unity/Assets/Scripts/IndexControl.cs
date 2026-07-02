using UnityEngine;
using System.IO.Ports;

public class IndexTracker : MonoBehaviour
{
    [Header("통신 설정")]
    public string portName = "COM3";
    public int baudRate = 500000;
    private SerialPort serialPort;

    [Header("유니티 뼈대 연결")]
    public Transform indexProx;  // 뿌리 관절 (MCP)
    public Transform indexInter; // 중간 관절 (PIP)
    public Transform indexDist;  // 끝 관절 (DIP)

    [Header("관절 오일러 각도 설정 (하드코딩)")]
    public Vector3 proxOpen = new Vector3(-7.843f, -3.073f, 29.806f);
    public Vector3 interOpen = new Vector3(1.483f, -0.726f, 1.429f);
    public Vector3 distOpen = new Vector3(-6.734f, 0.353f, -0.974f);
    
    public Vector3 proxClose = new Vector3(-67.521f, -60.344f, 63.975f);
    public Vector3 interClose = new Vector3(-73.230f, 172.581f, -172.958f);
    public Vector3 distClose = new Vector3(-68.662f, 2.715f, -2.658f);
    
    public Vector3 proxSpread = new Vector3(-7.843f, -3.073f, 8.888f);

    private Quaternion qProxOpen, qProxClose, qProxSpread;
    private Quaternion qInterOpen, qInterClose;
    private Quaternion qDistOpen, qDistClose;
    private Quaternion invProxOpen;

    [Header("센서 실시간 원시 데이터(Raw)")]
    public float rawSplay = 0f;     // C4: 뿌리 벌어짐
    public float rawProxFlex = 0f;  // C5: 뿌리 굽힘
    public float rawInterFlex = 0f; // C6: 중간 굽힘

    [Header("캘리브레이션 저장 값")]
    private float interFlexMin = 0f, proxFlexMin = 0f, splayCenter = 0f; 
    private float interFlexMax = 0f, proxFlexMax = 0f;
    private float splayMax = 0f;

    private bool isCalibJ = false, isCalibK = false, isCalibL = false;

    void Start()
    {
        qProxOpen = Quaternion.Euler(proxOpen);
        qProxClose = Quaternion.Euler(proxClose);
        qProxSpread = Quaternion.Euler(proxSpread);

        qInterOpen = Quaternion.Euler(interOpen);
        qInterClose = Quaternion.Euler(interClose);

        qDistOpen = Quaternion.Euler(distOpen);
        qDistClose = Quaternion.Euler(distClose);

        invProxOpen = Quaternion.Inverse(qProxOpen);

        try
        {
            serialPort = new SerialPort(portName, baudRate);
            serialPort.ReadTimeout = 50; 
            serialPort.Open();
            Debug.Log($"시리얼 포트 {portName} 연결 성공");
        }
        catch (System.Exception e) { Debug.LogError("통신 에러: " + e.Message); }
    }

    void Update()
    {
        ReceiveSerialData();
        HandleCalibration();
        ApplyPureKinematics();
    }

    private void ReceiveSerialData()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            try
            {
                string data = serialPort.ReadLine().Trim();
                string[] values = data.Split(',');

                // ESP32에서 15개의 데이터를 보내므로 최소 15개 이상인지 확인
                if (values.Length >= 15)
                {
                    // 매핑 수정: 배열 인덱스는 0부터 시작
                    // C4 -> index 3 (Splay)
                    // C5 -> index 4 (Proximal Flexion)
                    // C6 -> index 5 (Intermediate Flexion)
                    float.TryParse(values[3], out rawSplay);
                    float.TryParse(values[4], out rawProxFlex);
                    float.TryParse(values[5], out rawInterFlex);
                }
            }
            catch (System.TimeoutException) { }
            catch (System.Exception e) { Debug.LogWarning("데이터 파싱 에러: " + e.Message); }
        }
    }

    private void HandleCalibration()
    {
        // J키: 모두 쫙 핀 상태 (영점)
        if (Input.GetKeyDown(KeyCode.J))
        {
            interFlexMin = rawInterFlex;
            proxFlexMin = rawProxFlex;
            splayCenter = rawSplay;
            isCalibJ = true;
            Debug.Log("Calibration: 쫙 핌 완료");
        }

        // K키: 주먹 쥔 상태 (굽힘 Max)
        if (Input.GetKeyDown(KeyCode.K))
        {
            interFlexMax = rawInterFlex;
            proxFlexMax = rawProxFlex;
            isCalibK = true;
            Debug.Log("Calibration: 주먹 쥠 완료");
        }

        // L키: 쫙 벌린 상태 (벌어짐 Max)
        if (Input.GetKeyDown(KeyCode.L))
        {
            splayMax = rawSplay;
            isCalibL = true;
            Debug.Log("Calibration: 벌리기 완료");
        }
    }

    private void ApplyPureKinematics()
    {
        // 모든 캘리브레이션 단계가 완료되어야 작동
        if (indexProx == null || !isCalibJ || !isCalibK || !isCalibL) return;

        // 0으로 나누기 방지
        if (Mathf.Abs(interFlexMax - interFlexMin) < 0.1f || 
            Mathf.Abs(proxFlexMax - proxFlexMin) < 0.1f || 
            Mathf.Abs(splayMax - splayCenter) < 0.1f) return;

        // 센서값 비율 변환 (0.0 ~ 1.0)
        float interRatio = Mathf.Clamp01((rawInterFlex - interFlexMin) / (interFlexMax - interFlexMin));
        float proxRatio  = Mathf.Clamp01((rawProxFlex - proxFlexMin) / (proxFlexMax - proxFlexMin));
        float splayRatio = Mathf.Clamp01((rawSplay - splayCenter) / (splayMax - splayCenter));

        // 뼈대 회전 적용
        indexInter.localRotation = Quaternion.Slerp(qInterOpen, qInterClose, interRatio);
        indexDist.localRotation = Quaternion.Slerp(qDistOpen, qDistClose, interRatio); // 끝마디는 중간마디에 연동

        Quaternion targetFlex = Quaternion.Slerp(qProxOpen, qProxClose, proxRatio);
        Quaternion targetSplay = Quaternion.Slerp(qProxOpen, qProxSpread, splayRatio);

        Quaternion deltaFlex = targetFlex * invProxOpen;
        Quaternion deltaSplay = targetSplay * invProxOpen;

        indexProx.localRotation = deltaSplay * deltaFlex * qProxOpen;
    }

    void OnApplicationQuit()
    {
        if (serialPort != null && serialPort.IsOpen) serialPort.Close();
    }
}