using UnityEngine;
using System.IO.Ports;

public class FlexTracker : MonoBehaviour
{
    [Header("통신 설정")]
    public string portName = "COM3";
    public int baudRate = 115200;
    private SerialPort serialPort;

    [Header("유니티 뼈대 연결")]
    // unity 손가락 관절 요소랑 연결할 변수
    public Transform middleProx;
    public Transform middleInter;
    public Transform middleDist;

    [Header("관절별 오일러 각도 설정")]
    // 손을 핀 상태, 주먹을 쥔 상태일 때의 관절별 로컬 회전 값 하드코딩
    public Vector3 proxOpen = new Vector3(-17.457f, -4.409f, 2.188f);
    public Vector3 proxClose = new Vector3(-83.893f, -60.222f, 59.486f);
    
    public Vector3 interOpen = new Vector3(6.167f, 0.571f, 2.676f);
    public Vector3 interClose = new Vector3(-74.392f, -170.14f, 170.064f);
    
    public Vector3 distOpen = new Vector3(-6.498f, -0.677f, 0.644f);
    public Vector3 distClose = new Vector3(-64.576f, -1.950f, 1.491f);

    // 쿼터니언 변수 선언
    private Quaternion qProxOpen, qProxClose;
    private Quaternion qInterOpen, qInterClose;
    private Quaternion qDistOpen, qDistClose;

    [Header("센서 캘리브레이션 값")]   
    // 초기 값을 0으로 세팅
    // 손을 편 상태로 J키, 주먹을 쥔 상태로 K키를 누르면 변수 값 갱신
    public float mcpMin = 0f; 
    public float mcpMax = 0f; 
    public float pipMin = 0f;
    public float pipMax = 0f;
    
    // 캘리브레이션 여부 변수
    private bool isMinCalibrated = false;
    private bool isMaxCalibrated = false;

    // 과거 값 80% + 현재 값 20% Fusion
    [Header("필터 비율 설정")]
    [Range(0.01f, 1f)]
    public float smoothingFactor = 0.2f;

    // 필터링된 값을 저장할 변수
    private float smoothedMCP = 0f;
    private float smoothedPIP = 0f;

    // 처음 들어오는 값은 과거의 값이 없어서 무시하기 위한 bool 변수
    private bool isFirstData = true;

    // 센서의 실시간 원시 데이터 저장용 변수
    private float currentRawMCP = 0f;   // 뿌리 관절 센서 값
    private float currentRawPIP = 0f;   // 중간 관절 센서 값

    void Start()
    {
        // 1. 오일러 각도를 쿼터니언 각도로 변환
        qProxOpen = Quaternion.Euler(proxOpen);
        qProxClose = Quaternion.Euler(proxClose);
        
        qInterOpen = Quaternion.Euler(interOpen);
        qInterClose = Quaternion.Euler(interClose);
        
        qDistOpen = Quaternion.Euler(distOpen);
        qDistClose = Quaternion.Euler(distClose);

        // 2. 시리얼 포트 열기
        try
        {
            serialPort = new SerialPort(portName, baudRate);
            serialPort.ReadTimeout = 50;    // 0.05초 동안 응답 없으면 넘어감
            serialPort.Open();
            Debug.Log("시리얼 통신 연결 성공: " + portName);
        }
        catch (System.Exception e)
        {
            Debug.LogError("시리얼 포트 열기 실패: " + e.Message);
        }
    }

    void Update()
    {
        // 1. 키보드 캘리브레이션 (J: 손을 핀 상태, K: 주먹을 쥔 상태)
        if (Input.GetKeyDown(KeyCode.J))
        {
            mcpMin = currentRawMCP;
            pipMin = currentRawPIP;
            isMinCalibrated = true;
            Debug.Log($"[캘리브레이션] 손 쫙 핌 (Min) 세팅 완료! - MCP: {mcpMin}, PIP: {pipMin}");
        }

        if (Input.GetKeyDown(KeyCode.K))
        {
            mcpMax = currentRawMCP;
            pipMax = currentRawPIP;
            isMaxCalibrated = true;
            Debug.Log($"[캘리브레이션] 주먹 쥠 (Max) 세팅 완료! - MCP: {mcpMax}, PIP: {pipMax}");
        }

        // 2. 센서 데이터 수신 및 비율 계산
        if (serialPort != null && serialPort.IsOpen)
        {
            try
            {
                // ESP32에서 전달된 로그 한 줄("1500,1600") 읽기
                string dataString = serialPort.ReadLine();
                string[] sensorValues = dataString.Split(',');

                if (sensorValues.Length == 2)
                {
                    // 변환 가능한 정상 값일 때만 시도
                    if (float.TryParse(sensorValues[0], out float parsedMCP) && 
                        float.TryParse(sensorValues[1], out float parsedPIP))
                    {
                        if (isFirstData)
                        {
                            // 첫 데이터는 과거 값이 없으니 날것의 센서 값 입력
                            smoothedMCP = parsedMCP;
                            smoothedPIP = parsedPIP;
                            isFirstData = false;
                        }
                        else
                        {
                            // 과거 값에서 현재 값으로 비율만큼 부드럽게 다가감
                            smoothedMCP = Mathf.Lerp(smoothedMCP, parsedMCP, smoothingFactor);
                            smoothedPIP = Mathf.Lerp(smoothedPIP, parsedPIP, smoothingFactor);
                        }

                        // 필터링된 센서 값을 계속 업데이트
                        currentRawMCP = smoothedMCP;   // 뿌리 관절
                        currentRawPIP = smoothedPIP;   // 중간 관절

                        // 구부러짐 비율 계산
                        if (isMinCalibrated && isMaxCalibrated)
                        {
                            float ratioMCP = 0f;
                            if (mcpMax - mcpMin != 0) 
                            {
                                ratioMCP = Mathf.Clamp01((currentRawMCP - mcpMin) / (mcpMax - mcpMin));
                            }

                            float ratioPIP = 0f;
                            if (pipMax - pipMin != 0)
                            {
                                ratioPIP = Mathf.Clamp01((currentRawPIP - pipMin) / (pipMax - pipMin));
                            }

                            // 손가락 회전 적용
                            if (middleProx != null)
                                middleProx.localRotation = Quaternion.Lerp(qProxOpen, qProxClose, ratioMCP);

                            if (middleInter != null)
                                middleInter.localRotation = Quaternion.Lerp(qInterOpen, qInterClose, ratioPIP);

                            if (middleDist != null)
                                middleDist.localRotation = Quaternion.Lerp(qDistOpen, qDistClose, ratioPIP);
                        }
                    }
                }
            }
            catch (System.TimeoutException)
            {
                // 데이터가 늦게 와도 에러 띄우지 않고 무시
            }
        }
    }

    // 유니티 종료할 때 포트 닫기
    void OnApplicationQuit()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
            Debug.Log("시리얼 포트 닫힘");
        }
    }
}