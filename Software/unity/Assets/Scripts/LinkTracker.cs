using UnityEngine;
using System.IO.Ports;

[System.Serializable]
public class FingerData
{
    public string fingerName;
    
    [Header("유니티 뼈대 연결")]
    public Transform prox;  // 일반: 뿌리, 엄지: CMC(손바닥 안쪽)
    public Transform inter; // 일반: 중간, 엄지: MCP(뿌리)
    public Transform dist;  // 일반: 끝,   엄지: IP(끝)

    [Header("굽힘(Flex) 오일러 각도")]
    public Vector3 proxOpen;
    public Vector3 interOpen;
    public Vector3 distOpen;
    
    public Vector3 proxClose;
    public Vector3 interClose;
    public Vector3 distClose;
    
    [Header("벌어짐(Splay) 오일러 각도")]
    public Vector3 proxSplayMin;
    public Vector3 proxSplayMax;
    public Vector3 interSplayMax;

    [Header("가로지름(Oppose) 오일러 각도")]
    public Vector3 proxOpposeMax;
    public Vector3 interOpposeMax;

    [Header("벌어짐+가로지름(Splay+Oppose) 융합 오일러 각도")]
    public Vector3 proxSplayOpposeMax;
    public Vector3 interSplayOpposeMax;

    [Header("센서 방향 반전 설정")]
    public bool invertProxFlex = false;
    public bool invertInterFlex = false;
    public bool invertDistFlex = false;
    public bool invertSplay = false;
    public bool invertOppose = false;

    // 내부 계산용 쿼터니언 변수
    [HideInInspector] public Quaternion qProxOpen, qProxClose;
    [HideInInspector] public Quaternion qInterOpen, qInterClose;
    [HideInInspector] public Quaternion qDistOpen, qDistClose;
    [HideInInspector] public Quaternion qProxSplayMin, qProxSplayMax, invProxOpen, invInterOpen;
    [HideInInspector] public Quaternion qInterSplayMax;
    [HideInInspector] public Quaternion qProxOpposeMax, qInterOpposeMax; 
    [HideInInspector] public Quaternion qProxSplayOpposeMax, qInterSplayOpposeMax;

    // 실시간 센서 원시 데이터
    [HideInInspector] public float rawProxFlex = 0f;
    [HideInInspector] public float rawInterFlex = 0f;
    [HideInInspector] public float rawDistFlex = 0f;
    [HideInInspector] public float rawSplay = 0f;
    [HideInInspector] public float rawOppose = 0f;

    // 중앙값 저장 변수
    [HideInInspector] public float splayCenter = 0f;

    // 캘리브레이션용 저장 변수
    [HideInInspector] public float proxMin = 4095f, proxMax = 0f;
    [HideInInspector] public float interMin = 4095f, interMax = 0f;
    [HideInInspector] public float distMin = 4095f, distMax = 0f;
    [HideInInspector] public float splayMin = 4095f, splayMax = 0f;
    [HideInInspector] public float opposeMin = 4095f, opposeMax = 0f;

    // 초기화 함수
    public void InitQuaternions()
    {
        qProxOpen = Quaternion.Euler(proxOpen);
        qProxClose = Quaternion.Euler(proxClose);
        
        qInterOpen = Quaternion.Euler(interOpen);
        qInterClose = Quaternion.Euler(interClose);
        
        qDistOpen = Quaternion.Euler(distOpen);
        qDistClose = Quaternion.Euler(distClose);

        qProxSplayMin = Quaternion.Euler(proxSplayMin);
        qProxSplayMax = Quaternion.Euler(proxSplayMax);
        qInterSplayMax = Quaternion.Euler(interSplayMax);

        qProxOpposeMax = Quaternion.Euler(proxOpposeMax);
        qInterOpposeMax = Quaternion.Euler(interOpposeMax);

        qProxSplayOpposeMax = Quaternion.Euler(proxSplayOpposeMax);
        qInterSplayOpposeMax = Quaternion.Euler(interSplayOpposeMax);

        invProxOpen = Quaternion.Inverse(qProxOpen);
        invInterOpen = Quaternion.Inverse(qInterOpen);
    }

    // 캘리브레이션 시작 시 Min/Max 초기화
    public void ResetCalibrationLimits()
    {
        splayCenter = rawSplay; 
        splayMin = rawSplay; 
        splayMax = rawSplay; 
        
        proxMin = 4095f; proxMax = 0f;
        interMin = 4095f; interMax = 0f;
        distMin = 4095f; distMax = 0f;
        opposeMin = 4095f; opposeMax = 0f;
    }

    // 캘리브레이션 도중 실시간으로 한계값 업데이트
    public void UpdateCalibrationLimits()
    {
        proxMin = Mathf.Min(proxMin, rawProxFlex);
        proxMax = Mathf.Max(proxMax, rawProxFlex);

        interMin = Mathf.Min(interMin, rawInterFlex);
        interMax = Mathf.Max(interMax, rawInterFlex);

        distMin = Mathf.Min(distMin, rawDistFlex); 
        distMax = Mathf.Max(distMax, rawDistFlex);

        splayMin = Mathf.Min(splayMin, rawSplay);
        splayMax = Mathf.Max(splayMax, rawSplay);

        opposeMin = Mathf.Min(opposeMin, rawOppose);
        opposeMax = Mathf.Max(opposeMax, rawOppose);
    }
}

public class LinkTracker : MonoBehaviour
{
    [Header("통신 설정")]
    public string portName = "COM3";
    public int baudRate = 500000;
    private SerialPort serialPort;

    [Header("관절 오일러 각도 설정")]
    public FingerData thumb = new FingerData { fingerName = "Thumb"};
    public FingerData index = new FingerData { fingerName = "Index"};
    public FingerData middle = new FingerData { fingerName = "Middle"};
    public FingerData ring = new FingerData { fingerName = "Ring"};
    public FingerData pinky = new FingerData { fingerName = "Pinky"};

    // 캘리브레이션 상태
    private bool isCalibrating = false;
    private bool isCalibrated = false;
    
    void Start()
    {
        // 1. 관절 각도 설정
        InitializeFingerData();

        // 2. 쿼터니언 각도 초기화
        thumb.InitQuaternions();
        index.InitQuaternions();
        middle.InitQuaternions();
        ring.InitQuaternions();
        pinky.InitQuaternions();

        // 3. 시리얼 포트 열기
        try
        {
            serialPort = new SerialPort(portName, baudRate);
            serialPort.ReadTimeout = 50;    // 0.05초 대기
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
        ReceiveSerialData();
        HandleDynamicCalibration();
        
        // 캘리브레이션이 모두 완료된 경우에만 트래킹 수행
        if (isCalibrated && !isCalibrating)
        {
            ApplyKinematics(thumb);
            ApplyKinematics(index);
            ApplyKinematics(middle);
            ApplyKinematics(ring);
            ApplyKinematics(pinky);
        }
    }

    // ==========================================
    // 1. 시리얼 데이터 수신 및 파싱
    // ==========================================
    private void ReceiveSerialData()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            try
            {
                string lastDataString = null;
                while (serialPort.BytesToRead > 0)
                {
                    lastDataString = serialPort.ReadLine().Trim();
                }

                if (!string.IsNullOrEmpty(lastDataString))
                {
                    string[] values = lastDataString.Split(',');

                    if (values.Length >= 16)
                    {
                        float.TryParse(values[0], out thumb.rawSplay); float.TryParse(values[1], out thumb.rawInterFlex); float.TryParse(values[2], out thumb.rawDistFlex); float.TryParse(values[3], out thumb.rawOppose);
                        float.TryParse(values[4], out index.rawSplay); float.TryParse(values[5], out index.rawProxFlex); float.TryParse(values[6], out index.rawInterFlex);
                        float.TryParse(values[7], out middle.rawSplay); float.TryParse(values[8], out middle.rawProxFlex); float.TryParse(values[9], out middle.rawInterFlex);
                        float.TryParse(values[10], out ring.rawSplay); float.TryParse(values[11], out ring.rawProxFlex); float.TryParse(values[12], out ring.rawInterFlex);
                        float.TryParse(values[13], out pinky.rawSplay); float.TryParse(values[14], out pinky.rawProxFlex); float.TryParse(values[15], out pinky.rawInterFlex);
                    }
                }
            }
            catch (System.TimeoutException) {}
            catch (System.Exception e) { Debug.LogWarning("파싱 에러: " + e.Message); }
        }
    }

    // ==========================================
    // 2. 동적 캘리브레이션 (J, K)
    // ==========================================
    private void HandleDynamicCalibration()
    {
        // J키: 캘리브레이션 시작 (값 초기화)
        if (Input.GetKeyDown(KeyCode.J))
        {
            isCalibrating = true;
            isCalibrated = false; 
            
            thumb.ResetCalibrationLimits();
            index.ResetCalibrationLimits();
            middle.ResetCalibrationLimits();
            ring.ResetCalibrationLimits();
            pinky.ResetCalibrationLimits();
            
            Debug.Log("[캘리브레이션 시작] 손가락을 쫙 펴고, 주먹을 쥐고, 손가락을 벌려주세요.");
        }

        // K키: 캘리브레이션 종료
        if (Input.GetKeyDown(KeyCode.K))
        {
            if (isCalibrating)
            {
                isCalibrating = false;
                isCalibrated = true; 
                Debug.Log("[캘리브레이션 완료] 트래킹을 시작합니다.");
            }
        }

        if (isCalibrating)
        {
            thumb.UpdateCalibrationLimits();
            index.UpdateCalibrationLimits();
            middle.UpdateCalibrationLimits();
            ring.UpdateCalibrationLimits();
            pinky.UpdateCalibrationLimits();
        }
    }

    // ==========================================
    // 3. 기구학적 트래킹 알고리즘
    // ==========================================
    private void ApplyKinematics(FingerData f)
    {
        // 뼈대 할당 여부 확인
        if (f.prox == null || f.inter == null || f.dist == null) return;

        // 0 나누기 방지
        float proxRange = (f.proxMax - f.proxMin) + 0.001f;
        float interRange = (f.interMax - f.interMin) + 0.001f;
        float distRange = (f.distMax - f.distMin) + 0.001f;
        float splayRange = (f.splayMax - f.splayMin) + 0.001f;
        float opposeRange = (f.opposeMax - f.opposeMin) + 0.001f;

        // A. 센서 원시값을 0.0 ~ 1.0 비율(Ratio)로 정규화
        float proxRawRatio  = Mathf.Clamp01((f.rawProxFlex - f.proxMin) / proxRange);
        float interRawRatio = Mathf.Clamp01((f.rawInterFlex - f.interMin) / interRange);
        float distRawRatio  = Mathf.Clamp01((f.rawDistFlex - f.distMin) / distRange);
        float splayRawRatio = Mathf.Clamp01((f.rawSplay - f.splayMin) / splayRange);
        float opposeRawRatio = Mathf.Clamp01((f.rawOppose - f.opposeMin) / opposeRange);
        float centerRawRatio = Mathf.Clamp01((f.splayCenter - f.splayMin) / splayRange);

        // B. 포텐셔미터 장착 방향에 따른 Invert 로직 적용
        float proxRatio  = f.invertProxFlex ? 1f - proxRawRatio : proxRawRatio;
        float interRatio = f.invertInterFlex ? 1f - interRawRatio : interRawRatio;
        float distRatio  = f.invertDistFlex ? 1f - distRawRatio : distRawRatio;
        float splayRatio = f.invertSplay ? 1f - splayRawRatio : splayRawRatio;
        float opposeRatio = f.invertOppose ? 1f - opposeRawRatio : opposeRawRatio;
        float centerRatio = f.invertSplay ? 1f - centerRawRatio : centerRawRatio;

        // C. 벌어짐(Splay) 정규화
        float finalSplayRatio;
        if (f.fingerName == "Thumb")
        {
            float dynamicZero = centerRatio * opposeRatio;
            finalSplayRatio = Mathf.InverseLerp(dynamicZero, 1f, splayRatio);
        }
        else
        {
            finalSplayRatio = centerRatio + (splayRatio - centerRatio) * (1f - proxRatio);
        }

        // D. 유니티 뼈대 회전 적용
        if (f.fingerName == "Thumb")
        {
            // [1] Prox 뼈대 연산 (손바닥 안쪽)
            // 선 1: 펴진 상태에서의 벌어짐 (Open -> SplayMax)
            Quaternion qProxEdgeOpen = Quaternion.Slerp(f.qProxOpen, f.qProxSplayMax, finalSplayRatio);
            // 선 2: 가로지른 상태에서의 벌어짐 (OpposeMax -> SplayOpposeMax)
            Quaternion qProxEdgeOpposed = Quaternion.Slerp(f.qProxOpposeMax, f.qProxSplayOpposeMax, finalSplayRatio);
            
            // 면 생성: 두 선 사이를 가로지르기 비율(opposeRatio)로 섞어 베이스를 생성
            Quaternion targetBaseProx = Quaternion.Slerp(qProxEdgeOpen, qProxEdgeOpposed, opposeRatio);

            // 굽힘(Flex) 변화량을 구해서 베이스 위에 추가
            Quaternion targetProxFlex = Quaternion.Slerp(f.qProxOpen, f.qProxClose, proxRatio);
            Quaternion deltaProxFlex = targetProxFlex * f.invProxOpen;
            
            f.prox.localRotation = deltaProxFlex * targetBaseProx;

            // [2] Inter 뼈대 연산 (엄지 뿌리)
            // 선 1: 펴진 상태에서의 벌어짐 (Open -> SplayMax)
            Quaternion qInterEdgeOpen = Quaternion.Slerp(f.qInterOpen, f.qInterSplayMax, finalSplayRatio);
            // 선 2: 가로지른 상태에서의 벌어짐 (OpposeMax -> SplayOpposeMax)
            Quaternion qInterEdgeOpposed = Quaternion.Slerp(f.qInterOpposeMax, f.qInterSplayOpposeMax, finalSplayRatio);
            
            // 면 생성: 두 선 사이를 가로지르기 비율(opposeRatio)로 섞어 베이스를 생성
            Quaternion targetBaseInter = Quaternion.Slerp(qInterEdgeOpen, qInterEdgeOpposed, opposeRatio);

            // 굽힘(Flex) 변화량을 구해서 베이스 위에 추가
            Quaternion targetInterFlex = Quaternion.Slerp(f.qInterOpen, f.qInterClose, interRatio);
            Quaternion deltaInterFlex = targetInterFlex * f.invInterOpen;

            f.inter.localRotation = deltaInterFlex * targetBaseInter;

            // [3] Dist 뼈대 (엄지 끝)
            f.dist.localRotation = Quaternion.Slerp(f.qDistOpen, f.qDistClose, distRatio);
        }
        else
        {
            // 중간, 끝 마디 적용 (순수 굽힘)
            f.inter.localRotation = Quaternion.Slerp(f.qInterOpen, f.qInterClose, interRatio);
            f.dist.localRotation = Quaternion.Slerp(f.qDistOpen, f.qDistClose, interRatio);


            // 뿌리 마디 적용 (굽힘 + 벌어짐 합성)
            Quaternion targetFlex = Quaternion.Slerp(f.qProxOpen, f.qProxClose, proxRatio);
        
            Quaternion targetSplay;
            if (finalSplayRatio <= centerRatio)
            {
                // 구간 1: 최소 벌어짐 ~ 똑바른 상태(Center)
                // t 값은 0.0 에서 1.0 사이로 계산됨
                float t = (centerRatio > 0.001f) ? (finalSplayRatio / centerRatio) : 0f;
                targetSplay = Quaternion.Slerp(f.qProxSplayMin, f.qProxOpen, t);
            }
            else
            {
                // 구간 2: 똑바른 상태(Center) ~ 최대 벌어짐
                // t 값은 0.0 에서 1.0 사이로 계산됨
                float t = (centerRatio < 0.999f) ? ((finalSplayRatio - centerRatio) / (1f - centerRatio)) : 1f;
                targetSplay = Quaternion.Slerp(f.qProxOpen, f.qProxSplayMax, t);
            }

            Quaternion deltaFlex = targetFlex * f.invProxOpen;
            Quaternion deltaSplay = targetSplay * f.invProxOpen;

            f.prox.localRotation = deltaSplay * deltaFlex * f.qProxOpen;
        }
    }

    private void InitializeFingerData()
    {
        // 엄지 (C0, C1, C2, C3)
        // C0(Inter Splay) : Open=min, Spread=max -> 정상
        // C1(Inter Flex): Open=max, Closed=min -> 반전 필요
        // C2(Dist Flex) : Open=max, Closed=min -> 반전 필요
        // C3(Oppose) : Open=max, Closed=min -> 반전 필요
        thumb.invertSplay = false;
        thumb.invertInterFlex = true;
        thumb.invertDistFlex = true;
        thumb.invertOppose = true;

        // 1. 기본 자세 (Open)
        thumb.proxOpen = new Vector3(-27.437f, -74.588f, 2.521f);
        thumb.interOpen = new Vector3(25.689f, 3.163f, -16.036f);
        thumb.distOpen = new Vector3(-2.048f, 1.665f, 7.053f);
        thumb.proxSplayMin = thumb.proxOpen;

        // 2. 순수 굽힘 (Close)
        thumb.proxClose = new Vector3(-27.437f, -74.588f, 2.521f);
        thumb.interClose = new Vector3(22.072f, 5.721f, 53.624f);
        thumb.distClose = new Vector3(-2.048f, 1.665f, 104.796f);
        
        // 3. 순수 벌어짐 (SplayMax)
        thumb.proxSplayMax = new Vector3(-49.993f, -62.249f, -14.676f);
        thumb.interSplayMax = new Vector3(1.151f, 32.540f, -6.371f);

        // 4. 순수 가로지르기 (OpposeMax)
        thumb.proxOpposeMax = new Vector3(-31.067f, -35.385f, 9.723f);
        thumb.interOpposeMax = new Vector3(24.399f, 28.576f, -4.084f);

        // 5. 벌어짐 + 가로지름 (SplayOpposeMax)
        thumb.proxSplayOpposeMax = new Vector3(-43.635f, -38.467f, 12.312f);
        thumb.interSplayOpposeMax = new Vector3(-3.010f, 28.726f, -7.038f);

        // 검지 (C4, C5, C6)
        // C4(Prox Splay): Straight=min, Spread=max -> 정상
        // C5(Prox Flex): Straight=max, Closed=min -> 반전 필요
        // C6(Inter Flex): Straight=min, Closed=max -> 정상
        index.invertSplay = false;
        index.invertProxFlex = true;
        index.invertInterFlex = false;

        index.proxOpen = new Vector3(-7.843f, -3.073f, 29.806f); index.interOpen = new Vector3(1.483f, -0.726f, 1.429f); index.distOpen = new Vector3(-6.734f, 0.353f, -0.974f);
        index.proxClose = new Vector3(-67.521f, -60.344f, 63.975f); index.interClose = new Vector3(-73.230f, 172.581f, -172.958f); index.distClose = new Vector3(-68.662f, 2.715f, -2.658f);
        index.proxSplayMax = new Vector3(-7.843f, -3.073f, 8.888f);
        index.proxSplayMin = index.proxOpen;
        index.proxOpposeMax = index.proxOpen;
        index.interSplayMax = index.interOpen;

        // 중지 (C7, C8, C9)
        // C7(Prox Splay): 검지쪽(Min 회전상태)=max, 약지쪽(Max 회전상태)=min -> 반전 필요
        // C8(Prox Flex): Straight=max, Closed=min -> 반전 필요
        // C9(Inter Flex): Straight=min, Closed=max -> 정상
        middle.invertSplay = true;
        middle.invertProxFlex = true;
        middle.invertInterFlex = false;

        middle.proxOpen = new Vector3(-17.457f, -4.409f, 2.188f); middle.interOpen = new Vector3(6.167f, 0.571f, 2.676f); middle.distOpen = new Vector3(-6.498f, -0.677f, 0.644f);
        middle.proxClose = new Vector3(-83.893f, -60.222f, 59.486f); middle.interClose = new Vector3(-74.392f, -170.14f, 170.064f); middle.distClose = new Vector3(-64.576f, -1.950f, 1.491f);
        middle.proxSplayMin = new Vector3(-17.457f, -4.409f, -3.032f); // 검지 쪽으로 기울어진 자세
        middle.proxSplayMax = new Vector3(-17.457f, -4.409f, 12.852f); // 약지 쪽으로 기울어진 자세
        middle.proxOpposeMax = middle.proxOpen;
        middle.interSplayMax = middle.interOpen;

        // 약지 (C10, C11, C12)
        // C10(Prox Splay): Straight=max, Spread=min -> 반전 필요
        // C11(Prox Flex): Straight=max, Closed=min -> 반전 필요
        // C12(Inter Flex): Straight=min, Closed=max -> 정상
        ring.invertSplay = true;
        ring.invertProxFlex = true;
        ring.invertInterFlex = false;

        ring.proxOpen = new Vector3(-14.350f, 4.387f, -22.789f); ring.interOpen = new Vector3(1.192f, 0.628f, 2.755f); ring.distOpen = new Vector3(-5.425f, -2.367f, 1.922f);
        ring.proxClose = new Vector3(-80.328f, 51.251f, -53.693f); ring.interClose = new Vector3(-71.285f, -171.266f, 171.387f); ring.distClose = new Vector3(-63.749f, -6.070f, 4.330f); 
        ring.proxSplayMax = new Vector3(-14.350f, 4.387f, -3.204f);
        ring.proxSplayMin = ring.proxOpen;
        ring.proxOpposeMax = ring.proxOpen;
        ring.interSplayMax = ring.interOpen;

        // 새끼 (C13, C14, C15)
        // C13(Prox Splay): Straight=max, Spread=min -> 반전 필요
        // C14(Prox Flex): Straight=min, Closed=max -> 정상
        // C15(Inter Flex): Straight=min, Closed=max -> 정상
        pinky.invertSplay = true;
        pinky.invertProxFlex = false;
        pinky.invertInterFlex = false;

        pinky.proxOpen = new Vector3(-7.071f, 7.374f, -44.369f); pinky.interOpen = new Vector3(-177.230f, -181.131f, 178.519f); pinky.distOpen = new Vector3(-1.616f, 0.721f, 1.594f);
        pinky.proxClose = new Vector3(-64.125f, 63.102f, -64.182f); pinky.interClose = new Vector3(-70.447f, -179.531f, 178.330f); pinky.distClose = new Vector3(-62.597f, -2.309f, 3.463f);
        pinky.proxSplayMax = new Vector3(-7.071f, 7.374f, -4.142f);
        pinky.proxSplayMin = pinky.proxOpen;
        pinky.proxOpposeMax = pinky.proxOpen;
        pinky.interSplayMax = pinky.interOpen;
    }

    void OnApplicationQuit()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
            Debug.Log("시리얼 포트 닫힘");
        }
    }
}