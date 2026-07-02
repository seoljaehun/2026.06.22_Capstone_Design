using UnityEngine;

public class PureTrackingTest : MonoBehaviour
{
    [Header("유니티 뼈대 연결")]
    public Transform indexProx;  // 뿌리 관절
    public Transform indexInter; // 중간 관절
    public Transform indexDist;  // 끝 관절

    [Header("가상 센서 슬라이더 (실제 센서 비율 0.0 ~ 1.0)")]
    [Tooltip("중간/끝 관절 굽힘 (0=쫙 핌, 1=꽉 쥠)")]
    [Range(0f, 1f)] public float interFlexRatio = 0f;

    [Tooltip("뿌리 관절 굽힘 (0=쫙 핌, 1=꽉 쥠)")]
    [Range(0f, 1f)] public float proxFlexRatio = 0f;
    
    [Tooltip("뿌리 벌어짐 (0=모음, 1=쫙 벌림)")]
    [Range(0f, 1f)] public float splayRatio = 0f; 

    [Header("관절 오일러 각도 설정 (하드코딩)")]
    public Vector3 proxOpen = new Vector3(-7.843f, -3.073f, 29.806f);
    public Vector3 interOpen = new Vector3(1.483f, -0.726f, 1.429f);
    public Vector3 distOpen = new Vector3(-6.734f, 0.353f, -0.974f);
    
    public Vector3 proxClose = new Vector3(-67.521f, -60.344f, 63.975f);
    public Vector3 interClose = new Vector3(-73.230f, 172.581f, -172.958f);
    public Vector3 distClose = new Vector3(-68.662f, 2.715f, -2.658f);
    
    public Vector3 proxSpread = new Vector3(-7.843f, -3.073f, 8.888f);

    // --- 쿼터니언 변수 선언 ---
    private Quaternion qProxOpen, qProxClose, qProxSpread;
    private Quaternion qInterOpen, qInterClose;
    private Quaternion qDistOpen, qDistClose;
    
    // 역행렬 계산 최적화 변수
    private Quaternion invProxOpen; 

    void Start()
    {
        // 1. 하드코딩 오일러 값을 쿼터니언으로 변환
        qProxOpen = Quaternion.Euler(proxOpen);
        qProxClose = Quaternion.Euler(proxClose);
        qProxSpread = Quaternion.Euler(proxSpread);

        qInterOpen = Quaternion.Euler(interOpen);
        qInterClose = Quaternion.Euler(interClose);

        qDistOpen = Quaternion.Euler(distOpen);
        qDistClose = Quaternion.Euler(distClose);

        // 2. Prox 관절 합성을 위한 역행렬 계산 (1회만 수행)
        invProxOpen = Quaternion.Inverse(qProxOpen);
    }

    void Update()
    {
        if (indexProx == null || indexInter == null || indexDist == null) return;

        // =========================================================
        // Step 1. Inter & Dist (중간, 끝 마디) 적용
        // 특징: 벌어짐(Splay) 신경 쓰지 않고 오직 굽힘 비율만 1:1 대입
        // =========================================================
        indexInter.localRotation = Quaternion.Slerp(qInterOpen, qInterClose, interFlexRatio);
        indexDist.localRotation = Quaternion.Slerp(qDistOpen, qDistClose, interFlexRatio);

        // =========================================================
        // Step 2. Prox (뿌리 마디) 적용
        // 특징: 어떠한 한계선 제약도 없이, 입력받은 비율을 그대로 목표로 삼음
        // =========================================================
        
        // 1. 순수 비율(Ratio)을 이용해 각각의 목표 좌표 찾기
        Quaternion targetFlex = Quaternion.Slerp(qProxOpen, qProxClose, proxFlexRatio);
        Quaternion targetSplay = Quaternion.Slerp(qProxOpen, qProxSpread, splayRatio);

        // 2. 순수 회전량(Delta) 추출 (목표 위치 * 기준점 역행렬)
        Quaternion deltaFlex = targetFlex * invProxOpen;
        Quaternion deltaSplay = targetSplay * invProxOpen;

        // 3. 최종 합성 적용 (순수 벌어짐 * 순수 굽힘 * 기준점)
        indexProx.localRotation = deltaSplay * deltaFlex * qProxOpen;
    }
}