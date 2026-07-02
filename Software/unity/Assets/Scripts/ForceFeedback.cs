using UnityEngine;

public class ForceFeedback : MonoBehaviour
{
    [System.Serializable]
    public class Wall
    {
        public Transform wall;                              // 벽 오브젝트
        public Vector3 localNormal = new Vector3(0, 0, 1);  // 벽 로컬 법선
    }

    [Header("참조")]
    public Transform transparentHand;
    public Transform[] contactPoints;
    public Transform handProxy;

    [Header("벽 설정 (여러 개)")]
    public Wall[] walls = new Wall[3];   // ★ 벽 3개 (인스펙터에서 채움)

    [Header("전송 설정")]
    public HandTracker handTracker;
    public Transform robotBase;
    public bool sendToESP = false;
    public float sendRateHz = 50f;

    [Header("힘 파라미터")]
    public float stiffness = 200f;
    public float damping = 0f;

    [Header("디버그 (읽기용)")]
    public float dMax = 0f;
    public bool contact = false;
    public int activeWall = -1;          // ★ 현재 닿은 벽 인덱스
    public Vector3 forceWorld = Vector3.zero;
    public Vector3 forceBase = Vector3.zero;
    public float forceMag = 0f;

    private Quaternion rotOffset;
    private float prevDepth = 0f;
    private float vFiltered = 0f;
    private float lastSendTime = 0f;

    void Start()
    {
        if (handProxy && transparentHand)
            rotOffset = Quaternion.Inverse(transparentHand.rotation) * handProxy.rotation;
    }

    // 특정 벽의 법선 (월드)
    Vector3 WallNormalWorld(Wall w)
    {
        return w.wall.TransformDirection(w.localNormal.normalized);
    }

    // 특정 벽의 두께 절반 (법선 방향)
    float WallHalfThickness(Wall w)
    {
        Vector3 n = w.localNormal.normalized;
        Vector3 scale = w.wall.lossyScale;
        return Mathf.Abs(n.x)*scale.x*0.5f
             + Mathf.Abs(n.y)*scale.y*0.5f
             + Mathf.Abs(n.z)*scale.z*0.5f;
    }

    void Update()
    {
        if (contactPoints == null || contactPoints.Length == 0 || walls == null) return;

        // ---- 모든 벽 검사: 가장 깊이 박힌 (벽, 점) 찾기 ----
        dMax = 0f;
        activeWall = -1;
        Vector3 nWorldBest = Vector3.zero;

        for (int w = 0; w < walls.Length; w++)
        {
            if (walls[w] == null || walls[w].wall == null) continue;

            Vector3 nWorld = WallNormalWorld(walls[w]);
            float half = WallHalfThickness(walls[w]);
            Vector3 wallCenter = walls[w].wall.position;

            for (int i = 0; i < contactPoints.Length; i++)
            {
                if (contactPoints[i] == null) continue;
                Vector3 rel = contactPoints[i].position - wallCenter;
                float distAlongNormal = Vector3.Dot(rel, nWorld);
                float pen = half - distAlongNormal;
                if (pen > dMax)
                {
                    dMax = pen;
                    activeWall = w;
                    nWorldBest = nWorld;
                }
            }
        }

        contact = (dMax > 0f);
        if (!contact) { dMax = 0f; nWorldBest = Vector3.zero; }

        // 벽 법선 시각화 (닿은 벽만)
        if (contact && activeWall >= 0)
            Debug.DrawRay(walls[activeWall].wall.position, nWorldBest * 0.3f, Color.red);

        // 프록시 손 이동
        if (handProxy != null && transparentHand != null)
        {
            handProxy.rotation = transparentHand.rotation * rotOffset;
            handProxy.position = contact
                ? transparentHand.position + nWorldBest * dMax
                : transparentHand.position;
        }

        // F 계산 (스프링 + 비대칭 감쇠)
        if (contact)
        {
            float vRaw = (dMax - prevDepth) / Mathf.Max(Time.deltaTime, 1e-5f);
            vFiltered = Mathf.Lerp(vFiltered, vRaw, 0.2f);
            float fSpring = stiffness * dMax;
            float fDamp = (vFiltered > 0f) ? damping * vFiltered : 0f;
            forceMag = fSpring + fDamp;
            forceWorld = nWorldBest * forceMag;
        }
        else
        {
            forceMag = 0f;
            forceWorld = Vector3.zero;
        }
        prevDepth = dMax;

        // 로봇 베이스 좌표계로 변환
        forceBase = (robotBase != null)
            ? robotBase.InverseTransformDirection(forceWorld)
            : forceWorld;

        // ESP32로 F 패킷 전송
        if (sendToESP && handTracker != null)
        {
            if (Time.time - lastSendTime >= 1f / sendRateHz)
            {
                lastSendTime = Time.time;
                float espX = -forceBase.x;
                float espY =  forceBase.z;
                float espZ = -forceBase.y;
                handTracker.SendForce($"F,{espX:F3},{espY:F3},{espZ:F3}");
            }
        }
    }
}