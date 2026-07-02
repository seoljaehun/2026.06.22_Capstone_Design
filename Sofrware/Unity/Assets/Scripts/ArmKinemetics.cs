using UnityEngine;

public class ArmKinematics : MonoBehaviour
{
    [Header("센서 값 소스")]
    public HandTracker tracker;

    [Header("관절 GameObjects (체인 순서대로)")]
    public Transform joint1Base;
    public Transform joint2;
    public Transform joint3;
    public Transform gimbalPitch;
    public Transform gimbalYaw;
    public Transform gimbalRoll;

    [Header("손 모델")]
    public Transform handModel;

    [Header("k순간 손 목표 월드 회전 (손바닥-바닥+손가락-정면)")]
    // 에디터에서 손을 그 자세로 두고 handModel.rotation.eulerAngles 읽은 값
    public Vector3 handWorldTargetEuler = new Vector3(0f, 184.098f, 349f);

    [Header("각 관절 회전축 (로컬)")]
    public Vector3 axis1Dir = new Vector3(0, 1, 0);
    public Vector3 axis2Dir = new Vector3(1, 0, 0);
    public Vector3 axis3Dir = new Vector3(1, 0, 0);
    public Vector3 pitchDir = new Vector3(0, 1, 0);   // ★ 수정: 실제 pitch축
    public Vector3 yawDir   = new Vector3(1, 0, 0);   // ★ 수정: 실제 yaw축
    public Vector3 rollDir  = new Vector3(0, 0, 1);

    [Header("각 관절 부호 (방향 반대면 -1)")]
    public float sign1 = 1f;
    public float sign2 = 1f;
    public float sign3 = 1f;
    public float signPitch = 1f;
    public float signYaw = 1f;
    public float signRoll = 1f;

    // 각 관절 기본 로컬 회전 (시작 시 저장)
    private Quaternion base1, base2, base3, baseP, baseY, baseR;

    // GimbalRoll → 손 강체 상대회전 (불변 상수)
    private Quaternion L_const;

    // C 패킷 팔 절대각 기준
    private float calibBase = 0f, calibA2 = 0f, calibA3 = 0f;
    private bool synced = false;

    void Start()
    {
        if (joint1Base)  base1 = joint1Base.localRotation;
        if (joint2)      base2 = joint2.localRotation;
        if (joint3)      base3 = joint3.localRotation;
        if (gimbalPitch) baseP = gimbalPitch.localRotation;
        if (gimbalYaw)   baseY = gimbalYaw.localRotation;
        if (gimbalRoll)  baseR = gimbalRoll.localRotation;

        // GimbalRoll 기준 손의 불변 상대회전 (강체, 자세 무관 상수)
        if (gimbalRoll && handModel)
            L_const = Quaternion.Inverse(gimbalRoll.rotation) * handModel.rotation;
    }

    void Update()
    {
        if (tracker == null) return;

        // ★ C 패킷 도착: 팔 동기화 + 짐벌 원점(baseP/Y/R) 계산
        if (tracker.hasCalib)
        {
            calibBase = tracker.baseAbs;
            calibA2   = tracker.axis2Abs;
            calibA3   = tracker.axis3Abs;
            tracker.hasCalib = false;

            // ① 팔 3축을 동기화 자세로 먼저 적용 (WristCenter 월드 확정)
            ApplyArm(calibBase, calibA2, calibA3);

            // ② 짐벌 부모(WristCenter)의 월드 회전
            Quaternion W_parent = gimbalPitch.parent.rotation;

            // ③ 손 목표 월드 → GimbalRoll 목표 월드
            Quaternion R_target = Quaternion.Euler(handWorldTargetEuler)
                                  * Quaternion.Inverse(L_const);

            // ④ 짐벌 누적 로컬회전 = inverse(W_parent) × R_target
            Quaternion R_local = Quaternion.Inverse(W_parent) * R_target;

            Debug.Log($"[DIAG] W_parent={W_parent.eulerAngles}");
            Debug.Log($"[DIAG] R_target={R_target.eulerAngles}");
            Debug.Log($"[DIAG] R_local={R_local.eulerAngles}");

            // ⑤ Y(pitch)→X(yaw)→Z(roll) 순서로 분해
            DecomposeYXZ(R_local, out float tp, out float ty, out float tr);
            Debug.Log($"[DIAG] decompose result: pitch={tp:F1} yaw={ty:F1} roll={tr:F1}");

            // ⑥ 원점 세팅 (초기 로컬 0이므로 AngleAxis 그대로)
            baseP = Quaternion.AngleAxis(tp, pitchDir);
            baseY = Quaternion.AngleAxis(ty, yawDir);
            baseR = Quaternion.AngleAxis(tr, rollDir);

            Debug.Log($"[BASE] baseP={baseP.eulerAngles} baseY={baseY.eulerAngles} baseR={baseR.eulerAngles}");
            Debug.Log($"[DIR] pitchDir={pitchDir} yawDir={yawDir} rollDir={rollDir}");

            synced = true;
            Debug.Log($"[ArmKinematics] synced arm: base={calibBase:F1} a2={calibA2:F1} a3={calibA3:F1}");
            Debug.Log($"[ArmKinematics] gimbal origin: p={tp:F1} y={ty:F1} r={tr:F1}");
        }

        if (!synced || !tracker.hasData) return;

        // ★ 팔: 절대각 + 상대각
        float baseTotal = calibBase + tracker.baseDeg;
        float a2Total   = calibA2   + tracker.axis2Deg;
        float a3Total   = calibA3   + tracker.axis3Deg;
        ApplyArm(baseTotal, a2Total, a3Total);

        // ★ 짐벌: 원점(baseP/Y/R) × 센서 상대각
        if (gimbalPitch)
            gimbalPitch.localRotation = baseP * Quaternion.AngleAxis(signPitch * tracker.pitchDeg, pitchDir);
        if (gimbalYaw)
            gimbalYaw.localRotation = baseY * Quaternion.AngleAxis(signYaw * tracker.yawDeg, yawDir);
        if (gimbalRoll)
            gimbalRoll.localRotation = baseR * Quaternion.AngleAxis(signRoll * tracker.rollDeg, rollDir);
    }

    // 팔 3축 적용 (C 처리와 Update 공용)
    void ApplyArm(float baseT, float a2T, float a3T)
    {
        if (joint1Base)
            joint1Base.localRotation = base1 * Quaternion.AngleAxis(sign1 * baseT, axis1Dir);
        if (joint2)
            joint2.localRotation = base2 * Quaternion.AngleAxis(sign2 * a2T, axis2Dir);
        if (joint3)
            joint3.localRotation = base3 * Quaternion.AngleAxis(sign3 * a3T, axis3Dir);
    }

    // R = Ry(pitch, Y축) × Rx(yaw, X축) × Rz(roll, Z축) 분해
    // (짐벌 계층: GimbalPitch=Y, GimbalYaw=X, GimbalRoll=Z, 모두 초기 로컬 0)
    void DecomposeYXZ(Quaternion q, out float pitchY, out float yawX, out float rollZ)
    {
        // 회전행렬로 변환
        Vector3 r0 = q * Vector3.right;     // 1열
        Vector3 r1 = q * Vector3.up;        // 2열
        Vector3 r2 = q * Vector3.forward;   // 3열
        // m[row,col]
        float m00 = r0.x, m01 = r1.x, m02 = r2.x;
        float m10 = r0.y, m11 = r1.y, m12 = r2.y;
        float m20 = r0.z, m21 = r1.z, m22 = r2.z;

        // YXZ 순서: R = Ry * Rx * Rz
        // m12 = -sin(x)  →  x = yaw
        float sx = -m12;
        sx = Mathf.Clamp(sx, -1f, 1f);
        yawX = Mathf.Asin(sx) * Mathf.Rad2Deg;

        float cx = Mathf.Cos(Mathf.Asin(sx));
        if (Mathf.Abs(cx) > 1e-6f)
        {
            pitchY = Mathf.Atan2(m02, m22) * Mathf.Rad2Deg;  // Y
            rollZ  = Mathf.Atan2(m10, m11) * Mathf.Rad2Deg;  // Z
        }
        else
        {
            // 짐벌락 특이점 (거의 안 옴)
            pitchY = Mathf.Atan2(-m20, m00) * Mathf.Rad2Deg;
            rollZ  = 0f;
        }
    }
}