using UnityEngine;

// [ExecuteAlways]를 붙이면 게임을 플레이(▶)하지 않아도 에디터에서 스크립트가 실시간으로 작동합니다!
[ExecuteAlways]
public class WorldRotationViewer : MonoBehaviour
{
    [Header("🌍 실시간 월드 회전값 (보기 전용)")]
    [Tooltip("이 수치는 읽기 전용입니다. 여기서 숫자를 바꿔도 실제 회전이 변하지는 않습니다.")]
    public Vector3 worldRotation;

    [Header("디버그 옵션")]
    public bool printToConsole = false; // 콘솔 창에도 로그를 띄우고 싶을 때 체크하세요.

    void Update()
    {
        // transform.eulerAngles를 통해 부모의 꺾임과 상관없는 절대적인 월드 회전값을 가져옵니다.
        worldRotation = transform.eulerAngles;

        // 콘솔 출력이 켜져 있고, 게임이 실행 중일 때만 로그를 남깁니다.
        if (printToConsole && Application.isPlaying)
        {
            Debug.Log($"<color=orange>[{gameObject.name}]</color> 월드 회전: {worldRotation}");
        }
    }
}