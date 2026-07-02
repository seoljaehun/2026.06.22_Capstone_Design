using UnityEngine;

public class ArmVisualizer : MonoBehaviour
{
    [Header("시각화 설정")]
    public float jointSize = 0.02f;   // 관절(빈 오브젝트)에 그릴 구체의 크기
    public Color jointColor = Color.cyan;  // 관절 색상
    public Color boneColor = Color.yellow; // 연결 선 색상

    void OnDrawGizmos()
    {
        DrawJointsAndBones(transform);
    }

    void DrawJointsAndBones(Transform current)
    {
        // 1. 현재 관절 위치에 색상이 있는 구(Sphere) 그리기
        Gizmos.color = jointColor;
        Gizmos.DrawSphere(current.position, jointSize);

        // ★ 추가된 부분: 현재 관절의 이름이 "HandMount"라면 
        // 여기서 함수를 종료하여 자식(손 모델)으로 선이 이어지지 않게 차단합니다.
        if (current.name == "HandMount")
        {
            return; 
        }

        // 2. 현재 관절과 자식 관절들을 선으로 연결하기
        foreach (Transform child in current)
        {
            Gizmos.color = boneColor;
            Gizmos.DrawLine(current.position, child.position);
            
            // 자식의 자식도 똑같이 그리도록 함수 다시 호출
            DrawJointsAndBones(child);
        }
    }
}