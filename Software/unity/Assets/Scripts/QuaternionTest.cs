using UnityEngine;

public class QuaternionTest : MonoBehaviour
{
    [Header("테스트할 회전값 (오일러 각도)")]
    // 손을 쫙 폈을 때의 로컬 회전 값
    public Vector3 startEuler = new Vector3(6.167f, 0.571f, 2.676f); 
    // 주먹을 쥐었을 때의 로컬 회전 값
    public Vector3 endEuler = new Vector3(-74.392f, -170.14f, 170.064f); 

    [Header("마우스로 조절해볼 플렉스 센서 비율")]
    // 슬라이드 바
    [Range(0f, 1f)] 
    public float flexRatio = 0f;

    [Header("결과 확인 (실시간 변환된 오일러 각도)")]
    public Vector3 currentEuler;

    void Update()
    {
        // 1. 입력한 오일러 각도를 쿼터니언으로 변환
        Quaternion startRotation = Quaternion.Euler(startEuler);
        Quaternion endRotation = Quaternion.Euler(endEuler);

        // 2. 쿼터니언 선형 보간 (Lerp) 실행
        Quaternion lerpedRotation = Quaternion.Lerp(startRotation, endRotation, flexRatio);

        // 3. 이 스크립트가 달린 오브젝트를 실제로 회전시킴
        transform.localRotation = lerpedRotation;

        // 4. 현재 쿼터니언이 오일러 숫자로 어떻게 찍히는지 결과 출력
        currentEuler = lerpedRotation.eulerAngles;
    }
}