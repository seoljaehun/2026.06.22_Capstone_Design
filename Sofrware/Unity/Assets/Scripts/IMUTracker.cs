using UnityEngine;
using System.IO.Ports;

public class IMUTracker : MonoBehaviour
{
    [Header("통신 설정")]
    public string portName = "COM3";
    public int baudRate = 115200;
    private SerialPort serialPort;

    [Header("회전시킬 손 오브젝트")]
    public Transform forearmTransform; // 팔뚝 (비틀기 전용)
    public Transform wristTransform;   // 가상 손목 (꺾기 전용)

    [Header("팔뚝이 뻗어있는 방향 축 (Twist 축)")]
    public Vector3 twistAxis = Vector3.forward;

    // 기준점이 되는 회전 값 변수
    private Quaternion offsetRotation = Quaternion.identity;
    private bool isCalibrated = false;

    void Start()
    {
        try
        {
            serialPort = new SerialPort(portName, baudRate);
            serialPort.ReadTimeout = 50;
            serialPort.Open();
            Debug.Log("시리얼 포트 열기 성공: " + portName);
        }
        catch (System.Exception e)
        {
            Debug.LogError("시리얼 포트 열기 실패: " + e.Message);
        }
    }

    void Update()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            try
            {
                // ESP32에서 들어오는 "w,x,y,z" 한 줄 읽기
                string dataString = serialPort.ReadLine();
                string[] values = dataString.Split(',');

                if (values.Length == 4)
                {
                    if (float.TryParse(values[0], out float w) &&
                        float.TryParse(values[1], out float x) &&
                        float.TryParse(values[2], out float y) &&
                        float.TryParse(values[3], out float z))
                    {
                        // IMU의 물리적 방향에 따라 축 매핑
                        Quaternion sensorQ = new Quaternion(y, -x, z, w);

                        // 캘리브레이션
                        if (Input.GetKeyDown(KeyCode.J))
                        {
                            // 현재 센서 값의 역회전(Inverse)을 오프셋으로 저장
                            offsetRotation = Quaternion.Inverse(sensorQ);
                            isCalibrated = true;
                            Debug.Log("IMU 캘리브레이션 완료");
                        }

                        if (isCalibrated)
                        {
                            // 쿼터니언 곱셈: 현재 센서 회전 값 x 반대 회전 오프셋(캘리브레이션)
                            Quaternion currentQ = offsetRotation * sensorQ;

                            // 스윙-트위스트 분해
                            Vector3 qxyz = new Vector3(currentQ.x, currentQ.y, currentQ.z);  
                            Vector3 projection = Vector3.Project(qxyz, twistAxis.normalized);   // 팔뚝 방향의 회전 성분만 추출

                            // 비틀기 전용 쿼터니언 
                            Quaternion twistQ = new Quaternion(projection.x, projection.y, projection.z, currentQ.w);
                            twistQ = NormalizeQuaternion(twistQ);   // 정규화

                            // 꺾기 전용 쿼터니언 
                            Quaternion swingQ = currentQ * Quaternion.Inverse(twistQ);

                            // 회전 값 적용
                            if (forearmTransform != null)
                            {
                                forearmTransform.localRotation = twistQ;
                            }
                        
                            if (wristTransform != null)
                            {
                                wristTransform.localRotation = swingQ;
                            }
                        }
                    }
                }
            }
            catch (System.TimeoutException)
            {
                // 데이터 수신 대기 초과 무시
            }
            catch (System.Exception e)
            {
                Debug.LogWarning("에러 발생: " + e.Message);
            }
        }
    }

    void OnApplicationQuit()
    {
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
            Debug.Log("시리얼 포트 닫힘");
        }
    }

    // 길이를 1로 만드는 정규화 함수 정의
    private Quaternion NormalizeQuaternion(Quaternion q)
    {
        float length = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

        if (length == 0) return Quaternion.identity;

        return new Quaternion(q.x / length, q.y / length, q.z / length, q.w / length);
    }
}