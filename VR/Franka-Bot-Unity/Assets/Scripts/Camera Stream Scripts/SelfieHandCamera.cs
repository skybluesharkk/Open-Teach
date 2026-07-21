using UnityEngine;

/// <summary>
/// 트래킹된 손을 "나와 마주보는(내 시선 반대) 시점"으로 비추는 셀피 카메라.
///
/// 매 프레임 헤드(CenterEyeAnchor) 앞쪽 distance 만큼 카메라를 놓고,
/// 카메라가 나를 향해 되돌아보게(forward = 내 시선의 반대) 회전시킨다.
/// 이 카메라의 Target Texture(RenderTexture)를 UI RawImage에 꽂으면
/// 패널 안에 "정면에서 본 내 손 + 택타일 오버레이"가 나온다.
/// </summary>
public class SelfieHandCamera : MonoBehaviour
{
    [Tooltip("따라다닐 헤드 앵커 (OVRCameraRig/TrackingSpace/CenterEyeAnchor)")]
    public Transform head;

    [Tooltip("헤드 앞쪽으로 카메라를 얼마나 떨어뜨릴지 (m)")]
    public float distance = 0.6f;

    [Tooltip("손 높이에 맞춰 카메라를 얼마나 내릴지 (m, 음수=아래로)")]
    public float heightOffset = -0.15f;

    [Tooltip("고개를 숙이거나 기울여도 카메라 수평 유지 (yaw만 따라감)")]
    public bool yawOnly = true;

    void LateUpdate()
    {
        if (head == null) return;

        Vector3 fwd = head.forward;
        if (yawOnly)
        {
            fwd.y = 0f;
            if (fwd.sqrMagnitude < 1e-6f) return;   // 정면이 수직일 때 방어
            fwd.Normalize();
        }

        // 카메라 위치 = 내 앞쪽
        Vector3 pos = head.position + fwd * distance;
        pos.y += heightOffset;
        transform.position = pos;

        // 나를 바라보게: 카메라 forward = 내 시선의 반대 방향
        transform.rotation = Quaternion.LookRotation(-fwd, Vector3.up);
    }
}
