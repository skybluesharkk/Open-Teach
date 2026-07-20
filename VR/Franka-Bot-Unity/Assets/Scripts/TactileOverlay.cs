using System;
using System.Collections.Generic;
using System.Globalization;
using System.Threading;

using UnityEngine;

using NetMQ;
using NetMQ.Sockets;

/// <summary>
/// XHand 택타일 센서값을 트래킹된 오른손 손끝 위에 시각화.
///
/// PC의 tactile_viz_dummy.py(또는 실센서 퍼블리셔)가 보내는 텍스트 패킷을
/// NetMQ SUB로 수신해, 패킷 prefix에 따라 세 가지 모드로 렌더링한다:
///   F1  : 손끝 색상 히트맵 — 구체 색이 파랑(약함)→빨강(강함), 힘 없으면 숨김
///   F2A : 손끝당 합산 힘 벡터 화살표 1개 — 방향 + 크기(길이/색)
///   F2B : 손끝당 rows x cols 벡터장 — TactAR식 deformation field
///
/// 앵커: OVRSkeleton 오른손 손끝 본 5개 (ThumbTip~PinkyTip).
/// 씬 설정: 빈 GameObject에 이 스크립트를 붙이고 RightHandSkeleton을
/// 인스펙터에서 할당 (GestureDetector와 같은 오브젝트 재사용 가능).
/// 미할당 시 런타임에 자동 탐색한다.
/// </summary>
public class TactileOverlay : MonoBehaviour
{
    public OVRSkeleton RightHandSkeleton;

    [Header("F1 히트맵")]
    public float sphereDiameter = 0.018f;      // 손끝 구체 지름 (m)

    [Header("F2 화살표")]
    public float forceToLength = 0.015f;       // 1N 당 화살표 길이 (m)
    public float maxArrowLength = 0.08f;       // 화살표 최대 길이 (m)
    public float shaftWidth = 0.004f;          // F2A 축 두께 (m)
    public float gridShaftWidth = 0.0015f;     // F2B 축 두께 (m)
    public float gridSpacing = 0.004f;         // F2B 그리드 간격 (m)
    public float gridForceToLength = 0.006f;   // F2B 1N 당 길이 (m)

    [Header("색상")]
    public Color weakColor = new Color(0.15f, 0.4f, 1f);   // 파랑
    public Color strongColor = new Color(1f, 0.15f, 0.1f); // 빨강
    public float forceForFullColor = 5f;       // 이 힘(N)에서 완전 빨강

    // 손끝 본 ID (OVRSkeleton.BoneId) — 엄지,검지,중지,약지,소지 순
    private static readonly OVRSkeleton.BoneId[] TipBoneIds =
    {
        OVRSkeleton.BoneId.Hand_ThumbTip,
        OVRSkeleton.BoneId.Hand_IndexTip,
        OVRSkeleton.BoneId.Hand_MiddleTip,
        OVRSkeleton.BoneId.Hand_RingTip,
        OVRSkeleton.BoneId.Hand_PinkyTip,
    };
    private const int NumTips = 5;

    // ── 수신 ──────────────────────────────────────────────────────────
    private Thread receiverThread;
    private volatile string latestPacket;
    private bool connectionEstablished = false;
    private string communicationAddress;
    private NetworkManager netConfig;
    private SubscriberSocket socket;

    // ── 시각화 오브젝트 풀 ─────────────────────────────────────────────
    private Transform[] tipBones = new Transform[NumTips];
    private bool bonesReady = false;

    private GameObject[] heatSpheres = new GameObject[NumTips];        // F1
    private Material[] heatMats = new Material[NumTips];
    private GameObject[] tipArrows = new GameObject[NumTips];          // F2A
    private Transform[] tipArrowShafts = new Transform[NumTips];
    private Material[] tipArrowMats = new Material[NumTips];
    private GameObject[][] gridArrows;                                  // F2B [tip][point]
    private Transform[][] gridArrowShafts;
    private Material[][] gridArrowMats;
    private int gridRows = 0, gridCols = 0;

    private string currentMode = "";

    // ═══════════════════════════════════════════════════════════════════
    //  수신 스레드 (GraphStream.cs 패턴)
    // ═══════════════════════════════════════════════════════════════════

    private void StartReceiverThread()
    {
        communicationAddress = netConfig.getTactileAddress();
        if (String.Equals(communicationAddress, "tcp://:"))
            return;

        socket = new SubscriberSocket();
        socket.Options.ReceiveHighWatermark = 5;
        socket.Connect(communicationAddress);
        socket.Subscribe("");
        connectionEstablished = true;

        receiverThread = new Thread(ReceiveLoop);
        receiverThread.Start();
    }

    private void ReceiveLoop()
    {
        while (true)
        {
            string packet = socket.ReceiveFrameString();
            latestPacket = packet;   // 최신 패킷만 유지
        }
    }

    void Start()
    {
        GameObject netConfGame = GameObject.Find("NetworkConfigsLoader");
        netConfig = netConfGame.GetComponent<NetworkManager>();
    }

    void OnDestroy()
    {
        if (receiverThread != null) receiverThread.Abort();
        if (socket != null) socket.Close();
    }

    // ═══════════════════════════════════════════════════════════════════
    //  본 탐색
    // ═══════════════════════════════════════════════════════════════════

    private bool TryResolveBones()
    {
        if (RightHandSkeleton == null)
        {
            foreach (var sk in FindObjectsOfType<OVRSkeleton>())
            {
                if (sk.GetSkeletonType() == OVRSkeleton.SkeletonType.HandRight)
                {
                    RightHandSkeleton = sk;
                    break;
                }
            }
            if (RightHandSkeleton == null) return false;
        }

        if (RightHandSkeleton.Bones == null || RightHandSkeleton.Bones.Count == 0)
            return false;

        int found = 0;
        foreach (var bone in RightHandSkeleton.Bones)
        {
            for (int i = 0; i < NumTips; i++)
            {
                if (bone.Id == TipBoneIds[i])
                {
                    tipBones[i] = bone.Transform;
                    found++;
                }
            }
        }
        return found == NumTips;
    }

    // ═══════════════════════════════════════════════════════════════════
    //  시각화 오브젝트 생성 (프리팹 없이 프리미티브로)
    // ═══════════════════════════════════════════════════════════════════

    private Material MakeMat()
    {
        var mat = new Material(Shader.Find("Standard"));
        mat.EnableKeyword("_EMISSION");
        return mat;
    }

    private void SetMatColor(Material mat, Color c)
    {
        mat.color = c;
        mat.SetColor("_EmissionColor", c * 0.6f);
    }

    /// <summary>Cylinder 축 + Sphere 머리로 화살표 생성 (TactAR 방식).</summary>
    private GameObject MakeArrow(float width, out Transform shaft, out Material mat)
    {
        var root = new GameObject("TactArrow");

        var shaftGo = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        Destroy(shaftGo.GetComponent<Collider>());
        shaftGo.transform.parent = root.transform;
        shaftGo.transform.localPosition = new Vector3(0, 0.5f, 0); // 길이 1 기준, 스케일로 조절
        shaftGo.transform.localScale = new Vector3(width, 0.5f, width);

        var headGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        Destroy(headGo.GetComponent<Collider>());
        headGo.transform.parent = root.transform;
        headGo.transform.localPosition = new Vector3(0, 1f, 0);
        headGo.transform.localScale = Vector3.one * width * 2.2f;

        mat = MakeMat();
        shaftGo.GetComponent<Renderer>().material = mat;
        headGo.GetComponent<Renderer>().material = mat;

        shaft = root.transform;
        root.SetActive(false);
        return root;
    }

    private void EnsureF1Objects()
    {
        if (heatSpheres[0] != null) return;
        for (int i = 0; i < NumTips; i++)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            Destroy(go.GetComponent<Collider>());
            go.name = "TactHeat_" + i;
            go.transform.localScale = Vector3.one * sphereDiameter;
            heatMats[i] = MakeMat();
            go.GetComponent<Renderer>().material = heatMats[i];
            go.SetActive(false);
            heatSpheres[i] = go;
        }
    }

    private void EnsureF2AObjects()
    {
        if (tipArrows[0] != null) return;
        for (int i = 0; i < NumTips; i++)
        {
            tipArrows[i] = MakeArrow(shaftWidth, out tipArrowShafts[i], out tipArrowMats[i]);
            tipArrows[i].name = "TactVec_" + i;
        }
    }

    private void EnsureF2BObjects(int rows, int cols)
    {
        if (gridArrows != null && rows == gridRows && cols == gridCols) return;

        // 그리드 크기 변경 시 기존 오브젝트 제거 후 재생성
        if (gridArrows != null)
            foreach (var tipSet in gridArrows)
                foreach (var go in tipSet)
                    Destroy(go);

        gridRows = rows;
        gridCols = cols;
        int n = rows * cols;
        gridArrows = new GameObject[NumTips][];
        gridArrowShafts = new Transform[NumTips][];
        gridArrowMats = new Material[NumTips][];

        for (int t = 0; t < NumTips; t++)
        {
            gridArrows[t] = new GameObject[n];
            gridArrowShafts[t] = new Transform[n];
            gridArrowMats[t] = new Material[n];
            for (int p = 0; p < n; p++)
            {
                gridArrows[t][p] = MakeArrow(gridShaftWidth,
                    out gridArrowShafts[t][p], out gridArrowMats[t][p]);
                gridArrows[t][p].name = $"TactGrid_{t}_{p}";
            }
        }
    }

    private void HideAll(string exceptMode)
    {
        if (exceptMode != "F1" && heatSpheres[0] != null)
            foreach (var go in heatSpheres) go.SetActive(false);
        if (exceptMode != "F2A" && tipArrows[0] != null)
            foreach (var go in tipArrows) go.SetActive(false);
        if (exceptMode != "F2B" && gridArrows != null)
            foreach (var tipSet in gridArrows)
                foreach (var go in tipSet) go.SetActive(false);
    }

    // ═══════════════════════════════════════════════════════════════════
    //  렌더링
    // ═══════════════════════════════════════════════════════════════════

    private Color ForceColor(float forceN)
    {
        float k = Mathf.Clamp01(forceN / forceForFullColor);
        return Color.Lerp(weakColor, strongColor, k);
    }

    /// <summary>화살표를 world 시작점 + world 벡터로 배치.</summary>
    private void PlaceArrow(Transform arrowRoot, Vector3 worldStart, Vector3 worldVec, float length)
    {
        arrowRoot.position = worldStart;
        if (worldVec.sqrMagnitude > 1e-10f)
            arrowRoot.rotation = Quaternion.FromToRotation(Vector3.up, worldVec.normalized);
        arrowRoot.localScale = new Vector3(1f, length, 1f);
    }

    /// <summary>손끝 로컬 힘 벡터 → world 방향. fz=법선(손끝 바깥), fx/fy=접선.</summary>
    private Vector3 TipLocalToWorld(int tip, float fx, float fy, float fz)
    {
        Transform b = tipBones[tip];
        // OVR 손 본: 손끝 패드의 대략적 법선을 본의 -Y로 근사 (기기에서 보고 조정 가능)
        return b.right * fx + b.forward * fy + (-b.up) * fz;
    }

    private void RenderF1(string payload)
    {
        EnsureF1Objects();
        string[] vals = payload.Split(',');
        for (int i = 0; i < NumTips && i < vals.Length; i++)
        {
            float v = ParseF(vals[i]);   // 0~1
            if (v < 0.02f || tipBones[i] == null)
            {
                heatSpheres[i].SetActive(false);
                continue;
            }
            heatSpheres[i].SetActive(true);
            heatSpheres[i].transform.position = tipBones[i].position;
            SetMatColor(heatMats[i], Color.Lerp(weakColor, strongColor, v));
        }
    }

    private void RenderF2A(string payload)
    {
        EnsureF2AObjects();
        string[] vecs = payload.Split('|');
        for (int i = 0; i < NumTips && i < vecs.Length; i++)
        {
            string[] c = vecs[i].Split(',');
            if (c.Length < 3 || tipBones[i] == null) { tipArrows[i].SetActive(false); continue; }

            float fx = ParseF(c[0]), fy = ParseF(c[1]), fz = ParseF(c[2]);
            float mag = Mathf.Sqrt(fx * fx + fy * fy + fz * fz);
            if (mag < 0.1f) { tipArrows[i].SetActive(false); continue; }

            Vector3 dir = TipLocalToWorld(i, fx, fy, fz);
            float len = Mathf.Min(mag * forceToLength, maxArrowLength);

            tipArrows[i].SetActive(true);
            PlaceArrow(tipArrows[i].transform, tipBones[i].position, dir, len);
            SetMatColor(tipArrowMats[i], ForceColor(mag));
        }
    }

    private void RenderF2B(string payload)
    {
        // 형식: rows,cols;tip0|tip1|...  (tip = v,v,v,... rows*cols*3개)
        int semi = payload.IndexOf(';');
        if (semi < 0) return;
        string[] dims = payload.Substring(0, semi).Split(',');
        int rows = int.Parse(dims[0]), cols = int.Parse(dims[1]);
        EnsureF2BObjects(rows, cols);

        string[] tips = payload.Substring(semi + 1).Split('|');
        for (int t = 0; t < NumTips && t < tips.Length; t++)
        {
            if (tipBones[t] == null) continue;
            Transform bone = tipBones[t];
            string[] c = tips[t].Split(',');

            for (int r = 0; r < rows; r++)
            {
                for (int col = 0; col < cols; col++)
                {
                    int p = r * cols + col;
                    int o = p * 3;
                    if (o + 2 >= c.Length) break;

                    float fx = ParseF(c[o]), fy = ParseF(c[o + 1]), fz = ParseF(c[o + 2]);
                    float mag = Mathf.Sqrt(fx * fx + fy * fy + fz * fz);
                    var arrow = gridArrows[t][p];

                    if (mag < 0.05f) { arrow.SetActive(false); continue; }

                    // 그리드 점의 손끝 패드 위 로컬 배치 (본 로컬 접선 평면)
                    float gx = (col - (cols - 1) * 0.5f) * gridSpacing;
                    float gy = (r - (rows - 1) * 0.5f) * gridSpacing;
                    Vector3 start = bone.position
                                  + bone.right * gx
                                  + bone.forward * gy;

                    Vector3 dir = TipLocalToWorld(t, fx, fy, fz);
                    float len = Mathf.Min(mag * gridForceToLength, maxArrowLength * 0.5f);

                    arrow.SetActive(true);
                    PlaceArrow(arrow.transform, start, dir, len);
                    SetMatColor(gridArrowMats[t][p], ForceColor(mag));
                }
            }
        }
    }

    private static float ParseF(string s)
    {
        float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v);
        return v;
    }

    // ═══════════════════════════════════════════════════════════════════

    void Update()
    {
        if (!connectionEstablished)
        {
            if (netConfig != null) StartReceiverThread();
            return;
        }

        if (!bonesReady)
        {
            bonesReady = TryResolveBones();
            if (!bonesReady) return;
        }

        string packet = latestPacket;
        if (packet == null) return;

        int colon = packet.IndexOf(':');
        if (colon < 0) return;
        string mode = packet.Substring(0, colon);
        string payload = packet.Substring(colon + 1);

        if (mode != currentMode)
        {
            HideAll(mode);
            currentMode = mode;
        }

        switch (mode)
        {
            case "F1": RenderF1(payload); break;
            case "F2A": RenderF2A(payload); break;
            case "F2B": RenderF2B(payload); break;
        }
    }
}
