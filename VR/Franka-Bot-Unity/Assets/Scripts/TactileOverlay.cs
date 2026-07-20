using System;
using System.Globalization;
using System.Threading;

using UnityEngine;

using NetMQ;
using NetMQ.Sockets;

/// <summary>
/// XHand 택타일 센서값을 트래킹된 오른손 손가락 위에 시각화.
///
/// PC의 tactile_viz_dummy.py(또는 실센서 퍼블리셔)가 보내는 텍스트 패킷을
/// NetMQ SUB로 수신해, 패킷 prefix에 따라 세 가지 모드로 렌더링한다:
///   F1  : 손끝 마디 표면 색상 — 파랑(약함)→빨강(강함), 센서 존재 부위를 덮음
///   F2A : 손끝당 힘 벡터 화살표 1개 — 손가락에서 하늘로 솟는 화살표
///   F2B : 손끝당 rows x cols 벡터장 — 손끝 패드 중심의 deformation field
///
/// 좌표계 가정(사용자 시나리오): 손등이 하늘, 손바닥이 지면.
/// 힘 화살표는 손가락에서 '하늘(world +Y)' 방향으로 솟는다.
///
/// 앵커: OVRSkeleton 오른손 손끝/말단 본.
/// 씬 설정: 빈 GameObject에 붙이고 RightHandSkeleton 할당(미할당 시 자동 탐색).
///
/// 안정성: 트래킹 유효성 검사 + NaN 방어 + try/catch 로 손이 화면 밖으로
/// 나가거나 트래킹이 끊겨도 앱이 얼지 않는다(그냥 숨김 처리).
/// </summary>
public class TactileOverlay : MonoBehaviour
{
    public OVRSkeleton RightHandSkeleton;

    [Header("F1 표면 히트")]
    public float padThickness = 0.011f;    // 손끝 마디 덮개 두께/지름 (m)

    [Header("F2 화살표")]
    public float forceToLength = 0.015f;   // 1N 당 화살표 길이 (m)
    public float maxArrowLength = 0.09f;   // 화살표 최대 길이 (m)
    public float shaftWidth = 0.0035f;     // F2A 축 두께 (m)
    public float headLength = 0.014f;      // F2A 화살촉 길이 (m)
    public float gridShaftWidth = 0.0014f; // F2B 축 두께 (m)
    public float gridHeadLength = 0.006f;  // F2B 화살촉 길이 (m)
    public float gridSpacing = 0.003f;     // F2B 그리드 간격 (m)
    public float gridForceToLength = 0.008f; // F2B 1N 당 길이 (m)

    [Header("색상")]
    public Color weakColor = new Color(0.15f, 0.45f, 1f);   // 파랑
    public Color strongColor = new Color(1f, 0.12f, 0.08f); // 빨강
    public float forceForFullColor = 5f;   // 이 힘(N)에서 완전 빨강

    [Header("임계값")]
    public float scalarThreshold = 0.02f;  // F1 이하이면 숨김 (0~1)
    public float forceThreshold = 0.1f;    // F2 이하이면 숨김 (N)
    public int maxGridPerTip = 64;         // F2B 손끝당 최대 점 개수(폭주 방지)

    // 손끝 tip 본 — 엄지,검지,중지,약지,소지
    private static readonly OVRSkeleton.BoneId[] TipBoneIds =
    {
        OVRSkeleton.BoneId.Hand_ThumbTip,  OVRSkeleton.BoneId.Hand_IndexTip,
        OVRSkeleton.BoneId.Hand_MiddleTip, OVRSkeleton.BoneId.Hand_RingTip,
        OVRSkeleton.BoneId.Hand_PinkyTip,
    };
    // 손끝 말단(distal) 본 — 패드 중심 계산용
    private static readonly OVRSkeleton.BoneId[] DistalBoneIds =
    {
        OVRSkeleton.BoneId.Hand_Thumb3,  OVRSkeleton.BoneId.Hand_Index3,
        OVRSkeleton.BoneId.Hand_Middle3, OVRSkeleton.BoneId.Hand_Ring3,
        OVRSkeleton.BoneId.Hand_Pinky3,
    };
    private const int NumTips = 5;

    // ── 수신 ──────────────────────────────────────────────────────────
    private Thread receiverThread;
    private volatile bool running = false;
    private volatile string latestPacket;
    private bool connectionEstablished = false;
    private string communicationAddress;
    private NetworkManager netConfig;
    private SubscriberSocket socket;

    // ── 본 ─────────────────────────────────────────────────────────────
    private Transform[] tipBones = new Transform[NumTips];
    private Transform[] distalBones = new Transform[NumTips];
    private bool bonesReady = false;

    // ── 시각화 오브젝트 ─────────────────────────────────────────────────
    private GameObject[] padCaps = new GameObject[NumTips];        // F1
    private Material[] padMats = new Material[NumTips];
    private GameObject[] tipArrows = new GameObject[NumTips];      // F2A
    private Transform[] tipShafts = new Transform[NumTips];
    private Material[] tipMats = new Material[NumTips];
    private GameObject[][] gridArrows;                             // F2B
    private Transform[][] gridShafts;
    private Material[][] gridMats;
    private int gridRows = 0, gridCols = 0;

    private Mesh coneMesh;   // 화살촉 공유 메시
    private string currentMode = "";

    // ═══════════════════════════════════════════════════════════════════
    //  수신 스레드
    // ═══════════════════════════════════════════════════════════════════

    private void StartReceiverThread()
    {
        communicationAddress = netConfig.getTactileAddress();
        if (String.Equals(communicationAddress, "tcp://:"))
            return;

        try
        {
            socket = new SubscriberSocket();
            socket.Options.ReceiveHighWatermark = 5;
            socket.Connect(communicationAddress);
            socket.Subscribe("");
        }
        catch (Exception e)
        {
            Debug.LogWarning("[Tactile] 소켓 연결 실패: " + e.Message);
            return;
        }

        connectionEstablished = true;
        running = true;
        receiverThread = new Thread(ReceiveLoop) { IsBackground = true };
        receiverThread.Start();
    }

    private void ReceiveLoop()
    {
        while (running)
        {
            try
            {
                string packet = socket.ReceiveFrameString();
                latestPacket = packet;   // 최신 패킷만 유지
            }
            catch (Exception)
            {
                break;   // 소켓 닫힘 등 → 스레드 종료
            }
        }
    }

    void Start()
    {
        GameObject netConfGame = GameObject.Find("NetworkConfigsLoader");
        if (netConfGame != null)
            netConfig = netConfGame.GetComponent<NetworkManager>();
        coneMesh = BuildConeMesh(12);
    }

    void OnDestroy()
    {
        running = false;
        try { if (socket != null) socket.Close(); } catch { }
    }

    // ═══════════════════════════════════════════════════════════════════
    //  본 탐색
    // ═══════════════════════════════════════════════════════════════════

    private bool TryResolveBones()
    {
        if (RightHandSkeleton == null)
        {
            foreach (var sk in FindObjectsOfType<OVRSkeleton>())
                if (sk.GetSkeletonType() == OVRSkeleton.SkeletonType.HandRight)
                { RightHandSkeleton = sk; break; }
            if (RightHandSkeleton == null) return false;
        }
        if (RightHandSkeleton.Bones == null || RightHandSkeleton.Bones.Count == 0)
            return false;

        int found = 0;
        foreach (var bone in RightHandSkeleton.Bones)
        {
            for (int i = 0; i < NumTips; i++)
            {
                if (bone.Id == TipBoneIds[i])    { tipBones[i] = bone.Transform; found++; }
                if (bone.Id == DistalBoneIds[i]) { distalBones[i] = bone.Transform; }
            }
        }
        return found == NumTips;
    }

    private bool HandTracked()
    {
        return RightHandSkeleton != null
            && RightHandSkeleton.IsInitialized
            && RightHandSkeleton.IsDataValid;
    }

    private static bool Finite(Vector3 v)
    {
        return !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z)
              || float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));
    }

    // ═══════════════════════════════════════════════════════════════════
    //  프리미티브 생성
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
        mat.SetColor("_EmissionColor", c * 0.7f);
    }

    /// <summary>화살촉 원뿔 메시. apex(0,0,0) → base(0,height,1) 반경 0.5.</summary>
    private Mesh BuildConeMesh(int seg)
    {
        var mesh = new Mesh { name = "TactCone" };
        var verts = new Vector3[seg + 2];
        verts[0] = Vector3.zero;                 // apex (아래, 손가락쪽)
        verts[1] = new Vector3(0, 1, 0);         // base center
        for (int i = 0; i < seg; i++)
        {
            float a = (i / (float)seg) * Mathf.PI * 2f;
            verts[2 + i] = new Vector3(0.5f * Mathf.Cos(a), 1f, 0.5f * Mathf.Sin(a));
        }
        var tris = new System.Collections.Generic.List<int>();
        for (int i = 0; i < seg; i++)
        {
            int a = 2 + i, b = 2 + (i + 1) % seg;
            tris.Add(0); tris.Add(b); tris.Add(a);   // 옆면
            tris.Add(1); tris.Add(a); tris.Add(b);   // 밑면
        }
        mesh.vertices = verts;
        mesh.triangles = tris.ToArray();
        mesh.RecalculateNormals();
        return mesh;
    }

    /// <summary>
    /// 화살표: 손가락쪽(y=0)에 화살촉(apex 아래) + 위로 뻗는 축.
    /// 축 길이는 매 프레임 조절, 화살촉 크기는 고정.
    /// </summary>
    private GameObject MakeArrow(float width, float headLen, out Transform shaft, out Material mat)
    {
        var root = new GameObject("TactArrow");
        mat = MakeMat();

        var head = new GameObject("head");
        head.transform.SetParent(root.transform, false);
        head.AddComponent<MeshFilter>().sharedMesh = coneMesh;
        head.AddComponent<MeshRenderer>().sharedMaterial = mat;
        head.transform.localScale = new Vector3(width * 2.4f, headLen, width * 2.4f);
        head.transform.localPosition = Vector3.zero;   // apex가 손가락 접점

        var shaftGo = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        Destroy(shaftGo.GetComponent<Collider>());
        shaftGo.transform.SetParent(root.transform, false);
        shaftGo.GetComponent<Renderer>().sharedMaterial = mat;
        // 기본 실린더는 높이2(±1). 아래에서 PlaceArrow가 scale.y/pos.y 설정
        shaftGo.transform.localScale = new Vector3(width, 0.01f, width);
        shaftGo.transform.localPosition = new Vector3(0, headLen, 0);

        shaft = shaftGo.transform;
        root.SetActive(false);
        return root;
    }

    private void EnsureF1()
    {
        if (padCaps[0] != null) return;
        for (int i = 0; i < NumTips; i++)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            Destroy(go.GetComponent<Collider>());
            go.name = "TactPad_" + i;
            padMats[i] = MakeMat();
            go.GetComponent<Renderer>().sharedMaterial = padMats[i];
            go.SetActive(false);
            padCaps[i] = go;
        }
    }

    private void EnsureF2A()
    {
        if (tipArrows[0] != null) return;
        for (int i = 0; i < NumTips; i++)
        {
            tipArrows[i] = MakeArrow(shaftWidth, headLength, out tipShafts[i], out tipMats[i]);
            tipArrows[i].name = "TactVec_" + i;
        }
    }

    private void EnsureF2B(int rows, int cols)
    {
        if (gridArrows != null && rows == gridRows && cols == gridCols) return;

        if (gridArrows != null)
            foreach (var set in gridArrows)
                foreach (var go in set) Destroy(go);

        gridRows = rows; gridCols = cols;
        int n = rows * cols;
        gridArrows = new GameObject[NumTips][];
        gridShafts = new Transform[NumTips][];
        gridMats = new Material[NumTips][];
        for (int t = 0; t < NumTips; t++)
        {
            gridArrows[t] = new GameObject[n];
            gridShafts[t] = new Transform[n];
            gridMats[t] = new Material[n];
            for (int p = 0; p < n; p++)
            {
                gridArrows[t][p] = MakeArrow(gridShaftWidth, gridHeadLength,
                    out gridShafts[t][p], out gridMats[t][p]);
                gridArrows[t][p].name = $"TactGrid_{t}_{p}";
                gridMats[t][p] = gridArrows[t][p].GetComponentInChildren<Renderer>().sharedMaterial;
            }
        }
    }

    private void HideAll(string except)
    {
        if (except != "F1" && padCaps[0] != null)
            foreach (var go in padCaps) go.SetActive(false);
        if (except != "F2A" && tipArrows[0] != null)
            foreach (var go in tipArrows) go.SetActive(false);
        if (except != "F2B" && gridArrows != null)
            foreach (var set in gridArrows)
                foreach (var go in set) go.SetActive(false);
    }

    // ═══════════════════════════════════════════════════════════════════
    //  렌더링 헬퍼
    // ═══════════════════════════════════════════════════════════════════

    private Color ForceColor(float f)
    {
        return Color.Lerp(weakColor, strongColor, Mathf.Clamp01(f / forceForFullColor));
    }

    /// <summary>손끝 로컬 힘(fx 접선, fy 접선, fz 법선) → world 방향. 법선=하늘(+Y).</summary>
    private Vector3 ForceToWorldDir(float fx, float fy, float fz)
    {
        // 시나리오: 손등이 하늘. 누르는 반력은 하늘로 솟음.
        return (Vector3.right * fx + Vector3.up * fz + Vector3.forward * fy);
    }

    /// <summary>화살표 배치: 손가락에서 dir 방향으로 length 만큼.</summary>
    private void PlaceArrow(Transform root, Transform shaft, float headLen,
                            Vector3 start, Vector3 dir, float length, float width)
    {
        root.position = start;
        root.rotation = Quaternion.FromToRotation(Vector3.up, dir.normalized);
        // 축: headLen 위에서 length 만큼 (실린더 기본 높이 2 → scale.y = length/2)
        shaft.localScale = new Vector3(width, length * 0.5f, width);
        shaft.localPosition = new Vector3(0, headLen + length * 0.5f, 0);
    }

    private static float ParseF(string s)
    {
        float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v);
        return v;
    }

    // ── F1: 손끝 마디 표면 히트 ─────────────────────────────────────────
    private void RenderF1(string payload)
    {
        EnsureF1();
        string[] vals = payload.Split(',');
        for (int i = 0; i < NumTips; i++)
        {
            if (i >= vals.Length || tipBones[i] == null || distalBones[i] == null)
            { padCaps[i].SetActive(false); continue; }

            float v = ParseF(vals[i]);
            Vector3 tip = tipBones[i].position, dist = distalBones[i].position;
            if (v < scalarThreshold || !Finite(tip) || !Finite(dist))
            { padCaps[i].SetActive(false); continue; }

            // 말단~손끝 마디를 덮는 캡슐
            Vector3 center = (tip + dist) * 0.5f;
            Vector3 axis = tip - dist;
            float len = axis.magnitude;

            var tr = padCaps[i].transform;
            tr.position = center;
            if (len > 1e-5f) tr.rotation = Quaternion.FromToRotation(Vector3.up, axis.normalized);
            // 캡슐 기본 높이 2(±1) → scale.y = 마디길이/2 + 여유
            tr.localScale = new Vector3(padThickness, len * 0.5f + padThickness * 0.5f, padThickness);

            padCaps[i].SetActive(true);
            SetMatColor(padMats[i], Color.Lerp(weakColor, strongColor, v));
        }
    }

    // ── F2A: 손끝당 힘 벡터 화살표 ──────────────────────────────────────
    private void RenderF2A(string payload)
    {
        EnsureF2A();
        string[] vecs = payload.Split('|');
        for (int i = 0; i < NumTips; i++)
        {
            if (i >= vecs.Length || tipBones[i] == null) { tipArrows[i].SetActive(false); continue; }
            string[] c = vecs[i].Split(',');
            if (c.Length < 3) { tipArrows[i].SetActive(false); continue; }

            float fx = ParseF(c[0]), fy = ParseF(c[1]), fz = ParseF(c[2]);
            float mag = Mathf.Sqrt(fx * fx + fy * fy + fz * fz);
            Vector3 start = tipBones[i].position;
            Vector3 dir = ForceToWorldDir(fx, fy, fz);
            if (mag < forceThreshold || !Finite(start) || dir.sqrMagnitude < 1e-8f)
            { tipArrows[i].SetActive(false); continue; }

            float len = Mathf.Min(mag * forceToLength, maxArrowLength);
            tipArrows[i].SetActive(true);
            PlaceArrow(tipArrows[i].transform, tipShafts[i], headLength, start, dir, len, shaftWidth);
            SetMatColor(tipMats[i], ForceColor(mag));
        }
    }

    // ── F2B: 손끝 패드 벡터장 ───────────────────────────────────────────
    private void RenderF2B(string payload)
    {
        int semi = payload.IndexOf(';');
        if (semi < 0) return;
        string[] dims = payload.Substring(0, semi).Split(',');
        if (dims.Length < 2) return;
        if (!int.TryParse(dims[0], out int rows) || !int.TryParse(dims[1], out int cols)) return;
        if (rows < 1 || cols < 1 || rows * cols > maxGridPerTip) return;   // 폭주 방지
        EnsureF2B(rows, cols);

        string[] tips = payload.Substring(semi + 1).Split('|');
        for (int t = 0; t < NumTips; t++)
        {
            if (t >= tips.Length || tipBones[t] == null || distalBones[t] == null)
            { foreach (var go in gridArrows[t]) go.SetActive(false); continue; }

            Vector3 tip = tipBones[t].position, dist = distalBones[t].position;
            if (!Finite(tip) || !Finite(dist))
            { foreach (var go in gridArrows[t]) go.SetActive(false); continue; }

            // 패드 중심 = 말단~손끝 마디 중점, 그리드는 그 위에 펼침
            Vector3 padCenter = (tip + dist) * 0.5f;
            Vector3 longAxis = tip - dist;
            if (longAxis.sqrMagnitude < 1e-8f) longAxis = Vector3.forward;
            longAxis.Normalize();
            Vector3 widthAxis = Vector3.Cross(longAxis, Vector3.up);
            if (widthAxis.sqrMagnitude < 1e-6f) widthAxis = Vector3.right;
            widthAxis.Normalize();

            string[] c = tips[t].Split(',');
            for (int r = 0; r < rows; r++)
            {
                for (int col = 0; col < cols; col++)
                {
                    int p = r * cols + col, o = p * 3;
                    var arrow = gridArrows[t][p];
                    if (o + 2 >= c.Length) { arrow.SetActive(false); continue; }

                    float fx = ParseF(c[o]), fy = ParseF(c[o + 1]), fz = ParseF(c[o + 2]);
                    float mag = Mathf.Sqrt(fx * fx + fy * fy + fz * fz);
                    if (mag < forceThreshold * 0.5f) { arrow.SetActive(false); continue; }

                    float gx = (col - (cols - 1) * 0.5f) * gridSpacing;
                    float gy = (r - (rows - 1) * 0.5f) * gridSpacing;
                    Vector3 start = padCenter + widthAxis * gx + longAxis * gy;
                    Vector3 dir = ForceToWorldDir(fx, fy, fz);
                    if (!Finite(start) || dir.sqrMagnitude < 1e-8f) { arrow.SetActive(false); continue; }

                    float len = Mathf.Min(mag * gridForceToLength, maxArrowLength * 0.5f);
                    arrow.SetActive(true);
                    PlaceArrow(arrow.transform, gridShafts[t][p], gridHeadLength,
                               start, dir, len, gridShaftWidth);
                    SetMatColor(gridMats[t][p], ForceColor(mag));
                }
            }
        }
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

        // 트래킹 끊김/화면 밖 → 전부 숨기고 대기 (앱 얼음 방지 핵심)
        if (!HandTracked())
        {
            HideAll("");
            return;
        }

        string packet = latestPacket;
        if (packet == null) return;

        int colon = packet.IndexOf(':');
        if (colon < 0) return;
        string mode = packet.Substring(0, colon);
        string payload = packet.Substring(colon + 1);

        if (mode != currentMode) { HideAll(mode); currentMode = mode; }

        // 한 프레임의 파싱/렌더 오류가 앱 전체를 멈추지 않도록 격리
        try
        {
            switch (mode)
            {
                case "F1":  RenderF1(payload);  break;
                case "F2A": RenderF2A(payload); break;
                case "F2B": RenderF2B(payload); break;
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning("[Tactile] 렌더 예외(무시): " + e.Message);
            HideAll("");
        }
    }
}
