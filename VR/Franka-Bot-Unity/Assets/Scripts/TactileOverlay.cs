using System;
using System.Globalization;
using System.Threading;

using UnityEngine;

using NetMQ;
using NetMQ.Sockets;

/// <summary>
/// XHand 택타일 센서값을 트래킹된 오른손 손가락 위에 시각화.
///
/// 패킷(텍스트, NetMQ SUB):
///   F1:rows,cols;v,..|v,..     손끝 5개 × 택셀 스칼라(0~1)  → 택셀별 색상 히트맵
///   F2A:fx,fy,fz|...           손끝 5개 합산 벡터           → 하늘로 솟는 화살표 5개
///   F2B:rows,cols;vx,vy,vz,..  손끝 5개 × 택셀 벡터          → 패드 중심 deformation field
///
/// 성능/안정성 핵심:
///  - 패킷은 바뀔 때만 파싱해 float 캐시에 저장(매 프레임 문자열 Split 금지 → GC 폭주 방지).
///  - 매 프레임엔 캐시 + 현재 본 위치로 오브젝트 위치만 갱신.
///  - 트래킹 유효성 검사 + NaN 방어 + try/catch(로그 레이트리밋)로 앱 멈춤 방지.
///  - OnApplicationPause 시 소켓/스레드 정리 후 재개.
/// </summary>
public class TactileOverlay : MonoBehaviour
{
    public OVRSkeleton RightHandSkeleton;

    [Header("F1 택셀 히트맵")]
    public float f1CellSize = 0.0032f;     // 택셀 셀 한 변 (m)
    public float f1CellSpacing = 0.0034f;  // 택셀 간격 (m)
    public float f1SurfaceOffset = 0.004f; // 손가락 표면에서 띄우는 높이 (m)

    [Header("F2 화살표")]
    public float forceToLength = 0.015f;
    public float maxArrowLength = 0.09f;
    public float shaftWidth = 0.0035f;
    public float headLength = 0.014f;
    public float gridShaftWidth = 0.0014f;
    public float gridHeadLength = 0.006f;
    public float gridSpacing = 0.003f;
    public float gridForceToLength = 0.008f;
    public float fingerRadius = 0.008f;    // 손끝 곡면 반경 (m) — 택셀을 이 원통에 감음

    [Header("색상")]
    public Color weakColor = new Color(0.15f, 0.45f, 1f);
    public Color strongColor = new Color(1f, 0.12f, 0.08f);
    public float forceForFullColor = 5f;

    [Header("임계값")]
    public float forceThreshold = 0.1f;    // F2 이하이면 숨김 (N)
    public int maxGridPerTip = 96;         // 손끝당 택셀 상한(폭주 방지)

    private static readonly OVRSkeleton.BoneId[] TipBoneIds =
    {
        OVRSkeleton.BoneId.Hand_ThumbTip,  OVRSkeleton.BoneId.Hand_IndexTip,
        OVRSkeleton.BoneId.Hand_MiddleTip, OVRSkeleton.BoneId.Hand_RingTip,
        OVRSkeleton.BoneId.Hand_PinkyTip,
    };
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

    // ── 파싱 캐시(패킷 바뀔 때만 갱신) ──────────────────────────────────
    private string processedPacket = null;
    private string curMode = "";
    private int f1Rows, f1Cols;
    private float[][] f1Data;              // [tip][rows*cols] 스칼라
    private float[][] f2aData;             // [tip][3]
    private int f2bRows, f2bCols;
    private float[][] f2bData;             // [tip][rows*cols*3]
    private bool f1Valid, f2aValid, f2bValid;

    // ── 시각화 오브젝트 ─────────────────────────────────────────────────
    private GameObject[][] f1Cells; private Material[][] f1Mats; private int f1cR, f1cC;
    private GameObject[] tipArrows = new GameObject[NumTips];
    private Transform[] tipShafts = new Transform[NumTips];
    private Material[] tipMats = new Material[NumTips];
    private GameObject[][] gridArrows; private Transform[][] gridShafts; private Material[][] gridMats;
    private int gridR = 0, gridC = 0;

    // ── 점진적 생성(프레임당 조금씩 만들어 스파이크 제거) ──────────────
    public int buildPerFrame = 10;         // 한 프레임에 생성할 오브젝트 수
    private int f1Built, f2bBuilt;         // 지금까지 생성된 개수
    private bool f1Building, f2bBuilding;

    private static Shader _stdShader;      // Shader.Find 1회 캐싱(성능 함정 회피)
    private Mesh coneMesh;
    private float lastLogTime = -10f;

    // ═══════════════════════════════════════════════════════════════════
    //  수신
    // ═══════════════════════════════════════════════════════════════════

    private void StartReceiverThread()
    {
        communicationAddress = netConfig.getTactileAddress();
        if (String.Equals(communicationAddress, "tcp://:")) return;
        try
        {
            socket = new SubscriberSocket();
            socket.Options.ReceiveHighWatermark = 2;
            socket.Connect(communicationAddress);
            socket.Subscribe("");
        }
        catch (Exception e) { Debug.LogWarning("[Tactile] 소켓 실패: " + e.Message); return; }

        connectionEstablished = true;
        running = true;
        receiverThread = new Thread(ReceiveLoop) { IsBackground = true };
        receiverThread.Start();
    }

    private void ReceiveLoop()
    {
        while (running)
        {
            try { latestPacket = socket.ReceiveFrameString(); }
            catch (Exception) { break; }
        }
    }

    private void StopReceiver()
    {
        running = false;
        try { if (socket != null) { socket.Close(); socket = null; } } catch { }
        connectionEstablished = false;
        processedPacket = null;
    }

    void Start()
    {
        var g = GameObject.Find("NetworkConfigsLoader");
        if (g != null) netConfig = g.GetComponent<NetworkManager>();
        coneMesh = BuildConeMesh(12);
    }

    void OnApplicationPause(bool paused)
    {
        if (paused) StopReceiver();   // 재개 시 Update가 다시 연결
    }

    void OnDestroy() { StopReceiver(); }

    // ═══════════════════════════════════════════════════════════════════
    //  본
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
        if (RightHandSkeleton.Bones == null || RightHandSkeleton.Bones.Count == 0) return false;

        int found = 0;
        foreach (var bone in RightHandSkeleton.Bones)
            for (int i = 0; i < NumTips; i++)
            {
                if (bone.Id == TipBoneIds[i])    { tipBones[i] = bone.Transform; found++; }
                if (bone.Id == DistalBoneIds[i]) { distalBones[i] = bone.Transform; }
            }
        return found == NumTips;
    }

    private bool HandTracked()
    {
        return RightHandSkeleton != null && RightHandSkeleton.IsInitialized
            && RightHandSkeleton.IsDataValid;
    }

    private static bool Finite(Vector3 v)
    {
        return !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z)
              || float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));
    }

    /// <summary>손끝 패드 좌표계: 중심=(tip+distal)/2, 길이축, 폭축, 법선.</summary>
    private bool PadFrame(int i, out Vector3 center, out Vector3 lengthAxis,
                          out Vector3 widthAxis, out Vector3 normal)
    {
        center = lengthAxis = widthAxis = normal = Vector3.zero;
        if (tipBones[i] == null || distalBones[i] == null) return false;
        Vector3 tip = tipBones[i].position, dist = distalBones[i].position;
        if (!Finite(tip) || !Finite(dist)) return false;

        center = (tip + dist) * 0.5f;
        lengthAxis = tip - dist;
        if (lengthAxis.sqrMagnitude < 1e-8f) lengthAxis = Vector3.forward;
        lengthAxis.Normalize();
        widthAxis = Vector3.Cross(lengthAxis, Vector3.up);
        if (widthAxis.sqrMagnitude < 1e-6f) widthAxis = Vector3.right;
        widthAxis.Normalize();
        normal = Vector3.Cross(widthAxis, lengthAxis).normalized;
        return true;
    }

    // ═══════════════════════════════════════════════════════════════════
    //  프리미티브
    // ═══════════════════════════════════════════════════════════════════

    private Material MakeMat()
    {
        if (_stdShader == null) _stdShader = Shader.Find("Standard");   // 1회만
        var mat = new Material(_stdShader);
        mat.EnableKeyword("_EMISSION");
        return mat;
    }
    private void SetMatColor(Material mat, Color c)
    {
        mat.color = c;
        mat.SetColor("_EmissionColor", c * 0.7f);
    }

    private Mesh BuildConeMesh(int seg)
    {
        var mesh = new Mesh { name = "TactCone" };
        var verts = new Vector3[seg + 2];
        verts[0] = Vector3.zero; verts[1] = new Vector3(0, 1, 0);
        for (int i = 0; i < seg; i++)
        {
            float a = (i / (float)seg) * Mathf.PI * 2f;
            verts[2 + i] = new Vector3(0.5f * Mathf.Cos(a), 1f, 0.5f * Mathf.Sin(a));
        }
        var tris = new System.Collections.Generic.List<int>();
        for (int i = 0; i < seg; i++)
        {
            int a = 2 + i, b = 2 + (i + 1) % seg;
            tris.Add(0); tris.Add(b); tris.Add(a);
            tris.Add(1); tris.Add(a); tris.Add(b);
        }
        mesh.vertices = verts; mesh.triangles = tris.ToArray();
        mesh.RecalculateNormals();
        return mesh;
    }

    private GameObject MakeArrow(float width, float headLen, out Transform shaft, out Material mat)
    {
        var root = new GameObject("TactArrow");
        mat = MakeMat();
        var head = new GameObject("head");
        head.transform.SetParent(root.transform, false);
        head.AddComponent<MeshFilter>().sharedMesh = coneMesh;
        head.AddComponent<MeshRenderer>().sharedMaterial = mat;
        head.transform.localScale = new Vector3(width * 2.4f, headLen, width * 2.4f);
        var shaftGo = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        Destroy(shaftGo.GetComponent<Collider>());
        shaftGo.transform.SetParent(root.transform, false);
        shaftGo.GetComponent<Renderer>().sharedMaterial = mat;
        shaftGo.transform.localScale = new Vector3(width, 0.01f, width);
        shaftGo.transform.localPosition = new Vector3(0, headLen, 0);
        shaft = shaftGo.transform;
        root.SetActive(false);
        return root;
    }

    // 크기 바뀌면 배열만 잡고(오브젝트는 null), 실제 생성은 BuildF1이 프레임당 조금씩
    private void EnsureF1(int rows, int cols)
    {
        if (f1Cells != null && rows == f1cR && cols == f1cC) return;
        if (f1Cells != null)
            foreach (var set in f1Cells) foreach (var go in set) if (go) Destroy(go);
        f1cR = rows; f1cC = cols;
        int n = rows * cols;
        f1Cells = new GameObject[NumTips][];
        f1Mats  = new Material[NumTips][];
        for (int t = 0; t < NumTips; t++) { f1Cells[t] = new GameObject[n]; f1Mats[t] = new Material[n]; }
        f1Built = 0; f1Building = true;
    }

    private void BuildF1()
    {
        if (!f1Building) return;
        int n = f1cR * f1cC, total = NumTips * n, budget = buildPerFrame;
        while (f1Built < total && budget-- > 0)
        {
            int t = f1Built / n, p = f1Built % n;
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Destroy(go.GetComponent<Collider>());
            go.name = $"TactCell_{t}_{p}";
            f1Mats[t][p] = MakeMat();
            go.GetComponent<Renderer>().sharedMaterial = f1Mats[t][p];
            go.SetActive(false);
            f1Cells[t][p] = go;
            f1Built++;
        }
        if (f1Built >= total) f1Building = false;
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
        if (gridArrows != null && rows == gridR && cols == gridC) return;
        if (gridArrows != null)
            foreach (var set in gridArrows) foreach (var go in set) if (go) Destroy(go);
        gridR = rows; gridC = cols;
        int n = rows * cols;
        gridArrows = new GameObject[NumTips][];
        gridShafts = new Transform[NumTips][];
        gridMats   = new Material[NumTips][];
        for (int t = 0; t < NumTips; t++)
        {
            gridArrows[t] = new GameObject[n];
            gridShafts[t] = new Transform[n];
            gridMats[t]   = new Material[n];
        }
        f2bBuilt = 0; f2bBuilding = true;
    }

    private void BuildF2B()
    {
        if (!f2bBuilding) return;
        int n = gridR * gridC, total = NumTips * n, budget = buildPerFrame;
        while (f2bBuilt < total && budget-- > 0)
        {
            int t = f2bBuilt / n, p = f2bBuilt % n;
            gridArrows[t][p] = MakeArrow(gridShaftWidth, gridHeadLength,
                out gridShafts[t][p], out gridMats[t][p]);
            f2bBuilt++;
        }
        if (f2bBuilt >= total) f2bBuilding = false;
    }

    private void HideAll(string except)
    {
        if (except != "F1" && f1Cells != null)
            foreach (var set in f1Cells) foreach (var go in set) if (go) go.SetActive(false);
        if (except != "F2A" && tipArrows[0] != null)
            foreach (var go in tipArrows) if (go) go.SetActive(false);
        if (except != "F2B" && gridArrows != null)
            foreach (var set in gridArrows) foreach (var go in set) if (go) go.SetActive(false);
    }

    // ═══════════════════════════════════════════════════════════════════
    //  파싱 (패킷 바뀔 때 1회)
    // ═══════════════════════════════════════════════════════════════════

    private static float ParseF(string s)
    {
        float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v);
        return v;
    }

    private void ParsePacket(string packet)
    {
        f1Valid = f2aValid = f2bValid = false;
        int colon = packet.IndexOf(':');
        if (colon < 0) return;
        string mode = packet.Substring(0, colon);
        string payload = packet.Substring(colon + 1);

        if (mode != curMode) { HideAll(mode); curMode = mode; }

        if (mode == "F1")       ParseGrid(payload, 1, ref f1Rows, ref f1Cols, ref f1Data, ref f1Valid);
        else if (mode == "F2B") ParseGrid(payload, 3, ref f2bRows, ref f2bCols, ref f2bData, ref f2bValid);
        else if (mode == "F2A") ParseF2A(payload);
    }

    private void ParseF2A(string payload)
    {
        string[] vecs = payload.Split('|');
        if (f2aData == null) f2aData = new float[NumTips][];
        for (int i = 0; i < NumTips; i++)
        {
            if (f2aData[i] == null) f2aData[i] = new float[3];
            if (i < vecs.Length)
            {
                string[] c = vecs[i].Split(',');
                f2aData[i][0] = c.Length > 0 ? ParseF(c[0]) : 0;
                f2aData[i][1] = c.Length > 1 ? ParseF(c[1]) : 0;
                f2aData[i][2] = c.Length > 2 ? ParseF(c[2]) : 0;
            }
            else { f2aData[i][0] = f2aData[i][1] = f2aData[i][2] = 0; }
        }
        f2aValid = true;
    }

    /// <summary>"rows,cols;tip|tip.." → data[tip][rows*cols*stride]. stride=1(스칼라)/3(벡터).</summary>
    private void ParseGrid(string payload, int stride, ref int rows, ref int cols,
                           ref float[][] data, ref bool valid)
    {
        int semi = payload.IndexOf(';');
        if (semi < 0) return;
        string[] dims = payload.Substring(0, semi).Split(',');
        if (dims.Length < 2) return;
        if (!int.TryParse(dims[0], out int r) || !int.TryParse(dims[1], out int c)) return;
        if (r < 1 || c < 1 || r * c > maxGridPerTip) return;
        rows = r; cols = c;
        int need = r * c * stride;

        string[] tips = payload.Substring(semi + 1).Split('|');
        if (data == null) data = new float[NumTips][];
        for (int t = 0; t < NumTips; t++)
        {
            if (data[t] == null || data[t].Length != need) data[t] = new float[need];
            if (t < tips.Length)
            {
                string[] vals = tips[t].Split(',');
                int m = Mathf.Min(need, vals.Length);
                for (int k = 0; k < m; k++) data[t][k] = ParseF(vals[k]);
                for (int k = m; k < need; k++) data[t][k] = 0;
            }
            else { for (int k = 0; k < need; k++) data[t][k] = 0; }
        }
        valid = true;
    }

    // ═══════════════════════════════════════════════════════════════════
    //  렌더 (매 프레임, 캐시 + 현재 본 위치)
    // ═══════════════════════════════════════════════════════════════════

    private Color ForceColor(float f) => Color.Lerp(weakColor, strongColor, Mathf.Clamp01(f / forceForFullColor));

    private Vector3 ForceToWorldDir(float fx, float fy, float fz)
        => Vector3.right * fx + Vector3.up * fz + Vector3.forward * fy;   // 법선=하늘(+Y)

    private void PlaceArrow(Transform root, Transform shaft, float headLen,
                            Vector3 start, Vector3 dir, float length, float width)
    {
        root.position = start;
        root.rotation = Quaternion.FromToRotation(Vector3.up, dir.normalized);
        shaft.localScale = new Vector3(width, length * 0.5f, width);
        shaft.localPosition = new Vector3(0, headLen + length * 0.5f, 0);
    }

    private void RenderF1()
    {
        if (!f1Valid) return;
        EnsureF1(f1Rows, f1Cols);
        BuildF1();
        for (int t = 0; t < NumTips; t++)
        {
            if (!PadFrame(t, out var center, out var lengthAxis, out var widthAxis, out var normal))
            { foreach (var go in f1Cells[t]) if (go) go.SetActive(false); continue; }

            float[] d = f1Data[t];
            for (int r = 0; r < f1Rows; r++)
                for (int c = 0; c < f1Cols; c++)
                {
                    int p = r * f1Cols + c;
                    var cell = f1Cells[t][p];
                    if (cell == null) continue;                 // 아직 생성 전

                    // 손끝 곡면 래핑(F2B와 동일): 세로=길이축, 가로=원통 호
                    float gy  = (r - (f1Rows - 1) * 0.5f) * f1CellSpacing;
                    float arc = (c - (f1Cols - 1) * 0.5f) * f1CellSpacing;
                    float theta = arc / Mathf.Max(fingerRadius, 1e-4f);
                    float ct = Mathf.Cos(theta), st = Mathf.Sin(theta);
                    Vector3 nLocal = ct * normal + st * widthAxis;   // 국소 표면 법선
                    Vector3 pos = center + lengthAxis * gy + (fingerRadius + f1SurfaceOffset) * nLocal;
                    if (!Finite(pos)) { cell.SetActive(false); continue; }

                    // 셀을 곡면에 눕힘: 법선 방향이 셀의 위(+Y)
                    cell.SetActive(true);
                    cell.transform.SetPositionAndRotation(pos, Quaternion.LookRotation(lengthAxis, nLocal));
                    cell.transform.localScale = new Vector3(f1CellSize, 0.0007f, f1CellSize);
                    SetMatColor(f1Mats[t][p], Color.Lerp(weakColor, strongColor, Mathf.Clamp01(d[p])));
                }
        }
    }

    private void RenderF2A()
    {
        if (!f2aValid) return;
        EnsureF2A();
        for (int i = 0; i < NumTips; i++)
        {
            if (tipBones[i] == null) { tipArrows[i].SetActive(false); continue; }
            float fx = f2aData[i][0], fy = f2aData[i][1], fz = f2aData[i][2];
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

    private void RenderF2B()
    {
        if (!f2bValid) return;
        EnsureF2B(f2bRows, f2bCols);
        BuildF2B();
        for (int t = 0; t < NumTips; t++)
        {
            if (!PadFrame(t, out var center, out var lengthAxis, out var widthAxis, out var normal))
            { foreach (var go in gridArrows[t]) if (go) go.SetActive(false); continue; }

            float[] d = f2bData[t];
            for (int r = 0; r < f2bRows; r++)
                for (int c = 0; c < f2bCols; c++)
                {
                    int p = r * f2bCols + c, o = p * 3;
                    var arrow = gridArrows[t][p];
                    if (arrow == null) continue;                // 아직 생성 전
                    float fx = d[o], fy = d[o + 1], fz = d[o + 2];
                    float mag = Mathf.Sqrt(fx * fx + fy * fy + fz * fz);
                    if (mag < forceThreshold * 0.5f) { arrow.SetActive(false); continue; }

                    // 손끝 곡면 래핑: 세로=손가락 길이축, 가로=원통을 감는 호(arc)
                    float gy  = (r - (f2bRows - 1) * 0.5f) * gridSpacing;
                    float arc = (c - (f2bCols - 1) * 0.5f) * gridSpacing;
                    float theta = arc / Mathf.Max(fingerRadius, 1e-4f);
                    float ct = Mathf.Cos(theta), st = Mathf.Sin(theta);
                    Vector3 nLocal = ct * normal + st * widthAxis;      // 국소 표면 법선(방사형)
                    Vector3 tArc   = -st * normal + ct * widthAxis;     // 호 접선

                    Vector3 start = center + lengthAxis * gy + fingerRadius * nLocal;
                    // 화살표 방향: 법선(fz) 위주 + 접선 성분으로 살짝 기울임
                    Vector3 dir = nLocal * fz + tArc * fx + lengthAxis * fy;
                    if (!Finite(start) || dir.sqrMagnitude < 1e-8f) { arrow.SetActive(false); continue; }
                    float len = Mathf.Min(mag * gridForceToLength, maxArrowLength * 0.5f);
                    arrow.SetActive(true);
                    PlaceArrow(arrow.transform, gridShafts[t][p], gridHeadLength, start, dir, len, gridShaftWidth);
                    SetMatColor(gridMats[t][p], ForceColor(mag));
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

        // 패킷이 바뀔 때만 파싱(문자열 Split을 30Hz로 제한 → GC 폭주 방지)
        string packet = latestPacket;
        if (packet != null && !ReferenceEquals(packet, processedPacket))
        {
            try { ParsePacket(packet); }
            catch (Exception e) { RateLog("파싱 예외: " + e.Message); }
            processedPacket = packet;
        }

        // 트래킹 끊김/화면 밖 → 숨김
        if (!HandTracked()) { HideAll(""); return; }

        try
        {
            switch (curMode)
            {
                case "F1":  RenderF1();  break;
                case "F2A": RenderF2A(); break;
                case "F2B": RenderF2B(); break;
            }
        }
        catch (Exception e) { RateLog("렌더 예외: " + e.Message); HideAll(""); }
    }

    private void RateLog(string msg)
    {
        if (Time.time - lastLogTime > 2f) { Debug.LogWarning("[Tactile] " + msg); lastLogTime = Time.time; }
    }
}
