using System.Collections.Generic;
using UnityEngine;

// [twin] ArUco localization 칼만 필터가 동작하는지 보는 그림 (바닥 위, Top 뷰·unity_top.png 에 같이 찍힘)
//   주황 선·타원  : 원 측정 (live_loc LOC) 과 측정 공분산 R 의 95% 타원
//   하늘색 선·타원: 칼만 필터 결과 (live_loc KF, dashboard/loc_kf.py) 와 상태 공분산 P 의 95% 타원
//   빨간 점       : 칼만 필터가 이상치로 버린 측정 (NIS > 16.27)
//   타원 자취     : trailEvery 초마다 R·P 타원을 남김 → 마커가 안 보이는 곳에서 P 가 커지고, 측정이 들어오면 줄어드는지
// 데이터: twin_drive.m 이 대시보드에서 받은 LOC·KF 줄을 시뮬 좌표로 바꿔 Unity 에 보냄 (CarStateReceiver.locQueue · kfQueue)
// 숫자 기록은 version__1/results/loc_kf/loc_<시각>.csv (live_loc.py) 와 results/<run>_twin/timeseries.csv 의 kf_* 열.
// Play 를 누르면 CarStateReceiver 가 있을 때 자동으로 붙습니다 (장면 수정 필요 없음).
// 키 (Game 창 클릭 뒤): K 칼만 결과 · M 원 측정 · E 타원 · J 이 그림 전체
public class LocKfOverlay : MonoBehaviour
{
    public CarStateReceiver receiver;

    [Header("표시 (Play 중 키로도 변경)")]
    public bool showAll = true;        // J
    public bool showKf = true;         // K
    public bool showMeas = true;       // M
    public bool showEllipses = true;   // E

    [Header("모양")]
    [Tooltip("바닥에서 띄우는 높이 [m] (PathOverlay 0.02 보다 조금 위)")] public float height = 0.03f;
    [Tooltip("타원 크기: 2차원 95% = sqrt(5.991)")] public float ellipseScale = 2.4477f;
    public float measWidth = 0.012f;
    public float kfWidth = 0.03f;
    public float ellipseWidth = 0.012f;
    public float trailWidth = 0.006f;
    [Tooltip("타원 자취 간격 [s] (0 이면 자취 없음)")] public float trailEvery = 0.5f;
    public int trailMax = 600;
    public int pathMax = 20000;

    public Color measColor = new Color(1f, 0.55f, 0f);
    public Color kfColor = new Color(0f, 0.85f, 1f);
    public Color rejectColor = new Color(1f, 0f, 0f);

    LineRenderer measLine, kfLine, measEll, kfEll;
    readonly List<LineRenderer> trail = new List<LineRenderer>();
    readonly List<GameObject> rejects = new List<GameObject>();
    readonly List<Vector3> measPts = new List<Vector3>(), kfPts = new List<Vector3>();
    Material mat;
    int lastReset;
    float lastTrail = -1e9f;
    float[] lastMeas;

    // 화면 숫자
    int nMeas, nUsed, nRej, nPred;
    float lastSigMeas = float.NaN, lastSigKf = float.NaN, nisSum;
    int nisN;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        if (FindAnyObjectByType<LocKfOverlay>() != null) return;
        var r = FindAnyObjectByType<CarStateReceiver>();
        if (r == null) return;
        var go = new GameObject("LocKfOverlay");
        var po = FindAnyObjectByType<PathOverlay>();
        if (po != null) go.layer = po.gameObject.layer;   // PathOverlay 와 같은 레이어 (차 카메라 사진에는 안 찍힘)
        go.AddComponent<LocKfOverlay>().receiver = r;
    }

    void Start()
    {
        if (receiver == null) receiver = FindAnyObjectByType<CarStateReceiver>();
        if (receiver != null) lastReset = receiver.resetCount;
        mat = new Material(Shader.Find("Sprites/Default"));
        measLine = MakeLine("Meas path", measColor, measWidth, false);
        kfLine = MakeLine("KF path", kfColor, kfWidth, false);
        measEll = MakeLine("Meas R ellipse", measColor, ellipseWidth, true);
        kfEll = MakeLine("KF P ellipse", kfColor, ellipseWidth * 1.5f, true);
    }

    LineRenderer MakeLine(string name, Color c, float width, bool loop)
    {
        var go = new GameObject(name);
        go.layer = gameObject.layer;
        go.transform.SetParent(transform, false);
        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = true;
        lr.loop = loop;
        lr.sharedMaterial = mat;
        lr.startColor = lr.endColor = c;
        lr.startWidth = lr.endWidth = width;
        lr.numCapVertices = 2;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
        lr.positionCount = 0;
        return lr;
    }

    void Clear()
    {
        measPts.Clear(); kfPts.Clear();
        measLine.positionCount = kfLine.positionCount = measEll.positionCount = kfEll.positionCount = 0;
        foreach (var t in trail) Destroy(t.gameObject);
        trail.Clear();
        foreach (var g in rejects) Destroy(g);
        rejects.Clear();
        nMeas = nUsed = nRej = nPred = 0; nisSum = 0; nisN = 0;
        lastSigMeas = lastSigKf = float.NaN; lastMeas = null; lastTrail = -1e9f;
    }

    void Update()
    {
        HandleKeys();
        if (receiver == null) return;
        if (receiver.resetCount != lastReset) { lastReset = receiver.resetCount; Clear(); }

        bool measChanged = false, kfChanged = false;
        // LOC: x, y, yaw, n, Cxx, Cxy, Cyy
        while (receiver.locQueue.TryDequeue(out float[] m))
        {
            if (m.Length < 7) continue;
            lastMeas = m; nMeas++; measChanged = true;
            Add(measPts, W(m[0], m[1]));
            lastSigMeas = Mathf.Sqrt(m[4] + m[6]);
            Ellipse(measEll, m[0], m[1], m[4], m[5], m[6]);
        }
        // KF: x, y, yaw, v, Pxx, Pxy, Pyy, nis, flag
        while (receiver.kfQueue.TryDequeue(out float[] k))
        {
            if (k.Length < 9) continue;
            int flag = Mathf.RoundToInt(k[8]);
            if (flag == 1) { nUsed++; if (k[7] >= 0) { nisSum += k[7]; nisN++; } }
            else if (flag == 0) { nRej++; if (lastMeas != null) MarkReject(lastMeas[0], lastMeas[1]); }
            else if (flag == -1) nPred++;
            kfChanged = true;
            Add(kfPts, W(k[0], k[1]));
            lastSigKf = Mathf.Sqrt(k[4] + k[6]);
            Ellipse(kfEll, k[0], k[1], k[4], k[5], k[6]);
            if (trailEvery > 0 && Time.time - lastTrail >= trailEvery)   // 자취: 칼만 P (진하게) + 그때의 측정 R (옅게)
            {
                lastTrail = Time.time;
                AddTrail(k[0], k[1], k[4], k[5], k[6], kfColor);
                if (lastMeas != null && flag != -1)
                    AddTrail(lastMeas[0], lastMeas[1], lastMeas[4], lastMeas[5], lastMeas[6], new Color(measColor.r, measColor.g, measColor.b, 0.45f));
            }
        }
        if (measChanged) { measLine.positionCount = measPts.Count; measLine.SetPositions(measPts.ToArray()); }
        if (kfChanged) { kfLine.positionCount = kfPts.Count; kfLine.SetPositions(kfPts.ToArray()); }
        ApplyVisibility();
    }

    void Add(List<Vector3> pts, Vector3 p)
    {
        pts.Add(p);
        if (pts.Count > pathMax) pts.RemoveRange(0, pts.Count - pathMax);
    }

    void AddTrail(float x, float y, float a, float b, float c, Color col)
    {
        var lr = MakeLine("trail", col, trailWidth, true);
        Ellipse(lr, x, y, a, b, c);
        trail.Add(lr);
        if (trail.Count > trailMax) { Destroy(trail[0].gameObject); trail.RemoveAt(0); }
    }

    void MarkReject(float x, float y)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.name = "rejected";
        go.layer = gameObject.layer;
        Destroy(go.GetComponent<Collider>());
        go.transform.SetParent(transform, false);
        go.transform.localScale = Vector3.one * 0.06f;
        go.transform.position = W(x, y);
        var r = go.GetComponent<Renderer>();
        r.material = new Material(mat) { color = rejectColor };
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        rejects.Add(go);
        if (rejects.Count > 500) { Destroy(rejects[0]); rejects.RemoveAt(0); }
    }

    // 2×2 공분산 [a b; b c] 의 ellipseScale 배 타원 (시뮬 x → Unity X, 시뮬 y → Unity Z: 방향 그대로)
    void Ellipse(LineRenderer lr, float x, float y, float a, float b, float c)
    {
        if (float.IsNaN(a) || float.IsNaN(b) || float.IsNaN(c) || a <= 0 || c <= 0) { lr.positionCount = 0; return; }
        float m = 0.5f * (a + c), d = Mathf.Sqrt(0.25f * (a - c) * (a - c) + b * b);
        float r1 = ellipseScale * Mathf.Sqrt(Mathf.Max(m + d, 0f)), r2 = ellipseScale * Mathf.Sqrt(Mathf.Max(m - d, 0f));
        float ang = 0.5f * Mathf.Atan2(2f * b, a - c);
        float ca = Mathf.Cos(ang), sa = Mathf.Sin(ang);
        const int n = 48;
        lr.positionCount = n;
        for (int i = 0; i < n; i++)
        {
            float t = 2f * Mathf.PI * i / n, u = r1 * Mathf.Cos(t), v = r2 * Mathf.Sin(t);
            lr.SetPosition(i, W(x + ca * u - sa * v, y + sa * u + ca * v));
        }
    }

    void ApplyVisibility()
    {
        measLine.enabled = showAll && showMeas;
        measEll.enabled = showAll && showMeas && showEllipses;
        kfLine.enabled = showAll && showKf;
        kfEll.enabled = showAll && showKf && showEllipses;
        foreach (var t in trail) t.enabled = showAll && showEllipses;
        foreach (var g in rejects) g.SetActive(showAll && showMeas);
    }

    Vector3 W(float mx, float my) => receiver.MapToWorld(mx, my, height);

    void HandleKeys()
    {
        if (KeyDown('k')) showKf = !showKf;
        if (KeyDown('m')) showMeas = !showMeas;
        if (KeyDown('e')) showEllipses = !showEllipses;
        if (KeyDown('j')) showAll = !showAll;
    }

    bool KeyDown(char c)
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKeyDown((KeyCode)c);
#elif ENABLE_INPUT_SYSTEM
        var kb = UnityEngine.InputSystem.Keyboard.current;
        if (kb == null) return false;
        switch (c)
        {
            case 'k': return kb.kKey.wasPressedThisFrame;
            case 'm': return kb.mKey.wasPressedThisFrame;
            case 'e': return kb.eKey.wasPressedThisFrame;
            case 'j': return kb.jKey.wasPressedThisFrame;
        }
        return false;
#else
        return false;
#endif
    }

    // 화면 왼쪽 (PathOverlay 숫자 아래): 측정·칼만 횟수, 95% 위치 불확실성, 평균 NIS
    void OnGUI()
    {
        if (!showAll || receiver == null) return;
        string nis = nisN > 0 ? (nisSum / nisN).ToString("F2") : "-";
        string text =
            $"ArUco KF   meas {nMeas}   used {nUsed}   rejected {nRej}   predict-only {nPred}\n" +
            $"sigma_pos  meas {Cm(lastSigMeas)}  ->  KF {Cm(lastSigKf)}    mean NIS {nis} (ideal 3)\n" +
            "orange = measurement + R 95%   cyan = KF + P 95%   red = rejected\n" +
            "[K] KF  [M] Meas  [E] Ellipses  [J] All";
        GUI.Box(new Rect(8, 140, 470, 78), "");
        GUI.Label(new Rect(14, 143, 460, 76), text);
    }

    static string Cm(float v) => float.IsNaN(v) ? "-" : (v * 100f).ToString("F1") + " cm";
}
