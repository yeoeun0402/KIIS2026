using System.Collections.Generic;
using UnityEngine;

// MATLAB pure pursuit(pure_pursuit_control.m)가 보내는 값을 바닥 위에 그립니다.
//   레퍼런스 경로(노랑), 지나온 궤적(빨강), p_w(파랑)·p_d(초록)·p_l(빨강) 점,
//   차 → p_l 선(Lf), pure pursuit 원(얇게)과 차 → p_l 호(굵게), 왼쪽 위 숫자 표시
// 붙일 곳: 빈 오브젝트(예: PathOverlay). Receiver 칸이 비어 있으면 장면에서 CarStateReceiver를 찾습니다.
// 그리는 선·점은 이 오브젝트의 레이어(PathOverlay)를 따릅니다. Car_Camera의 Culling Mask에서 이 레이어를 빼서
// 저장 사진(CarCameraCapture)에는 경로가 찍히지 않고, Top·측면·추적 뷰에는 보입니다.
// Play 중 Game 창을 클릭한 뒤 키로 켜기·끄기:
//   R 레퍼런스 / T 궤적 / L 선행점 / C 원 / H 숫자 / O 전체
public class PathOverlay : MonoBehaviour
{
    [Header("대상 (비워두면 자동으로 찾음)")]
    public CarStateReceiver receiver;

    [Header("표시 (Play 중 키로도 변경)")]
    public bool showAll = true;          // O
    public bool showReference = true;    // R
    public bool showTrajectory = true;   // T
    public bool showLookahead = true;    // L
    public bool showCircle = true;       // C
    public bool showHud = true;          // H

    [Header("모양")]
    [Tooltip("바닥에서 띄우는 높이 [m]")] public float height = 0.02f;
    public float referenceWidth = 0.04f;
    public float trajectoryWidth = 0.03f;
    public float circleWidth = 0.015f;
    public float arcWidth = 0.04f;
    public float markerSize = 0.12f;
    [Tooltip("궤적 점을 추가하는 최소 이동 거리 [m]")] public float trajectoryStep = 0.01f;
    public int trajectoryMaxPoints = 20000;

    [Header("색 (MATLAB 그림·논문 그림과 같게)")]
    public Color referenceColor = new Color(1f, 0.9f, 0f);
    public Color trajectoryColor = new Color(1f, 0.1f, 0.1f);
    public Color pwColor = new Color(0f, 0.45f, 1f);
    public Color pdColor = new Color(0f, 0.8f, 0f);
    public Color plColor = new Color(1f, 0f, 0f);
    public Color circleColor = new Color(1f, 0f, 1f, 0.6f);
    public Color arcColor = new Color(1f, 0f, 1f);

    LineRenderer refLine, trajLine, lfLine, circleLine, arcLine;
    Transform pwMark, pdMark, plMark;
    Material mat;
    readonly List<Vector3> traj = new List<Vector3>();
    int lastPathVersion = -1, lastReset, lastCount = -1;
    bool hasPP;   // 받은 줄에 pure pursuit 값(21열)이 있는지

    void Start()
    {
        if (receiver == null) receiver = FindAnyObjectByType<CarStateReceiver>();
        if (receiver == null) Debug.LogWarning("[PathOverlay] CarStateReceiver를 찾을 수 없습니다.");
        if (receiver != null) lastReset = receiver.resetCount;

        mat = new Material(Shader.Find("Sprites/Default"));   // 조명 영향 없는 단색
        refLine    = MakeLine("Reference", referenceColor, referenceWidth, true);
        trajLine   = MakeLine("Trajectory", trajectoryColor, trajectoryWidth, false);
        circleLine = MakeLine("PP circle", circleColor, circleWidth, true);
        arcLine    = MakeLine("PP arc", arcColor, arcWidth, false);
        lfLine     = MakeLine("Lf line", plColor, circleWidth, false);
        pwMark = MakeMarker("p_w", pwColor);
        pdMark = MakeMarker("p_d", pdColor);
        plMark = MakeMarker("p_l", plColor);
    }

    LineRenderer MakeLine(string name, Color c, float width, bool loop)
    {
        var go = new GameObject(name);
        go.layer = gameObject.layer;   // PathOverlay 오브젝트와 같은 레이어 (Car_Camera Culling Mask에서 빼면 사진에 안 찍힘)
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

    Transform MakeMarker(string name, Color c)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.name = name;
        go.layer = gameObject.layer;   // PathOverlay 오브젝트와 같은 레이어
        Destroy(go.GetComponent<Collider>());
        go.transform.SetParent(transform, false);
        go.transform.localScale = Vector3.one * markerSize;
        var r = go.GetComponent<Renderer>();
        r.material = new Material(mat) { color = c };
        r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        go.SetActive(false);
        return go.transform;
    }

    void Update()
    {
        HandleKeys();
        if (receiver == null) return;

        // 레퍼런스 경로
        if (receiver.pathVersion != lastPathVersion)
        {
            lastPathVersion = receiver.pathVersion;
            var pts = new Vector3[receiver.path.Length];
            for (int i = 0; i < pts.Length; i++) pts[i] = W(receiver.path[i].x, receiver.path[i].y);
            refLine.positionCount = pts.Length;
            refLine.SetPositions(pts);
        }

        // 새 주행 시작: 궤적 지우기
        if (receiver.resetCount != lastReset)
        {
            lastReset = receiver.resetCount;
            traj.Clear();
            trajLine.positionCount = 0;
        }

        // 새 상태
        float[] f = receiver.fields;
        if (f != null && f.Length >= 9 && receiver.receivedCount != lastCount)
        {
            lastCount = receiver.receivedCount;
            Vector3 pv = W(f[1], f[2]);   // 뒤 차축 중앙 (MATLAB 기준점)

            if (traj.Count == 0 || (traj[traj.Count - 1] - pv).sqrMagnitude > trajectoryStep * trajectoryStep)
            {
                traj.Add(pv);
                if (traj.Count > trajectoryMaxPoints) traj.RemoveRange(0, traj.Count - trajectoryMaxPoints);
                trajLine.positionCount = traj.Count;
                trajLine.SetPositions(traj.ToArray());
            }

            hasPP = f.Length >= 21;
            if (hasPP) DrawPurePursuit(f, pv);
        }

        ApplyVisibility();
    }

    // f: t,x,y,yaw,v,delta,yawRate,e_lat,e_head, x_w,y_w,x_d,y_d,x_l,y_l,x_c,y_c,R,Lf,tau,delta_cmd
    void DrawPurePursuit(float[] f, Vector3 pv)
    {
        SetMarker(pwMark, f[9], f[10]);
        SetMarker(pdMark, f[11], f[12]);
        SetMarker(plMark, f[13], f[14]);

        bool hasPl = !float.IsNaN(f[13]) && !float.IsNaN(f[14]);
        Vector3 pl = hasPl ? W(f[13], f[14]) : pv;
        lfLine.positionCount = hasPl ? 2 : 0;
        if (hasPl) lfLine.SetPositions(new[] { pv, pl });

        float cx = f[15], cy = f[16], R = f[17];
        if (!hasPl || float.IsNaN(R) || Mathf.Abs(R) > 50f || float.IsNaN(cx))
        {
            // 거의 직선: 원 없이 차 → p_l 선분만
            circleLine.positionCount = 0;
            arcLine.positionCount = hasPl ? 2 : 0;
            if (hasPl) arcLine.SetPositions(new[] { pv, pl });
            return;
        }

        float r = Mathf.Abs(R);
        const int nCircle = 120;
        circleLine.positionCount = nCircle;
        for (int i = 0; i < nCircle; i++)
        {
            float a = 2f * Mathf.PI * i / nCircle;
            circleLine.SetPosition(i, W(cx + r * Mathf.Cos(a), cy + r * Mathf.Sin(a)));
        }

        // 차 → p_l 호: 왼쪽 회전(R>0)은 반시계, 오른쪽 회전은 시계 방향
        float a0 = Mathf.Atan2(f[2] - cy, f[1] - cx);
        float a1 = Mathf.Atan2(f[14] - cy, f[13] - cx);
        float sweep = R > 0 ? Repeat2Pi(a1 - a0) : -Repeat2Pi(a0 - a1);
        const int nArc = 40;
        arcLine.positionCount = nArc;
        for (int i = 0; i < nArc; i++)
        {
            float a = a0 + sweep * i / (nArc - 1);
            arcLine.SetPosition(i, W(cx + r * Mathf.Cos(a), cy + r * Mathf.Sin(a)));
        }
    }

    void SetMarker(Transform m, float mx, float my)
    {
        bool ok = !float.IsNaN(mx) && !float.IsNaN(my);
        m.gameObject.SetActive(ok && showAll && showLookahead);
        if (ok) m.position = W(mx, my) + Vector3.up * (markerSize * 0.5f);
    }

    void ApplyVisibility()
    {
        refLine.enabled = showAll && showReference;
        trajLine.enabled = showAll && showTrajectory;
        bool look = showAll && showLookahead && hasPP;
        lfLine.enabled = look;
        if (!look) { pwMark.gameObject.SetActive(false); pdMark.gameObject.SetActive(false); plMark.gameObject.SetActive(false); }
        circleLine.enabled = showAll && showCircle && hasPP;
        arcLine.enabled = showAll && showCircle && hasPP;
    }

    Vector3 W(float mx, float my) => receiver.MapToWorld(mx, my, height);

    static float Repeat2Pi(float a) => Mathf.Repeat(a, 2f * Mathf.PI);

    // ---------- 키 입력 (구/신 입력 시스템 모두 지원) ----------
    void HandleKeys()
    {
        if (KeyDown('r')) showReference = !showReference;
        if (KeyDown('t')) showTrajectory = !showTrajectory;
        if (KeyDown('l')) showLookahead = !showLookahead;
        if (KeyDown('c')) showCircle = !showCircle;
        if (KeyDown('h')) showHud = !showHud;
        if (KeyDown('o')) showAll = !showAll;
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
            case 'r': return kb.rKey.wasPressedThisFrame;
            case 't': return kb.tKey.wasPressedThisFrame;
            case 'l': return kb.lKey.wasPressedThisFrame;
            case 'c': return kb.cKey.wasPressedThisFrame;
            case 'h': return kb.hKey.wasPressedThisFrame;
            case 'o': return kb.oKey.wasPressedThisFrame;
        }
        return false;
#else
        return false;
#endif
    }

    // 화면 왼쪽 위 숫자 표시 (CameraViews 안내 글자 아래)
    void OnGUI()
    {
        if (!showAll || !showHud || receiver == null) return;
        float[] f = receiver.fields;
        string text;
        if (f == null || f.Length < 9)
        {
            text = "Waiting for MATLAB...";
        }
        else
        {
            text = $"t = {f[0]:F1} s    v = {f[4]:F2} m/s\n" +
                   $"delta = {f[5] * Mathf.Rad2Deg:F1} deg" +
                   (f.Length >= 21 ? $"   (cmd {f[20] * Mathf.Rad2Deg:F1})" : "") + "\n" +
                   $"e_lat = {f[7]:F3} m    e_head = {f[8] * Mathf.Rad2Deg:F1} deg";
            if (f.Length >= 21)
                text += $"\nLf = {f[18]:F2} m    tau = {Fmt(f[19])}    R = {Fmt(f[17])} m";
        }
        text += "\n[R] Ref [T] Traj [L] Look-ahead [C] Circle [H] HUD [O] All";
        GUI.Box(new Rect(8, 32, 380, f != null && f.Length >= 21 ? 100 : 82), "");
        GUI.Label(new Rect(14, 35, 370, 100), text);
    }

    static string Fmt(float v) => float.IsNaN(v) ? "-" : v.ToString("F2");
}
