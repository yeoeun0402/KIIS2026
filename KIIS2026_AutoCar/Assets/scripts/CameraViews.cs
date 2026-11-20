using UnityEngine;

// 카메라 3개(측면 전체 뷰, Top 뷰, 차량 추적 뷰)를 자동으로 배치하고 화면 배치를 바꿉니다.
// 붙일 곳: 빈 오브젝트(예: CameraRig)
// Play 중 Game 창을 클릭한 뒤 숫자키로 전환:
//   1 측면 / 2 Top / 3 추적 / 4 분할(3개 동시) / 5 분석(큰 Top + 추적, PathOverlay 보기용)
public class CameraViews : MonoBehaviour
{
    public enum Layout { Split, Overview, Top, Chase, Analysis }

    [Header("대상")]
    [Tooltip("따라갈 차량 (Hierarchy의 car)")]
    public Transform car;
    [Tooltip("맵 (Hierarchy의 Plane). 맵 크기에 맞춰 카메라 거리를 자동 계산합니다.")]
    public Renderer map;

    [Header("카메라 (비워두면 자동 생성)")]
    [Tooltip("Main Camera를 넣으면 측면 전체 뷰로 사용합니다.")]
    public Camera overviewCam;
    public Camera topCam;
    public Camera chaseCam;

    [Header("화면 배치 (Play 중 숫자키 1~4로도 변경)")]
    public Layout layout = Layout.Split;

    [Header("1. 측면 전체 뷰")]
    [Tooltip("내려다보는 각도. 클수록 위에서 보는 느낌")]
    [Range(10f, 80f)] public float overviewPitch = 35f;
    [Tooltip("바라보는 방향. 0이면 맵의 아래쪽(-Z)에서 위쪽을 봄")]
    public float overviewYaw = 0f;
    [Tooltip("1보다 작으면 확대, 크면 축소")]
    [Range(0.4f, 1.5f)] public float overviewZoom = 0.8f;

    [Header("2. Top 뷰")]
    [Tooltip("1보다 크면 맵 주변 여백이 생김")]
    [Range(0.8f, 1.5f)] public float topMargin = 1.05f;

    [Header("3. 추적 뷰")]
    public float chaseDistance = 2.0f;   // 차 뒤로 떨어진 거리 [m]
    public float chaseHeight = 1.0f;     // 카메라 높이 [m]
    public float lookAhead = 1.0f;       // 차 앞쪽 어디를 바라볼지 [m]
    public float lookHeight = 0.3f;      // 바라보는 지점 높이 [m]
    [Range(1f, 20f)] public float chaseSmooth = 6f; // 클수록 빠르게 따라감

    bool chaseInitialized;

    void Start()
    {
        if (overviewCam == null) overviewCam = CreateCamera("OverviewCam");
        if (topCam == null) topCam = CreateCamera("TopCam");
        if (chaseCam == null) chaseCam = CreateCamera("ChaseCam");

        topCam.orthographic = true;
        chaseCam.nearClipPlane = 0.05f;

        if (map == null) Debug.LogWarning("[CameraViews] Map 칸에 Plane을 넣어주세요.");
        if (car == null) Debug.LogWarning("[CameraViews] Car 칸에 car를 넣어주세요.");
    }

    Camera CreateCamera(string name)
    {
        var go = new GameObject(name);
        go.transform.SetParent(transform, false);
        return go.AddComponent<Camera>();
    }

    // 차가 Update에서 움직인 뒤에 카메라를 움직여야 떨림이 없어서 LateUpdate 사용
    void LateUpdate()
    {
        HandleKeys();
        ApplyLayout();

        if (map != null)
        {
            UpdateOverview();
            UpdateTop();
        }
        if (car != null) UpdateChase();
    }

    // ---------- 화면 배치 ----------
    void HandleKeys()
    {
        if (KeyDown(1)) layout = Layout.Overview;
        if (KeyDown(2)) layout = Layout.Top;
        if (KeyDown(3)) layout = Layout.Chase;
        if (KeyDown(4)) layout = Layout.Split;
        if (KeyDown(5)) layout = Layout.Analysis;
    }

    void ApplyLayout()
    {
        var full = new Rect(0f, 0f, 1f, 1f);
        switch (layout)
        {
            case Layout.Split:
                // 왼쪽 위: 측면, 왼쪽 아래: Top, 오른쪽 전체: 추적
                Show(overviewCam, new Rect(0f, 0.5f, 0.5f, 0.5f));
                Show(topCam, new Rect(0f, 0f, 0.5f, 0.5f));
                Show(chaseCam, new Rect(0.5f, 0f, 0.5f, 1f));
                break;
            case Layout.Overview: ShowOnly(overviewCam, full); break;
            case Layout.Top: ShowOnly(topCam, full); break;
            case Layout.Chase: ShowOnly(chaseCam, full); break;
            case Layout.Analysis:
                // 왼쪽 큰 화면: Top (경로·궤적·선행점·원 보기), 오른쪽: 추적
                overviewCam.enabled = false;
                Show(topCam, new Rect(0f, 0f, 0.62f, 1f));
                Show(chaseCam, new Rect(0.62f, 0f, 0.38f, 1f));
                break;
        }
    }

    void Show(Camera c, Rect r) { c.enabled = true; c.rect = r; }

    void ShowOnly(Camera c, Rect r)
    {
        overviewCam.enabled = (c == overviewCam);
        topCam.enabled = (c == topCam);
        chaseCam.enabled = (c == chaseCam);
        c.rect = r;
    }

    // ---------- 1. 측면 전체 뷰 ----------
    void UpdateOverview()
    {
        Bounds b = map.bounds;
        overviewCam.orthographic = false;

        // 맵 전체를 감싸는 구가 화면에 들어오는 거리 계산
        float radius = new Vector3(b.extents.x, 0.5f, b.extents.z).magnitude;
        float vFov = overviewCam.fieldOfView * Mathf.Deg2Rad;
        float hFov = 2f * Mathf.Atan(Mathf.Tan(vFov / 2f) * overviewCam.aspect);
        float fov = Mathf.Min(vFov, hFov);
        float dist = radius / Mathf.Sin(fov / 2f) * overviewZoom;

        Quaternion rot = Quaternion.Euler(overviewPitch, overviewYaw, 0f);
        overviewCam.transform.SetPositionAndRotation(b.center - rot * Vector3.forward * dist, rot);
    }

    // ---------- 2. Top 뷰 ----------
    void UpdateTop()
    {
        Bounds b = map.bounds;
        // 화면 위쪽 = +Z, 오른쪽 = +X  →  MATLAB plot(x, y)와 같은 방향
        topCam.orthographicSize = Mathf.Max(b.extents.z, b.extents.x / topCam.aspect) * topMargin;
        topCam.transform.SetPositionAndRotation(b.center + Vector3.up * 50f, Quaternion.Euler(90f, 0f, 0f));
    }

    // ---------- 3. 추적 뷰 ----------
    void UpdateChase()
    {
        Vector3 fwd = car.forward;
        fwd.y = 0f;
        if (fwd.sqrMagnitude < 1e-6f) fwd = Vector3.forward;
        fwd.Normalize();

        Vector3 desired = car.position - fwd * chaseDistance + Vector3.up * chaseHeight;
        Vector3 pos = chaseCam.transform.position;

        // 처음이거나 차가 멀리 순간이동했으면 바로 붙고, 아니면 부드럽게 따라감
        if (!chaseInitialized || (pos - desired).sqrMagnitude > 25f)
        {
            pos = desired;
            chaseInitialized = true;
        }
        else
        {
            float k = 1f - Mathf.Exp(-chaseSmooth * Time.deltaTime);
            pos = Vector3.Lerp(pos, desired, k);
        }

        Vector3 look = car.position + fwd * lookAhead + Vector3.up * lookHeight;
        chaseCam.transform.SetPositionAndRotation(pos, Quaternion.LookRotation(look - pos, Vector3.up));
    }

    // ---------- 키 입력 (구/신 입력 시스템 모두 지원) ----------
    bool KeyDown(int n)
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKeyDown((KeyCode)((int)KeyCode.Alpha0 + n))
            || Input.GetKeyDown((KeyCode)((int)KeyCode.Keypad0 + n));
#elif ENABLE_INPUT_SYSTEM
        var kb = UnityEngine.InputSystem.Keyboard.current;
        if (kb == null) return false;
        switch (n)
        {
            case 1: return kb.digit1Key.wasPressedThisFrame || kb.numpad1Key.wasPressedThisFrame;
            case 2: return kb.digit2Key.wasPressedThisFrame || kb.numpad2Key.wasPressedThisFrame;
            case 3: return kb.digit3Key.wasPressedThisFrame || kb.numpad3Key.wasPressedThisFrame;
            case 4: return kb.digit4Key.wasPressedThisFrame || kb.numpad4Key.wasPressedThisFrame;
            case 5: return kb.digit5Key.wasPressedThisFrame || kb.numpad5Key.wasPressedThisFrame;
        }
        return false;
#else
        return false;
#endif
    }

    // 화면 왼쪽 위에 키 안내 표시
    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 500, 25), "1: Side   2: Top   3: Chase   4: Split   5: Analysis");
    }
}
