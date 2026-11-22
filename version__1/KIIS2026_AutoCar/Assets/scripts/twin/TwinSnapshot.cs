using System.Globalization;
using System.IO;
using UnityEngine;

// [twin] 위에서 본 맵 + PathOverlay(레퍼런스 경로·지나온 궤적)를 PNG로 저장합니다.
//   - MATLAB(twin_drive.m)이 종료할 때 "CMD,SNAP,<png 경로>"를 보내면 그 경로에 저장 (결과 폴더에 같이 남음)
//   - Play 중 Game 창에서 F12: Application.persistentDataPath 밑에 시각 이름으로 저장 (수동 확인용)
// 이미지 축척이 정확하도록 맵(Plane) 크기에 딱 맞춘 직교 카메라를 따로 만들어 그립니다 (화면 배치와 무관).
//   pixelsPerMeter = 100 → 16 m × 11 m 맵이 1600 × 1100 px (1 px = 1 cm)
//   rotateToTrack = true → 실차 트랙 좌표(~/demo/track/track.yaml: 원점 매트 왼쪽 아래, x 오른쪽, y 위)와 같은 방향
//     (Unity 맵 텍스처는 도면과 180° 돌아가 있음: x_track = 8 - x_sim, y_track = 5.5 - y_sim)
//     이때 픽셀 (u, v_아래에서) = (x_track, y_track) × pixelsPerMeter
// PNG 옆에 같은 이름의 .json으로 축척·방향을 같이 저장합니다.
// Play를 누르면 장면에 CarStateReceiver가 있을 때 자동으로 붙습니다.
public class TwinSnapshot : MonoBehaviour
{
    public CarStateReceiver receiver;
    [Tooltip("비워두면 CameraViews의 Map, 없으면 이름이 Plane인 오브젝트")]
    public Renderer map;
    public int pixelsPerMeter = 100;
    public bool rotateToTrack = true;
    public Color background = new Color(0.2f, 0.2f, 0.2f);

    Camera snapCam;
    string lastSaved = "";

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        if (FindAnyObjectByType<TwinSnapshot>() != null) return;
        var r = FindAnyObjectByType<CarStateReceiver>();
        if (r == null) return;
        new GameObject("TwinSnapshot").AddComponent<TwinSnapshot>().receiver = r;
    }

    void Start()
    {
        if (receiver == null) receiver = FindAnyObjectByType<CarStateReceiver>();
        if (map == null)
        {
            var views = FindAnyObjectByType<CameraViews>();
            if (views != null) map = views.map;
        }
        if (map == null)
        {
            var plane = GameObject.Find("Plane");
            if (plane != null) map = plane.GetComponent<Renderer>();
        }
    }

    // MATLAB이 "CMD,HUD,<글>"로 보낸 상태 글 (예: "PWM +30 / 40  (real car)"). TwinInput.OnGUI가 보여 줌.
    public static string Hud = "";

    void Update()
    {
        if (receiver != null)
        {
            while (receiver.commands.TryDequeue(out string cmd))
            {
                if (cmd.StartsWith("SNAP,")) Save(cmd.Substring(5).Trim());
                else if (cmd.StartsWith("HUD,")) Hud = cmd.Substring(4).Trim();   // TwinInput 화면 아래 표시 (지금 PWM)
            }
        }
        if (F12Pressed())
        {
            string name = "twin_top_" + System.DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".png";
            Save(Path.Combine(Application.persistentDataPath, name));
        }
    }

    public void Save(string path)
    {
        if (map == null) { Debug.LogWarning("[TwinSnapshot] 맵(Plane)을 찾지 못해 저장하지 않음"); return; }
        Bounds b = map.bounds;
        int w = Mathf.RoundToInt(b.size.x * pixelsPerMeter);
        int h = Mathf.RoundToInt(b.size.z * pixelsPerMeter);

        if (snapCam == null)
        {
            var go = new GameObject("TwinSnapshotCam");
            go.transform.SetParent(transform, false);
            snapCam = go.AddComponent<Camera>();
            snapCam.enabled = false;               // 화면에는 안 그림, Render()로만
            snapCam.orthographic = true;
            snapCam.clearFlags = CameraClearFlags.SolidColor;
            snapCam.nearClipPlane = 0.1f;
            snapCam.farClipPlane = 200f;
        }
        snapCam.backgroundColor = background;
        snapCam.orthographicSize = b.extents.z;
        // Euler(90, 0, 0): 화면 위 = +Z, 오른쪽 = +X (MATLAB plot과 같음) / Euler(90, 180, 0): 180° 돌림 = 트랙 방향
        snapCam.transform.SetPositionAndRotation(b.center + Vector3.up * 50f,
            Quaternion.Euler(90f, rotateToTrack ? 180f : 0f, 0f));

        var rt = new RenderTexture(w, h, 24);
        var tex = new Texture2D(w, h, TextureFormat.RGB24, false);
        RenderTexture prevActive = RenderTexture.active;
        try
        {
            snapCam.aspect = (float)w / h;
            snapCam.targetTexture = rt;
            snapCam.Render();
            RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, w, h), 0, 0);
            tex.Apply();
            string dir = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
            File.WriteAllBytes(path, tex.EncodeToPNG());

            var inv = CultureInfo.InvariantCulture;
            string json = "{\n" +
                $"  \"image\": \"{Path.GetFileName(path)}\",\n" +
                $"  \"width_px\": {w},\n  \"height_px\": {h},\n" +
                $"  \"pixels_per_meter\": {pixelsPerMeter},\n" +
                $"  \"orientation\": \"{(rotateToTrack ? "track" : "sim")}\",\n" +
                "  \"note\": \"track: pixel (u, v from bottom) = (x_track, y_track) * pixels_per_meter, x_track = 8 - x_sim, y_track = 5.5 - y_sim. sim: (u, v from bottom) = (x_sim + 8, y_sim + 5.5) * pixels_per_meter\",\n" +
                $"  \"map_center_unity\": [{b.center.x.ToString(inv)}, {b.center.z.ToString(inv)}],\n" +
                $"  \"map_size_m\": [{b.size.x.ToString(inv)}, {b.size.z.ToString(inv)}]\n" +
                "}\n";
            File.WriteAllText(Path.ChangeExtension(path, ".json"), json);
            lastSaved = path;
            Debug.Log($"[TwinSnapshot] 저장: {path} ({w}x{h})");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[TwinSnapshot] 저장 실패 {path}: {e.Message}");
        }
        finally
        {
            snapCam.targetTexture = null;
            RenderTexture.active = prevActive;
            rt.Release();
            Destroy(rt);
            Destroy(tex);
        }
    }

    void OnGUI()
    {
        if (string.IsNullOrEmpty(lastSaved)) return;
        GUI.Label(new Rect(10, Screen.height - 114, 1200, 22), "  saved top view: " + lastSaved + "   [F12] save again");
    }

    bool F12Pressed()
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKeyDown(KeyCode.F12);
#elif ENABLE_INPUT_SYSTEM
        var kb = UnityEngine.InputSystem.Keyboard.current;
        return kb != null && kb.f12Key.wasPressedThisFrame;
#else
        return false;
#endif
    }
}
