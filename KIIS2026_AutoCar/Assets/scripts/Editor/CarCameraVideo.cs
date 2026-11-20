using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEditor.Media;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;

// Car_Camera 시점 주행 영상 (mp4, H.264)을 만듭니다. Edit 모드에서 실행 (Play 하지 않음, MATLAB 필요 없음).
// 메뉴: Tools → Car_Camera 영상 만들기
//
// 하는 일: 저장된 MATLAB 주행 기록(timeseries.csv)의 차 위치 x, y, yaw, 조향각을 영상 fps로 보간해
//   매 프레임 car를 그 자리에 놓고 Car_Camera를 두 번 렌더링합니다.
//     <이름>_ref.mp4   : 레퍼런스 경로(노란 선, PathOverlay와 같은 색·굵기·높이)가 보이는 영상
//     <이름>_noref.mp4 : 레퍼런스가 안 보이는 영상 (Car_Camera 원래 화면 = YOLO 사진과 같은 화면)
//   두 영상은 같은 프레임에서 같은 위치로 찍으므로, 레퍼런스 선만 다릅니다.
//
// 주행 기록 CSV 형식 (열 이름으로 찾음):
//   switching_2/3 data/<run>/timeseries.csv : 열 앞글자 "bl_" (블랜디드 차 = Unity로 보내는 차) → bl_x_m, bl_y_m, bl_yaw_rad, bl_delta_out_rad
//   results/<run>/timeseries.csv (RunLogger) : 앞글자 없음 → x_m, y_m, yaw_rad, delta_rad
//   시간 열은 t_s. 앞글자 열이 없으면 앞글자 없이 다시 찾습니다.
// 차 놓는 방법은 CarStateReceiver.Update와 같음 (MATLAB x → Unity X, y → Z, 회전 Y = 90 − yaw[deg] + Yaw Offset).
//   MATLAB은 0.1 s마다 상태를 보내지만 영상은 그 사이를 선형 보간해서 부드럽게 움직입니다.
// 끝나면 car·바퀴·Car_Camera 설정을 원래대로 되돌립니다 (장면 파일은 바뀌지 않음).
public class CarCameraVideoWindow : EditorWindow
{
    CarCameraVideo.Settings s = new CarCameraVideo.Settings();
    Vector2 scroll;

    [MenuItem("Tools/Car_Camera 영상 만들기")]
    static void Open() => GetWindow<CarCameraVideoWindow>("Car_Camera 영상");

    void OnGUI()
    {
        scroll = EditorGUILayout.BeginScrollView(scroll);
        EditorGUILayout.LabelField("주행 기록 (상대 경로는 Unity 프로젝트 폴더 KIIS2026_AutoCar 기준)", EditorStyles.boldLabel);
        s.runCsv = PathField("주행 기록 CSV", s.runCsv, "csv");
        s.columnPrefix = EditorGUILayout.TextField(new GUIContent("열 앞글자", "switching_2/3은 bl_ (블랜디드 차), RunLogger는 비움"), s.columnPrefix);
        s.referenceCsv = PathField("레퍼런스 경로 CSV", s.referenceCsv, "csv");

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("영상", EditorStyles.boldLabel);
        s.fps = EditorGUILayout.IntSlider("fps", s.fps, 10, 60);
        s.width = EditorGUILayout.IntField(new GUIContent("가로 [px]", "640x480 YOLO 사진과 같은 화각이려면 4:3"), s.width);
        s.height = EditorGUILayout.IntField("세로 [px]", s.height);
        s.supersample = EditorGUILayout.IntSlider(new GUIContent("Supersample", "n배 크게 렌더링 후 줄여서 얇은 선의 계단·깜빡임을 줄임"), s.supersample, 1, 3);
        s.bitrateMbps = EditorGUILayout.IntSlider("Bitrate [Mbps]", s.bitrateMbps, 2, 40);
        s.startTime = EditorGUILayout.FloatField("시작 t [s]", s.startTime);
        s.endTime = EditorGUILayout.FloatField(new GUIContent("끝 t [s]", "0 이하면 기록 끝까지"), s.endTime);
        s.makeRef = EditorGUILayout.Toggle("레퍼런스 보이는 영상", s.makeRef);
        s.makeNoRef = EditorGUILayout.Toggle("레퍼런스 안 보이는 영상", s.makeNoRef);

        EditorGUILayout.Space();
        s.outputFolder = EditorGUILayout.TextField("저장 폴더", s.outputFolder);
        s.name = EditorGUILayout.TextField(new GUIContent("파일 이름", "비우면 주행 기록 폴더 이름으로 정함"), s.name);

        EditorGUILayout.Space();
        using (new EditorGUI.DisabledScope(EditorApplication.isPlayingOrWillChangePlaymode))
        {
            if (GUILayout.Button("영상 만들기", GUILayout.Height(30))) CarCameraVideo.Record(s);
        }
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            EditorGUILayout.HelpBox("Play를 멈춘 뒤 실행하세요 (Edit 모드에서 차를 직접 옮겨 찍습니다).", MessageType.Info);
        if (!string.IsNullOrEmpty(CarCameraVideo.status)) EditorGUILayout.HelpBox(CarCameraVideo.status, MessageType.None);
        EditorGUILayout.EndScrollView();
    }

    static string PathField(string label, string value, string ext)
    {
        using (new EditorGUILayout.HorizontalScope())
        {
            value = EditorGUILayout.TextField(label, value);
            if (GUILayout.Button("...", GUILayout.Width(28)))
            {
                string p = EditorUtility.OpenFilePanel(label, Path.GetDirectoryName(CarCameraVideo.Resolve(value)), ext);
                if (!string.IsNullOrEmpty(p)) value = p;
                GUI.FocusControl(null);
            }
        }
        return value;
    }
}

public static class CarCameraVideo
{
    [Serializable]
    public class Settings
    {
        public string runCsv = "../MATLAB/switching_2/data/20261116_194238/timeseries.csv";
        public string columnPrefix = "bl_";
        public string referenceCsv = "../MATLAB/reference_path_outer_lane.csv";
        public int fps = 30;
        public int width = 1280, height = 960;
        public int supersample = 2;
        public int bitrateMbps = 16;
        public float startTime = 0f, endTime = 0f;
        public bool makeRef = true, makeNoRef = true;
        public string outputFolder = "../video";
        public string name = "";
        public bool showProgressBar = true;
    }

    public static string status = "";      // 마지막 실행 결과 (CLI·eval에서 확인용)
    public static bool running;
    public static string lastRefPath, lastNoRefPath;
    public static int lastFrames;

    public static string Resolve(string p) =>
        Path.GetFullPath(Path.IsPathRooted(p) ? p : Path.Combine(Application.dataPath, "..", p));

    // CLI·eval용: 다음 Editor 업데이트에서 실행 (호출이 바로 끝나도록). 결과는 status, running으로 확인
    public static void RecordLater(Settings s)
    {
        running = true;
        status = "대기 중";
        EditorApplication.delayCall += () => Record(s);
    }

    class Track
    {
        public float[] t, x, y, yaw, delta;
    }

    public static bool Record(Settings s)
    {
        running = true;
        status = "시작";
        if (EditorApplication.isPlayingOrWillChangePlaymode) return Fail("Play 중에는 실행할 수 없습니다. Play를 멈춘 뒤 실행하세요.");
        if (!s.makeRef && !s.makeNoRef) return Fail("만들 영상이 없습니다.");
        if (s.width % 2 != 0 || s.height % 2 != 0 || s.width < 16 || s.height < 16) return Fail("가로·세로는 16 이상의 짝수여야 합니다 (H.264).");

        // ---- 장면 오브젝트 ----
        var receiver = UnityEngine.Object.FindAnyObjectByType<CarStateReceiver>();
        if (receiver == null) return Fail("장면에 CarStateReceiver(car)가 없습니다.");
        Camera cam = null;
        foreach (var c in receiver.GetComponentsInChildren<Camera>(true))
            if (c.name == "Car_Camera") { cam = c; break; }
        if (cam == null) return Fail("car 아래에 Car_Camera가 없습니다.");
        var overlay = UnityEngine.Object.FindAnyObjectByType<PathOverlay>();

        // ---- 데이터 ----
        Track tr;
        string runPath = Resolve(s.runCsv);
        try { tr = LoadTrack(runPath, s.columnPrefix); }
        catch (Exception e) { return Fail($"주행 기록을 읽을 수 없습니다: {runPath}\n{e.Message}"); }
        List<Vector2> refPts = null;
        if (s.makeRef)
        {
            string refPath = Resolve(s.referenceCsv);
            try { refPts = LoadReference(refPath); }
            catch (Exception e) { return Fail($"레퍼런스 경로를 읽을 수 없습니다: {refPath}\n{e.Message}"); }
        }

        float t0 = Mathf.Max(s.startTime, tr.t[0]);
        float t1 = s.endTime > 0f ? Mathf.Min(s.endTime, tr.t[tr.t.Length - 1]) : tr.t[tr.t.Length - 1];
        int nFrames = Mathf.FloorToInt((t1 - t0) * s.fps + 1e-4f) + 1;
        if (nFrames < 2) return Fail($"영상 구간이 너무 짧습니다 (t {t0:F2} ~ {t1:F2} s).");

        string outDir = Resolve(s.outputFolder);
        Directory.CreateDirectory(outDir);
        string name = string.IsNullOrWhiteSpace(s.name) ? DefaultName(runPath) : s.name.Trim();
        string refOut = Path.Combine(outDir, name + "_ref.mp4");
        string noRefOut = Path.Combine(outDir, name + "_noref.mp4");

        // ---- 원래 상태 저장 (끝나면 되돌림) ----
        Transform car = receiver.transform;
        Vector3 carPos0 = car.position;
        Quaternion carRot0 = car.rotation;
        Transform fl = receiver.frontLeftWheel, fr = receiver.frontRightWheel;
        Quaternion flRot0 = fl ? fl.localRotation : Quaternion.identity;
        Quaternion frRot0 = fr ? fr.localRotation : Quaternion.identity;
        RenderTexture camTarget0 = cam.targetTexture;
        Rect camRect0 = cam.rect;
        int camMask0 = cam.cullingMask;
        float baseHeight = carPos0.y;   // CarStateReceiver.Start와 같음 (Play 시작 때 높이)

        // ---- 레퍼런스 선 (PathOverlay와 같은 모양, 같은 레이어) ----
        int layer = overlay != null ? overlay.gameObject.layer : LayerMask.NameToLayer("PathOverlay");
        if (layer < 0) layer = 31;
        GameObject refGo = null;
        LineRenderer refLine = null;
        Material lineMat = null;
        if (s.makeRef)
        {
            Color col = overlay ? overlay.referenceColor : new Color(1f, 0.9f, 0f);
            float w = overlay ? overlay.referenceWidth : 0.04f;
            float h = overlay ? overlay.height : 0.02f;
            refGo = new GameObject("CarCameraVideo_Reference") { hideFlags = HideFlags.HideAndDontSave, layer = layer };
            refLine = refGo.AddComponent<LineRenderer>();
            lineMat = new Material(Shader.Find("Sprites/Default")) { hideFlags = HideFlags.HideAndDontSave };
            refLine.useWorldSpace = true;
            refLine.loop = true;
            refLine.sharedMaterial = lineMat;
            refLine.startColor = refLine.endColor = col;
            refLine.startWidth = refLine.endWidth = w;
            refLine.numCapVertices = 2;
            refLine.shadowCastingMode = ShadowCastingMode.Off;
            refLine.receiveShadows = false;
            var pts = new Vector3[refPts.Count];
            for (int i = 0; i < pts.Length; i++) pts[i] = receiver.MapToWorld(refPts[i].x, refPts[i].y, h);
            refLine.positionCount = pts.Length;
            refLine.SetPositions(pts);
        }

        int ss = Mathf.Max(1, s.supersample);
        var bigRt = new RenderTexture(s.width * ss, s.height * ss, 24, GraphicsFormat.R8G8B8A8_SRGB) { name = "CarCameraVideoBig", hideFlags = HideFlags.HideAndDontSave };
        var outRt = ss > 1 ? new RenderTexture(s.width, s.height, 0, GraphicsFormat.R8G8B8A8_SRGB) { name = "CarCameraVideoOut", hideFlags = HideFlags.HideAndDontSave } : bigRt;
        bigRt.Create();
        if (outRt != bigRt) { outRt.Create(); }
        var tex = new Texture2D(s.width, s.height, TextureFormat.RGBA32, false) { hideFlags = HideFlags.HideAndDontSave };

        MediaEncoder encRef = null, encNoRef = null;
        bool ok = false, cancelled = false;
        int done = 0;
        double start = EditorApplication.timeSinceStartup;
        try
        {
            if (s.makeRef) encRef = NewEncoder(refOut, s);
            if (s.makeNoRef) encNoRef = NewEncoder(noRefOut, s);

            cam.targetTexture = bigRt;
            cam.rect = new Rect(0f, 0f, 1f, 1f);
            cam.aspect = (float)s.width / s.height;

            int seg = 0;
            for (int i = 0; i < nFrames; i++)
            {
                float t = t0 + (float)i / s.fps;
                while (seg < tr.t.Length - 2 && tr.t[seg + 1] <= t) seg++;
                float a = Mathf.Clamp01((t - tr.t[seg]) / Mathf.Max(1e-6f, tr.t[seg + 1] - tr.t[seg]));
                float x = Mathf.Lerp(tr.x[seg], tr.x[seg + 1], a);
                float y = Mathf.Lerp(tr.y[seg], tr.y[seg + 1], a);
                float yaw = tr.yaw[seg] + a * Mathf.DeltaAngle(tr.yaw[seg] * Mathf.Rad2Deg, tr.yaw[seg + 1] * Mathf.Rad2Deg) * Mathf.Deg2Rad;
                float delta = Mathf.Lerp(tr.delta[seg], tr.delta[seg + 1], a);

                // CarStateReceiver.Update와 같은 변환
                Vector3 origin = receiver.mapOrigin ? receiver.mapOrigin.position : Vector3.zero;
                car.SetPositionAndRotation(new Vector3(origin.x + x, baseHeight, origin.z + y),
                                           Quaternion.Euler(0f, 90f - yaw * Mathf.Rad2Deg + receiver.yawOffsetDeg, 0f));
                float steerDeg = -delta * Mathf.Rad2Deg;
                SteerWheel(car, fl, flRot0, steerDeg);
                SteerWheel(car, fr, frRot0, steerDeg);

                if (encRef != null)
                {
                    refLine.enabled = true;
                    cam.cullingMask = camMask0 | (1 << layer);
                    RenderInto(cam, bigRt, outRt, tex);
                    encRef.AddFrame(tex);
                }
                if (encNoRef != null)
                {
                    if (refLine) refLine.enabled = false;
                    cam.cullingMask = camMask0;
                    RenderInto(cam, bigRt, outRt, tex);
                    encNoRef.AddFrame(tex);
                }
                done = i + 1;

                if (i % 10 == 0)
                {
                    status = $"렌더링 중 {done}/{nFrames} (t = {t:F1} s)";
                    if (s.showProgressBar && EditorUtility.DisplayCancelableProgressBar("Car_Camera 영상", status, (float)done / nFrames))
                    {
                        cancelled = true;
                        break;
                    }
                }
            }
            ok = !cancelled;
        }
        catch (Exception e)
        {
            status = $"실패 ({done}/{nFrames} 프레임에서): {e.GetType().Name}: {e.Message}";
            Debug.LogException(e);
        }
        finally
        {
            encRef?.Dispose();
            encNoRef?.Dispose();
            if (s.showProgressBar) EditorUtility.ClearProgressBar();

            car.SetPositionAndRotation(carPos0, carRot0);
            if (fl) fl.localRotation = flRot0;
            if (fr) fr.localRotation = frRot0;
            cam.targetTexture = camTarget0;
            cam.rect = camRect0;
            cam.cullingMask = camMask0;
            cam.ResetAspect();

            if (refGo) UnityEngine.Object.DestroyImmediate(refGo);
            if (lineMat) UnityEngine.Object.DestroyImmediate(lineMat);
            if (outRt != bigRt) { outRt.Release(); UnityEngine.Object.DestroyImmediate(outRt); }
            bigRt.Release();
            UnityEngine.Object.DestroyImmediate(bigRt);
            UnityEngine.Object.DestroyImmediate(tex);
            SceneView.RepaintAll();
            running = false;
        }

        double sec = EditorApplication.timeSinceStartup - start;
        lastFrames = done;
        lastRefPath = s.makeRef ? refOut : null;
        lastNoRefPath = s.makeNoRef ? noRefOut : null;
        if (ok)
        {
            status = $"완료: {done}프레임 ({(done - 1f) / s.fps:F1} s, {s.fps} fps, {s.width}x{s.height}), 걸린 시간 {sec:F0} s\n" +
                     (s.makeRef ? refOut + "\n" : "") + (s.makeNoRef ? noRefOut : "");
            Debug.Log("[CarCameraVideo] " + status);
        }
        else if (cancelled)
        {
            status = $"취소됨: {done}/{nFrames} 프레임까지 저장";
            Debug.LogWarning("[CarCameraVideo] " + status);
        }
        else
        {
            Debug.LogError("[CarCameraVideo] " + status);
        }
        return ok;
    }

    static bool Fail(string msg)
    {
        status = "실패: " + msg;
        running = false;
        Debug.LogError("[CarCameraVideo] " + msg);
        return false;
    }

    static MediaEncoder NewEncoder(string path, Settings s)
    {
        if (File.Exists(path)) File.Delete(path);
        var h264 = new H264EncoderAttributes
        {
            gopSize = (uint)s.fps,
            numConsecutiveBFrames = 2,
            profile = VideoEncodingProfile.H264High
        };
        var attr = new VideoTrackEncoderAttributes(h264)
        {
            frameRate = new MediaRational(s.fps),
            width = (uint)s.width,
            height = (uint)s.height,
            targetBitRate = (uint)(s.bitrateMbps * 1000000)
        };
        return new MediaEncoder(path, attr);
    }

    // Car_Camera → bigRt (supersample) → outRt (축소) → tex (CPU)
    static void RenderInto(Camera cam, RenderTexture bigRt, RenderTexture outRt, Texture2D tex)
    {
        cam.Render();
        if (outRt != bigRt) Graphics.Blit(bigRt, outRt);
        var req = AsyncGPUReadback.Request(outRt, 0, TextureFormat.RGBA32);
        req.WaitForCompletion();
        if (req.hasError) throw new Exception("GPU 읽기 실패");
        tex.SetPixelData(req.GetData<byte>(), 0);
        tex.Apply(false);
    }

    // CarStateReceiver.SteerWheel과 같음
    static void SteerWheel(Transform car, Transform wheel, Quaternion baseRot, float steerDeg)
    {
        if (!wheel) return;
        Vector3 upInParent = wheel.parent ? wheel.parent.InverseTransformDirection(car.up) : Vector3.up;
        wheel.localRotation = Quaternion.AngleAxis(steerDeg, upInParent) * baseRot;
    }

    // ".../switching_2/data/20261116_194238/timeseries.csv" → "switching_2_20261116_194238"
    // ".../results/20261116_001020_pp-paper_.../timeseries.csv" → "20261116_001020_pp-paper_..."
    static string DefaultName(string csvPath)
    {
        string dir = Path.GetDirectoryName(csvPath);
        string run = Path.GetFileName(dir);
        string parent = Path.GetFileName(Path.GetDirectoryName(dir));
        if (parent == "data")
            return Path.GetFileName(Path.GetDirectoryName(Path.GetDirectoryName(dir))) + "_" + run;
        return run;
    }

    static Track LoadTrack(string path, string prefix)
    {
        string[] lines = File.ReadAllLines(path);
        if (lines.Length < 3) throw new Exception("데이터 줄이 2개보다 적습니다.");
        string[] head = lines[0].Trim().Split(',');
        int Col(string n)
        {
            int i = Array.IndexOf(head, n);
            return i;
        }
        string p = prefix ?? "";
        if (Col(p + "x_m") < 0) p = "";   // 앞글자 열이 없으면 앞글자 없이
        int ct = Col("t_s"), cx = Col(p + "x_m"), cy = Col(p + "y_m"), cyaw = Col(p + "yaw_rad");
        int cd = Col(p + "delta_out_rad");
        if (cd < 0) cd = Col(p + "delta_rad");
        if (ct < 0 || cx < 0 || cy < 0 || cyaw < 0)
            throw new Exception($"열을 찾을 수 없습니다: t_s, {p}x_m, {p}y_m, {p}yaw_rad");

        var t = new List<float>(); var x = new List<float>(); var y = new List<float>();
        var yaw = new List<float>(); var d = new List<float>();
        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;
            string[] f = lines[i].Split(',');
            if (!Num(f, ct, out float vt) || !Num(f, cx, out float vx) || !Num(f, cy, out float vy) || !Num(f, cyaw, out float vyaw)) continue;
            if (t.Count > 0 && vt <= t[t.Count - 1]) continue;   // 시간이 늘지 않는 줄은 건너뜀
            float vd = 0f;
            if (cd >= 0 && !Num(f, cd, out vd)) vd = 0f;
            t.Add(vt); x.Add(vx); y.Add(vy); yaw.Add(vyaw); d.Add(vd);
        }
        if (t.Count < 2) throw new Exception("읽은 줄이 2개보다 적습니다.");
        return new Track { t = t.ToArray(), x = x.ToArray(), y = y.ToArray(), yaw = yaw.ToArray(), delta = d.ToArray() };
    }

    static List<Vector2> LoadReference(string path)
    {
        string[] lines = File.ReadAllLines(path);
        string[] head = lines[0].Trim().Split(',');
        int cx = Array.IndexOf(head, "x_m"), cy = Array.IndexOf(head, "y_m");
        if (cx < 0 || cy < 0) throw new Exception("열 x_m, y_m을 찾을 수 없습니다.");
        var pts = new List<Vector2>();
        for (int i = 1; i < lines.Length; i++)
        {
            string[] f = lines[i].Split(',');
            if (Num(f, cx, out float x) && Num(f, cy, out float y)) pts.Add(new Vector2(x, y));
        }
        if (pts.Count < 2) throw new Exception("점이 2개보다 적습니다.");
        return pts;
    }

    static bool Num(string[] f, int i, out float v)
    {
        v = 0f;
        return i >= 0 && i < f.Length &&
               float.TryParse(f[i], NumberStyles.Float, CultureInfo.InvariantCulture, out v) && !float.IsNaN(v);
    }
}
