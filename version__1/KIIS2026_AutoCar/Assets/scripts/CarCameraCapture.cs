using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;

// Car_Camera 화면을 MATLAB 시간 t 기준으로 일정 주기마다 이미지로 저장합니다.
// YOLOv8-seg 학습용(yolo)과 제어기 전환 분석용(analysis)을 따로 저장합니다.
// 붙일 곳: Car_Camera (Camera가 있는 오브젝트)
//
// 저장 주기 바꾸기: Inspector에서 Car_Camera → Car Camera Capture → "저장 주기" → Period [s] (기본 0.1 = PP_PID의 Ts)
//   Play 중에 바꿔도 다음 이미지부터 적용됩니다.
//
// 언제 찍나: MATLAB이 보낸 상태의 시간 t가 다음 주기(Period의 배수)에 도달할 때마다 한 장.
//   PP_PID (blended_discrete, Ts 0.1 s)              : 매 스텝 (t = 0, 0.1, 0.2, ...)
//   pure_pursuit_control, PID_control (dt 0.02 s)  : 5스텝마다
//   MATLAB이 상태를 보내지 않으면 (연결 전, 키보드 대기, 종료 후) 찍지 않습니다.
//   화면 배치(CameraViews 1~5)와 상관없이 Car_Camera만 따로 렌더링해서 저장합니다.
//
// 저장 위치 (기본값, 상대 경로는 Unity 프로젝트 폴더 KIIS2026_AutoCar 기준):
//   KIIS2026_Unity/yolo/<Play 시각>/images/<Play 시각>_r01_k000229.png   YOLO 학습용 사진 (Play 한 번 = 폴더 하나)
//   KIIS2026_Unity/yolo/<Play 시각>/masks/<같은 이름>.png                 차선 마스크 (바깥 실선 빨강, 중앙 점선 초록, 안쪽 실선 파랑)
//   KIIS2026_Unity/yolo/<Play 시각>/labels/                              YOLO 라벨: lane_yolo/mask_to_yolo.py가 masks에서 만듦
//   KIIS2026_Unity/analysis/<Play 시각>_capture_log.csv                   제어기 전환 분석용: 사진마다 MATLAB t와 차 상태
//   <Play 시각> = Play를 누른 시각 yyyyMMdd_HHmmss (MATLAB results 폴더와 같은 형식)
//   r01 = 이 Play 안에서 MATLAB 실행 번호 (RESET을 받거나 t가 줄어들면 1 늘어남)
//   k   = round(t / MATLAB Ts) = MATLAB 스텝 번호 (PP_PID 그림 제목의 k와 같음)
// CSV 열 (이름·단위는 MATLAB timeseries.csv 규칙):
//   play,run,k,t_s,x_m,y_m,yaw_rad,v_mps,delta_rad,yawrate_radps,e_lat_m,e_head_rad,image,unity_time_s,frame
//   image = CSV 파일 위치 기준 사진 상대 경로 (MATLAB: fullfile(csv 폴더, image))
//
// 제어기 전환 (MATLAB switch_discrete.m): 찍은 사진을 lane_yolo/lane_server.py로 보내고 (TCP 55002),
//   받은 판단 "LANE,k,PP_PID|PID,..."를 CarStateReceiver 연결로 MATLAB에 그대로 전달합니다.
//   MATLAB이 스텝마다 판단을 기다리므로 Period는 MATLAB Ts(0.1 s)와 같아야 합니다.
//
// 사진은 찍은 프레임 안에서 GPU에서 읽고, 인코딩·저장은 별도 스레드에서 합니다.
public class CarCameraCapture : MonoBehaviour
{
    public enum ImageFormat { PNG, JPG }

    [Header("저장 주기")]
    [Tooltip("MATLAB 시간 t 기준 저장 간격 [s]. 기본 0.1 = PP_PID(blended_discrete)의 Ts")]
    [Min(0.001f)] public float period = 0.1f;
    [Tooltip("파일 이름과 CSV의 k = round(t / MATLAB Ts). PP_PID는 0.1, pure_pursuit_control·PID_control은 0.02")]
    [Min(0.001f)] public float matlabTs = 0.1f;

    [Header("대상 (비워두면 자동으로 찾음)")]
    [Tooltip("비워두면 이 오브젝트의 Camera (Car_Camera)")]
    public Camera cam;
    [Tooltip("비워두면 장면에서 CarStateReceiver를 찾습니다 (car)")]
    public CarStateReceiver receiver;

    [Header("이미지")]
    public int width = 640;
    public int height = 480;
    public ImageFormat format = ImageFormat.PNG;
    [Range(1, 100)] public int jpgQuality = 95;

    [Header("저장 폴더 (상대 경로면 Unity 프로젝트 폴더 KIIS2026_AutoCar 기준. Assets 안은 피하세요)")]
    [Tooltip("YOLO 학습용 사진: <이 폴더>/<Play 시각>/images, labels")]
    public string yoloFolder = "../yolo";
    [Tooltip("제어기 전환 분석용 CSV: <이 폴더>/<Play 시각>_capture_log.csv")]
    public string analysisFolder = "../analysis";

    [Header("YOLO 라벨용 차선 마스크 (yolo/<Play 시각>/masks)")]
    [Tooltip("사진마다 같은 시점에서 맵을 차선 라벨 텍스처로 바꿔 한 번 더 렌더링해 저장 (lane_yolo/mask_to_yolo.py가 labels로 변환)")]
    public bool saveMasks = true;
    [Tooltip("lane_yolo/make_label_texture.py가 만든 텍스처. 상대 경로면 Unity 프로젝트 폴더 기준")]
    public string labelTexturePath = "../lane_yolo/lane_label_texture.png";
    [Tooltip("맵 Plane의 Renderer. 비워두면 이름이 Plane인 오브젝트")]
    public Renderer mapRenderer;
    [Tooltip("카메라에서 이 거리 [m]보다 먼 차선은 마스크에서 뺌 (맵 반대편 도로의 선이 섞이지 않게)")]
    [Min(0.5f)] public float labelMaxDistance = 6f;

    [Header("실시간 차선 판단 서버 (lane_yolo/lane_server.py, MATLAB switch_discrete.m용)")]
    [Tooltip("찍은 사진을 lane_server로 보내고, 받은 판단 줄(LANE,k,...)을 MATLAB으로 그대로 전달. 서버가 없으면 1초마다 다시 접속 시도")]
    public bool sendToLaneServer = true;
    public string laneServerHost = "127.0.0.1";
    public int laneServerPort = 55002;

    [Header("상태 (확인용, 수정하지 마세요)")]
    public bool laneServerConnected;
    public int laneSent, laneReplies;
    public string lastLane;
    public string playName;        // 이번 Play 시각 (yyyyMMdd_HHmmss)
    public string imageFolder;     // 이번 Play의 사진 폴더 (전체 경로)
    public string csvPath;         // 이번 Play의 CSV (전체 경로)
    public int run = 1;            // MATLAB 실행 번호
    public int savedCount;         // 저장을 마친 이미지 수
    public int skippedSlots;       // Unity 프레임이 MATLAB보다 느려서 건너뛴 주기 수
    public int failedCount;        // 읽기·저장 실패 수
    public float lastSavedT = float.NaN;
    public float lastCaptureMs;    // 한 장(사진 + 마스크) 렌더링·GPU 읽기에 걸린 시간 [ms]

    // ---- 아래는 내부 동작용 ----
    class Job
    {
        public string path, csvImage, maskPath;
        public int run, frame, width, height, quality;
        public long k;
        public ImageFormat format;
        public float t, x, y, yaw, v, delta, yawRate, eLat, eHead, unityTime;
        public byte[] rgba, maskRgba;
    }

    class Pending
    {
        public AsyncGPUReadbackRequest img, mask;
        public bool hasMask;
        public Job job;
    }

    RenderTexture rt, maskRt;
    Material labelMat;
    Texture2D labelTex;
    string maskFolder;
    readonly Queue<Pending> pending = new Queue<Pending>();
    BlockingCollection<Job> queue;
    Thread writer;
    StreamWriter csv;
    string analysisDir;
    int saved, failed;             // 스레드에서 올리는 값 (Interlocked)
    string lastError;

    // lane_server 연결 (접속·읽기 스레드 하나, 보내기 스레드 하나)
    BlockingCollection<Job> laneQueue;
    Thread laneConnThread, laneSendThread;
    volatile bool laneRunning;
    volatile NetworkStream laneStream;
    TcpClient laneClient;
    int laneSentT, laneRepliesT;
    volatile string lastLaneT;

    int seenResets, seenReceived;
    float lastRecvT = float.NaN;   // 마지막으로 받은 상태의 t
    int capturedInRun;
    long lastNameK = long.MinValue;
    bool warnedSkip;

    void Start()
    {
        if (cam == null) cam = GetComponent<Camera>();
        if (cam != null)
        {
            // Car_Camera는 사진 저장용(cam.Render()로 따로 렌더링)이라 Game 화면에는 그리지 않음.
            // 켜 두면 화면 전체(depth 0)를 덮어서 CameraViews의 1~5 화면 전환이 보이지 않음 (2026-11-21)
            cam.enabled = false;
            var listener = cam.GetComponent<AudioListener>();
            if (listener != null) listener.enabled = false;   // Main Camera와 AudioListener 2개 경고 방지
        }
        if (receiver == null) receiver = FindFirstObjectByType<CarStateReceiver>();
        if (cam == null) { Debug.LogError("[CarCameraCapture] Camera가 없습니다. Car_Camera에 붙여주세요."); enabled = false; return; }
        if (receiver == null) { Debug.LogError("[CarCameraCapture] 장면에 CarStateReceiver가 없습니다."); enabled = false; return; }

        // Play 시작 시각으로 이름 정함 (MATLAB results 폴더와 같은 형식). 같은 초에 두 번 Play하면 _2, _3 ...
        string yoloRoot = ResolveFolder(yoloFolder);
        analysisDir = ResolveFolder(analysisFolder);
        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
        playName = stamp;
        for (int i = 2; Directory.Exists(Path.Combine(yoloRoot, playName)) ||
                        File.Exists(Path.Combine(analysisDir, playName + "_capture_log.csv")); i++)
            playName = $"{stamp}_{i}";

        imageFolder = Path.Combine(yoloRoot, playName, "images");
        Directory.CreateDirectory(imageFolder);
        Directory.CreateDirectory(Path.Combine(yoloRoot, playName, "labels"));
        Directory.CreateDirectory(analysisDir);

        if (saveMasks) SetupMasks(Path.Combine(yoloRoot, playName, "masks"));

        csvPath = Path.Combine(analysisDir, playName + "_capture_log.csv");
        csv = new StreamWriter(csvPath, false, new UTF8Encoding(false));
        csv.WriteLine("play,run,k,t_s,x_m,y_m,yaw_rad,v_mps,delta_rad,yawrate_radps,e_lat_m,e_head_rad,image,unity_time_s,frame");
        csv.Flush();

        queue = new BlockingCollection<Job>();
        writer = new Thread(WriteLoop) { IsBackground = true, Name = "CarCameraCapture" };
        writer.Start();

        if (sendToLaneServer)
        {
            laneRunning = true;
            laneQueue = new BlockingCollection<Job>();
            laneConnThread = new Thread(LaneConnectLoop) { IsBackground = true, Name = "LaneServerConn" };
            laneSendThread = new Thread(LaneSendLoop) { IsBackground = true, Name = "LaneServerSend" };
            laneConnThread.Start();
            laneSendThread.Start();
        }

        seenResets = receiver.resetCount;
        seenReceived = receiver.receivedCount;
        Debug.Log($"[CarCameraCapture] 사진: {imageFolder}\n분석 CSV: {csvPath}  (주기 {period} s, MATLAB t 기준)");
    }

    // 라벨 텍스처를 파일에서 직접 읽음 (Unity import 압축·밉맵이 없어 색이 정확히 유지됨)
    void SetupMasks(string folder)
    {
        if (mapRenderer == null) { var p = GameObject.Find("Plane"); if (p) mapRenderer = p.GetComponent<Renderer>(); }
        string texPath = ResolveFolder(labelTexturePath);
        if (mapRenderer == null || !File.Exists(texPath))
        {
            Debug.LogError($"[CarCameraCapture] 차선 마스크를 저장할 수 없습니다 (맵 Renderer {(mapRenderer ? "있음" : "없음")}, 라벨 텍스처 {texPath} {(File.Exists(texPath) ? "있음" : "없음")}). 사진만 저장합니다.");
            return;
        }
        labelTex = new Texture2D(2, 2, TextureFormat.RGBA32, false, false) { name = "LaneLabel" };
        labelTex.LoadImage(File.ReadAllBytes(texPath));
        labelTex.filterMode = FilterMode.Point;      // 색을 섞지 않음 (클래스 색 그대로)
        labelTex.wrapMode = TextureWrapMode.Clamp;
        var src = mapRenderer.sharedMaterial;
        labelMat = new Material(Shader.Find("Unlit/Texture")) { name = "LaneLabelMat", mainTexture = labelTex };
        labelMat.mainTextureScale = src.mainTextureScale;
        labelMat.mainTextureOffset = src.mainTextureOffset;
        maskFolder = folder;
        Directory.CreateDirectory(maskFolder);
    }

    static string ResolveFolder(string p) =>
        Path.GetFullPath(Path.IsPathRooted(p) ? p : Path.Combine(Application.dataPath, "..", p));

    // CarStateReceiver.Update가 차를 옮긴 뒤에 찍어야 하므로 LateUpdate
    void LateUpdate()
    {
        if (queue == null) return;
        CheckForCapture();
        CollectReadbacks(false);

        savedCount = Volatile.Read(ref saved);
        failedCount = Volatile.Read(ref failed);
        laneServerConnected = laneStream != null;
        laneSent = Volatile.Read(ref laneSentT);
        laneReplies = Volatile.Read(ref laneRepliesT);
        lastLane = lastLaneT;
        string err = Interlocked.Exchange(ref lastError, null);
        if (err != null) Debug.LogError($"[CarCameraCapture] 저장 실패: {err}");
    }

    void CheckForCapture()
    {
        // MATLAB이 다시 실행됨 → 새 run
        if (receiver.resetCount != seenResets) { seenResets = receiver.resetCount; NewRun(); }

        if (receiver.receivedCount == seenReceived) return;   // 이번 프레임에 새 상태 없음
        seenReceived = receiver.receivedCount;

        float t = receiver.t;
        if (!float.IsNaN(lastRecvT) && t < lastRecvT - 1e-4f) NewRun();   // RESET 없이 t가 처음부터 시작한 경우
        lastRecvT = t;

        // MATLAB 실행은 t = 0부터 시작하므로, 첫 장이 t = 0이 아니면 그 앞의 주기도 건너뛴 것으로 셈
        long slot = Slot(t);
        long slotLast = capturedInRun > 0 ? Slot(lastSavedT) : -1;
        if (slot <= slotLast) return;                          // 아직 다음 주기가 아님
        if (slot > slotLast + 1)
        {
            skippedSlots += (int)(slot - slotLast - 1);
            if (!warnedSkip)
            {
                warnedSkip = true;
                Debug.LogWarning("[CarCameraCapture] Unity가 MATLAB 주기를 따라가지 못해 건너뛴 이미지가 있습니다 " +
                                 "(MATLAB을 realtime으로 실행했는지, Play가 완전히 시작된 뒤 MATLAB을 실행했는지 확인하세요). " +
                                 "건너뛴 수는 Skipped Slots에 표시됩니다.");
            }
        }
        Capture(t);
    }

    // t가 몇 번째 주기에 속하는지 (0.30000001 / 0.1 같은 소수 오차는 1e-3 여유로 흡수)
    long Slot(float t) => (long)Math.Floor((double)t / period + 1e-3);

    void NewRun()
    {
        if (capturedInRun == 0) return;   // 아직 찍은 것이 없으면 번호 그대로
        run++;
        capturedInRun = 0;
        lastSavedT = float.NaN;
        lastRecvT = float.NaN;
        lastNameK = long.MinValue;
    }

    void EnsureRt(ref RenderTexture t, string name)
    {
        if (t != null && t.width == width && t.height == height) return;
        if (t != null) t.Release();
        t = new RenderTexture(width, height, 24, GraphicsFormat.R8G8B8A8_SRGB) { name = name };
        t.Create();
    }

    void Capture(float t)
    {
        double captureStart = Time.realtimeSinceStartupAsDouble;
        EnsureRt(ref rt, "CarCameraCapture");
        bool withMask = labelMat != null;
        if (withMask) EnsureRt(ref maskRt, "CarCameraCaptureMask");

        // 화면 출력 설정은 그대로 두고, 이번 한 번만 rt에 렌더링
        RenderTexture prevTarget = cam.targetTexture;
        Rect prevRect = cam.rect;
        cam.targetTexture = rt;
        cam.rect = new Rect(0f, 0f, 1f, 1f);
        cam.aspect = (float)width / height;
        cam.Render();

        // 차선 마스크: 같은 시점, 맵 재질만 라벨 텍스처(조명 없음)로, 배경 검정, labelMaxDistance보다 먼 곳은 그리지 않음.
        // 차체·핸들은 그대로 그려서 가려진 차선은 마스크에서도 빠짐
        if (withMask)
        {
            Material prevMat = mapRenderer.sharedMaterial;
            CameraClearFlags prevClear = cam.clearFlags;
            Color prevBg = cam.backgroundColor;
            float prevFar = cam.farClipPlane;
            cam.targetTexture = maskRt;
            mapRenderer.sharedMaterial = labelMat;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = Color.black;
            cam.farClipPlane = labelMaxDistance;
            cam.Render();
            mapRenderer.sharedMaterial = prevMat;
            cam.clearFlags = prevClear;
            cam.backgroundColor = prevBg;
            cam.farClipPlane = prevFar;
        }

        cam.targetTexture = prevTarget;
        cam.rect = prevRect;
        cam.ResetAspect();

        // 파일 이름: <Play 시각>_r01_k000229.png  (Period < MATLAB Ts라서 k가 겹치면 뒤에 _<t ms>를 붙임)
        long k = (long)Math.Round((double)t / matlabTs);
        string name = $"{playName}_r{run:00}_k{k:000000}";
        if (k == lastNameK) name += $"_{Mathf.RoundToInt(t * 1000f)}";
        lastNameK = k;
        name += format == ImageFormat.PNG ? ".png" : ".jpg";
        string path = Path.Combine(imageFolder, name);

        var job = new Job
        {
            path = path,
            csvImage = Path.GetRelativePath(analysisDir, path).Replace('\\', '/'),
            run = run, k = k, frame = Time.frameCount, unityTime = Time.time,
            width = width, height = height, format = format, quality = jpgQuality,
            t = t, x = receiver.x, y = receiver.y, yaw = receiver.yaw, v = receiver.v,
            delta = receiver.delta, yawRate = receiver.yawRate, eLat = receiver.eLat, eHead = receiver.eHead,
        };
        var item = new Pending { job = job, img = AsyncGPUReadback.Request(rt, 0, TextureFormat.RGBA32), hasMask = withMask };
        if (withMask)
        {
            job.maskPath = Path.Combine(maskFolder, Path.ChangeExtension(name, ".png"));   // 마스크는 항상 PNG (무손실)
            item.mask = AsyncGPUReadback.Request(maskRt, 0, TextureFormat.RGBA32);
        }
        // 같은 프레임 안에서 GPU 읽기를 끝냄. 비동기로 두면 Editor가 수백 fps로 돌 때 요청이 몇 프레임(수 ms) 안에
        // 끝나지 않아 실패 처리되는 경우가 있었음 (2026-11-15: 350장 중 8~11장, 모두 두 번째 요청인 마스크).
        // 0.1 s에 한 번 몇 ms 기다리는 것이라 프레임에는 거의 영향이 없음. 인코딩·저장은 그대로 별도 스레드
        item.img.WaitForCompletion();
        if (withMask) item.mask.WaitForCompletion();
        pending.Enqueue(item);
        lastCaptureMs = (float)(Time.realtimeSinceStartupAsDouble - captureStart) * 1000f;

        capturedInRun++;
        lastSavedT = t;
    }

    // GPU 읽기가 끝난 것부터 순서대로 저장 스레드에 넘김 (wait = true: 끝날 때까지 기다림)
    void CollectReadbacks(bool wait)
    {
        while (pending.Count > 0)
        {
            Pending p = pending.Peek();
            if (wait) { p.img.WaitForCompletion(); if (p.hasMask) p.mask.WaitForCompletion(); }
            else if (!p.img.done || (p.hasMask && !p.mask.done)) break;
            pending.Dequeue();

            if (p.img.hasError || (p.hasMask && p.mask.hasError))
            {
                Interlocked.Increment(ref failed);
                Debug.LogWarning($"[CarCameraCapture] GPU 읽기 실패: {Path.GetFileName(p.job.path)} (사진 {(p.img.hasError ? "실패" : "성공")}, " +
                                 $"마스크 {(p.hasMask ? (p.mask.hasError ? "실패" : "성공") : "없음")}, 요청 프레임 {p.job.frame}, 지금 {Time.frameCount})");
                continue;
            }
            p.job.rgba = p.img.GetData<byte>().ToArray();
            if (p.hasMask) p.job.maskRgba = p.mask.GetData<byte>().ToArray();
            queue.Add(p.job);
            if (laneQueue != null && laneStream != null) laneQueue.Add(p.job);   // 연결돼 있을 때만 (없으면 쌓이지 않게)
        }
    }

    // 별도 스레드: RGBA → RGB, 인코딩, 파일 저장, CSV 한 줄
    void WriteLoop()
    {
        foreach (Job job in queue.GetConsumingEnumerable())
        {
            try
            {
                byte[] rgb = ToRgb(job.rgba, job.width * job.height);
                uint w = (uint)job.width, h = (uint)job.height;
                byte[] bytes = job.format == ImageFormat.PNG
                    ? ImageConversion.EncodeArrayToPNG(rgb, GraphicsFormat.R8G8B8_SRGB, w, h)
                    : ImageConversion.EncodeArrayToJPG(rgb, GraphicsFormat.R8G8B8_SRGB, w, h, 0, job.quality);
                File.WriteAllBytes(job.path, bytes);
                if (job.maskRgba != null)
                    File.WriteAllBytes(job.maskPath, ImageConversion.EncodeArrayToPNG(
                        ToRgb(job.maskRgba, job.width * job.height), GraphicsFormat.R8G8B8_SRGB, w, h));

                csv.WriteLine(string.Join(",",
                    playName, I(job.run), job.k.ToString(CultureInfo.InvariantCulture),
                    F(job.t), F(job.x), F(job.y), F(job.yaw), F(job.v), F(job.delta), F(job.yawRate),
                    F(job.eLat), F(job.eHead), job.csvImage, F(job.unityTime), I(job.frame)));
                csv.Flush();
                Interlocked.Increment(ref saved);
            }
            catch (Exception e)
            {
                Interlocked.Increment(ref failed);
                lastError = $"{Path.GetFileName(job.path)}: {e.Message}";
            }
        }
    }

    // 스레드: lane_server에 접속하고, 받은 줄 중 "LANE,"으로 시작하는 줄을 MATLAB으로 전달
    void LaneConnectLoop()
    {
        bool warned = false;
        while (laneRunning)
        {
            try
            {
                laneClient = new TcpClient { NoDelay = true };
                laneClient.Connect(laneServerHost, laneServerPort);
                laneStream = laneClient.GetStream();
                Debug.Log($"[CarCameraCapture] lane_server 연결됨 ({laneServerHost}:{laneServerPort})");
                warned = false;
                using (var reader = new StreamReader(laneStream, Encoding.ASCII))
                {
                    string line;
                    while (laneRunning && (line = reader.ReadLine()) != null)
                    {
                        if (!line.StartsWith("LANE,")) continue;
                        Interlocked.Increment(ref laneRepliesT);
                        lastLaneT = line;
                        receiver.SendLine(line);   // MATLAB이 연결돼 있지 않으면 버려짐
                    }
                }
            }
            catch (Exception e)
            {
                if (!warned && laneRunning)
                {
                    Debug.Log($"[CarCameraCapture] lane_server({laneServerHost}:{laneServerPort})에 연결할 수 없습니다 ({e.GetType().Name}). " +
                              "1초마다 다시 시도합니다 (전환 실험이 아니면 무시해도 됨).");
                    warned = true;
                }
            }
            finally
            {
                laneStream = null;
                try { laneClient?.Close(); } catch { }
            }
            if (laneRunning) Thread.Sleep(1000);
        }
    }

    // 스레드: 사진을 "IMG,k,t,run,w,h,nbytes" + RGB 바이트로 보냄
    void LaneSendLoop()
    {
        foreach (Job job in laneQueue.GetConsumingEnumerable())
        {
            NetworkStream s = laneStream;
            if (s == null) continue;
            try
            {
                byte[] rgb = ToRgb(job.rgba, job.width * job.height);
                byte[] header = Encoding.ASCII.GetBytes(string.Join(",", "IMG",
                    job.k.ToString(CultureInfo.InvariantCulture), F(job.t), I(job.run),
                    I(job.width), I(job.height), I(rgb.Length)) + "\n");
                s.Write(header, 0, header.Length);
                s.Write(rgb, 0, rgb.Length);
                Interlocked.Increment(ref laneSentT);
            }
            catch (Exception)
            {
                // 연결이 끊기면 접속 스레드가 다시 연결함
            }
        }
    }

    static byte[] ToRgb(byte[] rgba, int n)
    {
        var rgb = new byte[n * 3];
        for (int i = 0, j = 0; i < n; i++, j += 4)
        {
            rgb[3 * i] = rgba[j];
            rgb[3 * i + 1] = rgba[j + 1];
            rgb[3 * i + 2] = rgba[j + 2];
        }
        return rgb;
    }

    static string F(float v) => v.ToString("0.######", CultureInfo.InvariantCulture);
    static string I(int v) => v.ToString(CultureInfo.InvariantCulture);

    // Play를 멈출 때: 남은 GPU 읽기와 저장을 모두 끝낸 뒤 파일을 닫음
    void OnDestroy()
    {
        if (queue == null) return;
        CollectReadbacks(true);
        queue.CompleteAdding();
        if (laneQueue != null)
        {
            laneRunning = false;
            laneQueue.CompleteAdding();
            try { laneClient?.Close(); } catch { }
            laneConnThread?.Join(1000);
            laneSendThread?.Join(1000);
            laneQueue = null;
        }
        if (writer != null && !writer.Join(10000))
            Debug.LogWarning("[CarCameraCapture] 저장 스레드가 10초 안에 끝나지 않았습니다. 마지막 이미지 일부가 빠졌을 수 있습니다.");
        try { csv?.Dispose(); } catch { }
        queue = null;
        if (rt != null) { rt.Release(); Destroy(rt); rt = null; }
        if (maskRt != null) { maskRt.Release(); Destroy(maskRt); maskRt = null; }
        if (labelMat != null) { Destroy(labelMat); Destroy(labelTex); labelMat = null; }
        Debug.Log($"[CarCameraCapture] 저장 완료: {Volatile.Read(ref saved)}장 → {imageFolder}");
    }
}
