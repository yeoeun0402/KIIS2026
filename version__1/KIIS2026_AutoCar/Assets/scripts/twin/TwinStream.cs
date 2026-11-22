using System;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;

// [twin] Game 화면을 웹 대시보드(dashboard/twin_dash.py)로 보내기: 0.1초마다 JPEG 한 장을 덮어씀.
//   파일: /dev/shm/twin_live/sim.jpg (TWIN_LIVE_DIR 환경 변수로 폴더 변경)
//   폴더가 있을 때만 동작 (대시보드가 만듦) → 대시보드 없이 Unity만 쓸 때는 아무것도 안 함.
//   화면 그대로 (숫자키 1~5로 바꾼 시점 · 궤적 · 화면 글씨 포함), 폭 maxWidth로 줄임.
//   GPU에서 줄이고 비동기로 읽어 와서 JPEG 만들기는 다른 스레드 → Game 화면 프레임 저하 거의 없음.
// Play를 누르면 장면에 CarStateReceiver가 있을 때 자동으로 붙습니다.
public class TwinStream : MonoBehaviour
{
    public float interval = 0.1f;
    public int maxWidth = 960;
    [Range(10, 100)] public int quality = 70;

    string dir, path;
    RenderTexture full, small;
    volatile bool busy;
    float next, nextDirCheck;
    bool enabledByDir;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        if (FindAnyObjectByType<TwinStream>() != null) return;
        if (FindAnyObjectByType<CarStateReceiver>() == null) return;
        new GameObject("TwinStream").AddComponent<TwinStream>();
    }

    void Start()
    {
        dir = Environment.GetEnvironmentVariable("TWIN_LIVE_DIR");
        if (string.IsNullOrEmpty(dir)) dir = "/dev/shm/twin_live";
        path = Path.Combine(dir, "sim.jpg");
        StartCoroutine(Loop());
    }

    System.Collections.IEnumerator Loop()
    {
        var eof = new WaitForEndOfFrame();
        while (true)
        {
            yield return eof;
            if (Time.unscaledTime >= nextDirCheck)
            {
                nextDirCheck = Time.unscaledTime + 1f;
                enabledByDir = Directory.Exists(dir);
            }
            if (!enabledByDir || busy || Time.unscaledTime < next) continue;
            next = Time.unscaledTime + interval;
            Capture();
        }
    }

    void Capture()
    {
        int sw = Screen.width, sh = Screen.height;
        if (sw < 16 || sh < 16) return;
        if (full == null || full.width != sw || full.height != sh)
        {
            if (full != null) full.Release();
            full = new RenderTexture(sw, sh, 0, RenderTextureFormat.ARGB32);
        }
        int w = Mathf.Min(maxWidth, sw);
        int h = Mathf.Max(2, Mathf.RoundToInt(sh * (float)w / sw / 2f) * 2);
        if (small == null || small.width != w || small.height != h)
        {
            if (small != null) small.Release();
            small = new RenderTexture(w, h, 0, RenderTextureFormat.ARGB32);
        }
        ScreenCapture.CaptureScreenshotIntoRenderTexture(full);
        // Vulkan·Metal·D3D(UV 원점 위)에서는 화면 캡처가 위아래 뒤집혀 들어옴 → 줄일 때 바로잡기
        if (SystemInfo.graphicsUVStartsAtTop) Graphics.Blit(full, small, new Vector2(1, -1), new Vector2(0, 1));
        else Graphics.Blit(full, small);

        busy = true;
        AsyncGPUReadback.Request(small, 0, TextureFormat.RGBA32, req =>
        {
            if (req.hasError) { busy = false; return; }
            byte[] px = req.GetData<byte>().ToArray();
            string dst = path;
            int q = quality;
            Task.Run(() =>
            {
                try
                {
                    byte[] jpg = ImageConversion.EncodeArrayToJPG(px, GraphicsFormat.R8G8B8A8_SRGB, (uint)w, (uint)h, 0, q);
                    string tmp = dst + ".tmp";
                    File.WriteAllBytes(tmp, jpg);
                    if (File.Exists(dst)) File.Replace(tmp, dst, null); else File.Move(tmp, dst);   // 원자적 교체
                }
                catch (Exception) { }
                finally { busy = false; }
            });
        });
    }

    void OnDestroy()
    {
        if (full != null) full.Release();
        if (small != null) small.Release();
    }
}
