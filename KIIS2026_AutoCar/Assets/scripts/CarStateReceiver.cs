using System;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

// MATLAB이 TCP로 보내는 한 줄(t,x,y,yaw,v,delta,yawrate,e_lat,e_head)을 받아
// 이 오브젝트(car)의 위치와 방향에 적용합니다.
// 붙일 곳: Hierarchy의 car (Rigidbody가 있는 부모 오브젝트)
//
// pure_pursuit_control.m용 추가 (manual_drive.m은 그대로 동작):
//   - 9개 뒤에 붙는 열(x_w,y_w,x_d,y_d,x_l,y_l,x_c,y_c,R,Lf,tau,delta_cmd)도 fields에 보관
//   - "PATH,n,x1,y1,..." 줄: 레퍼런스 경로 → path (pathVersion 증가)
//   - "RESET" 줄: 궤적 지우기 신호 → resetCount 증가
//   표시는 PathOverlay가 이 값들을 읽어서 합니다.
// discrete_path_drive.m용 추가: SendLine()으로 같은 연결을 통해 MATLAB에 한 줄을 보냄 (StepInput의 키 입력)
public class CarStateReceiver : MonoBehaviour
{
    [Header("통신 설정")]
    public string ip = "127.0.0.1";
    public int port = 55001;

    [Header("좌표 설정")]
    [Tooltip("MATLAB (0,0)이 될 위치. Plane을 넣으면 Plane 가운데가 원점이 됩니다.")]
    public Transform mapOrigin;

    [Tooltip("차가 옆이나 뒤를 보고 달리면 90, -90, 180 중에서 골라 보정합니다.")]
    public float yawOffsetDeg = 0f;

    [Tooltip("MATLAB 전송 주기가 느려서 화면이 끊겨 보일 때 켭니다.")]
    public bool smooth = false;
    [Range(1f, 50f)] public float smoothSpeed = 20f;

    [Header("앞바퀴 조향 표시 (선택, 비워둬도 됨)")]
    public Transform frontLeftWheel;
    public Transform frontRightWheel;

    [Header("수신 상태 (확인용, 수정하지 마세요)")]
    public bool connected;
    public int receivedCount;
    public float t, x, y, yaw, v, delta, yawRate, eLat, eHead;

    [Header("Pure pursuit 표시용 (확인용, 수정하지 마세요)")]
    public float[] fields = new float[0];      // 마지막으로 받은 한 줄 전체 (앞 9개 + 추가 열)
    public Vector2[] path = new Vector2[0];    // 레퍼런스 경로 (MATLAB x, y)
    public int pathVersion;                    // PATH를 받을 때마다 +1
    public int resetCount;                     // RESET을 받을 때마다 +1

    // ---- 아래는 내부 동작용 ----
    const int FieldCount = 9;

    readonly object lockObj = new object();
    readonly object sendLock = new object();
    float[] latest;
    bool hasNew;
    int threadCount;
    Vector2[] pendingPath;
    int pendingResets;

    volatile bool running;
    volatile bool clientConnected;
    TcpListener listener;
    TcpClient client;
    volatile NetworkStream stream;
    Thread thread;

    float baseHeight;
    Quaternion flBase, frBase;

    void Start()
    {
        Application.runInBackground = true; // MATLAB 창을 보고 있어도 Unity가 계속 갱신되도록

        var rb = GetComponent<Rigidbody>();
        if (rb != null) rb.isKinematic = true; // Unity 물리 대신 MATLAB 값으로만 움직임

        baseHeight = transform.position.y;
        if (frontLeftWheel) flBase = frontLeftWheel.localRotation;
        if (frontRightWheel) frBase = frontRightWheel.localRotation;

        running = true;
        thread = new Thread(ReceiveLoop) { IsBackground = true };
        thread.Start();
    }

    // 별도 스레드: 연결을 기다리고, 한 줄씩 읽어서 최신 값으로 저장
    void ReceiveLoop()
    {
        try
        {
            listener = new TcpListener(IPAddress.Parse(ip), port);
            listener.Server.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
            listener.Start();
            Debug.Log($"[CarStateReceiver] {ip}:{port} 에서 MATLAB 연결을 기다리는 중");
        }
        catch (Exception e)
        {
            Debug.LogError($"[CarStateReceiver] 포트를 열 수 없습니다: {e.Message}");
            return;
        }

        while (running)
        {
            try
            {
                client = listener.AcceptTcpClient();
                stream = client.GetStream();
                clientConnected = true;
                Debug.Log("[CarStateReceiver] MATLAB 연결됨");

                using (var reader = new StreamReader(stream, Encoding.ASCII))
                {
                    string line;
                    while (running && (line = reader.ReadLine()) != null)
                    {
                        if (line.StartsWith("PATH"))
                        {
                            Vector2[] p = ParsePath(line);
                            if (p != null) lock (lockObj) { pendingPath = p; }
                            continue;
                        }
                        if (line.StartsWith("RESET"))
                        {
                            lock (lockObj) { pendingResets++; }
                            continue;
                        }

                        float[] values = Parse(line);
                        if (values == null) continue;

                        lock (lockObj)
                        {
                            latest = values;
                            hasNew = true;
                            threadCount++;
                        }
                    }
                }
            }
            catch (Exception)
            {
                if (!running) break;
                Thread.Sleep(100);
            }
            finally
            {
                clientConnected = false;
                stream = null;
                try { client?.Close(); } catch { }
                client = null;
                if (running) Debug.Log("[CarStateReceiver] MATLAB 연결 끊김. 다시 기다리는 중");
            }
        }
    }

    // "t,x,y,..." 한 줄을 숫자 배열로 변환. 앞 9개 형식이 틀리면 null
    // 9개 뒤의 열은 그대로 보관 (읽을 수 없는 값은 NaN)
    float[] Parse(string line)
    {
        string[] parts = line.Trim().Split(',');
        if (parts.Length < FieldCount) return null;

        var vals = new float[parts.Length];
        for (int i = 0; i < parts.Length; i++)
        {
            if (!float.TryParse(parts[i], NumberStyles.Float, CultureInfo.InvariantCulture, out vals[i]))
            {
                if (i < FieldCount) return null;
                vals[i] = float.NaN;
            }
        }
        return vals;
    }

    // "PATH,n,x1,y1,x2,y2,..." 를 점 배열로 변환. 형식이 틀리면 null
    Vector2[] ParsePath(string line)
    {
        string[] parts = line.Trim().Split(',');
        if (parts.Length < 2 || !int.TryParse(parts[1], out int n) || parts.Length < 2 + 2 * n) return null;

        var pts = new Vector2[n];
        for (int i = 0; i < n; i++)
        {
            if (!float.TryParse(parts[2 + 2 * i], NumberStyles.Float, CultureInfo.InvariantCulture, out float px) ||
                !float.TryParse(parts[3 + 2 * i], NumberStyles.Float, CultureInfo.InvariantCulture, out float py))
                return null;
            pts[i] = new Vector2(px, py);
        }
        return pts;
    }

    // MATLAB에 한 줄 보내기 (메인 스레드에서 호출). MATLAB이 연결돼 있지 않으면 false
    // 받는 스레드가 같은 연결에서 읽는 중이어도, 읽기와 쓰기는 동시에 해도 됨
    public bool SendLine(string line)
    {
        NetworkStream s = stream;
        if (!clientConnected || s == null) return false;
        try
        {
            byte[] bytes = Encoding.ASCII.GetBytes(line + "\n");
            lock (sendLock) { s.Write(bytes, 0, bytes.Length); }
            return true;
        }
        catch (Exception)
        {
            return false;
        }
    }

    // MATLAB 좌표 (x, y) → Unity 월드 좌표 (바닥에서 height 위)
    public Vector3 MapToWorld(float mx, float my, float height)
    {
        Vector3 origin = mapOrigin ? mapOrigin.position : Vector3.zero;
        return new Vector3(origin.x + mx, origin.y + height, origin.z + my);
    }

    // 메인 스레드: 최신 값을 차에 적용
    void Update()
    {
        float[] s = null;
        lock (lockObj)
        {
            if (hasNew) { s = latest; hasNew = false; }
            receivedCount = threadCount;
            if (pendingPath != null) { path = pendingPath; pendingPath = null; pathVersion++; }
            resetCount += pendingResets;
            pendingResets = 0;
        }
        connected = clientConnected;

        if (s != null)
        {
            t = s[0]; x = s[1]; y = s[2]; yaw = s[3]; v = s[4];
            delta = s[5]; yawRate = s[6]; eLat = s[7]; eHead = s[8];
            fields = s;
        }
        if (receivedCount == 0) return; // 아직 받은 값이 없으면 제자리

        // 좌표 변환: MATLAB x → Unity X, MATLAB y → Unity Z, 회전 Y = 90 - yaw(도)
        Vector3 origin = mapOrigin ? mapOrigin.position : Vector3.zero;
        Vector3 targetPos = new Vector3(origin.x + x, baseHeight, origin.z + y);
        Quaternion targetRot = Quaternion.Euler(0f, 90f - yaw * Mathf.Rad2Deg + yawOffsetDeg, 0f);

        if (smooth)
        {
            float k = 1f - Mathf.Exp(-smoothSpeed * Time.deltaTime);
            transform.SetPositionAndRotation(
                Vector3.Lerp(transform.position, targetPos, k),
                Quaternion.Slerp(transform.rotation, targetRot, k));
        }
        else
        {
            transform.SetPositionAndRotation(targetPos, targetRot);
        }

        // 앞바퀴 조향: MATLAB은 왼쪽이 +, Unity의 Y축 회전은 오른쪽이 +라서 부호 반대
        float steerDeg = -delta * Mathf.Rad2Deg;
        SteerWheel(frontLeftWheel, flBase, steerDeg);
        SteerWheel(frontRightWheel, frBase, steerDeg);
    }

    // 차의 위쪽 방향을 축으로 바퀴를 돌림 (모델마다 다른 바퀴 축 방향에 상관없이 동작)
    void SteerWheel(Transform wheel, Quaternion baseRot, float steerDeg)
    {
        if (!wheel) return;
        Vector3 upInParent = wheel.parent
            ? wheel.parent.InverseTransformDirection(transform.up)
            : Vector3.up;
        wheel.localRotation = Quaternion.AngleAxis(steerDeg, upInParent) * baseRot;
    }

    // Play를 멈출 때 포트를 닫아서, 다음 Play에서 "포트 사용 중" 오류가 나지 않게 함
    void OnDisable()
    {
        if (!running) return;
        running = false;
        try { listener?.Stop(); } catch { }
        try { client?.Close(); } catch { }
        if (thread != null && thread.IsAlive) thread.Join(500);
    }
}
