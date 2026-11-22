using UnityEngine;

// [twin] 실차 연동 수동 주행용 키 입력: Game 창에서 누른 키를 MATLAB(twin_drive.m)으로 보냅니다.
// MATLAB은 이 키로 시뮬레이션 차를 움직이고, 같은 키를 실차 주행 코드(drive.sh --twin)에도 그대로 보냅니다.
// 실차 터미널 조작(~/demo/drive.sh)과 같은 키·같은 동작:
//   W : 속도 +step (PWM)      S : 속도 -step (멈춘 뒤 계속 누르면 후진)
//   A : 왼쪽 조향 펄스        D : 오른쪽 조향 펄스   (실차: 한 번에 120 ms 조향 모터, 손을 떼도 그 각도 유지)
//   Space : 정지 (속도 0 + 조향 모터 멈춤)          Q : 종료 (MATLAB이 기록 저장 + 이미지 저장)
// 키를 누르고 있으면 터미널의 키 반복처럼 repeatDelay 뒤 repeatInterval마다 다시 보냅니다
// (실차 터미널에서 키를 누르고 있을 때와 같은 동작이 되도록).
// 보내는 줄: "KEY,w" / "KEY,s" / "KEY,a" / "KEY,d" / "KEY,space" / "KEY,q"
// Play를 누르면 장면에 CarStateReceiver가 있을 때 자동으로 붙습니다 (장면에 따로 추가할 필요 없음).
public class TwinInput : MonoBehaviour
{
    [Tooltip("비워두면 장면에서 CarStateReceiver를 찾습니다.")]
    public CarStateReceiver receiver;

    [Tooltip("키를 누르고 있을 때 반복을 시작하기까지 [s] (Ubuntu 기본 키 반복과 비슷하게)")]
    public float repeatDelay = 0.5f;
    [Tooltip("반복 간격 [s] (Ubuntu 기본 약 30 Hz)")]
    public float repeatInterval = 0.033f;
    public bool showHelp = true;

    enum K { W, S, A, D, Space, Q }
    static readonly K[] Keys = { K.W, K.S, K.A, K.D, K.Space, K.Q };
    static readonly string[] Names = { "w", "s", "a", "d", "space", "q" };

    readonly float[] nextRepeat = new float[6];
    string lastSent = "";
    int sentCount;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        if (FindAnyObjectByType<TwinInput>() != null) return;
        var r = FindAnyObjectByType<CarStateReceiver>();
        if (r == null) return;
        new GameObject("TwinInput").AddComponent<TwinInput>().receiver = r;
    }

    void Start()
    {
        if (receiver == null) receiver = FindAnyObjectByType<CarStateReceiver>();
    }

    void Update()
    {
        if (receiver == null) return;
        float now = Time.unscaledTime;
        for (int i = 0; i < Keys.Length; i++)
        {
            if (Pressed(Keys[i]))
            {
                Send(Names[i]);
                nextRepeat[i] = now + repeatDelay;
            }
            else if (Keys[i] != K.Q && Held(Keys[i]) && now >= nextRepeat[i])
            {
                Send(Names[i]);
                nextRepeat[i] = now + repeatInterval;
            }
        }
    }

    void Send(string name)
    {
        if (receiver.SendLine("KEY," + name)) { lastSent = name; sentCount++; }
    }

    void OnGUI()
    {
        if (!showHelp) return;
        string state = receiver != null && receiver.connected ? $"MATLAB connected, last key: {lastSent} ({sentCount})" : "MATLAB not connected";
        GUI.Label(new Rect(10, Screen.height - 70, 900, 22),
            "Twin drive (real car follows):  [W/S] speed +/-   [A/D] steer pulse   [Space] stop   [Q] quit + save");
        GUI.Label(new Rect(10, Screen.height - 92, 900, 22), "  (" + state + ")");
        if (!string.IsNullOrEmpty(TwinSnapshot.Hud))
        {
            var big = new GUIStyle(GUI.skin.label) { fontSize = 22, fontStyle = FontStyle.Bold };
            big.normal.textColor = Color.yellow;
            GUI.Label(new Rect(10, Screen.height - 128, 900, 32), TwinSnapshot.Hud, big);
        }
    }

    // ---------- 키 입력 (구/신 입력 시스템 모두 지원, StepInput과 같은 방식) ----------
    bool Pressed(K key)
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKeyDown(Code(key));
#elif ENABLE_INPUT_SYSTEM
        var k = Control(key);
        return k != null && k.wasPressedThisFrame;
#else
        return false;
#endif
    }

    bool Held(K key)
    {
#if ENABLE_LEGACY_INPUT_MANAGER
        return Input.GetKey(Code(key));
#elif ENABLE_INPUT_SYSTEM
        var k = Control(key);
        return k != null && k.isPressed;
#else
        return false;
#endif
    }

#if ENABLE_LEGACY_INPUT_MANAGER
    static KeyCode Code(K key)
    {
        switch (key)
        {
            case K.W: return KeyCode.W;
            case K.S: return KeyCode.S;
            case K.A: return KeyCode.A;
            case K.D: return KeyCode.D;
            case K.Space: return KeyCode.Space;
            default: return KeyCode.Q;
        }
    }
#elif ENABLE_INPUT_SYSTEM
    static UnityEngine.InputSystem.Controls.KeyControl Control(K key)
    {
        var kb = UnityEngine.InputSystem.Keyboard.current;
        if (kb == null) return null;
        switch (key)
        {
            case K.W: return kb.wKey;
            case K.S: return kb.sKey;
            case K.A: return kb.aKey;
            case K.D: return kb.dKey;
            case K.Space: return kb.spaceKey;
            default: return kb.qKey;
        }
    }
#endif
}
