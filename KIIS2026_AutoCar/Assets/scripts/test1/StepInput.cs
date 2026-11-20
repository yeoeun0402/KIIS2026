using UnityEngine;

// 이산 경로 주행(MATLAB/test1/discrete_path_drive.m)용 키 입력: 누른 키를 MATLAB으로 보냅니다 (MATLAB이 k → k+1 계산).
// Play를 누르면 장면에 CarStateReceiver가 있을 때 자동으로 붙습니다 (장면에 따로 추가할 필요 없음).
// Game 창을 클릭한 뒤:
//   Space / → : 한 스텝 (누르고 있으면 repeatInterval마다 계속)   → MATLAB에 "STEP,↑,↓"
//   ↑ / ↓     : 스텝 키와 함께 누르면 그 스텝의 가속 / 감속 (키보드 버전)
//   P         : 자동 진행 / 일시정지 (자동 버전)                   → "PLAY"
//   Backspace : 처음으로 (k = 0)                                   → "RESET"
//   Q         : 종료 (MATLAB이 기록 저장)                          → "QUIT"
// MATLAB이 연결되어 있지 않으면 아무 일도 하지 않습니다 (pure pursuit·PID 실행 중에는 MATLAB이 읽지 않음).
public class StepInput : MonoBehaviour
{
    [Tooltip("비워두면 장면에서 CarStateReceiver를 찾습니다.")]
    public CarStateReceiver receiver;

    [Tooltip("스텝 키를 누르고 있을 때 반복 간격 [s]")]
    public float repeatInterval = 0.1f;
    [Tooltip("스텝 키를 누른 뒤 반복을 시작하기까지 기다리는 시간 [s]")]
    public float repeatDelay = 0.3f;
    public bool showHelp = true;

    enum K { Space, Right, Up, Down, P, Backspace, Q }

    float nextRepeat;
    string lastSent = "";

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Install()
    {
        if (FindAnyObjectByType<StepInput>() != null) return;
        var r = FindAnyObjectByType<CarStateReceiver>();
        if (r == null) return;
        new GameObject("StepInput").AddComponent<StepInput>().receiver = r;
    }

    void Start()
    {
        if (receiver == null) receiver = FindAnyObjectByType<CarStateReceiver>();
    }

    void Update()
    {
        if (receiver == null) return;

        bool stepHeld = Held(K.Space) || Held(K.Right);
        if (Pressed(K.Space) || Pressed(K.Right))
        {
            SendStep();
            nextRepeat = Time.unscaledTime + repeatDelay;
        }
        else if (stepHeld && Time.unscaledTime >= nextRepeat)
        {
            SendStep();
            nextRepeat = Time.unscaledTime + repeatInterval;
        }

        if (Pressed(K.P)) Send("PLAY");
        if (Pressed(K.Backspace)) Send("RESET");
        if (Pressed(K.Q)) Send("QUIT");
    }

    void SendStep() => Send($"STEP,{(Held(K.Up) ? 1 : 0)},{(Held(K.Down) ? 1 : 0)}");

    void Send(string line)
    {
        if (receiver.SendLine(line)) lastSent = line;
    }

    // 화면 왼쪽 아래 키 안내
    void OnGUI()
    {
        if (!showHelp) return;
        string state = receiver != null && receiver.connected ? $"connected, last: {lastSent}" : "MATLAB not connected";
        GUI.Label(new Rect(10, Screen.height - 48, 900, 22),
            "Discrete step:  [Space/Right] step   [Up/Down + step] accel/brake   [P] play/pause   [Backspace] reset   [Q] quit");
        GUI.Label(new Rect(10, Screen.height - 26, 900, 22), "  (" + state + ")");
    }

    // ---------- 키 입력 (구/신 입력 시스템 모두 지원) ----------
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
            case K.Space: return KeyCode.Space;
            case K.Right: return KeyCode.RightArrow;
            case K.Up: return KeyCode.UpArrow;
            case K.Down: return KeyCode.DownArrow;
            case K.P: return KeyCode.P;
            case K.Backspace: return KeyCode.Backspace;
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
            case K.Space: return kb.spaceKey;
            case K.Right: return kb.rightArrowKey;
            case K.Up: return kb.upArrowKey;
            case K.Down: return kb.downArrowKey;
            case K.P: return kb.pKey;
            case K.Backspace: return kb.backspaceKey;
            default: return kb.qKey;
        }
    }
#endif
}
