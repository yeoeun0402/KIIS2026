using System;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// twin.sh(한 번에 실행)가 Unity를 자동으로 Play 하게 하는 에디터 스크립트.
//
// 하는 일: 에디터가 켜져 있는 동안 0.5초마다 프로젝트 폴더의 twin_play.request 파일을 확인합니다.
//   파일이 생기면 지우고 → SampleScene을 열고 (다른 장면이 열려 있으면) → Play를 누릅니다.
//   컴파일·에셋 가져오기 중이면 끝날 때까지 기다렸다가 누릅니다. 이미 Play 중이면 파일만 지웁니다.
//   15분보다 오래된 요청 파일은 (지난 실행이 비정상 종료된 흔적) 무시하고 지웁니다.
// 손으로 Play 하는 것과 똑같으므로, 이 스크립트가 있어도 평소 사용에는 영향이 없습니다.
[InitializeOnLoad]
public static class TwinAutoPlay
{
    const string ScenePath = "Assets/Scenes/SampleScene.unity";
    static readonly string RequestFile =
        Path.Combine(Directory.GetParent(Application.dataPath).FullName, "twin_play.request");
    static double nextCheck;

    static TwinAutoPlay()
    {
        EditorApplication.update += Poll;
    }

    static void Poll()
    {
        if (EditorApplication.timeSinceStartup < nextCheck) return;
        nextCheck = EditorApplication.timeSinceStartup + 0.5;
        if (!File.Exists(RequestFile)) return;

        if (DateTime.Now - File.GetLastWriteTime(RequestFile) > TimeSpan.FromMinutes(15))
        {
            TryDelete();
            Debug.Log("[twin] 오래된 Play 요청을 무시했습니다.");
            return;
        }
        if (EditorApplication.isPlaying)
        {
            TryDelete();
            return;
        }
        if (EditorApplication.isCompiling || EditorApplication.isUpdating ||
            EditorApplication.isPlayingOrWillChangePlaymode) return;

        TryDelete();
        if (EditorSceneManager.GetActiveScene().path != ScenePath)
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            EditorSceneManager.OpenScene(ScenePath);
        }
        Debug.Log("[twin] twin.sh 요청으로 Play 합니다.");
        EditorApplication.isPlaying = true;
    }

    static void TryDelete()
    {
        try { File.Delete(RequestFile); } catch (Exception) { }
    }
}
