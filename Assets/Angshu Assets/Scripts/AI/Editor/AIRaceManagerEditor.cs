#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(AIRaceManager))]
public class AIRaceManagerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        
        AIRaceManager raceManager = (AIRaceManager)target;
        
        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("Quick Preset Application", EditorStyles.boldLabel);
        
        GUILayout.BeginHorizontal();
        
        if (GUILayout.Button("Competitive\n(Recommended)", GUILayout.Height(40)))
        {
            AIRaceManagerPresets.ApplyThreeRacerPreset(raceManager);
            EditorUtility.SetDirty(raceManager);
        }
        
        if (GUILayout.Button("Casual\n(Easy)", GUILayout.Height(40)))
        {
            AIRaceManagerPresets.ApplyThreeRacerCasualPreset(raceManager);
            EditorUtility.SetDirty(raceManager);
        }
        
        if (GUILayout.Button("Hardcore\n(Challenging)", GUILayout.Height(40)))
        {
            AIRaceManagerPresets.ApplyThreeRacerHardcorePreset(raceManager);
            EditorUtility.SetDirty(raceManager);
        }
        
        GUILayout.EndHorizontal();
        
        EditorGUILayout.Space(5);
        EditorGUILayout.HelpBox(
            "Competitive: Balanced, exciting races with strong rubber banding\n" +
            "Casual: Easier AI, more forgiving for new players\n" +
            "Hardcore: Very skilled AI, minimal rubber banding", 
            MessageType.Info);
    }
}
#endif
