#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(AIRaceManagerPresetApplier))]
public class AIRaceManagerPresetEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        
        AIRaceManagerPresetApplier presetApplier = (AIRaceManagerPresetApplier)target;
        
        if (presetApplier.targetRaceManager == null)
        {
            EditorGUILayout.HelpBox("Please assign a Target Race Manager", MessageType.Warning);
            return;
        }
        
        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("Quick Preset Application", EditorStyles.boldLabel);
        
        GUILayout.BeginHorizontal();
        
        if (GUILayout.Button("Competitive\n(Recommended)", GUILayout.Height(40)))
        {
            AIRaceManagerPresets.ApplyThreeRacerPreset(presetApplier.targetRaceManager);
            EditorUtility.SetDirty(presetApplier.targetRaceManager);
        }
        
        if (GUILayout.Button("Casual\n(Easy)", GUILayout.Height(40)))
        {
            AIRaceManagerPresets.ApplyThreeRacerCasualPreset(presetApplier.targetRaceManager);
            EditorUtility.SetDirty(presetApplier.targetRaceManager);
        }
        
        if (GUILayout.Button("Hardcore\n(Challenging)", GUILayout.Height(40)))
        {
            AIRaceManagerPresets.ApplyThreeRacerHardcorePreset(presetApplier.targetRaceManager);
            EditorUtility.SetDirty(presetApplier.targetRaceManager);
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