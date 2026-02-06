using UnityEngine;
using UnityEditor;
using System.IO;

/// <summary>
/// Custom inspector for AIPersonalityData to show personality description and provide quick randomization
/// </summary>
[CustomEditor(typeof(AIPersonalityData))]
public class AIPersonalityDataEditor : Editor
{
    private AIPersonalityData.PersonalityType previousPersonalityType;
    
    private void OnEnable()
    {
        AIPersonalityData personality = (AIPersonalityData)target;
        previousPersonalityType = personality.personalityType;
    }
    
    public override void OnInspectorGUI()
    {
        AIPersonalityData personality = (AIPersonalityData)target;
        
        // Show personality description in a help box
        EditorGUILayout.Space();
        EditorGUILayout.HelpBox(personality.GetPersonalityDescription(), MessageType.Info);
        EditorGUILayout.Space();
        
        // Check if personality type changed
        EditorGUI.BeginChangeCheck();
        
        // Draw default inspector
        DrawDefaultInspector();
        
        if (EditorGUI.EndChangeCheck())
        {
            // Check if personality type was changed
            if (previousPersonalityType != personality.personalityType)
            {
                // Ask user if they want to update values
                if (EditorUtility.DisplayDialog(
                    "Personality Type Changed",
                    $"You changed the personality type from {previousPersonalityType} to {personality.personalityType}.\n\n" +
                    "Do you want to update all values to match the new personality type?\n\n" +
                    "• Yes: Reset all values to defaults for this type\n" +
                    "• No: Keep current values",
                    "Yes, Update Values",
                    "No, Keep Current Values"))
                {
                    Undo.RecordObject(personality, "Update Personality Values");
                    personality.RandomizeForType(personality.personalityType);
                    EditorUtility.SetDirty(personality);
                }
                
                // Always rename the asset
                RenameAsset(personality);
                
                previousPersonalityType = personality.personalityType;
            }
        }
        
        EditorGUILayout.Space();
        
        // Add randomize button
        if (GUILayout.Button("Randomize Values for Current Type", GUILayout.Height(30)))
        {
            Undo.RecordObject(personality, "Randomize Personality");
            personality.RandomizeForType(personality.personalityType);
            EditorUtility.SetDirty(personality);
        }
        
        EditorGUILayout.Space();
        
        // Show summary of key traits
        EditorGUILayout.LabelField("Trait Summary", EditorStyles.boldLabel);
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        
        DrawTraitBar("Aggression", personality.aggression, Color.red);
        DrawTraitBar("Skill", personality.skill, Color.green);
        DrawTraitBar("Consistency", personality.consistency, Color.blue);
        DrawTraitBar("Risk Taking", personality.riskTaking, Color.yellow);
        DrawTraitBar("Patience", personality.patience, Color.cyan);
        
        EditorGUILayout.EndVertical();
    }
    
    private void RenameAsset(AIPersonalityData personality)
    {
        string assetPath = AssetDatabase.GetAssetPath(personality);
        if (string.IsNullOrEmpty(assetPath))
            return;
        
        string directory = Path.GetDirectoryName(assetPath);
        string newName = $"{personality.personalityType}_Personality.asset";
        string newPath = Path.Combine(directory, newName);
        
        // Check if a file with this name already exists
        if (File.Exists(newPath) && newPath != assetPath)
        {
            // Add a number suffix to make it unique
            int counter = 1;
            string baseName = $"{personality.personalityType}_Personality";
            while (File.Exists(newPath))
            {
                newName = $"{baseName}_{counter}.asset";
                newPath = Path.Combine(directory, newName);
                counter++;
            }
        }
        
        // Only rename if the path is different
        if (newPath != assetPath)
        {
            string error = AssetDatabase.RenameAsset(assetPath, Path.GetFileNameWithoutExtension(newName));
            if (string.IsNullOrEmpty(error))
            {
                AssetDatabase.SaveAssets();
                Debug.Log($"Renamed personality asset to: {newName}");
            }
            else
            {
                Debug.LogWarning($"Failed to rename asset: {error}");
            }
        }
    }
    
    private void DrawTraitBar(string label, float value, Color color)
    {
        EditorGUILayout.BeginHorizontal();
        EditorGUILayout.LabelField(label, GUILayout.Width(100));
        
        Rect rect = EditorGUILayout.GetControlRect(GUILayout.Height(18));
        EditorGUI.DrawRect(rect, new Color(0.2f, 0.2f, 0.2f));
        
        Rect fillRect = new Rect(rect.x, rect.y, rect.width * value, rect.height);
        EditorGUI.DrawRect(fillRect, color);
        
        EditorGUILayout.LabelField($"{(value * 100):F0}%", GUILayout.Width(40));
        EditorGUILayout.EndHorizontal();
    }
}
