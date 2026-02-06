using UnityEngine;
using UnityEditor;
using System.IO;

/// <summary>
/// Editor utility to quickly create preset AI personalities
/// </summary>
public class AIPersonalityCreator : EditorWindow
{
    [MenuItem("Tools/AI/Create Personality Presets")]
    public static void CreatePersonalityPresets()
    {
        string folderPath = "Assets/Angshu Assets/Personalities";
        
        // Create folder if it doesn't exist
        if (!AssetDatabase.IsValidFolder(folderPath))
        {
            string parentFolder = "Assets/Angshu Assets";
            if (!AssetDatabase.IsValidFolder(parentFolder))
            {
                AssetDatabase.CreateFolder("Assets", "Angshu Assets");
            }
            AssetDatabase.CreateFolder(parentFolder, "Personalities");
        }
        
        // Create one of each personality type
        foreach (AIPersonalityData.PersonalityType type in System.Enum.GetValues(typeof(AIPersonalityData.PersonalityType)))
        {
            string assetPath = $"{folderPath}/{type}_Personality.asset";
            
            // Skip if already exists
            if (File.Exists(assetPath))
            {
                Debug.Log($"Personality preset already exists: {assetPath}");
                continue;
            }
            
            // Create the personality
            var personality = ScriptableObject.CreateInstance<AIPersonalityData>();
            personality.personalityType = type;
            personality.RandomizeForType(type);
            
            // Save as asset
            AssetDatabase.CreateAsset(personality, assetPath);
            Debug.Log($"Created personality preset: {assetPath}");
        }
        
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        
        EditorUtility.DisplayDialog("Personality Presets Created", 
            $"Created {System.Enum.GetValues(typeof(AIPersonalityData.PersonalityType)).Length} personality presets in:\n{folderPath}", 
            "OK");
    }
    
    [MenuItem("Tools/AI/Create Custom Personality")]
    public static void CreateCustomPersonality()
    {
        string folderPath = "Assets/Angshu Assets/Personalities";
        
        // Create folder if it doesn't exist
        if (!AssetDatabase.IsValidFolder(folderPath))
        {
            string parentFolder = "Assets/Angshu Assets";
            if (!AssetDatabase.IsValidFolder(parentFolder))
            {
                AssetDatabase.CreateFolder("Assets", "Angshu Assets");
            }
            AssetDatabase.CreateFolder(parentFolder, "Personalities");
        }
        
        // Create a blank personality
        var personality = ScriptableObject.CreateInstance<AIPersonalityData>();
        
        // Save as asset
        string assetPath = AssetDatabase.GenerateUniqueAssetPath($"{folderPath}/Custom_Personality.asset");
        AssetDatabase.CreateAsset(personality, assetPath);
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        
        // Select and ping the new asset
        Selection.activeObject = personality;
        EditorGUIUtility.PingObject(personality);
        
        Debug.Log($"Created custom personality: {assetPath}");
    }
}
