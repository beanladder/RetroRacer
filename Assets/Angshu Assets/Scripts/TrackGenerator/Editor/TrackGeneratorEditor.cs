using Track;
using UnityEditor;
using UnityEngine;
using UnityEngine.Splines;
using System.IO;
using System.Linq;

[CustomEditor(typeof(TrackGenerator), true)]
public class TrackGeneratorEditor : UnityEditor.Editor
{
    private TrackGenerator _trackGenerator;
    private TrackTerrainIntegrator _terrainIntegrator;

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        _trackGenerator = (TrackGenerator)target;
        _terrainIntegrator = _trackGenerator.GetComponent<TrackTerrainIntegrator>();

        EditorGUILayout.Space();

        if (GUILayout.Button(new GUIContent("Generate", "Generate track with procedural desert terrain")))
        {
            if (Application.isPlaying)
            {
                // Check if terrain integrator exists
                if (_terrainIntegrator == null)
                {
                    EditorGUILayout.HelpBox("Add TrackTerrainIntegrator component to auto-generate terrain!", MessageType.Info);
                }
                _trackGenerator.Generate();
            }
            else
                Debug.LogWarning("Generate in Play mode!");
        }

        if (GUILayout.Button(new GUIContent("Generate Mesh", "Generate the track mesh")))
        {
            if (Application.isPlaying)
                _trackGenerator.StartCoroutine(_trackGenerator.GenerateMesh());
            else
                Debug.LogWarning("Generate in Play mode!");
        }

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Terrain Setup", EditorStyles.boldLabel);
        
        if (_terrainIntegrator == null)
        {
            EditorGUILayout.HelpBox("Add TrackTerrainIntegrator to auto-generate desert terrain when clicking Generate.", MessageType.Info);
            if (GUILayout.Button(new GUIContent("Add Terrain Integrator", "Enable automatic desert generation")))
            {
                _trackGenerator.gameObject.AddComponent<TrackTerrainIntegrator>();
            }
        }
        else
        {
            EditorGUILayout.HelpBox("Terrain will be auto-generated when you click Generate above.", MessageType.Info);
        }

        EditorGUILayout.Space();

        if (GUILayout.Button(new GUIContent("Save as Prefab", "Save generated track as prefab")))
        {
            SaveTrackAsPrefab(_trackGenerator.gameObject);
        }
    }

    private int GetNextSeedNumber()
    {
        int maxSeed = 0;

        // Check existing prefabs
        string[] prefabGuids = AssetDatabase.FindAssets("t:Prefab", new[] { "Assets/Prefabs/Tracks" });
        foreach (string guid in prefabGuids)
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            string fileName = System.IO.Path.GetFileNameWithoutExtension(path);
            string[] parts = fileName.Split('_');
            if (parts.Length > 0 && parts[0].StartsWith("TrackSeed"))
            {
                string seedStr = parts[0].Substring("TrackSeed".Length);
                if (int.TryParse(seedStr, out int seed) && seed > maxSeed)
                    maxSeed = seed;
            }
        }

        // Check existing meshes
        string[] meshGuids = AssetDatabase.FindAssets("t:Mesh", new[] { "Assets/Meshes" });
        foreach (string guid in meshGuids)
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            string fileName = System.IO.Path.GetFileNameWithoutExtension(path);
            if (fileName.EndsWith("_VisualMesh") && fileName.StartsWith("TrackSeed"))
            {
                string basePart = fileName.Substring(0, fileName.Length - "_VisualMesh".Length);
                string seedStr = basePart.Substring("TrackSeed".Length);
                if (int.TryParse(seedStr, out int seed) && seed > maxSeed)
                    maxSeed = seed;
            }
        }

        return maxSeed + 1;
    }

    private void SaveTrackAsPrefab(GameObject trackObject)
    {
        if (!Application.isPlaying)
        {
            Debug.LogWarning("You need to be in Play Mode to save!");
            return;
        }

        // Create required directories
        Directory.CreateDirectory("Assets/Meshes");
        Directory.CreateDirectory("Assets/Prefabs/Tracks");

        // Generate unique seed-based names
        int seedNumber = GetNextSeedNumber();
        string baseName = $"TrackSeed{seedNumber}";
        string visualMeshName = $"{baseName}_VisualMesh";
        string prefabName = $"{baseName}_Track";

        // Save the visual mesh
        MeshFilter meshFilter = trackObject.GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null)
        {
            Debug.LogError("No mesh to save!");
            return;
        }

        Mesh visualMesh = Instantiate(meshFilter.sharedMesh);
        visualMesh.name = visualMeshName;
        string visualMeshPath = System.IO.Path.Combine("Assets/Meshes", $"{visualMeshName}.asset");
        AssetDatabase.CreateAsset(visualMesh, visualMeshPath);

        // Handle spline data
        SplineContainer splineContainer = trackObject.GetComponent<SplineContainer>();
        Spline savedSpline = new Spline(splineContainer.Spline);
        splineContainer.Spline = savedSpline;

        // Create prefab instance with correct naming
        GameObject prefabInstance = Instantiate(trackObject);
        prefabInstance.name = baseName;

        // Assign the saved mesh to prefab components
        Mesh loadedVisualMesh = AssetDatabase.LoadAssetAtPath<Mesh>(visualMeshPath);
        MeshFilter instanceFilter = prefabInstance.GetComponent<MeshFilter>();
        instanceFilter.sharedMesh = loadedVisualMesh;

        // Handle collider if present
        if (prefabInstance.TryGetComponent<MeshCollider>(out var prefabCollider))
        {
            prefabCollider.sharedMesh = loadedVisualMesh;
        }

        // Save as prefab
        string prefabPath = System.IO.Path.Combine("Assets/Prefabs/Tracks", $"{prefabName}.prefab");
        PrefabUtility.SaveAsPrefabAsset(prefabInstance, prefabPath);

        // Finalize and cleanup
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        DestroyImmediate(prefabInstance);

        Debug.Log($"Saved track assets:\n- Prefab: {prefabPath}\n- Mesh: {visualMeshPath}");
    }
}