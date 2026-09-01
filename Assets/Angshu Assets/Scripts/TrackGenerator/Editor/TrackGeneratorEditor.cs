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

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        _trackGenerator = (TrackGenerator)target;

        EditorGUILayout.Space();

        if (GUILayout.Button(new GUIContent("Generate", "Generate track")))
        {
            if (Application.isPlaying)
                _trackGenerator.Generate();
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

        if (GUILayout.Button(new GUIContent("Generate Walls", "Rebuild only the edge walls")))
        {
            if (Application.isPlaying)
                _trackGenerator.GenerateWalls();
            else
                Debug.LogWarning("Generate in Play mode!");
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
        string wallMeshName = $"{baseName}_WallMesh";
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

        // Save the wall mesh too, otherwise the prefab would keep a runtime-only mesh reference
        Mesh loadedWallMesh = null;
        Transform wallSource = trackObject.transform.Find(TrackGenerator.WallsObjectName);
        if (wallSource != null && wallSource.TryGetComponent(out MeshFilter wallFilter) && wallFilter.sharedMesh != null)
        {
            Mesh wallMesh = Instantiate(wallFilter.sharedMesh);
            wallMesh.name = wallMeshName;
            string wallMeshPath = System.IO.Path.Combine("Assets/Meshes", $"{wallMeshName}.asset");
            AssetDatabase.CreateAsset(wallMesh, wallMeshPath);
            loadedWallMesh = AssetDatabase.LoadAssetAtPath<Mesh>(wallMeshPath);
        }

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

        // Point the prefab's walls at the saved wall mesh
        if (loadedWallMesh != null)
        {
            Transform wallInstance = prefabInstance.transform.Find(TrackGenerator.WallsObjectName);
            if (wallInstance != null)
            {
                if (wallInstance.TryGetComponent<MeshFilter>(out var instanceWallFilter))
                    instanceWallFilter.sharedMesh = loadedWallMesh;

                if (wallInstance.TryGetComponent<MeshCollider>(out var instanceWallCollider))
                    instanceWallCollider.sharedMesh = loadedWallMesh;
            }
        }

        // Save as prefab
        string prefabPath = System.IO.Path.Combine("Assets/Prefabs/Tracks", $"{prefabName}.prefab");
        PrefabUtility.SaveAsPrefabAsset(prefabInstance, prefabPath);

        // Finalize and cleanup
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        DestroyImmediate(prefabInstance);

        string wallReport = loadedWallMesh != null ? $"\n- Wall Mesh: {AssetDatabase.GetAssetPath(loadedWallMesh)}" : string.Empty;
        Debug.Log($"Saved track assets:\n- Prefab: {prefabPath}\n- Mesh: {visualMeshPath}{wallReport}");
    }
}