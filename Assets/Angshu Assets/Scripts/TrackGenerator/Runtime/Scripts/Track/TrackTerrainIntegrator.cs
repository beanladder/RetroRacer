using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;

namespace Track
{
    /// <summary>
    /// Procedurally generates desert terrain and conforms track to it.
    /// </summary>
    [RequireComponent(typeof(TrackGenerator))]
    public class TrackTerrainIntegrator : MonoBehaviour
    {
        [Header("Terrain Reference")]
        [SerializeField] private Terrain _terrain;
        
        [Header("Terrain Creation")]
        [SerializeField, Tooltip("Padding around track (multiplier of track size)")]
        private float _terrainPaddingMultiplier = 2.5f;
        
        [SerializeField, Tooltip("Minimum terrain size")]
        private int _minTerrainSize = 1000;
        
        [SerializeField, Tooltip("Terrain heightmap resolution")]
        private int _terrainResolution = 513;
        
        [SerializeField, Tooltip("Auto-create terrain if missing")]
        private bool _autoCreateTerrain = true;
        
        [Header("Desert Generation")]
        [SerializeField] private int _seed = 0;
        [SerializeField] private bool _randomizeSeed = true;
        
        [SerializeField, Tooltip("Base height of terrain (0-1)")]
        private float _baseHeight = 0.3f;
        
        [SerializeField, Tooltip("Height of large dunes")]
        private float _duneHeight = 0.25f;
        
        [SerializeField, Tooltip("Scale of large dunes (lower = bigger dunes)")]
        private float _duneScale = 0.005f;
        
        [SerializeField, Tooltip("Height of small ripples")]
        private float _rippleHeight = 0.03f;
        
        [SerializeField, Tooltip("Scale of small ripples")]
        private float _rippleScale = 0.05f;
        
        [SerializeField, Tooltip("Number of random rock formations")]
        private int _rockFormationCount = 8;
        
        [SerializeField, Tooltip("Max height of rock formations")]
        private float _rockHeight = 0.15f;
        
        [Header("Road Settings")]
        [SerializeField, Tooltip("Height offset above terrain")]
        private float _roadHeightOffset = 0.5f;
        
        [SerializeField, Tooltip("Flatten terrain under road")]
        private bool _flattenUnderRoad = true;
        
        [SerializeField, Range(0f, 1f), Tooltip("How much to flatten (higher = flatter road)")]
        private float _flattenStrength = 0.7f;
        
        [SerializeField, Tooltip("Distance to blend flattening")]
        private float _flattenBlendDistance = 15f;
        
        [Header("Texture Painting")]
        [SerializeField] private int _roadTextureLayer = 1;
        [SerializeField] private int _sandTextureLayer = 0;
        [SerializeField] private float _textureExtraWidth = 3f;
        [SerializeField] private float _textureBlendWidth = 4f;
        
        private TrackGenerator _trackGenerator;
        private TerrainData _terrainData;
        private System.Random _random;
        
        public Terrain Terrain => _terrain;
        
        private void Awake()
        {
            _trackGenerator = GetComponent<TrackGenerator>();
        }
        
        public void GenerateDesertAndIntegrate()
        {
            if (_trackGenerator.Vertices == null || _trackGenerator.Vertices.Count < 3)
            {
                Debug.LogError("Generate track first!");
                return;
            }
            
            // Create terrain if needed
            if (_terrain == null && _autoCreateTerrain)
            {
                CreateTerrain();
            }
            
            if (_terrain == null)
            {
                Debug.LogError("TrackTerrainIntegrator: No terrain assigned!");
                return;
            }
            
            _terrainData = _terrain.terrainData;
            
            if (_randomizeSeed)
                _seed = UnityEngine.Random.Range(0, int.MaxValue);
            _random = new System.Random(_seed);
            
            StartCoroutine(GenerationRoutine());
        }
        
        private void CreateTerrain()
        {
            // Remove old terrain if exists
            Terrain oldTerrain = FindObjectOfType<Terrain>();
            if (oldTerrain != null)
            {
                if (Application.isPlaying)
                    Destroy(oldTerrain.gameObject);
                else
                    DestroyImmediate(oldTerrain.gameObject);
            }
            
            // Calculate terrain size based on track bounds
            var trackBounds = GetTrackBounds();
            float trackWidth = Mathf.Max(trackBounds.size.x, trackBounds.size.z);
            int terrainSize = Mathf.Max(_minTerrainSize, Mathf.CeilToInt(trackWidth * _terrainPaddingMultiplier));
            
            // Round to nearest power of 2 + 1 for better performance
            terrainSize = Mathf.NextPowerOfTwo(terrainSize);
            
            Debug.Log($"Track bounds: {trackBounds.size}, Creating terrain size: {terrainSize}x{terrainSize}");
            
            // Create terrain data
            TerrainData terrainData = new TerrainData();
            terrainData.heightmapResolution = _terrainResolution;
            terrainData.size = new Vector3(terrainSize, 100, terrainSize);
            terrainData.alphamapResolution = 512;
            
            // Create terrain GameObject
            GameObject terrainObj = Terrain.CreateTerrainGameObject(terrainData);
            terrainObj.name = "ProceduralTerrain";
            _terrain = terrainObj.GetComponent<Terrain>();
            
            // Position terrain centered on track
            Vector3 terrainPos = trackBounds.center - new Vector3(terrainSize * 0.5f, 0, terrainSize * 0.5f);
            terrainPos.y = trackBounds.min.y - 20f; // Below track
            _terrain.transform.position = terrainPos;
            
            Debug.Log($"Created terrain at {terrainPos} with size {terrainSize}x{terrainSize}");
        }
        
        private IEnumerator GenerationRoutine()
        {
            Debug.Log($"Generating flat terrain...");
            
            // Step 1: Generate flat terrain
            GenerateDesertTerrain();
            yield return new WaitForSeconds(0.2f);
            
            // Step 2: Conform track vertices to terrain
            ConformTrackToTerrain();
            yield return new WaitForSeconds(0.1f);
            
            // Step 3: Regenerate track mesh with new heights
            Debug.Log("Regenerating track spline and mesh...");
            yield return _trackGenerator.StartCoroutine(_trackGenerator.GenerateSpline());
            yield return _trackGenerator.StartCoroutine(_trackGenerator.GenerateMesh());
            
            // Step 4: Paint textures
            if (_terrainData.alphamapLayers > 1)
            {
                PaintRoadTexture();
            }
            else
            {
                Debug.LogWarning("Terrain needs at least 2 layers for road painting. Add terrain layers!");
            }
            
            Debug.Log("Terrain generation complete!");
        }

        
        private void GenerateDesertTerrain()
        {
            int resolution = _terrainData.heightmapResolution;
            float[,] heights = new float[resolution, resolution];
            
            // Completely flat terrain for now
            for (int z = 0; z < resolution; z++)
            {
                for (int x = 0; x < resolution; x++)
                {
                    heights[z, x] = _baseHeight;
                }
            }
            
            _terrainData.SetHeights(0, 0, heights);
            Debug.Log($"Generated flat terrain at height {_baseHeight}");
        }
        
        private Bounds GetTrackBounds()
        {
            var vertices = _trackGenerator.Vertices;
            if (vertices.Count == 0) return new Bounds();
            
            Bounds bounds = new Bounds(transform.TransformPoint(vertices[0]), Vector3.zero);
            foreach (var v in vertices)
                bounds.Encapsulate(transform.TransformPoint(v));
            return bounds;
        }
        
        private bool IsNearTrack(Vector2 normalizedPos, float threshold)
        {
            Vector3 terrainPos = _terrain.transform.position;
            Vector3 terrainSize = _terrainData.size;
            
            float worldX = terrainPos.x + normalizedPos.x * terrainSize.x;
            float worldZ = terrainPos.z + normalizedPos.y * terrainSize.z;
            
            foreach (var v in _trackGenerator.Vertices)
            {
                Vector3 worldV = transform.TransformPoint(v);
                float dist = Vector2.Distance(
                    new Vector2(worldX, worldZ),
                    new Vector2(worldV.x, worldV.z)
                );
                float normalizedDist = dist / Mathf.Max(terrainSize.x, terrainSize.z);
                if (normalizedDist < threshold)
                    return true;
            }
            return false;
        }

        
        private void FlattenUnderTrack()
        {
            int resolution = _terrainData.heightmapResolution;
            float[,] heights = _terrainData.GetHeights(0, 0, resolution, resolution);
            
            Vector3 terrainPos = _terrain.transform.position;
            Vector3 terrainSize = _terrainData.size;
            float trackWidth = _trackGenerator.Width;
            
            // Get spline points for accurate path following
            var spline = _trackGenerator.Spline;
            int sampleCount = 500;
            List<Vector3> splinePoints = new List<Vector3>();
            List<float> splineHeights = new List<float>();
            
            for (int i = 0; i < sampleCount; i++)
            {
                float t = i / (float)sampleCount;
                spline.Evaluate(t, out float3 pos, out float3 tangent, out float3 up);
                Vector3 worldPos = transform.TransformPoint(pos);
                splinePoints.Add(worldPos);
                
                // Sample current terrain height at this point
                int hx = Mathf.RoundToInt((worldPos.x - terrainPos.x) / terrainSize.x * (resolution - 1));
                int hz = Mathf.RoundToInt((worldPos.z - terrainPos.z) / terrainSize.z * (resolution - 1));
                hx = Mathf.Clamp(hx, 0, resolution - 1);
                hz = Mathf.Clamp(hz, 0, resolution - 1);
                splineHeights.Add(heights[hz, hx]);
            }
            
            // Smooth the spline heights to avoid bumpy roads
            float[] smoothedHeights = new float[sampleCount];
            int smoothWindow = 10;
            for (int i = 0; i < sampleCount; i++)
            {
                float sum = 0f;
                int count = 0;
                for (int j = -smoothWindow; j <= smoothWindow; j++)
                {
                    int idx = (i + j + sampleCount) % sampleCount;
                    sum += splineHeights[idx];
                    count++;
                }
                smoothedHeights[i] = sum / count;
            }
            
            // Apply flattening
            float totalWidth = trackWidth + _flattenBlendDistance * 2f;
            
            for (int z = 0; z < resolution; z++)
            {
                for (int x = 0; x < resolution; x++)
                {
                    float worldX = terrainPos.x + (x / (float)(resolution - 1)) * terrainSize.x;
                    float worldZ = terrainPos.z + (z / (float)(resolution - 1)) * terrainSize.z;
                    
                    // Find closest spline point
                    float minDist = float.MaxValue;
                    float targetHeight = heights[z, x];
                    
                    for (int i = 0; i < splinePoints.Count; i++)
                    {
                        float dist = Vector2.Distance(
                            new Vector2(worldX, worldZ),
                            new Vector2(splinePoints[i].x, splinePoints[i].z)
                        );
                        
                        if (dist < minDist)
                        {
                            minDist = dist;
                            targetHeight = smoothedHeights[i];
                        }
                    }
                    
                    if (minDist < totalWidth)
                    {
                        float blend = 1f;
                        if (minDist > trackWidth * 0.5f)
                        {
                            blend = 1f - ((minDist - trackWidth * 0.5f) / _flattenBlendDistance);
                            blend = Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(blend));
                        }
                        
                        heights[z, x] = Mathf.Lerp(heights[z, x], targetHeight, blend * _flattenStrength);
                    }
                }
            }
            
            _terrainData.SetHeights(0, 0, heights);
        }
        
        private void ConformTrackToTerrain()
        {
            var vertices = _trackGenerator.Vertices;
            Vector3 terrainPos = _terrain.transform.position;
            
            Debug.Log($"Conforming {vertices.Count} track vertices to terrain...");
            
            int conformedCount = 0;
            for (int i = 0; i < vertices.Count; i++)
            {
                Vector3 worldPos = transform.TransformPoint(vertices[i]);
                float terrainHeight = _terrain.SampleHeight(worldPos) + terrainPos.y;
                
                // Debug first few vertices
                if (i < 3)
                {
                    Debug.Log($"Vertex {i}: World XZ=({worldPos.x:F1}, {worldPos.z:F1}), " +
                             $"Old Y={worldPos.y:F2}, Terrain Y={terrainHeight:F2}, " +
                             $"New Y={terrainHeight + _roadHeightOffset:F2}");
                }
                
                worldPos.y = terrainHeight + _roadHeightOffset;
                vertices[i] = transform.InverseTransformPoint(worldPos);
                conformedCount++;
            }
            
            Debug.Log($"Conformed {conformedCount} vertices to terrain surface.");
        }

        
        private void PaintRoadTexture()
        {
            int resolution = _terrainData.alphamapResolution;
            int layers = _terrainData.alphamapLayers;
            float[,,] alphamaps = new float[resolution, resolution, layers];
            
            // Start with all sand
            for (int z = 0; z < resolution; z++)
            {
                for (int x = 0; x < resolution; x++)
                {
                    alphamaps[z, x, _sandTextureLayer] = 1f;
                }
            }
            
            Vector3 terrainPos = _terrain.transform.position;
            Vector3 terrainSize = _terrainData.size;
            
            var spline = _trackGenerator.Spline;
            float trackWidth = _trackGenerator.Width;
            float paintWidth = trackWidth + _textureExtraWidth;
            float totalWidth = paintWidth + _textureBlendWidth;
            
            // Sample spline
            int sampleCount = 500;
            List<Vector3> splinePoints = new List<Vector3>();
            
            for (int i = 0; i < sampleCount; i++)
            {
                float t = i / (float)sampleCount;
                spline.Evaluate(t, out float3 pos, out float3 tangent, out float3 up);
                splinePoints.Add(transform.TransformPoint(pos));
            }
            
            // Paint road
            for (int z = 0; z < resolution; z++)
            {
                for (int x = 0; x < resolution; x++)
                {
                    float worldX = terrainPos.x + (x / (float)(resolution - 1)) * terrainSize.x;
                    float worldZ = terrainPos.z + (z / (float)(resolution - 1)) * terrainSize.z;
                    
                    float minDist = float.MaxValue;
                    for (int i = 0; i < splinePoints.Count; i++)
                    {
                        float dist = Vector2.Distance(
                            new Vector2(worldX, worldZ),
                            new Vector2(splinePoints[i].x, splinePoints[i].z)
                        );
                        minDist = Mathf.Min(minDist, dist);
                    }
                    
                    float roadWeight = 0f;
                    if (minDist < paintWidth * 0.5f)
                    {
                        roadWeight = 1f;
                    }
                    else if (minDist < totalWidth * 0.5f)
                    {
                        roadWeight = 1f - ((minDist - paintWidth * 0.5f) / _textureBlendWidth);
                        roadWeight = Mathf.SmoothStep(0f, 1f, roadWeight);
                    }
                    
                    if (roadWeight > 0f)
                    {
                        alphamaps[z, x, _roadTextureLayer] = roadWeight;
                        alphamaps[z, x, _sandTextureLayer] = 1f - roadWeight;
                    }
                }
            }
            
            _terrainData.SetAlphamaps(0, 0, alphamaps);
        }
        
        // Keep for backwards compatibility
        public void IntegrateWithTrack() => GenerateDesertAndIntegrate();
        
        public void ResetTerrain()
        {
            Debug.Log("Use 'Generate Desert' to create new terrain.");
        }
    }
}
