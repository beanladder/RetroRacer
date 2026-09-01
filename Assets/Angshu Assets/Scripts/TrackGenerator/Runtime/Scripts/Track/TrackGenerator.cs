using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Splines;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Track
{
    [RequireComponent(typeof(SplineContainer), typeof(MeshFilter), typeof(MeshRenderer))]
    public abstract class TrackGenerator : MonoBehaviour
    {
        public const string WallsObjectName = "TrackWalls";

        [Header("Racing Line Configuration")]
        [SerializeField, Tooltip("Whether to generate a racing line for AI vehicles")]
        private bool generateRacingLine = true;
        
        [SerializeField, Range(0f, 1f), Tooltip("Controls how much the racing line deviates from the center on straights.")]
        private float minDeviation = 0.2f;
        
        [SerializeField, Range(0f, 2f), Tooltip("Controls how much the racing line deviates from the center in corners.")]
        private float maxDeviation = 0.9f;
        
        [SerializeField, Range(1f, 20f), Tooltip("The scale of the racing line's deviation. Lower values create smoother, longer waves.")]
        private float deviationScale = 8f;
        
        [SerializeField, Tooltip("Show racing line visualization in editor")]
        private bool showRacingLine = true;
        
        [SerializeField, Tooltip("Color of the racing line visualization")]
        private Color racingLineColor = Color.red;
        
        [SerializeField]
        private List<Vector3> _racingLinePoints = new List<Vector3>();
        [SerializeField]
        private List<float> _racingLineSpeeds = new List<float>();
        
        private RacingLine _racingLine = new RacingLine();
        public RacingLine RacingLine => _racingLine;
        // Add new serialized field for checkpoint prefab
        [Header("Checkpoint Configuration")]
        [SerializeField, Tooltip("Checkpoint prefab to place along the track")]
        private GameObject checkpointPrefab;
        
        [SerializeField, Tooltip("Start/Finish Line prefab to place at the straightest section of the track")]
        private GameObject startFinishLinePrefab;
        
        [SerializeField, Tooltip("Number of checkpoints to place along the track"), Range(4, 100)]
        private int numberOfCheckpoints = 10;
        
        [SerializeField, Tooltip("Vertical offset above the track surface, measured at the reference width. Scales with the track.")]
        private float checkpointVerticalOffset = 1f;

        [SerializeField, Tooltip("Scale the checkpoint and start/finish prefabs so they always span the track mesh.")]
        private bool scaleMarkersToTrackWidth = true;

        [SerializeField, Tooltip("The Width the checkpoint and start/finish prefabs were authored against. They are rescaled by Width / this value.")]
        private float markerReferenceWidth = 20f;

        public int NumberOfCheckpoints => numberOfCheckpoints;

        /// <summary>
        /// Uniform scale that maps prefabs authored for <see cref="markerReferenceWidth"/> onto the current track width.
        /// </summary>
        public float MarkerScale =>
            scaleMarkersToTrackWidth && markerReferenceWidth > 0.001f ? Width / markerReferenceWidth : 1f;

        [field: SerializeField, Tooltip("The resolution of the track. The number of segments that'll be used to generate the track mesh.")]
        public int Resolution { get; private set; } = 10000;

        [field: SerializeField, Tooltip("Half the width of the track: the distance from the centre line out to either edge of the mesh.")]
        public float Width { get; private set; } = 3f;

        [field: SerializeField, Tooltip("Add a Mesh Collider to the track.")]
        public bool GenerateCollider { get; private set; } = true;

        [Header("Terrain Integration")]
        [SerializeField, Tooltip("Reference to the terrain the track should follow")]
        private Terrain _terrain;

        [SerializeField, Tooltip("Vertical offset above terrain surface")]
        private float _terrainOffset = 0.1f;

        [Header("Track Verticality")]
        [SerializeField, Tooltip("Maximum height variation from base terrain")]
        private float _maxVerticality = 5f;

        [SerializeField, Tooltip("Scale of verticality noise")]
        private float _verticalityNoiseScale = 0.5f;

        [Header("Track Thickness")]
        [SerializeField, Tooltip("Vertical thickness of the track mesh.")]
        private float _thickness = 1.0f;

        [Header("Track Walls")]
        [SerializeField, Tooltip("Build solid barrier walls along both edges of the track mesh so cars cannot fall off.")]
        private bool generateWalls = true;

        [SerializeField, Tooltip("How tall the walls stand above the track surface.")]
        private float wallHeight = 6f;

        [SerializeField, Tooltip("How thick the walls are, measured outward from the track edge.")]
        private float wallThickness = 1.5f;

        [SerializeField, Tooltip("How far the inner face of the wall sits inside the track edge. Keep it slightly above zero so the wall does not z-fight with the side of the road slab.")]
        private float wallInset = 0.1f;

        [SerializeField, Tooltip("How far the walls extend below the track surface so no gap shows on slopes.")]
        private float wallSkirtDepth = 2f;

        [SerializeField, Range(50, 5000), Tooltip("Number of segments used around the track for the wall mesh.")]
        private int wallResolution = 1500;

        [SerializeField, Tooltip("Material for the walls. Falls back to the track material when empty.")]
        private Material wallMaterial;

        [SerializeField, Tooltip("Physics material for the walls. A low friction material keeps cars from catching on them.")]
        private PhysicsMaterial wallPhysicsMaterial;

        [SerializeField, Tooltip("Texture repeats per world unit along the walls.")]
        private float wallUVScale = 0.1f;

        [Header("Material Animation")]
        [SerializeField] private float _integrationDuration = 2f;
        [SerializeField] private AnimationCurve _integrationCurve = AnimationCurve.EaseInOut(0, 0, 1, 1);

        [Header("Debug Visualization")]
        [SerializeField] private bool _showDebugLines = false;
        [SerializeField] private bool _showVertexNormals = false;
        [SerializeField] private Color _debugColor = Color.cyan;
        [SerializeField] private float _debugLineLength = 2f;

        private static readonly int IntegrationValueProperty = Shader.PropertyToID("_IntegrationValue");
        private bool _isAnimatingMaterial = false;

        private SplineContainer _splineContainer;
        private MeshFilter _meshFilter;
        private MeshRenderer _meshRenderer;
        private MeshCollider _meshCollider;

        [SerializeField] private List<Vector3> vertices = new List<Vector3>();
        public List<Vector3> Vertices => vertices;

        public Spline Spline
        {
            get => _splineContainer.Spline;
            set => _splineContainer.Spline = value;
        }

        private void Awake()
        {
            _splineContainer = GetComponent<SplineContainer>();
            _meshFilter = GetComponent<MeshFilter>();
            _meshRenderer = GetComponent<MeshRenderer>();

            if (GenerateCollider && !TryGetComponent<MeshCollider>(out _meshCollider))
            {
                _meshCollider = gameObject.AddComponent<MeshCollider>();
            }
            
            // Load racing line data from serialized fields if available
            if (_racingLinePoints.Count > 0 && _racingLineSpeeds.Count > 0)
            {
                _racingLine = new RacingLine();
                _racingLine.Points.AddRange(_racingLinePoints);
                _racingLine.RecommendedSpeeds.AddRange(_racingLineSpeeds);
            }
        }

        public void Generate()
        {
            StartCoroutine(StartRoutines());
        }

        // Add new method for checkpoint and start/finish line generation
        private IEnumerator GenerateCheckpoints()
        {
            yield return new WaitForSeconds(0.1f);
            
            if ((checkpointPrefab == null && startFinishLinePrefab == null) || vertices.Count < 2)
                yield break;
        
            // Clear existing checkpoints and start/finish lines
            foreach (Transform child in transform)
            {
                if (child.name.StartsWith("Checkpoint") || child.name.StartsWith("StartFinishLine"))
                    Destroy(child.gameObject);
            }
        
            // Prefabs are authored for a fixed track width, so rescale them onto the current one
            float markerScale = MarkerScale;

            // Calculate equal spacing based on spline length
            float totalLength = Spline.GetLength();
            float interval = totalLength / numberOfCheckpoints;
            
            // Offset starting position to prevent overlap
            float currentDistance = interval * 0.5f;
            
            // Prepare to skip checkpoint at Start/Finish Line
            float startFinishDistance = -1f;
            
            // Find Start/Finish Line position first if prefab exists
            if (startFinishLinePrefab != null)
            {
                float straightestDistance = 0f;
                float minCurvature = float.MaxValue;
                float sfDistance = interval * 0.5f;
                for (int i = 0; i < numberOfCheckpoints; i++)
                {
                    Spline.Evaluate(Spline.ConvertIndexUnit(sfDistance, PathIndexUnit.Distance, PathIndexUnit.Normalized), 
                        out float3 position, 
                        out float3 tangent, 
                        out float3 up);
                    float nextDistance = Mathf.Min(sfDistance + totalLength * 0.02f, totalLength);
                    Spline.Evaluate(Spline.ConvertIndexUnit(nextDistance, PathIndexUnit.Distance, PathIndexUnit.Normalized), 
                        out float3 nextPosition, 
                        out float3 nextTangent, 
                        out float3 nextUp);
                    float curvature = Vector3.Angle(tangent, nextTangent);
                    if (curvature < minCurvature)
                    {
                        minCurvature = curvature;
                        straightestDistance = sfDistance;
                    }
                    sfDistance += interval;
                    if (sfDistance > totalLength) break;
                }
                startFinishDistance = straightestDistance;
                // Place the Start/Finish Line at the straightest checkpoint position
                Spline.Evaluate(Spline.ConvertIndexUnit(straightestDistance, PathIndexUnit.Distance, PathIndexUnit.Normalized), 
                    out float3 sfPosition, 
                    out float3 sfTangent, 
                    out float3 sfUp);
                Vector3 sfWorldPos = transform.TransformPoint(sfPosition);
                Quaternion sfRotation = Quaternion.LookRotation(transform.TransformDirection(sfTangent));
                sfRotation *= Quaternion.Euler(0, 0, 0); 
                sfWorldPos += Vector3.up * (checkpointVerticalOffset * markerScale);
                GameObject sfLine = Instantiate(
                    startFinishLinePrefab,
                    sfWorldPos,
                    sfRotation,
                    transform
                );
                sfLine.transform.localScale = startFinishLinePrefab.transform.localScale * markerScale;
                sfLine.name = "StartFinishLine";
                sfLine.tag = "StartFinish";
            }
            // Place checkpoints, skipping the one at Start/Finish Line
            currentDistance = interval * 0.5f;
            for (int i = 0; i < numberOfCheckpoints; i++)
            {
                // If this checkpoint is at (or very close to) the Start/Finish Line, skip it
                if (startFinishDistance >= 0f && Mathf.Abs(currentDistance - startFinishDistance) < interval * 0.25f)
                {
                    currentDistance += interval;
                    continue;
                }
                Spline.Evaluate(Spline.ConvertIndexUnit(currentDistance, PathIndexUnit.Distance, PathIndexUnit.Normalized), 
                    out float3 position, 
                    out float3 tangent, 
                    out float3 up);
                Vector3 worldPos = transform.TransformPoint(position);
                Quaternion rotation = Quaternion.LookRotation(transform.TransformDirection(tangent));
                rotation *= Quaternion.Euler(0, 90, 0); 
                worldPos += Vector3.up * (checkpointVerticalOffset * markerScale);
                if (checkpointPrefab != null)
                {
                    GameObject checkpoint = Instantiate(
                        checkpointPrefab,
                        worldPos,
                        rotation,
                        transform
                    );
                    checkpoint.transform.localScale = checkpointPrefab.transform.localScale * markerScale;
                    checkpoint.name = $"Checkpoint_{i}";
                    checkpoint.tag = "Checkpoint";
                }
                currentDistance += interval;
                if (currentDistance > totalLength) break;
            }
        }

        // Modify existing coroutine to include checkpoints and racing line
        public IEnumerator StartRoutines()
        {
            StartCoroutine(GenerateVertices());
            yield return new WaitForSeconds(0.4f);
            StartCoroutine(GenerateSpline());
            yield return new WaitForSeconds(0.4f);
            StartCoroutine(GenerateMesh());
            yield return new WaitForSeconds(0.4f);
            // Add or update the fall trigger after mesh generation
            CreateOrUpdateFallTrigger();
            StartCoroutine(GenerateCheckpoints());
            yield return new WaitForSeconds(0.4f);
            if (generateRacingLine)
            {
                GenerateRacingLine();
            }
        }
        
        private void GenerateRacingLine()
        {
            if (vertices.Count < 3) return;
            
            _racingLine.Generate(Spline, Width, Resolution, minDeviation, maxDeviation, deviationScale);
            
            // Save racing line data to serialized fields for prefab saving
            _racingLinePoints.Clear();
            _racingLineSpeeds.Clear();
            _racingLinePoints.AddRange(_racingLine.Points);
            _racingLineSpeeds.AddRange(_racingLine.RecommendedSpeeds);
        }

        protected abstract Path GetPath();

        public IEnumerator GenerateVertices()
        {
            yield return new WaitForSeconds(0.01f);
            using (Path path = GetPath())
            {
                path.Generate(transform);
                vertices = path.Vertices.ConvertAll(v => (Vector3)v);

                // Apply terrain height and verticality
                for (int i = 0; i < vertices.Count; i++)
                {
                    Vector3 worldPos = transform.TransformPoint(vertices[i]);

                    // Get base terrain height
                    float terrainHeight = _terrain != null ?
                        _terrain.SampleHeight(worldPos) :
                        worldPos.y;

                    // Add procedural verticality
                    float verticality = Mathf.PerlinNoise(
                        worldPos.x * _verticalityNoiseScale,
                        worldPos.z * _verticalityNoiseScale
                    ) * _maxVerticality;

                    // Combine terrain and verticality
                    worldPos.y = terrainHeight + _terrainOffset + verticality;
                    vertices[i] = transform.InverseTransformPoint(worldPos);
                }
            }
        }

        public IEnumerator GenerateSpline()
        {
            yield return new WaitForSeconds(0.01f);
            Spline.Clear();

            foreach (Vector3 vertex in vertices)
            {
                Spline.Add(transform.InverseTransformPoint(vertex));
            }

            Spline.Closed = true;
        }

        public IEnumerator GenerateMesh()
        {
            yield return new WaitForSeconds(0.01f);
            int vLength = (Resolution + 2) * 2;

            NativeArray<float3> verticesNative = new NativeArray<float3>(vLength, Allocator.TempJob);
            NativeArray<float2> uvs = new NativeArray<float2>(vLength, Allocator.TempJob);
            NativeArray<int> triangles = new NativeArray<int>(vLength * 3, Allocator.TempJob);
            NativeSpline spline = new NativeSpline(Spline, Allocator.TempJob);

            new CalculateUVsAndVerticesJob
            {
                Vertices = verticesNative,
                UVs = uvs,
                Length = vLength,
                Spline = spline,
                Width = Width
            }.Schedule(vLength, 8).Complete();

            new CalculateTrianglesJob
            {
                Triangles = triangles,
                VerticesLength = vLength
            }.Schedule(triangles.Length, 8).Complete();

            Spline = new Spline(spline, true);

            // Convert job output to mesh
            Vector3[] topVertices = Array.ConvertAll(verticesNative.ToArray(), v => (Vector3)v);

            // Add thickness by duplicating vertices downward
            Vector3[] allVertices = new Vector3[topVertices.Length * 2];
            List<int> allTriangles = new List<int>(triangles.ToArray());

            // Copy top vertices and create bottom vertices
            for (int i = 0; i < topVertices.Length; i++)
            {
                allVertices[i] = topVertices[i]; // Top vertex
                allVertices[i + topVertices.Length] = new Vector3(
                    topVertices[i].x,
                    topVertices[i].y - _thickness,
                    topVertices[i].z
                ); // Bottom vertex
            }

            // Generate side triangles with corrected winding order
            for (int i = 0; i < topVertices.Length - 2; i += 2)
            {
                // Current segment indices
                int tl = i;          // Top left
                int tr = i + 1;      // Top right
                int bl = i + topVertices.Length;     // Bottom left
                int br = i + 1 + topVertices.Length; // Bottom right

                // Next segment indices (for connecting sides between segments)
                int ntl = i + 2;     // Next top left
                int ntr = i + 3;     // Next top right
                int nbl = i + 2 + topVertices.Length; // Next bottom left
                int nbr = i + 3 + topVertices.Length; // Next bottom right

                // Left side (facing outward)
                allTriangles.Add(tl); allTriangles.Add(bl); allTriangles.Add(ntl);
                allTriangles.Add(ntl); allTriangles.Add(bl); allTriangles.Add(nbl);

                // Right side (facing outward)
                allTriangles.Add(tr); allTriangles.Add(ntr); allTriangles.Add(br);
                allTriangles.Add(br); allTriangles.Add(ntr); allTriangles.Add(nbr);

                // Bottom face (optional)
                allTriangles.Add(bl); allTriangles.Add(br); allTriangles.Add(nbl);
                allTriangles.Add(nbl); allTriangles.Add(br); allTriangles.Add(nbr);
            }

            // Fix UVs (same as before)
            Vector2[] originalUVs = Array.ConvertAll(uvs.ToArray(), v => (Vector2)v);
            Vector2[] allUVs = new Vector2[originalUVs.Length * 2];
            originalUVs.CopyTo(allUVs, 0);
            originalUVs.CopyTo(allUVs, originalUVs.Length);

            // Update mesh
            _meshFilter.mesh.Clear();
            _meshFilter.mesh.vertices = allVertices;
            _meshFilter.mesh.uv = allUVs;
            _meshFilter.mesh.triangles = allTriangles.ToArray();

            // Fixed Y-axis tiling at 40
            if (_meshRenderer.sharedMaterial != null)
            {
                _meshRenderer.sharedMaterial.SetFloat(IntegrationValueProperty, 0f);
                if (!_isAnimatingMaterial)
                {
                    StartCoroutine(AnimateIntegrationValue());
                }
                _meshRenderer.sharedMaterial.mainTextureScale = new Vector2(1, 40);
            }

            _meshFilter.mesh.RecalculateNormals();

            if (_meshCollider != null && GenerateCollider)
            {
                _meshCollider.sharedMesh = _meshFilter.sharedMesh;
            }

            // Walls follow the same spline and width, so rebuild them whenever the mesh changes
            GenerateWalls();

            // Debug visualization
            if (_showDebugLines)
            {
                DrawDebugLines();
            }

            // Dispose
            verticesNative.Dispose();
            uvs.Dispose();
            triangles.Dispose();
            spline.Dispose();
        }

        private IEnumerator AnimateIntegrationValue()
        {
            _isAnimatingMaterial = true;
            Material material = _meshRenderer.sharedMaterial;
            float elapsedTime = 0f;

            // Reset integration value at start
            material.SetFloat(IntegrationValueProperty, 0f);

            while (elapsedTime < _integrationDuration)
            {
                elapsedTime += Time.deltaTime;
                float normalizedTime = elapsedTime / _integrationDuration;
                float curveValue = _integrationCurve.Evaluate(normalizedTime);

                material.SetFloat(IntegrationValueProperty, curveValue);
                yield return null;
            }

            // Ensure we end at exactly 1
            material.SetFloat(IntegrationValueProperty, 1f);
            _isAnimatingMaterial = false;
        }

        private void Update()
        {
            if (_showDebugLines)
            {
                // Add pulsing effect to lines
                float pulse = Mathf.PingPong(Time.time, 1f);
                _debugColor.a = pulse;

                // Randomly shift line length for "scanning" effect
                _debugLineLength = 1.5f + Mathf.Sin(Time.time) * 0.5f;
            }
            
            // Draw racing line in editor
            if (showRacingLine && _racingLine != null && _racingLine.Points.Count > 1)
            {
                for (int i = 0; i < _racingLine.Points.Count - 1; i++)
                {
                    Vector3 start = transform.TransformPoint(_racingLine.Points[i]);
                    Vector3 end = transform.TransformPoint(_racingLine.Points[i + 1]);
                    Debug.DrawLine(start, end, racingLineColor);
                    
                    // Draw speed indicators
                    float speed = _racingLine.RecommendedSpeeds[i];
                    Color speedColor = Color.Lerp(Color.red, Color.green, speed);
                    Debug.DrawLine(start, start + Vector3.up * speed * 2f, speedColor);
                }
                
                // Connect last point to first point
                if (_racingLine.Points.Count > 0)
                {
                    Debug.DrawLine(
                        transform.TransformPoint(_racingLine.Points[_racingLine.Points.Count - 1]),
                        transform.TransformPoint(_racingLine.Points[0]),
                        racingLineColor
                    );
                }
            }
        }

        private void DrawDebugLines()
        {
            if (vertices.Count < 2) return;

            // Draw main track path
            for (int i = 0; i < vertices.Count - 1; i++)
            {
                Vector3 start = transform.TransformPoint(vertices[i]);
                Vector3 end = transform.TransformPoint(vertices[i + 1]);
                Debug.DrawLine(start, end, _debugColor);

                // Add geeky perpendicular lines
                Vector3 mid = Vector3.Lerp(start, end, 0.5f);
                Vector3 perpendicular = Vector3.Cross((end - start).normalized, Vector3.up);
                Debug.DrawLine(mid, mid + perpendicular * _debugLineLength, Color.magenta);
            }

            // Draw connection between first and last vertex
            Debug.DrawLine(
                transform.TransformPoint(vertices[0]),
                transform.TransformPoint(vertices[vertices.Count - 1]),
                _debugColor
            );

            if (_showVertexNormals)
            {
                // Draw fake normals with random directions
                foreach (Vector3 vertex in vertices)
                {
                    Vector3 worldPos = transform.TransformPoint(vertex);
                    Vector3 randomDir = UnityEngine.Random.onUnitSphere * _debugLineLength;
                    Debug.DrawLine(worldPos, worldPos + randomDir, Color.green);
                }
            }

            // Add computational-looking grid
            float gridSize = 10f;
            Vector3 boundsCenter = transform.TransformPoint(vertices[0]);
            for (float x = -gridSize; x <= gridSize; x += gridSize / 2)
            {
                for (float z = -gridSize; z <= gridSize; z += gridSize / 2)
                {
                    Vector3 gridPoint = boundsCenter + new Vector3(x, 0, z);
                    Debug.DrawLine(gridPoint, gridPoint + Vector3.up * 0.5f, Color.yellow);
                }
            }
        }

        /// <summary>
        /// Rebuilds the barrier walls that run along both edges of the track mesh.
        /// </summary>
        public void GenerateWalls()
        {
            Transform existing = transform.Find(WallsObjectName);

            if (!generateWalls)
            {
                if (existing != null)
                {
                    DestroyImmediate(existing.gameObject);
                }

                return;
            }

            if (Spline == null || Spline.Count < 2) return;

            Mesh wallMesh = BuildWallMesh();
            if (wallMesh == null) return;

            GameObject walls = existing != null ? existing.gameObject : new GameObject(WallsObjectName);

            walls.transform.SetParent(transform, false);
            walls.transform.localPosition = Vector3.zero;
            walls.transform.localRotation = Quaternion.identity;
            walls.transform.localScale = Vector3.one;
            walls.layer = gameObject.layer;

            if (!walls.TryGetComponent(out MeshFilter filter))
            {
                filter = walls.AddComponent<MeshFilter>();
            }

            filter.sharedMesh = wallMesh;

            if (!walls.TryGetComponent(out MeshRenderer wallRenderer))
            {
                wallRenderer = walls.AddComponent<MeshRenderer>();
            }

            wallRenderer.sharedMaterial = wallMaterial != null
                ? wallMaterial
                : (_meshRenderer != null ? _meshRenderer.sharedMaterial : null);

            if (!walls.TryGetComponent(out MeshCollider wallCollider))
            {
                wallCollider = walls.AddComponent<MeshCollider>();
            }

            wallCollider.sharedMesh = wallMesh;
            wallCollider.convex = false;
            wallCollider.sharedMaterial = wallPhysicsMaterial;
        }

        /// <summary>
        /// Samples the spline and extrudes both track edges into a single closed wall mesh.
        /// </summary>
        private Mesh BuildWallMesh()
        {
            int segments = Mathf.Max(16, wallResolution);
            int sampleCount = segments + 1;

            float innerOffset = Mathf.Max(0.01f, Width - wallInset);
            float outerOffset = innerOffset + Mathf.Max(0.01f, wallThickness);
            float skirt = Mathf.Max(0f, wallSkirtDepth);
            float height = Mathf.Max(0.01f, wallHeight);

            Vector3[] innerLeft = new Vector3[sampleCount];
            Vector3[] outerLeft = new Vector3[sampleCount];
            Vector3[] innerRight = new Vector3[sampleCount];
            Vector3[] outerRight = new Vector3[sampleCount];
            Vector3[] ups = new Vector3[sampleCount];
            float[] distances = new float[sampleCount];

            Vector3 previousPoint = Vector3.zero;

            for (int i = 0; i < sampleCount; i++)
            {
                // The last sample wraps onto the first so the loop closes without a seam
                float t = (i % segments) / (float)segments;

                if (!Spline.Evaluate(t, out float3 position, out float3 tangent, out float3 up))
                    return null;

                // Same frame the mesh job uses, so the walls land exactly on the mesh edges
                Vector3 point = position;
                Vector3 side = math.normalize(math.cross(tangent, up));
                Vector3 upDirection = math.normalize(up);

                innerLeft[i] = point + side * innerOffset;
                outerLeft[i] = point + side * outerOffset;
                innerRight[i] = point - side * innerOffset;
                outerRight[i] = point - side * outerOffset;
                ups[i] = upDirection;
                distances[i] = i == 0 ? 0f : distances[i - 1] + Vector3.Distance(previousPoint, point);
                previousPoint = point;
            }

            List<Vector3> wallVertices = new List<Vector3>(sampleCount * 12);
            List<Vector2> wallUVs = new List<Vector2>(sampleCount * 12);
            List<int> wallTriangles = new List<int>(segments * 36);

            AppendWall(innerLeft, outerLeft, ups, distances, segments, height, skirt, false, wallVertices, wallUVs, wallTriangles);
            AppendWall(innerRight, outerRight, ups, distances, segments, height, skirt, true, wallVertices, wallUVs, wallTriangles);

            Mesh mesh = new Mesh
            {
                name = WallsObjectName,
                indexFormat = IndexFormat.UInt32
            };

            mesh.SetVertices(wallVertices);
            mesh.SetUVs(0, wallUVs);
            mesh.SetTriangles(wallTriangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            return mesh;
        }

        /// <summary>
        /// Extrudes one track edge into an inner face, an outer face and a cap.
        /// <paramref name="mirrored"/> flips the winding for the wall on the other side of the centre line.
        /// </summary>
        private void AppendWall(Vector3[] inner, Vector3[] outer, Vector3[] ups, float[] distances, int segments,
            float height, float skirt, bool mirrored,
            List<Vector3> wallVertices, List<Vector2> wallUVs, List<int> wallTriangles)
        {
            int sampleCount = segments + 1;
            float verticalUV = (height + skirt) * wallUVScale;
            float capUV = Mathf.Max(0.01f, wallThickness) * wallUVScale;

            // Each face gets its own rail of vertices so the wall keeps hard edges where the faces meet
            int innerBase = wallVertices.Count;
            for (int i = 0; i < sampleCount; i++)
            {
                wallVertices.Add(inner[i] - ups[i] * skirt);
                wallVertices.Add(inner[i] + ups[i] * height);
                wallUVs.Add(new Vector2(distances[i] * wallUVScale, 0f));
                wallUVs.Add(new Vector2(distances[i] * wallUVScale, verticalUV));
            }

            int outerBase = wallVertices.Count;
            for (int i = 0; i < sampleCount; i++)
            {
                wallVertices.Add(outer[i] - ups[i] * skirt);
                wallVertices.Add(outer[i] + ups[i] * height);
                wallUVs.Add(new Vector2(distances[i] * wallUVScale, 0f));
                wallUVs.Add(new Vector2(distances[i] * wallUVScale, verticalUV));
            }

            int capBase = wallVertices.Count;
            for (int i = 0; i < sampleCount; i++)
            {
                wallVertices.Add(inner[i] + ups[i] * height);
                wallVertices.Add(outer[i] + ups[i] * height);
                wallUVs.Add(new Vector2(distances[i] * wallUVScale, 0f));
                wallUVs.Add(new Vector2(distances[i] * wallUVScale, capUV));
            }

            for (int i = 0; i < segments; i++)
            {
                int current = 2 * i;
                int next = 2 * (i + 1);

                AddQuad(wallTriangles, innerBase + current, innerBase + current + 1, innerBase + next + 1, innerBase + next, mirrored);
                AddQuad(wallTriangles, outerBase + current, outerBase + current + 1, outerBase + next + 1, outerBase + next, !mirrored);
                AddQuad(wallTriangles, capBase + current, capBase + current + 1, capBase + next + 1, capBase + next, mirrored);
            }
        }

        private static void AddQuad(List<int> wallTriangles, int v0, int v1, int v2, int v3, bool flip)
        {
            if (flip)
            {
                wallTriangles.Add(v0); wallTriangles.Add(v2); wallTriangles.Add(v1);
                wallTriangles.Add(v0); wallTriangles.Add(v3); wallTriangles.Add(v2);
            }

            else
            {
                wallTriangles.Add(v0); wallTriangles.Add(v1); wallTriangles.Add(v2);
                wallTriangles.Add(v0); wallTriangles.Add(v2); wallTriangles.Add(v3);
            }
        }

        // Add a method to create or update the fall trigger
        private void CreateOrUpdateFallTrigger()
        {
            // Remove any existing fall trigger
            Transform existing = transform.Find("FallTrigger");
            if (existing != null)
            {
                DestroyImmediate(existing.gameObject);
            }

            float totalLength = Spline.GetLength();
            float boxSize = totalLength * 1.5f;
            float height = 100f; // Tall enough to catch any car
            float yOffset = 0f;
            if (vertices.Count > 0)
            {
                // Find average Y for centering
                float sumY = 0f;
                foreach (var v in vertices) sumY += transform.TransformPoint(v).y;
                yOffset = sumY / vertices.Count;
            }

            GameObject fallTrigger = new GameObject("FallTrigger");
            fallTrigger.transform.parent = transform;
            fallTrigger.transform.localPosition = Vector3.zero;
            fallTrigger.transform.localRotation = Quaternion.identity;
            fallTrigger.tag = "Fall";

            BoxCollider box = fallTrigger.AddComponent<BoxCollider>();
            box.isTrigger = true;
            box.size = new Vector3(boxSize, height, boxSize);
            box.center = new Vector3(0, yOffset - (height / 2) - 190f, 0); // Place below the track
        }
    }
}