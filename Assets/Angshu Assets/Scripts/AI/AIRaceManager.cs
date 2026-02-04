using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Track;
using System.Linq;
using Ashsvp;

public class AIRaceManager : MonoBehaviour
{
    [Header("AI Racers Configuration")]
    [SerializeField] private TrackGenerator trackGenerator;
    [SerializeField] private List<GameObject> aiVehiclePrefabs;
    [SerializeField, Range(1, 10)] private int numberOfAIRacers = 3;
    [SerializeField] private float startingOffset = 15f;
    
    [Header("AI Difficulty Settings")]
    [SerializeField, Range(0f, 1f)] private float minSkillLevel = 0.5f;
    [SerializeField, Range(0f, 1f)] private float maxSkillLevel = 0.9f;
    [SerializeField, Range(0f, 1f)] private float minAggressiveness = 0.3f;
    [SerializeField, Range(0f, 1f)] private float maxAggressiveness = 0.8f;
    
    [Header("Rubber Banding")]
    [SerializeField, Range(0f, 1f), Tooltip("How strongly the rubber banding effect pulls cars together.")]
    private float rubberBandingStrength = 0.5f;
    [SerializeField, Tooltip("The max speed boost given to cars that are behind.")]
    private float maxSpeedBoost = 1.2f; // 20% speed boost
    [SerializeField, Tooltip("The max speed penalty given to the car in the lead.")]
    private float maxSpeedPenalty = 0.9f; // 10% speed penalty
    
    [Header("Race Game Mode")]
    [SerializeField, Range(1, 20)] private int numberOfLaps = 3;
    [SerializeField] private GameObject playerVehiclePrefab;
    [SerializeField] private bool spawnPlayer = true;
    [SerializeField] private string playerTag = "Player";
    
    [Header("Auto Setup")]
    [SerializeField] private bool autoSetupAllVehicles = true;
    [SerializeField] private bool setupPlayerVehicles = true;
    [SerializeField] private float respawnHeight = 2f;
    [SerializeField] private float respawnForwardOffset = 5f;
    [SerializeField] private float respawnInvulnerabilityTime = 2f;
    [Header("AI Car Appearance")]
    [SerializeField] private List<Material> aiCarMaterials = new List<Material>();
    
    private List<AIVehicleController> aiRacers = new List<AIVehicleController>();
    private bool raceIsActive = false;
    
    // Position tracking
    public List<AIVehicleController> SortedRacers { get; private set; } = new List<AIVehicleController>();
    public Dictionary<AIVehicleController, int> CarPositions { get; private set; } = new Dictionary<AIVehicleController, int>();
    private Dictionary<AIVehicleController, float> overtakeCooldowns = new Dictionary<AIVehicleController, float>();
    
    // Race state for all cars (AI + player)
    private class RaceCarState {
        public GameObject car;
        public int currentLap = 1;
        public int lastCheckpoint = -1;
        public bool finished = false;
        public float finishTime = 0f;
        public int position = 0;
        public AIVehicleController aiController; // Reference to AI controller if it's an AI car
    }
    private List<RaceCarState> raceCars = new List<RaceCarState>();
    private RaceCarState playerState = null;
    private bool raceFinished = false;
    private float raceStartTime = 0f;
    private int totalCheckpoints = 0;
    
    private void Start()
    {
        // Ensure RaceContextManager exists
        if (RaceContextManager.Instance == null)
        {
            var contextManagerGO = new GameObject("RaceContextManager");
            contextManagerGO.AddComponent<RaceContextManager>();
        }
        
        // Find track generator if not assigned
        if (trackGenerator == null)
        {
            trackGenerator = FindFirstObjectByType<TrackGenerator>();
            if (trackGenerator == null)
            {
                Debug.LogError("No TrackGenerator found in scene. AI race manager will not function properly.");
                enabled = false;
                return;
            }
        }
        
        // Subscribe to race controller events
        RaceController.OnRaceStart += OnRaceStarted;
        
        // Wait for track generation to complete before spawning AI racers
        StartCoroutine(SpawnAIRacersWhenReady());
        StartCoroutine(UpdatePositionsRoutine());
        StartCoroutine(PeriodicOvertakeOrders());
        
        // Count checkpoints for lap logic
        totalCheckpoints = 0;
        foreach (Transform child in trackGenerator.transform)
        {
            if (child.CompareTag("Checkpoint")) totalCheckpoints++;
        }
        
        // Auto-setup all vehicles if enabled
        if (autoSetupAllVehicles)
        {
            Invoke(nameof(SetupAllExistingVehicles), 2f); // Delay to ensure all objects are spawned
        }
    }
    
    private void OnDestroy()
    {
        // Unsubscribe from events
        RaceController.OnRaceStart -= OnRaceStarted;
    }
    
    private void OnRaceStarted()
    {
        raceStartTime = Time.time;
        
        // Enable racing for all AI
        foreach(var racer in aiRacers)
        {
            if (racer != null)
            {
                racer.IsRacing = true;
            }
        }
        
        // Start rubber banding updates
        if (!raceIsActive)
        {
            StartCoroutine(UpdateRubberBanding());
            raceIsActive = true;
        }
        
        Debug.Log("[AIRaceManager] Race started - AI racers enabled");
    }
    
    private IEnumerator SpawnAIRacersWhenReady()
    {
        // Wait for track generation to complete
        yield return new WaitForSeconds(2f); // Give time for track generation to complete
        
        // Make sure racing line is generated
        if (trackGenerator.RacingLine == null || trackGenerator.RacingLine.Points.Count == 0)
        {
            Debug.LogWarning("No racing line found on track. AI racers will not be spawned.");
            yield break;
        }
        
        SpawnAIRacers();

        // Start the race with a countdown (handled by RaceController now)
        // The RaceController will manage the countdown and enable AI racing
        Debug.Log("AI racers spawned. RaceController will handle countdown and race start.");
    }
    
    private void SpawnAIRacers()
    {
        if (aiVehiclePrefabs == null || aiVehiclePrefabs.Count == 0)
        {
            Debug.LogError("AI vehicle prefabs list is not assigned or is empty!");
            return;
        }
        
        // Clear any existing AI racers
        foreach (var racer in aiRacers)
        {
            if (racer != null)
            {
                Destroy(racer.gameObject);
            }
        }
        aiRacers.Clear();
        
        // Clear race state
        raceCars.Clear();
        playerState = null;
        raceFinished = false;
        
        // Find the Start/Finish Line
        Transform startFinishLine = null;
        foreach (Transform child in trackGenerator.transform)
        {
            if (child.name == "StartFinishLine")
            {
                startFinishLine = child;
                break;
            }
        }
        
        if (startFinishLine == null)
        {
            Debug.LogWarning("Start/Finish Line not found! Falling back to racing line start position.");
            // Fallback to original racing line method
            Vector3 fallbackPosition = trackGenerator.transform.TransformPoint(trackGenerator.RacingLine.Points[0]);
            Vector3 fallbackDirection = trackGenerator.transform.TransformPoint(trackGenerator.RacingLine.Points[5]) - fallbackPosition;
            fallbackDirection.y = 0;
            fallbackDirection.Normalize();
            SpawnCarsAtPosition(fallbackPosition, fallbackDirection);
            return;
        }
        
        // Get position and orientation from the Start/Finish Line
        Vector3 startPosition = startFinishLine != null ? startFinishLine.position : trackGenerator.transform.TransformPoint(trackGenerator.RacingLine.Points[0]);
        Vector3 startDirection = startFinishLine != null ? startFinishLine.forward : (trackGenerator.transform.TransformPoint(trackGenerator.RacingLine.Points[5]) - startPosition).normalized;
        startDirection.y = 0;
        startDirection.Normalize();
        
        // Find ground level at the start position
        Vector3 groundPosition = FindGroundPosition(startPosition);
        
        SpawnCarsAtPosition(groundPosition, startDirection);
    }
    
    private Vector3 FindGroundPosition(Vector3 startPosition)
    {
        // Raycast downward to find the ground
        RaycastHit hit;
        if (Physics.Raycast(startPosition + Vector3.up * 10f, Vector3.down, out hit, 20f))
        {
            return hit.point + Vector3.up * 0.5f; // Slight offset above ground
        }
        
        // Fallback: use the start position but at a reasonable height
        return new Vector3(startPosition.x, startPosition.y - 2f, startPosition.z);
    }
    
    private void SpawnCarsAtPosition(Vector3 startPosition, Vector3 startDirection)
    {
        Vector3 sideDirection = Vector3.Cross(startDirection, Vector3.up);
        int carsPerRow = 2;
        float rowSpacing = 12f;
        float colSpacing = 6f;
        int totalCars = numberOfAIRacers + (spawnPlayer && playerVehiclePrefab != null ? 1 : 0);
        
        // Randomize player position in the grid
        int playerGridPosition = -1;
        if (spawnPlayer && playerVehiclePrefab != null)
        {
            playerGridPosition = Random.Range(0, totalCars);
        }
        
        int aiIndex = 0;
        int carIndex = 0;
        for (int i = 0; i < totalCars; i++)
        {
            int row = i / carsPerRow;
            int col = i % carsPerRow;
            Vector3 rowOffset = -startDirection * (row * rowSpacing + startingOffset);
            float gridCenterOffset = (carsPerRow - 1) * 0.5f;
            Vector3 colOffset = sideDirection * (col - gridCenterOffset) * colSpacing;
            Vector3 position = startPosition + rowOffset + colOffset;
            position.y += 0.5f;
            Quaternion rotation = Quaternion.LookRotation(startDirection);
            GameObject carObj = null;
            bool isPlayer = (spawnPlayer && playerVehiclePrefab != null && i == playerGridPosition);
            if (isPlayer)
            {
                carObj = Instantiate(playerVehiclePrefab, position, rotation);
                carObj.name = "PlayerCar";
                carObj.tag = playerTag;
            }
            else
            {
                GameObject prefabToSpawn = aiVehiclePrefabs[Random.Range(0, aiVehiclePrefabs.Count)];
                carObj = Instantiate(prefabToSpawn, position, rotation);
                carObj.name = $"AI_Racer_{aiIndex+1}";
                
                // Assign a unique material if available
                if (aiCarMaterials != null && aiCarMaterials.Count > 0)
                {
                    bool materialApplied = false;
                    
                    // Try to find the Body child object
                    Transform body = carObj.transform.Find("Body");
                    if (body != null)
                    {
                        Renderer rend = body.GetComponent<Renderer>();
                        if (rend != null)
                        {
                            Material mat = aiCarMaterials[aiIndex % aiCarMaterials.Count];
                            rend.material = mat;
                            materialApplied = true;
                            // Material assignment log removed - setup spam
                        }
                        else
                        {
                            Debug.LogWarning($"[AIRaceManager] Body object found but no Renderer component on {carObj.name}");
                        }
                    }
                    else
                    {
                        // If Body not found, try to find any child with a Renderer
                        Renderer[] renderers = carObj.GetComponentsInChildren<Renderer>();
                        if (renderers.Length > 0)
                        {
                            // Find the main body renderer (usually the largest one)
                            Renderer mainRenderer = renderers[0];
                            float maxVolume = 0f;
                            
                            foreach (var rend in renderers)
                            {
                                if (rend.bounds.size.x * rend.bounds.size.y * rend.bounds.size.z > maxVolume)
                                {
                                    maxVolume = rend.bounds.size.x * rend.bounds.size.y * rend.bounds.size.z;
                                    mainRenderer = rend;
                                }
                            }
                            
                            Material mat = aiCarMaterials[aiIndex % aiCarMaterials.Count];
                            mainRenderer.material = mat;
                            materialApplied = true;
                            // Material assignment log removed - setup spam
                        }
                        else
                        {
                            Debug.LogWarning($"[AIRaceManager] No Body object or renderers found on {carObj.name}");
                        }
                    }
                    
                    if (!materialApplied)
                    {
                        Debug.LogError($"[AIRaceManager] Failed to apply material to {carObj.name} - no suitable renderer found");
                    }
                }
                else
                {
                    Debug.LogWarning("[AIRaceManager] No AI car materials assigned in inspector");
                }
                
                // Ensure AIVehicleController is present
                var aiController = carObj.GetComponent<AIVehicleController>();
                if (aiController == null)
                {
                    aiController = carObj.AddComponent<AIVehicleController>();
                }
                aiIndex++;
            }
            // Add to race state
            var state = new RaceCarState { car = carObj };
            raceCars.Add(state);
            if (isPlayer) playerState = state;
            
            // Add RobustRespawnSystem component to handle fall/checkpoint detection
            var respawnSystem = carObj.GetComponent<RobustRespawnSystem>();
            if (respawnSystem == null)
            {
                respawnSystem = carObj.AddComponent<RobustRespawnSystem>();
                ConfigureRespawnSystem(respawnSystem);
            }
            
            // Configure AI controller if present
            var aiControllerConfig = carObj.GetComponent<AIVehicleController>();
            if (aiControllerConfig != null && !isPlayer)
            {
                aiControllerConfig.trackGenerator = trackGenerator;
                SetRandomDifficulty(aiControllerConfig, aiIndex-1);
                aiControllerConfig.IsRacing = false;
                aiControllerConfig.isMischiefCar = (Random.value < 0.4f);
                aiRacers.Add(aiControllerConfig);
                state.aiController = aiControllerConfig; // Store reference to AI controller
            }
            carIndex++;
        }
        
        // After all cars are spawned, update AI cars' otherVehicles list to include all cars
        StartCoroutine(UpdateAICarListsAfterDelay());
    }
    
    private IEnumerator UpdateAICarListsAfterDelay()
    {
        yield return new WaitForSeconds(0.5f); // Wait for all cars to be fully initialized
        
        // Get all AI cars and update their otherVehicles list
        foreach (var aiCar in aiRacers)
        {
            if (aiCar != null)
            {
                // Use reflection to access the private otherVehicles field
                var otherVehiclesField = typeof(AIVehicleController).GetField("otherVehicles", 
                    System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                
                if (otherVehiclesField != null)
                {
                    var otherVehicles = new List<AIVehicleController>();
                    
                    // Add all other AI cars
                    foreach (var otherAI in aiRacers)
                    {
                        if (otherAI != null && otherAI != aiCar)
                        {
                            otherVehicles.Add(otherAI);
                        }
                    }
                    
                    // Add player car if it exists and has an AIVehicleController (for consistency)
                    if (playerState != null && playerState.car != null)
                    {
                        var playerAI = playerState.car.GetComponent<AIVehicleController>();
                        if (playerAI != null)
                        {
                            otherVehicles.Add(playerAI);
                        }
                    }
                    
                    otherVehiclesField.SetValue(aiCar, otherVehicles);
                }
            }
        }
    }

    private IEnumerator UpdatePositionsRoutine()
    {
        while (true)
        {
            UpdateRacePositions();
            yield return new WaitForSeconds(1f);
        }
    }

    private void UpdateRacePositions()
    {
        // Create a list of all cars with their progress
        var allCarsWithProgress = new List<(RaceCarState state, float progress)>();
        
        foreach (var state in raceCars)
        {
            if (state.finished) continue;
            
            float progress = 0f;
            if (state.aiController != null)
            {
                // AI car - use AI progress
                progress = state.aiController.RaceProgress;
            }
            else if (state.car != null)
            {
                // Player car - calculate progress based on lap and checkpoint
                progress = (state.currentLap - 1) + (state.lastCheckpoint + 1) / (float)totalCheckpoints;
            }
            
            allCarsWithProgress.Add((state, progress));
        }
        
        // Sort by progress (highest first)
        allCarsWithProgress.Sort((a, b) => b.progress.CompareTo(a.progress));
        
        // Update positions
        CarPositions.Clear();
        SortedRacers.Clear();
        
        for (int i = 0; i < allCarsWithProgress.Count; i++)
        {
            var (state, progress) = allCarsWithProgress[i];
            state.position = i + 1;
            
            if (state.aiController != null)
            {
                CarPositions[state.aiController] = i + 1;
                SortedRacers.Add(state.aiController);
            }
        }
        
        // Position tracking removed - was too noisy
    }

    private IEnumerator UpdateRubberBanding()
    {
        while (true)
        {
            // Wait for a short interval before recalculating.
            yield return new WaitForSeconds(1.0f); 

            if (aiRacers.Count < 1) continue;

            float leadProgress = 0f;
            AIVehicleController leader = null;

            // Find the leader among all cars (AI + player)
            foreach (var state in raceCars)
            {
                if (state.finished) continue;
                
                float progress = 0f;
                if (state.aiController != null)
                {
                    progress = state.aiController.RaceProgress;
                }
                else if (state.car != null)
                {
                    // Player car progress
                    progress = (state.currentLap - 1) + (state.lastCheckpoint + 1) / (float)totalCheckpoints;
                }
                
                if (progress > leadProgress)
                {
                    leadProgress = progress;
                    leader = state.aiController; // leader will be null for player car
                }
            }

            if (leader == null) continue;

            // Apply rubber banding to all AI cars
            foreach (var racer in aiRacers)
            {
                float progressDifference = leadProgress - racer.RaceProgress;
                
                if (racer == leader)
                {
                    // The leader gets slowed down based on how far they are from the car in 2nd place.
                    float secondProgress = 0;
                    foreach(var state in raceCars)
                    {
                        if (state.finished || state.aiController == leader) continue;
                        
                        float otherProgress = 0f;
                        if (state.aiController != null)
                        {
                            otherProgress = state.aiController.RaceProgress;
                        }
                        else if (state.car != null)
                        {
                            otherProgress = (state.currentLap - 1) + (state.lastCheckpoint + 1) / (float)totalCheckpoints;
                        }
                        
                        if (otherProgress > secondProgress)
                        {
                            secondProgress = otherProgress;
                        }
                    }
                    float leadAdvantage = leadProgress - secondProgress;
                    float penaltyFactor = Mathf.InverseLerp(0f, 0.1f, leadAdvantage); // 10% of track ahead
                    racer.RubberBandingFactor = Mathf.Lerp(1f, maxSpeedPenalty, penaltyFactor * rubberBandingStrength);
                }
                else
                {
                    // Other cars get a boost based on how far they are behind the leader.
                    float boostFactor = Mathf.InverseLerp(0f, 0.2f, progressDifference); // 20% of track behind
                    racer.RubberBandingFactor = Mathf.Lerp(1f, maxSpeedBoost, boostFactor * rubberBandingStrength);
                }
            }
        }
    }
    
    // Remove per-car overtaking coroutines and add a single periodic overtake order coroutine
    private IEnumerator PeriodicOvertakeOrders()
    {
        while (true)
        {
            yield return new WaitForSeconds(10f);
            if (!raceIsActive) continue;
            if (SortedRacers.Count < 2) continue;
            
            // Get all cars sorted by position (including player)
            var allCarsWithProgress = new List<(RaceCarState state, float progress)>();
            foreach (var state in raceCars)
            {
                if (state.finished) continue;
                
                float progress = 0f;
                if (state.aiController != null)
                {
                    progress = state.aiController.RaceProgress;
                }
                else if (state.car != null)
                {
                    progress = (state.currentLap - 1) + (state.lastCheckpoint + 1) / (float)totalCheckpoints;
                }
                
                allCarsWithProgress.Add((state, progress));
            }
            
            allCarsWithProgress.Sort((a, b) => b.progress.CompareTo(a.progress));
            
            // Order cars to overtake the car ahead of them
            for (int i = 1; i < allCarsWithProgress.Count; i++)
            {
                var currentState = allCarsWithProgress[i].state;
                var aheadState = allCarsWithProgress[i - 1].state;
                
                // Only AI cars can be ordered to overtake
                if (currentState.aiController != null)
                {
                    if (aheadState.aiController != null)
                    {
                        // AI overtaking AI
                        Debug.Log($"[RaceManager] {currentState.aiController.gameObject.name} (P{i+1}) ordered to overtake {aheadState.aiController.gameObject.name} (P{i})");
                        currentState.aiController.ForceOvertake(aheadState.aiController);
                    }
                    else if (aheadState.car != null && aheadState.car.CompareTag(playerTag))
                    {
                        // AI overtaking player
                        Debug.Log($"[RaceManager] {currentState.aiController.gameObject.name} (P{i+1}) ordered to overtake Player (P{i})");
                        // Note: Player doesn't have an AIVehicleController, so we can't use ForceOvertake
                        // The AI will naturally try to overtake the player through its normal logic
                    }
                }
            }
        }
    }
    
    private void SetRandomDifficulty(AIVehicleController aiController, int racerIndex)
    {
        // Access the serialized fields using reflection
        var skillField = aiController.GetType().GetField("skillLevel", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
        var aggressivenessField = aiController.GetType().GetField("aggressiveness", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
        
        if (skillField != null && aggressivenessField != null)
        {
            // Calculate skill level - lead cars are generally more skilled
            float normalizedIndex = (float)racerIndex / Mathf.Max(1, numberOfAIRacers - 1);
            float skillLevel = Mathf.Lerp(maxSkillLevel, minSkillLevel, normalizedIndex);
            
            // Add some randomness
            skillLevel += Random.Range(-0.1f, 0.1f);
            skillLevel = Mathf.Clamp(skillLevel, minSkillLevel, maxSkillLevel);
            
            // Calculate aggressiveness - random for each car
            float aggressiveness = Random.Range(minAggressiveness, maxAggressiveness);
            
            // Set values
            skillField.SetValue(aiController, skillLevel);
            aggressivenessField.SetValue(aiController, aggressiveness);
        }
    }
    
    // Method to reset race (can be called from other scripts)
    public void ResetRace()
    {
        StartCoroutine(SpawnAIRacersWhenReady());
    }
    
    // Public method to get current race positions for all cars
    public List<(string carName, int position, float progress)> GetCurrentPositions()
    {
        var positions = new List<(string carName, int position, float progress)>();
        
        foreach (var state in raceCars)
        {
            if (state.finished) continue;
            
            float progress = 0f;
            if (state.aiController != null)
            {
                // AI car - use AI progress
                progress = state.aiController.RaceProgress;
            }
            else if (state.car != null)
            {
                // Player car - calculate progress based on lap and checkpoint
                progress = (state.currentLap - 1) + (state.lastCheckpoint + 1) / (float)totalCheckpoints;
            }
            
            string carName = state.aiController != null ? state.aiController.gameObject.name : state.car.name;
            positions.Add((carName, state.position, progress));
        }
        
        // Sort by position
        positions.Sort((a, b) => a.position.CompareTo(b.position));
        return positions;
    }

    // --- Checkpoint and Lap Logic ---
    // Note: OnTriggerEnter removed from here since AIRaceManager doesn't have a Collider
    // Fall detection is now handled by individual car components

    // Public method to handle fall detection (called from car components)
    public void HandleCarFall(GameObject car)
    {
        var state = raceCars.Find(s => s.car == car);
        if (state == null || state.finished) return;
        
        // Get or add the robust respawn system
        RobustRespawnSystem respawnSystem = car.GetComponent<RobustRespawnSystem>();
        if (respawnSystem == null)
        {
            respawnSystem = car.AddComponent<RobustRespawnSystem>();
            ConfigureRespawnSystem(respawnSystem);
        }
        
        // Check if car is already respawning or in invulnerability period
        if (respawnSystem.IsRespawning() || respawnSystem.IsInvulnerable())
        {
            return;
        }
        
        // Respawn to last checkpoint
        if (state.lastCheckpoint >= 0)
        {
            Transform checkpoint = GetCheckpointTransform(state.lastCheckpoint);
            if (checkpoint != null)
            {
                respawnSystem.RespawnAtCheckpoint(checkpoint);
                Debug.Log($"{car.name} fell! Respawning at checkpoint {state.lastCheckpoint}");
            }
            else
            {
                Debug.LogWarning($"Checkpoint {state.lastCheckpoint} not found for {car.name}");
                RespawnAtStartLine(car, respawnSystem);
            }
        }
        else
        {
            // If no checkpoint reached yet, respawn at start
            RespawnAtStartLine(car, respawnSystem);
        }
    }
    
    private void RespawnAtStartLine(GameObject car, RobustRespawnSystem respawnSystem)
    {
        Transform startFinishLine = null;
        foreach (Transform child in trackGenerator.transform)
        {
            if (child.name == "StartFinishLine")
            {
                startFinishLine = child;
                break;
            }
        }
        
        if (startFinishLine != null)
        {
            respawnSystem.RespawnAtStartLine(startFinishLine);
            Debug.Log($"{car.name} fell! Respawning at start line (no checkpoint reached)");
        }
        else
        {
            // Fallback: use racing line start position
            if (trackGenerator.RacingLine != null && trackGenerator.RacingLine.Points.Count > 0)
            {
                Vector3 startPos = trackGenerator.transform.TransformPoint(trackGenerator.RacingLine.Points[0]);
                Vector3 nextPos = trackGenerator.transform.TransformPoint(trackGenerator.RacingLine.Points[5]);
                Vector3 direction = (nextPos - startPos).normalized;
                Quaternion rotation = Quaternion.LookRotation(direction, Vector3.up);
                
                respawnSystem.RespawnAt(startPos, rotation);
                Debug.Log($"{car.name} fell! Respawning at racing line start (fallback)");
            }
            else
            {
                Debug.LogError($"No valid respawn position found for {car.name}");
            }
        }
    }

    // Public method to handle checkpoint detection (called from car components)
    public void HandleCheckpoint(GameObject car, Transform checkpoint)
    {
        var state = raceCars.Find(s => s.car == car);
        if (state == null || state.finished) return;
        
        int checkpointIndex = GetCheckpointIndex(checkpoint);
        if (checkpointIndex != -1 && checkpointIndex != state.lastCheckpoint)
        {
            state.lastCheckpoint = checkpointIndex;
            Debug.Log($"{car.name} reached checkpoint {checkpointIndex}");
        }
    }

    // Public method to handle start/finish line detection (called from car components)
    public void HandleStartFinish(GameObject car)
    {
        var state = raceCars.Find(s => s.car == car);
        if (state == null || state.finished) return;
        
        // Only count lap if all checkpoints were crossed
        if (state.lastCheckpoint == totalCheckpoints-1)
        {
            state.currentLap++;
            state.lastCheckpoint = -1;
            Debug.Log($"{car.name} completed lap {state.currentLap - 1}!");
            if (state.currentLap > numberOfLaps)
            {
                state.finished = true;
                state.finishTime = Time.time - raceStartTime;
                Debug.Log($"{car.name} FINISHED! Time: {state.finishTime:F2}s");
                
                // Stop the car when it finishes
                StopCar(car);
                
                // Check if all cars have finished
                CheckRaceCompletion();
            }
        }
    }

    private void StopCar(GameObject car)
    {
        // Stop AI cars
        var aiController = car.GetComponent<AIVehicleController>();
        if (aiController != null)
        {
            aiController.StopCar();
        }
        
        // For player car, you might want to disable input or show a message
        if (car.CompareTag(playerTag))
        {
            Debug.Log("Player has finished the race!");
            // You can add UI logic here to show race completion
        }
    }
    
    private void CheckRaceCompletion()
    {
        bool allFinished = true;
        foreach (var state in raceCars)
        {
            if (!state.finished)
            {
                allFinished = false;
                break;
            }
        }
        
        if (allFinished && !raceFinished)
        {
            raceFinished = true;
            Debug.Log("ALL CARS HAVE FINISHED! Race complete!");
            
            // Stop all remaining cars
            foreach (var state in raceCars)
            {
                if (state.car != null)
                {
                    StopCar(state.car);
                }
            }
            
            // Stop the race manager updates
            raceIsActive = false;
        }
    }

    private int GetCheckpointIndex(Transform checkpoint)
    {
        int idx = 0;
        foreach (Transform child in trackGenerator.transform)
        {
            if (child.CompareTag("Checkpoint"))
            {
                if (child == checkpoint) return idx;
                idx++;
            }
        }
        return -1;
    }
    private Transform GetCheckpointTransform(int index)
    {
        int idx = 0;
        foreach (Transform child in trackGenerator.transform)
        {
            if (child.CompareTag("Checkpoint"))
            {
                if (idx == index) return child;
                idx++;
            }
        }
        return null;
    }
    
    // Public method to apply materials to an existing AI car
    public void ApplyMaterialToAICar(GameObject carObj, int materialIndex = -1)
    {
        if (aiCarMaterials == null || aiCarMaterials.Count == 0)
        {
            Debug.LogWarning("[AIRaceManager] No AI car materials assigned in inspector");
            return;
        }
        
        if (materialIndex < 0)
        {
            // Find the car in our AI racers list to get its index
            materialIndex = aiRacers.FindIndex(ai => ai.gameObject == carObj);
            if (materialIndex < 0)
            {
                Debug.LogWarning($"[AIRaceManager] Car {carObj.name} not found in AI racers list");
                return;
            }
        }
        
        Material mat = aiCarMaterials[materialIndex % aiCarMaterials.Count];
        bool materialApplied = false;
        
        // Try to find the Body child object
        Transform body = carObj.transform.Find("Body");
        if (body != null)
        {
            Renderer rend = body.GetComponent<Renderer>();
            if (rend != null)
            {
                rend.material = mat;
                materialApplied = true;
                // Material assignment log removed - setup spam
            }
        }
        
        if (!materialApplied)
        {
            // Fallback: find any renderer
            Renderer[] renderers = carObj.GetComponentsInChildren<Renderer>();
            if (renderers.Length > 0)
            {
                Renderer mainRenderer = renderers[0];
                float maxVolume = 0f;
                
                foreach (var rend in renderers)
                {
                    if (rend.bounds.size.x * rend.bounds.size.y * rend.bounds.size.z > maxVolume)
                    {
                        maxVolume = rend.bounds.size.x * rend.bounds.size.y * rend.bounds.size.z;
                        mainRenderer = rend;
                    }
                }
                
                mainRenderer.material = mat;
                materialApplied = true;
                // Material assignment log removed - setup spam
            }
        }
        
        if (!materialApplied)
        {
            Debug.LogError($"[AIRaceManager] Failed to apply material to {carObj.name} - no suitable renderer found");
        }
    }
    
    // Public method to refresh materials for all AI cars
    public void RefreshAllAICarMaterials()
    {
        for (int i = 0; i < aiRacers.Count; i++)
        {
            if (aiRacers[i] != null)
            {
                ApplyMaterialToAICar(aiRacers[i].gameObject, i);
            }
        }
    }
    
    // === INTEGRATED RACE SETUP FUNCTIONALITY ===
    
    /// <summary>
    /// Setup all existing vehicles in the scene (integrated from RaceSetupManager)
    /// </summary>
    [ContextMenu("Setup All Existing Vehicles")]
    public void SetupAllExistingVehicles()
    {
        Debug.Log("[AIRaceManager] Setting up all existing vehicles...");
        
        // Find all vehicles with SimcadeVehicleController
        var allVehicles = FindObjectsByType<SimcadeVehicleController>(FindObjectsSortMode.None);
        
        int playerCount = 0;
        int aiCount = 0;
        
        foreach (var vehicle in allVehicles)
        {
            if (vehicle == null) continue;
            
            bool isPlayer = vehicle.CompareTag("Player") || vehicle.GetComponent<AIVehicleController>() == null;
            
            if (isPlayer && setupPlayerVehicles)
            {
                SetupPlayerVehicle(vehicle.gameObject);
                playerCount++;
            }
            else if (!isPlayer)
            {
                SetupExistingAIVehicle(vehicle.gameObject);
                aiCount++;
            }
        }
        
        Debug.Log($"[AIRaceManager] Vehicle setup complete: {playerCount} player vehicles, {aiCount} AI vehicles");
    }
    
    private void SetupPlayerVehicle(GameObject vehicle)
    {
        // Add robust respawn system
        RobustRespawnSystem respawnSystem = vehicle.GetComponent<RobustRespawnSystem>();
        if (respawnSystem == null)
        {
            respawnSystem = vehicle.AddComponent<RobustRespawnSystem>();
            ConfigureRespawnSystem(respawnSystem);
        }
        
        // Ensure InputManager exists
        var inputManager = vehicle.GetComponent<Ashsvp.InputManager_SVP>();
        if (inputManager == null)
        {
            Debug.LogWarning($"[AIRaceManager] Player vehicle {vehicle.name} missing InputManager_SVP component!");
        }
        
        // Setup logs removed - too verbose during setup
    }
    
    private void SetupExistingAIVehicle(GameObject vehicle)
    {
        // Add robust respawn system
        RobustRespawnSystem respawnSystem = vehicle.GetComponent<RobustRespawnSystem>();
        if (respawnSystem == null)
        {
            respawnSystem = vehicle.AddComponent<RobustRespawnSystem>();
            ConfigureRespawnSystem(respawnSystem);
        }
        
        // Add smooth steering controller
        SmoothSteeringController smoothSteering = vehicle.GetComponent<SmoothSteeringController>();
        if (smoothSteering == null)
        {
            smoothSteering = vehicle.AddComponent<SmoothSteeringController>();
        }
        
        // Ensure AI components exist
        AIVehicleController aiController = vehicle.GetComponent<AIVehicleController>();
        if (aiController == null)
        {
            Debug.LogWarning($"[AIRaceManager] AI vehicle {vehicle.name} missing AIVehicleController component!");
        }
        
        // Setup logs removed - too verbose during setup
    }
    
    private void ConfigureRespawnSystem(RobustRespawnSystem respawnSystem)
    {
        // Use reflection to set private fields since they're not exposed as public properties
        var respawnHeightField = typeof(RobustRespawnSystem).GetField("respawnHeight", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (respawnHeightField != null)
        {
            respawnHeightField.SetValue(respawnSystem, respawnHeight);
        }
        
        var respawnOffsetField = typeof(RobustRespawnSystem).GetField("respawnForwardOffset", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (respawnOffsetField != null)
        {
            respawnOffsetField.SetValue(respawnSystem, respawnForwardOffset);
        }
        
        var invulnerabilityField = typeof(RobustRespawnSystem).GetField("respawnInvulnerabilityTime", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (invulnerabilityField != null)
        {
            invulnerabilityField.SetValue(respawnSystem, respawnInvulnerabilityTime);
        }
    }
    
    [ContextMenu("Test Player Respawn")]
    public void TestPlayerRespawn()
    {
        // Find player vehicle and test respawn
        var allVehicles = FindObjectsByType<SimcadeVehicleController>(FindObjectsSortMode.None);
        
        foreach (var vehicle in allVehicles)
        {
            if (vehicle.CompareTag("Player"))
            {
                RobustRespawnSystem respawnSystem = vehicle.GetComponent<RobustRespawnSystem>();
                if (respawnSystem != null)
                {
                    respawnSystem.TestRespawnHere();
                    Debug.Log($"[AIRaceManager] Tested respawn for player vehicle: {vehicle.name}");
                    return;
                }
            }
        }
        
        Debug.LogWarning("[AIRaceManager] No player vehicle found for respawn test");
    }
    
    // === PRESET CONFIGURATIONS ===
    
    [ContextMenu("Apply 3-Racer Competitive Preset (RECOMMENDED)")]
    public void ApplyCompetitivePreset()
    {
        AIRaceManagerPresets.ApplyThreeRacerPreset(this);
    }
    
    [ContextMenu("Apply 3-Racer Casual Preset")]
    public void ApplyCasualPreset()
    {
        AIRaceManagerPresets.ApplyThreeRacerCasualPreset(this);
    }
    
    [ContextMenu("Apply 3-Racer Hardcore Preset")]
    public void ApplyHardcorePreset()
    {
        AIRaceManagerPresets.ApplyThreeRacerHardcorePreset(this);
    }
}