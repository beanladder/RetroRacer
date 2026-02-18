using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Ashsvp;
using Track;

[RequireComponent(typeof(SimcadeVehicleController))]
public class AIVehicleController : MonoBehaviour
{
    [Header("AI Configuration")]
    [SerializeField, Tooltip("Reference to the track generator that contains the racing line for the AI to follow")]
    public TrackGenerator trackGenerator;
    
    [SerializeField, Range(1, 20), Tooltip("How many points ahead on the racing line the AI will target. Higher values make the AI look further ahead and take smoother lines")]
    private int lookaheadPoints = 8;
    
    [SerializeField, Range(0f, 1f), Tooltip("Maximum steering angle the AI can use. Lower values make the AI take wider turns, higher values allow sharper turning")]
    private float maxSteeringAngle = 0.7f;
    
    [SerializeField, Range(0f, 1f), Tooltip("Maximum speed multiplier applied to the racing line's recommended speed. Higher values allow the AI to drive closer to the maximum possible speed")]
    private float maxSpeedMultiplier = 0.85f;
    
    [SerializeField, Range(0f, 1f), Tooltip("Minimum speed multiplier applied to the racing line's recommended speed. Higher values prevent the AI from driving too slowly")]
    private float minSpeedMultiplier = 0.6f;
    
    [SerializeField, Range(0f, 10f), Tooltip("How quickly the AI adjusts its steering. Higher values make steering more responsive but potentially less stable")]
    private float steeringSpeed = 2.5f;
    
    [SerializeField, Range(0f, 10f), Tooltip("How quickly the AI adjusts its acceleration/braking. Higher values make throttle control more responsive")]
    private float accelerationSpeed = 1.5f;
    
  
    
    [SerializeField, Tooltip("Layer mask for sensor raycasts (should include the layer that cars are on)")]
    private LayerMask carDetectionLayerMask = -1;
    
    [Header("Corner Handling")]
    [SerializeField, Range(5, 30), Tooltip("How many points ahead the AI looks to detect corners. Higher values allow earlier corner detection")]
    private int cornerDetectionLookahead = 25;
    
    [SerializeField, Range(0.1f, 1f), Tooltip("How much the AI reduces speed in corners. Lower values cause more aggressive braking in corners")]
    private float cornerSpeedReductionFactor = 0.5f;
    
    [SerializeField, Range(0.05f, 0.5f), Tooltip("Minimum curvature threshold to consider a section as a corner. Lower values detect more subtle corners")]
    private float cornerDetectionThreshold = 0.12f;
    
    [SerializeField, Range(1f, 20f), Tooltip("Distance at which the AI starts braking for corners. Higher values make the AI brake earlier before corners")]
    private float brakingDistance = 15f;
    
    [SerializeField, Range(0.1f, 5f), Tooltip("Multiplier for braking intensity. Higher values make the AI brake more aggressively")]
    private float brakingIntensityMultiplier = 1.8f;
    

    
    // Difficulty settings
    [Header("Difficulty Settings")]
    [SerializeField, Range(0f, 1f), Tooltip("Overall driving skill of the AI. Higher values improve cornering precision, braking timing, and racing line following")]
    private float skillLevel = 0.75f;
    
    [SerializeField, Range(0f, 1f), Tooltip("How aggressively the AI drives. Higher values increase target speeds and make the AI take more risks")]
    private float aggressiveness = 0.6f;
    
    // Debug visualization
    [Header("Debug")]
    [SerializeField, Tooltip("Whether to show debug visualization for AI decision making")]
    private bool showDebugInfo = true;
    
    [SerializeField, Tooltip("Color used for visualizing the target point the AI is steering towards")]
    private Color targetPointColor = Color.blue;
    
    [SerializeField, Tooltip("Color used for visualizing the racing line path ahead")]
    private Color pathColor = Color.yellow;
    
    [SerializeField, Tooltip("Show waypoint-based randomness visualization")]
    private bool showRandomnessDebug = true;
    
    [Header("Humanization")]
    [SerializeField, Range(0f, 2f), Tooltip("How far (in meters) the AI can randomly deviate from the racing line. Set to 0 to disable wiggling completely")]
    private float pathRandomness = 0; // Reduced from 0.8 to minimize wiggling

    [SerializeField, Range(5, 50), Tooltip("How many waypoints before changing random offset. Higher = smoother, less wiggling")]
    private int randomnessWaypointInterval = 20; // Increased from 15 for more stability
    
    // Waypoint-based randomness (prevents wiggling)
    private float currentRandomOffset = 0f;
    private float targetRandomOffset = 0f;
    private int lastRandomnessWaypoint = 0;
     
    [Header("Overtaking")]
    [SerializeField, Tooltip("How long to be stuck behind a car before attempting to overtake.")]
    private float overtakeTriggerTime = 2.0f;
    [SerializeField, Tooltip("How far to the side to move when overtaking.")]
    private float overtakeLaneOffset = 5.0f;
    [SerializeField, Tooltip("The maximum corner sharpness where an overtake is allowed.")]
    private float maxOvertakeCornerFactor = 0.2f;
    [SerializeField, Tooltip("Time to wait after completing an overtake before starting another.")]
    private float overtakeCooldown = 3.0f;
    
    [Header("Starting Grid & Racing Lines")]
    [SerializeField, Range(0f, 10f), Tooltip("How long to maintain starting grid lane after race start")]
    private float startingGridDuration = 5f;
    [SerializeField, Range(0f, 5f), Tooltip("Personality-based racing line offset range")]
    private float personalityLineOffset = 2.5f;
    
    // Public properties for external management
    public bool IsRacing { get; set; } = false;
    public float RaceProgress { get; private set; }
    public float RubberBandingFactor { get; set; } = 1f;
    
    // Private variables
    private SimcadeVehicleController vehicleController;
    private RacingLine racingLine;
    private int currentWaypointIndex = 0;
    private float currentSteer = 0f;
    private float currentAcceleration = 0f;
    private float currentBrake = 0f;
    private float currentHandbrake = 0f;
    private List<AIVehicleController> otherVehicles = new List<AIVehicleController>();
    private float perlinSeed;
    private bool isOvertaking = false;
    private float timeStuck = 0f;
    private float overtakeCooldownTimer = 0f;
    private float targetOvertakeOffset = 0f;
    private float currentOvertakeOffset = 0f;
    private AIVehicleController carToOvertake = null;
    private float originalAcceleration;
    private bool isForcedOvertake = false; // Track if overtake was forced by race manager
    
    // Starting grid and personality-based racing lines
    private float startingGridLaneOffset = 0f; // Assigned at spawn based on grid position
    private float personalityBasedOffset = 0f; // Based on personality type
    private float raceStartTime = 0f;
    private bool hasCalculatedPersonalityOffset = false;
    
    // Avoidance system state
    private float committedAvoidanceOffset = 0f;
    private float avoidanceCommitmentTimer = 0f;
    private float avoidanceCommitmentDuration = 1.5f;
    private float lastAvoidanceDecisionTime = 0f;
    private const float avoidanceDetectionRadius = 10f;
    private const float avoidanceAwarenessAngle = 120f; // degrees
    
    // Defensive driving state
    private bool isDefending = false;
    private float defenseTimer = 0f;
    private float defenseDuration = 0f;
    // Handbrake parameters (randomized per car)
    private float handbrakeStrength;
    private float handbrakeThresholdRandomized;
    // Strategic Nitro system (consolidated from SmartNitroSystem)
    private float nitroDecisionCooldown = 0f;
    private float lastNitroUseTime = 0f;
    private NitroStrategy currentNitroStrategy = NitroStrategy.Conservative;
    
    // Race context for nitro decisions
    private int currentPosition = 1;
    private int totalRacers = 1;
    private bool hasOpportunityAhead = false;
    
    public enum NitroStrategy
    {
        Conservative,   // Save nitro for key moments
        Aggressive,     // Use nitro to attack
        Defensive,      // Use nitro to defend position
        Opportunistic,  // Use nitro when opportunity arises
        Desperate       // Use nitro when far behind
    }
    
    // Nitro slowdown state
    private float nitroSlowdownTimer = 0f;
    private float nitroSlowdownDuration = 2.5f; // seconds to enforce slowdown after nitro
    private bool isNitroSlowingDown = false;
    
    // Nitro at race start
    private bool forceStartNitro = false;
    private float startNitroTimer = 0f;
    
    // Enhanced AI systems
    private AIPersonalityManager personalityManager;
    private SmoothSteeringController smoothSteeringController;
    private bool wasRacing = false;
    private float startNitroDuration = 3f;
    
    // Behavior modification system
    private float behaviorModifier_braking = 0f;
    private float behaviorModifier_steering = 0f;
    private float behaviorModifier_throttle = 0f;
    private float behaviorModifier_aggression = 0f;
    private float behaviorModifier_blocking = 0f;
    private float behaviorModifier_speed = 0f;
    private float behaviorModifier_defense = 0f;
    private float behaviorModifier_pathRandomness = 0f;
    
    // Base values for behavior modification (stored to prevent accumulation)
    private float baseAggressiveness;
    private float baseSkillLevel;
    private float baseMaxSpeedMultiplier;
    private float basePathRandomness;
    
    [Header("Ramming System")]
    [SerializeField] private bool enableRamming = true;
    [SerializeField, Range(0f, 1f)] private float rammingAggressiveness = 0.5f;
    [SerializeField] private float rammingCooldown = 3f;
    [SerializeField] private float rammingDetectionRange = 12f;
    
    private float rammingCooldownTimer = 0f;
    private AIVehicleController rammingTarget = null;
    private string rammingType = ""; // "side", "back", "front"
    
    private void Awake()
    {
        vehicleController = GetComponent<SimcadeVehicleController>();
        
        // Always enable Auto Counter Steer for AI
        vehicleController.AutoCounterSteer = true;
        
        // Disable the camera for AI cars (if present)
        if (vehicleController.cinemachineCamera != null)
        {
            vehicleController.cinemachineCamera.gameObject.SetActive(false);
            // Optionally, also disable alternativeCamera if needed:
            // if (vehicleController.alternativeCamera != null) vehicleController.alternativeCamera.gameObject.SetActive(false);
        }
        
        // Find track generator if not assigned
        if (trackGenerator == null)
        {
            trackGenerator = FindFirstObjectByType<TrackGenerator>();
            if (trackGenerator == null)
            {
                Debug.LogError("No TrackGenerator found in scene. AI vehicle will not function properly.");
                enabled = false;
                return;
            }
        }
        
        // Get racing line from track generator
        racingLine = trackGenerator.RacingLine;
        if (racingLine == null || racingLine.Points.Count == 0)
        {
            Debug.LogError("No racing line found on track generator. AI vehicle will not function properly.");
            enabled = false;
            return;
        }
        
        // Find other AI vehicles in the scene
        AIVehicleController[] allVehicles = FindObjectsByType<AIVehicleController>(FindObjectsSortMode.None);
        foreach (var vehicle in allVehicles)
        {
            if (vehicle != this)
            {
                otherVehicles.Add(vehicle);
            }
        }
        
        // Ensure InputManager is enabled for AI control
        if (vehicleController.inputManager != null)
        {
            vehicleController.inputManager.enabled = true;
        }

        // Initialize a random seed for this vehicle to make its behavior unique
        perlinSeed = Random.Range(0f, 1000f);
        // Store the original acceleration value from the vehicle controller
        originalAcceleration = vehicleController.Acceleration;

        // Drifty/non-drifty switch
        
            handbrakeStrength = Random.Range(0.05f, 0.1f);
            handbrakeThresholdRandomized = Random.Range(0.12f, 0.19f);
            vehicleController.driftFactor = Random.Range(0.52f, 0.63f);
      
        // AI setup details removed - too verbose

        // Randomize nitro acceleration multiplier for this AI car
        vehicleController.nitroAccelerationMultiplier = Random.Range(1.01f, 1.05f);
        vehicleController.nitroMaxSpeedMultiplier = Random.Range(1.01f, 1.05f);
        
        // Set higher turn angle for all AI cars
        vehicleController.MaxTurnAngle = Random.Range(32f, 35f);
        
        // Initialize enhanced AI systems
        personalityManager = GetComponent<AIPersonalityManager>();
        if (personalityManager == null)
        {
            personalityManager = gameObject.AddComponent<AIPersonalityManager>();
        }
        
        // Initialize strategic nitro system
        DetermineInitialNitroStrategy();
        
        // Register with race context manager
        if (RaceContextManager.Instance != null)
        {
            RaceContextManager.Instance.RegisterAICar(this);
        }
        
        // Initialize smooth steering controller
        smoothSteeringController = GetComponent<SmoothSteeringController>();
        if (smoothSteeringController == null)
        {
            smoothSteeringController = gameObject.AddComponent<SmoothSteeringController>();
        }
        
        // Store base values for behavior modification system
        baseAggressiveness = aggressiveness;
        baseSkillLevel = skillLevel;
        baseMaxSpeedMultiplier = maxSpeedMultiplier;
        basePathRandomness = pathRandomness;
    }
    
    private void Start()
    {
        // Find initial position on racing line
        Vector3 localPosition = trackGenerator.transform.InverseTransformPoint(transform.position);
        (currentWaypointIndex, _, _) = racingLine.GetClosestPoint(localPosition);
        
        // Configure input manager for AI control
        if (vehicleController.inputManager != null)
        {
            // Ensure the InputManager is enabled
            vehicleController.inputManager.enabled = true;
            
            // Set initial AI inputs
            vehicleController.inputManager.SetAIInputs(0f, 0f, 0f, false);
        }
        
        // Refresh the list of other vehicles after a short delay to ensure all cars are spawned
        StartCoroutine(RefreshOtherVehiclesAfterDelay());
    }
    
    private IEnumerator RefreshOtherVehiclesAfterDelay()
    {
        yield return new WaitForSeconds(3f); // Wait 3 seconds for all cars to spawn
        RefreshOtherVehicles();
    }
    
    private void RefreshOtherVehicles()
    {
        otherVehicles.Clear();
        AIVehicleController[] allVehicles = FindObjectsByType<AIVehicleController>(FindObjectsSortMode.None);
        foreach (var vehicle in allVehicles)
        {
            if (vehicle != this && vehicle != null)
            {
                otherVehicles.Add(vehicle);
            }
        }
    }
    
    private void Update()
    {
        // Detect race start and track time
        if (!wasRacing && IsRacing)
        {
            forceStartNitro = true;
            startNitroTimer = startNitroDuration;
            raceStartTime = Time.time; // Track when race started
            CalculatePersonalityOffset(); // Calculate personality-based offset once
        }
        wasRacing = IsRacing;

        if (!IsRacing)
        {
            // Before the race starts, keep the handbrake on to stay stationary.
            if(vehicleController.inputManager != null)
            {
                vehicleController.inputManager.SetAIInputs(0f, 0f, 1f, false);
            }
            return;
        }

        if (racingLine == null || racingLine.Points.Count == 0) return;
        
        // --- 1. Find our position and progress on the racing line ---
        Vector3 localPosition = trackGenerator.transform.InverseTransformPoint(transform.position);
        (currentWaypointIndex, _, _) = racingLine.GetClosestPoint(localPosition);
        
        if (racingLine.Points.Count > 0)
        {
            int nextWaypointIndex = (currentWaypointIndex + 1) % racingLine.Points.Count;
            float distanceToNext = Vector3.Distance(localPosition, racingLine.Points[nextWaypointIndex]);
            float segmentLength = Vector3.Distance(racingLine.Points[currentWaypointIndex], racingLine.Points[nextWaypointIndex]);
            
            float progressBetweenWaypoints = (segmentLength > 0) ? Mathf.Clamp01(1f - (distanceToNext / segmentLength)) : 0f;
            RaceProgress = (currentWaypointIndex + progressBetweenWaypoints) / racingLine.Points.Count;
        }

        // --- 2. Determine our ideal state (target point and speed) ---
        int adjustedLookahead = Mathf.RoundToInt(lookaheadPoints * (1f + skillLevel));
        (Vector3 targetPoint, float recommendedSpeed) = racingLine.GetNextTargetPoint(currentWaypointIndex, adjustedLookahead);
        float cornerFactor = DetectUpcomingCorners();
        
        float idealTargetSpeed = recommendedSpeed;
        if (cornerFactor > 0)
        {
            idealTargetSpeed *= Mathf.Lerp(1.0f, cornerSpeedReductionFactor, Mathf.Pow(cornerFactor, 0.7f));
        }
        idealTargetSpeed *= Mathf.Lerp(minSpeedMultiplier, maxSpeedMultiplier, skillLevel);
        idealTargetSpeed *= (1f + aggressiveness * 0.2f);

        // --- 3. Check for overtaking and apply lateral offsets ---
        UpdateOvertakingLogic(idealTargetSpeed);
        currentOvertakeOffset = Mathf.Lerp(currentOvertakeOffset, targetOvertakeOffset, Time.deltaTime * 2f);

        // --- NEW: Dynamic Avoidance System (Smoothed) ---
        float rawAvoidanceOffset = CalculateDynamicAvoidanceOffset();
        
        // Smooth the avoidance offset to prevent sudden changes
        float avoidanceOffset = smoothSteeringController.SmoothLateralOffset(rawAvoidanceOffset);
        
        // Commitment logic: only change direction every X seconds (but smoother)
        if (Time.time - lastAvoidanceDecisionTime > avoidanceCommitmentDuration)
        {
            committedAvoidanceOffset = avoidanceOffset;
            avoidanceCommitmentTimer = avoidanceCommitmentDuration;
            lastAvoidanceDecisionTime = Time.time;
        }
        else
        {
            // Gradually blend towards new avoidance offset instead of sudden changes
            committedAvoidanceOffset = Mathf.Lerp(committedAvoidanceOffset, avoidanceOffset, Time.deltaTime * 1.5f);
            avoidanceCommitmentTimer -= Time.deltaTime;
        }

        // Apply behavior modifiers from personality system (smoothed)
        float effectivePathRandomness = pathRandomness + behaviorModifier_pathRandomness;
        
        // Waypoint-based randomness (prevents wiggling)
        // Only update random offset when we pass certain waypoints
        int waypointsSinceLastChange = Mathf.Abs(currentWaypointIndex - lastRandomnessWaypoint);
        if (waypointsSinceLastChange >= randomnessWaypointInterval || lastRandomnessWaypoint == 0)
        {
            lastRandomnessWaypoint = currentWaypointIndex;
            targetRandomOffset = Random.Range(-1f, 1f) * effectivePathRandomness;
        }
        
        // Smoothly transition to target offset
        currentRandomOffset = Mathf.Lerp(currentRandomOffset, targetRandomOffset, Time.deltaTime * 2f);
        
        // Adaptive path randomness: reduce on straight paths, maintain on curves
        float straightFactor = 1f - Mathf.Clamp01(cornerFactor * 2f);
        float adaptiveRandomOffset = currentRandomOffset * Mathf.Lerp(0.2f, 1.0f, straightFactor);

        // === STARTING GRID & PERSONALITY-BASED RACING LINES ===
        // Blend from starting grid lane to personality-based offset over time
        float timeSinceRaceStart = Time.time - raceStartTime;
        float gridToPersonalityBlend = Mathf.Clamp01(timeSinceRaceStart / startingGridDuration);
        
        // Calculate the strategic racing line offset
        float strategicLineOffset = Mathf.Lerp(startingGridLaneOffset, personalityBasedOffset, gridToPersonalityBlend);
        
        // Smooth and limit the total lateral offset to prevent sudden path changes
        float rawTotalLateralOffset = adaptiveRandomOffset + currentOvertakeOffset + committedAvoidanceOffset + strategicLineOffset;
        float totalLateralOffset = smoothSteeringController.SmoothLateralOffset(rawTotalLateralOffset);

        if (totalLateralOffset != 0)
        {
            int targetIndex = (currentWaypointIndex + adjustedLookahead) % racingLine.Points.Count;
            int nextPointIndex = (targetIndex + 1) % racingLine.Points.Count;
            Vector3 tangent = (racingLine.Points[nextPointIndex] - racingLine.Points[targetIndex]).normalized;
            Vector3 normal = Vector3.Cross(tangent, Vector3.up);
            targetPoint += normal * totalLateralOffset;
        }
        
        Vector3 worldTargetPoint = trackGenerator.transform.TransformPoint(targetPoint);
        
        // --- 4. Calculate final speed and steering with behavior modifiers ---
        float rawTargetSteer = GetTargetSteer(worldTargetPoint);
        
        // Apply smoothed steering behavior modifier
        float smoothedBehaviorModifier = smoothSteeringController.SmoothBehaviorModifier(behaviorModifier_steering);
        rawTargetSteer += smoothedBehaviorModifier;
        rawTargetSteer = Mathf.Clamp(rawTargetSteer, -maxSteeringAngle, maxSteeringAngle);
        
        // Check if this is an emergency avoidance situation
        bool isEmergencyAvoidance = false;
        if (otherVehicles.Count > 0)
        {
            foreach (var other in otherVehicles)
            {
                if (other != null && other.isActiveAndEnabled)
                {
                    float distance = Vector3.Distance(transform.position, other.transform.position);
                    if (smoothSteeringController.ShouldUseEmergencyAvoidance(other.transform.position, distance))
                    {
                        isEmergencyAvoidance = true;
                        break;
                    }
                }
            }
        }
        
        // Apply smooth steering to prevent sudden unrealistic turns
        float targetSteer = smoothSteeringController.SmoothSteeringInput(rawTargetSteer, isEmergencyAvoidance);
        
        // Final steering application with smoothing
        currentSteer = Mathf.Lerp(currentSteer, targetSteer, Time.deltaTime * steeringSpeed * (1f + skillLevel));
        float finalTargetSpeed = idealTargetSpeed * RubberBandingFactor;

        // Determine acceleration/braking
        float currentSpeed = vehicleController.carVelocity.magnitude / vehicleController.MaxSpeed;
        
        // Calculate how much we need to slow down based on corner factor - more gradual curve
        float cornerBrakingFactor = Mathf.Pow(cornerFactor, 0.8f) * 1.2f; // Less aggressive exponential curve
        
        // NEW: Enhanced braking when nitro is active or approaching sharp corners
        if (vehicleController.isNitroActive && cornerFactor > 0.1f)
        {
            // More aggressive braking when nitro is active and approaching corners
            cornerBrakingFactor *= 2.0f;
        }
        
        // NEW: Emergency braking for very sharp corners
        if (cornerFactor > 0.25f)
        {
            cornerBrakingFactor *= 1.5f;
        }
        
        // NEW: Two-phase braking approach for more realistic driving
        // Phase 1: Cut throttle when approaching corners or slightly over target speed
        // Phase 2: Apply brakes only when significantly over target speed or in sharp corners
        
        bool shouldAccelerate = currentSpeed < finalTargetSpeed && cornerFactor < 0.1f;
        bool needsThrottleCut = cornerFactor > 0.05f || currentSpeed > finalTargetSpeed * 1.05f;
        bool needsActiveBraking = currentSpeed > finalTargetSpeed * 1.15f || cornerFactor > 0.3f;
        
        // NEW: Enhanced braking triggers when nitro is active
        if (vehicleController.isNitroActive)
        {
            needsThrottleCut = cornerFactor > 0.03f || currentSpeed > finalTargetSpeed * 1.02f; // More sensitive
            needsActiveBraking = currentSpeed > finalTargetSpeed * 1.1f || cornerFactor > 0.2f; // More aggressive
        }
        
        if (shouldAccelerate)
        {
            // Need to accelerate - only when well below target speed and not approaching corners
            float accelerationFactor = 1.0f - (cornerFactor * 3.0f); // More aggressive acceleration reduction near corners
            accelerationFactor = Mathf.Max(0.05f, accelerationFactor); // Lower minimum acceleration
            
            // Apply throttle behavior modifier
            accelerationFactor += behaviorModifier_throttle;
            accelerationFactor = Mathf.Clamp01(accelerationFactor);
            
            currentAcceleration = Mathf.Lerp(currentAcceleration, accelerationFactor, Time.deltaTime * accelerationSpeed);
            currentBrake = Mathf.Lerp(currentBrake, 0f, Time.deltaTime * accelerationSpeed * 2.0f); // Release brakes quickly
        }
        else if (needsThrottleCut)
        {
            // Phase 1: Cut throttle to slow down naturally
            // This creates a more realistic "lift off" behavior before applying brakes
            
            // Cut throttle completely when approaching corners or over target speed
            currentAcceleration = Mathf.Lerp(currentAcceleration, 0f, Time.deltaTime * accelerationSpeed * 1.5f);
            
            // Only apply light braking if significantly over target speed
            float speedDifference = currentSpeed - finalTargetSpeed;
            float lightBrakingIntensity = 0f;
            
            if (speedDifference > 0.1f) // Only brake if 10% over target speed
            {
                lightBrakingIntensity = Mathf.Clamp01(speedDifference * brakingIntensityMultiplier * 0.3f); // Very light braking
            }
            
            currentBrake = Mathf.Lerp(currentBrake, lightBrakingIntensity, Time.deltaTime * accelerationSpeed * 1.5f);
        }
        else if (needsActiveBraking)
        {
            // Phase 2: Apply active braking when necessary
            float speedDifference = currentSpeed - finalTargetSpeed;
            
            // Calculate braking intensity based on speed difference and corner factor
            float brakingIntensity = Mathf.Clamp01(speedDifference * brakingIntensityMultiplier * 0.8f);
                
            // Add corner-based braking for sharp turns
            if (cornerFactor > 0.3f)
            {
                float cornerBraking = cornerBrakingFactor * 0.6f; // Gentler corner braking
                brakingIntensity = Mathf.Max(brakingIntensity, cornerBraking);
            }
            
            // Apply braking behavior modifier
            brakingIntensity += behaviorModifier_braking;
            brakingIntensity = Mathf.Clamp01(brakingIntensity);
            
            // Cut throttle completely when actively braking
            currentAcceleration = Mathf.Lerp(currentAcceleration, 0f, Time.deltaTime * accelerationSpeed * 2.0f);
            
            // Apply brakes gradually
            currentBrake = Mathf.Lerp(currentBrake, brakingIntensity, Time.deltaTime * accelerationSpeed * 1.5f);
        }
        else
        {
            // Coasting - gradually reduce inputs
            currentAcceleration = Mathf.Lerp(currentAcceleration, 0f, Time.deltaTime * accelerationSpeed);
            currentBrake = Mathf.Lerp(currentBrake, 0f, Time.deltaTime * accelerationSpeed);
        }
        
        // Use handbrake for sharp corners to prevent loss of control
        // NEW: Slam handbrake at sharp corners, release smoothly (player-like drift)
        if (cornerFactor > handbrakeThresholdRandomized)
        {
            // "Press" handbrake hard at the start of a sharp corner
            currentHandbrake = Mathf.Lerp(currentHandbrake, 1.0f, Time.deltaTime * 2f);
            // Drifting log removed - too frequent
        }
        else
        {
            // Release handbrake smoothly
            currentHandbrake = Mathf.Lerp(currentHandbrake, 0f, Time.deltaTime * 2f);
        }
        
        // --- Strategic AI Nitro System ---
        nitroDecisionCooldown -= Time.deltaTime;
        if (nitroSlowdownTimer > 0f) nitroSlowdownTimer -= Time.deltaTime;
        
        // Update race context for nitro decisions
        UpdateNitroRaceContext();
        
        // Determine current strategy
        UpdateNitroStrategy();
        
        // Make strategic nitro decision
        bool useNitro = false;
        if (nitroDecisionCooldown <= 0f)
        {
            useNitro = MakeStrategicNitroDecision();
            nitroDecisionCooldown = Random.Range(0.5f, 1.5f); // Check every 0.5-1.5 seconds
        }
        
        // Handle forced start nitro
        if (forceStartNitro)
        {
            startNitroTimer -= Time.deltaTime;
            float lookaheadCorner = GetLookaheadCornerFactor(1f);
            float doubleLookaheadCorner = GetLookaheadCornerFactor(2f);
            bool isSafeForNitro = lookaheadCorner < 0.1f && doubleLookaheadCorner < 0.1f;
            
            if (startNitroTimer > 0f && isSafeForNitro)
            {
                useNitro = true;
            }
            else
            {
                forceStartNitro = false;
            }
        }
        
        if (vehicleController.inputManager != null)
        {
            vehicleController.inputManager.SetAIInputs(currentSteer, currentAcceleration, currentHandbrake > 0f ? currentHandbrake : currentBrake, useNitro);
        }
        
        // After nitro, enforce slowdown to 90% of recommended speed
        if (isNitroSlowingDown)
        {
            if (nitroSlowdownTimer > 0f)
            {
                float slowdownTarget = recommendedSpeed * 0.9f;
                float slowdownCurrentSpeed = vehicleController.carVelocity.magnitude / vehicleController.MaxSpeed;
                if (slowdownCurrentSpeed > slowdownTarget)
                {
                    // Aggressively cut throttle and apply brakes
                    currentAcceleration = Mathf.Lerp(currentAcceleration, 0f, Time.deltaTime * 3f);
                    currentBrake = Mathf.Lerp(currentBrake, 1f, Time.deltaTime * 2f);
                }
            }
            else
            {
                isNitroSlowingDown = false;
            }
        }
        
        // Debug visualization
        if (showDebugInfo)
        {
            // Draw line to target point
            Debug.DrawLine(transform.position, worldTargetPoint, targetPointColor);
            
            // Draw racing line ahead
            int startIdx = currentWaypointIndex;
            int endIdx = (currentWaypointIndex + 20) % racingLine.Points.Count;
            for (int i = startIdx; i != endIdx; i = (i + 1) % racingLine.Points.Count)
            {
                int nextIdx = (i + 1) % racingLine.Points.Count;
                Vector3 start = trackGenerator.transform.TransformPoint(racingLine.Points[i]);
                Vector3 end = trackGenerator.transform.TransformPoint(racingLine.Points[nextIdx]);
                Debug.DrawLine(start, end, pathColor);
            }
            
            // Draw handbrake activation
            if (currentHandbrake > 0f)
            {
                Debug.DrawRay(transform.position + Vector3.up * 4.5f, Vector3.right * currentHandbrake * 5f, Color.magenta);
            }
            
            // Draw nitro lookahead (green line)
            int longLookahead = Mathf.RoundToInt(lookaheadPoints * 2.5f);
            int lookIdx = (currentWaypointIndex + longLookahead) % racingLine.Points.Count;
            Vector3 lookaheadPoint = trackGenerator.transform.TransformPoint(racingLine.Points[lookIdx]);
            Debug.DrawLine(transform.position + Vector3.up * 2f, lookaheadPoint + Vector3.up * 2f, Color.green);
            
            // Draw waypoint-based randomness
            if (showRandomnessDebug)
            {
                // Show current random offset as a line from car
                Vector3 offsetDirection = transform.right * currentRandomOffset;
                Debug.DrawRay(transform.position + Vector3.up * 2.5f, offsetDirection, Color.cyan, 0f, false);
                
                // Show target random offset
                Vector3 targetOffsetDirection = transform.right * targetRandomOffset;
                Debug.DrawRay(transform.position + Vector3.up * 3f, targetOffsetDirection, Color.yellow, 0f, false);
                
                // Show last randomness waypoint
                if (lastRandomnessWaypoint > 0 && lastRandomnessWaypoint < racingLine.Points.Count)
                {
                    Vector3 waypointPos = trackGenerator.transform.TransformPoint(racingLine.Points[lastRandomnessWaypoint]);
                    Debug.DrawLine(transform.position, waypointPos, Color.magenta);
                }
            }
        }

        // Defensive driving timer
        if (isDefending)
        {
            defenseTimer += Time.deltaTime;
            if (defenseTimer >= defenseDuration)
            {
                isDefending = false;
                vehicleController.Acceleration = originalAcceleration;
                // Defense log removed - too frequent
            }
        }

        // Universal Ramming System (all cars can ram)
        if (enableRamming && IsRacing)
        {
            rammingCooldownTimer -= Time.deltaTime;
            
            // Personality affects ramming frequency
            float personalityRammingChance = 1.0f;
            if (personalityManager?.GetPersonality() != null)
            {
                var personality = personalityManager.GetPersonality();
                // Aggressive/Hothead personalities ram more, Conservative/Veteran less
                personalityRammingChance = personality.aggression * personality.riskTaking;
            }
            
            if (rammingCooldownTimer <= 0f && Random.value < personalityRammingChance)
            {
                // Find ramming opportunity
                FindRammingTarget();
                
                if (rammingTarget != null)
                {
                    // Check if we're in a corner - reduce ramming in sharp corners
                    if (cornerFactor > 0.4f)
                    {
                        // Skip ramming in sharp corners
                        rammingCooldownTimer = rammingCooldown * 0.3f;
                        rammingTarget = null;
                    }
                    else
                    {
                        // Execute ramming maneuver
                        ExecuteRamming();
                        
                        // Reset cooldown (shorter for aggressive personalities)
                        float cooldownMultiplier = personalityRammingChance > 0.7f ? 0.7f : 1.0f;
                        rammingCooldownTimer = rammingCooldown * Random.Range(0.8f, 1.2f) * cooldownMultiplier;
                    }
                }
                else
                {
                    // No target found, check again soon
                    rammingCooldownTimer = rammingCooldown * 0.5f;
                }
            }
        }
    }

    private float GetTargetSteer(Vector3 worldTargetPoint)
    {
        // Calculate the steering angle needed to follow the racing line.
        Vector3 directionToTarget = worldTargetPoint - transform.position;
        Vector3 localDirection = transform.InverseTransformDirection(directionToTarget);
        float targetSteer = Mathf.Clamp(localDirection.x / localDirection.magnitude, -maxSteeringAngle, maxSteeringAngle);
        targetSteer *= Mathf.Lerp(1.5f, 1.0f, skillLevel);

        return targetSteer;
    }
    
    private void UpdateOvertakingLogic(float idealSpeedNormalized)
    {
        // Don't interrupt forced overtakes with natural overtaking logic
        if (isForcedOvertake && isOvertaking)
        {
            // Let forced overtake complete naturally
            if (carToOvertake == null || Vector3.Dot(transform.forward, carToOvertake.transform.position - transform.position) < 0)
            {
                // Forced overtake complete
                isOvertaking = false;
                isForcedOvertake = false;
                targetOvertakeOffset = 0f;
                overtakeCooldownTimer = overtakeCooldown;
                carToOvertake = null;
                vehicleController.Acceleration = originalAcceleration;
            }
            return;
        }
        
        if (overtakeCooldownTimer > 0)
        {
            overtakeCooldownTimer -= Time.deltaTime;
            return;
        }

        if (isOvertaking)
        {
            // Check if overtake is complete
            if (carToOvertake == null) 
            {
                isOvertaking = false;
                isForcedOvertake = false;
                targetOvertakeOffset = 0f;
                overtakeCooldownTimer = overtakeCooldown;
                carToOvertake = null;
                vehicleController.Acceleration = originalAcceleration;
                return;
            }

            Vector3 directionToTargetCar = carToOvertake.transform.position - transform.position;
            if (Vector3.Dot(transform.forward, directionToTargetCar) < 0)
            {
                isOvertaking = false;
                isForcedOvertake = false;
                targetOvertakeOffset = 0f;
                overtakeCooldownTimer = overtakeCooldown;
                carToOvertake = null;
                vehicleController.Acceleration = originalAcceleration;
            }
            return; 
        }

        // Natural overtaking logic
        AIVehicleController leadCar = FindCarToOvertake(idealSpeedNormalized);
        
        if (leadCar != null)
        {
            timeStuck += Time.deltaTime;
            
            if (timeStuck > overtakeTriggerTime)
            {
                isOvertaking = true;
                isForcedOvertake = false; // Natural overtake
                carToOvertake = leadCar;
                timeStuck = 0f;
                float boostFactor = Random.Range(1.1f, 1.3f);
                vehicleController.Acceleration = originalAcceleration * boostFactor;
                
                float overtakeDirection = (Random.value > 0.5f) ? 1f : -1f;
                targetOvertakeOffset = overtakeLaneOffset * overtakeDirection;

                leadCar.TryStartDefense(vehicleController.Acceleration);
            }
        }
        else
        {
            timeStuck = 0f;
        }
    }

    private AIVehicleController FindCarToOvertake(float idealSpeedNormalized)
    {
        float closestDistance = 15f; 
        AIVehicleController potentialTarget = null;

        foreach (var vehicle in otherVehicles)
        {
            if (vehicle == null || !vehicle.isActiveAndEnabled) continue;
            Vector3 directionToVehicle = vehicle.transform.position - transform.position;
            float distance = directionToVehicle.magnitude;
            
            // More lenient "in front" check - reduce from 0.8 to 0.6 (about 53 degrees)
            bool isInFront = Vector3.Dot(transform.forward, directionToVehicle.normalized) > 0.6f;

            if (isInFront && distance < closestDistance)
            {
                closestDistance = distance;
                potentialTarget = vehicle;
            }
        }

        bool isTargetValid = false;
        if (potentialTarget != null)
        {
            float desiredSpeed = idealSpeedNormalized * vehicleController.MaxSpeed;
            float targetSpeed = potentialTarget.vehicleController.carVelocity.magnitude;
            
            // More lenient speed comparison - consider target slower if we want to go 5% faster
            bool isSlower = desiredSpeed > targetSpeed * 1.05f;
            bool onStraight = DetectUpcomingCorners() < maxOvertakeCornerFactor;

            isTargetValid = isSlower && onStraight;
        }

        if (showDebugInfo)
        {
            foreach (var vehicle in otherVehicles)
            {
                if (vehicle == null || !vehicle.isActiveAndEnabled) continue;
                
                Color debugColor = Color.grey; 
                if (vehicle == potentialTarget)
                {
                    debugColor = isTargetValid ? Color.green : Color.yellow;
                }
                Debug.DrawLine(transform.position, vehicle.transform.position, debugColor);
            }
        }
        
        return isTargetValid ? potentialTarget : null;
    }

    /// <summary>
    /// Detects upcoming corners by analyzing multiple points ahead on the racing line
    /// Returns a value between 0 and 1 indicating the sharpness of upcoming corners
    /// </summary>
    private float DetectUpcomingCorners()
    {
        if (racingLine == null || racingLine.Points.Count == 0) return 0f;
        
        float maxCurvature = 0f;
        float distanceToCorner = float.MaxValue;
        Vector3 cornerPosition = Vector3.zero;
        
        // Look ahead multiple points to detect upcoming corners
        for (int i = 1; i <= cornerDetectionLookahead; i++)
        {
            int idx1 = currentWaypointIndex;
            int idx2 = (currentWaypointIndex + i) % racingLine.Points.Count;
            int idx3 = (currentWaypointIndex + i * 2) % racingLine.Points.Count;
            
            if (idx1 >= racingLine.Points.Count || idx2 >= racingLine.Points.Count || idx3 >= racingLine.Points.Count)
                continue;
                
            Vector3 p1 = racingLine.Points[idx1];
            Vector3 p2 = racingLine.Points[idx2];
            Vector3 p3 = racingLine.Points[idx3];
            
            // Calculate vectors between points
            Vector3 v1 = (p2 - p1).normalized;
            Vector3 v2 = (p3 - p2).normalized;
            
            // Calculate the angle between the vectors (indicates curvature)
            float dot = Vector3.Dot(v1, v2);
            // Normalize to 0-1 range, where 1 is a sharp 180-degree turn
            float curvature = 1f - (dot + 1f) / 2f; 
            
            // Only consider significant curves - lowered threshold to detect more corners
            if (curvature > cornerDetectionThreshold * 0.8f) // 20% lower threshold for initial detection
            {
                // Calculate approximate distance to this corner
                float distance = 0f;
                for (int j = 0; j < i; j++)
                {
                    int fromIdx = (currentWaypointIndex + j) % racingLine.Points.Count;
                    int toIdx = (currentWaypointIndex + j + 1) % racingLine.Points.Count;
                    distance += Vector3.Distance(racingLine.Points[fromIdx], racingLine.Points[toIdx]);
                }
                
                // Weight the curvature by distance - closer corners matter more
                // Reduced the divisor to make distance weighting more significant
                float distanceWeight = Mathf.Clamp01(1.0f - (distance / (brakingDistance * 5f))); // More aggressive distance weighting
                float weightedCurvature = curvature * distanceWeight;
                
                // Keep track of the sharpest corner and its distance
                if (weightedCurvature > maxCurvature)
                {
                    maxCurvature = weightedCurvature;
                    distanceToCorner = distance;
                    cornerPosition = trackGenerator.transform.TransformPoint(p2); // Store corner position for visualization
                }
            }
        }
        
        // Apply braking distance factor - start slowing down earlier for sharper corners
        // Increased the influence of braking distance to start slowing down earlier
        float cornerFactor = maxCurvature * Mathf.Clamp01((brakingDistance * 2.5f) / Mathf.Max(0.1f, distanceToCorner));
        
        // Apply skill level - better drivers anticipate corners better
        cornerFactor *= Mathf.Lerp(1.5f, 1.0f, skillLevel);
        
        // Enhanced debug visualization for corner detection
        if (showDebugInfo && cornerFactor > 0.05f) // Lower threshold to see more corner detections
        {            
            // Draw a line to the detected corner position
            Debug.DrawLine(transform.position, cornerPosition, Color.red);
            
            // Display the corner factor as text above the vehicle
            Debug.DrawRay(transform.position + Vector3.up * 2f, Vector3.up * cornerFactor * 5f, Color.red);
            
            // Draw a sphere at the corner position
            Debug.DrawRay(cornerPosition, Vector3.up * 2f, Color.red);
            Debug.DrawRay(cornerPosition, Vector3.right * 2f, Color.red);
            Debug.DrawRay(cornerPosition, Vector3.forward * 2f, Color.red);
        }
        
        return Mathf.Clamp01(cornerFactor);
    }

    // Defensive driving: called by overtaking car
    public void TryStartDefense(float attackerAcceleration)
    {
        if (isDefending) return; // Already defending

        if (Random.value < 0.5f) // 50% chance
        {
            isDefending = true;
            defenseDuration = Random.Range(5f, 10f);
            defenseTimer = 0f;
            vehicleController.Acceleration = attackerAcceleration;
            // Defense log removed - too frequent
        }
    }

    // Force an overtake attempt on a specific car (called by AIRaceManager)
    public void ForceOvertake(AIVehicleController target)
    {
        if (target == null) return;
        
        // Allow forced overtakes to override natural overtaking
        // But don't interrupt an existing forced overtake of the same target
        if (isForcedOvertake && carToOvertake == target) return;
        
        isOvertaking = true;
        isForcedOvertake = true; // Mark as forced - has priority
        carToOvertake = target;
        timeStuck = 0f;
        float boostFactor = Random.Range(1.1f, 1.3f);
        vehicleController.Acceleration = originalAcceleration * boostFactor;
        float overtakeDirection = (Random.value > 0.5f) ? 1f : -1f;
        targetOvertakeOffset = overtakeLaneOffset * overtakeDirection;
        overtakeCooldownTimer = 0f; // Reset cooldown for forced overtakes
        
        // Ask the car being overtaken to defend
        target.TryStartDefense(vehicleController.Acceleration);
    }

    // Public method to stop the AI car (called when race finishes)
    public void StopCar()
    {
        IsRacing = false;
        if (vehicleController.inputManager != null)
        {
            vehicleController.inputManager.SetAIInputs(0f, 0f, 1f, false); // Full brake
        }
    }
    
    /// <summary>
    /// Set the starting grid lane offset for this AI car (called by race manager at spawn)
    /// </summary>
    public void SetStartingGridLane(float laneOffset)
    {
        startingGridLaneOffset = laneOffset;
    }
    
    /// <summary>
    /// Calculate personality-based racing line offset
    /// </summary>
    private void CalculatePersonalityOffset()
    {
        if (hasCalculatedPersonalityOffset) return;
        
        var personality = personalityManager?.GetPersonality();
        if (personality == null)
        {
            personalityBasedOffset = 0f;
            hasCalculatedPersonalityOffset = true;
            return;
        }
        
        // Different personalities prefer different racing lines
        switch (personality.personalityType)
        {
            case AIPersonalityData.PersonalityType.Aggressive:
            case AIPersonalityData.PersonalityType.Hothead:
                // Aggressive drivers take the inside line (tighter, riskier)
                personalityBasedOffset = Random.Range(-personalityLineOffset, -personalityLineOffset * 0.5f);
                break;
                
            case AIPersonalityData.PersonalityType.Conservative:
            case AIPersonalityData.PersonalityType.Rookie:
                // Conservative/Rookie drivers take the outside line (safer, wider)
                personalityBasedOffset = Random.Range(personalityLineOffset * 0.5f, personalityLineOffset);
                break;
                
            case AIPersonalityData.PersonalityType.Veteran:
                // Veterans stick close to optimal racing line
                personalityBasedOffset = Random.Range(-0.5f, 0.5f);
                break;
                
            case AIPersonalityData.PersonalityType.Blocker:
                // Blockers take defensive middle line
                personalityBasedOffset = Random.Range(-1f, 1f);
                break;
                
            case AIPersonalityData.PersonalityType.Speedster:
                // Speedsters vary their line for overtaking opportunities
                personalityBasedOffset = Random.Range(-personalityLineOffset * 0.7f, personalityLineOffset * 0.7f);
                break;
                
            case AIPersonalityData.PersonalityType.Opportunist:
                // Opportunists adapt their line
                personalityBasedOffset = Random.Range(-personalityLineOffset * 0.6f, personalityLineOffset * 0.6f);
                break;
                
            default:
                personalityBasedOffset = 0f;
                break;
        }
        
        hasCalculatedPersonalityOffset = true;
    }

    // --- NEW: Dynamic Threat Field Avoidance System (Improved) ---
    private float CalculateDynamicAvoidanceOffset()
    {
        float totalOffset = 0f;
        float totalThreat = 0f;
        Vector3 myPos = transform.position;
        Vector3 myFwd = transform.forward;
        float mySpeed = vehicleController.carVelocity.magnitude;

        foreach (var other in otherVehicles)
        {
            if (other == null || !other.isActiveAndEnabled) continue;
            Vector3 toOther = other.transform.position - myPos;
            float distance = toOther.magnitude;
            if (distance > avoidanceDetectionRadius) continue;
            float angle = Vector3.Angle(myFwd, toOther);
            if (angle > avoidanceAwarenessAngle * 0.5f) continue;

            // Threat score: closer, more in front, and higher relative speed = higher threat
            float relSpeed = Vector3.Dot(other.vehicleController.carVelocity - vehicleController.carVelocity, myFwd);
            float threat = Mathf.Lerp(1.0f, 0.1f, distance / avoidanceDetectionRadius);
            threat *= Mathf.Lerp(1.0f, 0.2f, angle / (avoidanceAwarenessAngle * 0.5f));
            threat *= 1.0f + Mathf.Clamp01(relSpeed / Mathf.Max(1f, mySpeed));

            // Aggression: aggressive AIs tolerate more, cautious AIs avoid more
            float aggressionFactor = Mathf.Lerp(1.2f, 0.7f, aggressiveness);
            threat *= aggressionFactor;

            // IMPROVED: Reduce avoidance strength to prevent sudden turns
            Vector3 localToOther = transform.InverseTransformPoint(other.transform.position);
            float side = Mathf.Sign(localToOther.x);
            
            // Reduced offset range and made it more gradual
            float baseOffset = Mathf.Lerp(0.8f, 2.0f, threat); // Reduced from 1.5f-3.0f
            
            // Apply distance-based reduction - closer cars get more avoidance
            float distanceFactor = Mathf.InverseLerp(avoidanceDetectionRadius, 2f, distance);
            baseOffset *= distanceFactor;
            
            // Reduce avoidance when in corners to prevent track departure
            float cornerFactor = DetectUpcomingCorners();
            if (cornerFactor > 0.3f)
            {
                baseOffset *= Mathf.Lerp(1f, 0.4f, cornerFactor); // Reduce avoidance in corners
            }
            
            float offset = side * baseOffset;
            totalOffset += offset * threat;
            totalThreat += threat;

            // Debug: draw threat lines
            if (showDebugInfo)
            {
                Debug.DrawLine(myPos + Vector3.up * 2f, other.transform.position + Vector3.up * 2f, Color.red);
                Debug.DrawRay(other.transform.position + Vector3.up * 2f, Vector3.up * threat * 2f, Color.magenta);
            }
        }

        // Average the offset by total threat
        float avoidanceOffset = (totalThreat > 0f) ? totalOffset / totalThreat : 0f;

        // If overtaking, bias to the chosen overtake side (but less aggressively)
        if (isOvertaking && carToOvertake != null)
        {
            avoidanceOffset += Mathf.Sign(targetOvertakeOffset) * 0.5f; // Reduced from 1.0f
        }

        // Clamp avoidance offset to reasonable limits
        avoidanceOffset = Mathf.Clamp(avoidanceOffset, -3.5f, 3.5f);
        return avoidanceOffset;
    }
    
    public float GetLookaheadCornerFactor(float multiplier)
    {
        if (racingLine == null || racingLine.Points.Count == 0) return 0f;
        int lookahead = Mathf.RoundToInt(lookaheadPoints * multiplier);
        int idx1 = currentWaypointIndex;
        int idx2 = (currentWaypointIndex + lookahead) % racingLine.Points.Count;
        int idx3 = (currentWaypointIndex + lookahead * 2) % racingLine.Points.Count;
        if (idx1 >= racingLine.Points.Count || idx2 >= racingLine.Points.Count || idx3 >= racingLine.Points.Count)
            return 0f;
        Vector3 p1 = racingLine.Points[idx1];
        Vector3 p2 = racingLine.Points[idx2];
        Vector3 p3 = racingLine.Points[idx3];
        Vector3 v1 = (p2 - p1).normalized;
        Vector3 v2 = (p3 - p2).normalized;
        float dot = Vector3.Dot(v1, v2);
        float curvature = 1f - (dot + 1f) / 2f;
        return Mathf.Clamp01(curvature);
    }
    
    // Public methods for enhanced AI systems
    public float GetCornerFactor()
    {
        return DetectUpcomingCorners();
    }
    
    public float GetCurrentSteerInput() => currentSteer;
    public float GetCurrentAccelerationInput() => currentAcceleration;
    public float GetCurrentBrakeInput() => currentHandbrake > 0f ? currentHandbrake : currentBrake;
    
    // Personality setters - clean alternative to reflection
    public void SetSkillLevel(float skill) 
    {
        skillLevel = Mathf.Clamp01(skill);
        baseSkillLevel = skillLevel; // Update base value when personality is set
    }
    
    public void SetAggressiveness(float aggro) 
    {
        aggressiveness = Mathf.Clamp01(aggro);
        baseAggressiveness = aggressiveness; // Update base value when personality is set
    }
    
    public void SetRiskTaking(float risk)
    {
        // Risk taking could affect corner speed and overtaking behavior
        // For now, we can map it to existing parameters or store it
        // This is a placeholder for future risk-based behavior
    }
    
    // Behavior modification methods for personality system
    public void ModifyBehavior(string behaviorType, float modifier)
    {
        switch (behaviorType.ToLower())
        {
            case "braking":
                behaviorModifier_braking = modifier;
                break;
            case "steering":
                behaviorModifier_steering = modifier;
                break;
            case "throttle":
                behaviorModifier_throttle = modifier;
                break;
            case "aggression":
                behaviorModifier_aggression = modifier;
                aggressiveness = Mathf.Clamp01(baseAggressiveness + modifier); // Use base value!
                break;
            case "blocking":
                behaviorModifier_blocking = modifier;
                break;
            case "speed":
                behaviorModifier_speed = modifier;
                maxSpeedMultiplier = Mathf.Clamp01(baseMaxSpeedMultiplier + modifier); // Use base value!
                break;
            case "defense":
                behaviorModifier_defense = modifier;
                break;
            case "pathrandomness":
                behaviorModifier_pathRandomness = modifier;
                pathRandomness = Mathf.Max(0f, basePathRandomness + modifier); // Use base value!
                break;
        }
    }
    
    public void ResetBehaviorModifiers()
    {
        behaviorModifier_braking = 0f;
        behaviorModifier_steering = 0f;
        behaviorModifier_throttle = 0f;
        behaviorModifier_aggression = 0f;
        behaviorModifier_blocking = 0f;
        behaviorModifier_speed = 0f;
        behaviorModifier_defense = 0f;
        behaviorModifier_pathRandomness = 0f;
        
        // Reset smooth steering state when behavior is reset
        if (smoothSteeringController != null)
        {
            smoothSteeringController.ResetSmoothingState();
        }
        
        // Reset modified values to base values
        aggressiveness = baseAggressiveness;
        skillLevel = baseSkillLevel;
        maxSpeedMultiplier = baseMaxSpeedMultiplier;
        pathRandomness = basePathRandomness;
    }
    
    // === STRATEGIC NITRO SYSTEM METHODS ===
    
    private void DetermineInitialNitroStrategy()
    {
        if (personalityManager?.GetPersonality() == null) return;
        
        var personality = personalityManager.GetPersonality();
        
        if (personality.nitroAggression > 0.7f)
            currentNitroStrategy = NitroStrategy.Aggressive;
        else if (personality.nitroDefense > 0.7f)
            currentNitroStrategy = NitroStrategy.Defensive;
        else if (personality.nitroConservation > 0.7f)
            currentNitroStrategy = NitroStrategy.Conservative;
        else
            currentNitroStrategy = NitroStrategy.Opportunistic;
    }
    
    private void UpdateNitroRaceContext()
    {
        // Use shared race context manager for efficient context updates
        if (RaceContextManager.Instance != null)
        {
            var context = RaceContextManager.Instance.GetRaceContext(this);
            currentPosition = context.position;
            totalRacers = context.totalRacers;
            hasOpportunityAhead = context.hasOpportunityAhead;
        }
        else
        {
            // Fallback to direct calculations
            var raceManager = FindFirstObjectByType<AIRaceManager>();
            if (raceManager != null)
            {
                currentPosition = GetCurrentNitroPosition(raceManager);
                totalRacers = raceManager.SortedRacers.Count + 1;
            }
            
            hasOpportunityAhead = HasOvertakingOpportunity();
        }
    }
    
    private void UpdateNitroStrategy()
    {
        if (personalityManager?.GetPersonality() == null) return;
        
        var personality = personalityManager.GetPersonality();
        
        // Adapt strategy based on race situation, but respect personality traits
        if (currentPosition > totalRacers * 0.7f && RaceProgress > 0.5f)
        {
            // Far behind in late race - but respect conservative personalities
            if (personality.nitroConservation > 0.7f)
            {
                // Conservative personalities stay opportunistic even when desperate
                currentNitroStrategy = NitroStrategy.Opportunistic;
            }
            else
            {
                currentNitroStrategy = NitroStrategy.Desperate;
            }
        }
        else if (hasOpportunityAhead && personality.overtakingAggression > 0.6f)
        {
            // Opportunity to overtake - go aggressive
            currentNitroStrategy = NitroStrategy.Aggressive;
        }
        else if (RaceProgress < 0.3f)
        {
            // Early race - respect personality's natural strategy
            if (personality.nitroAggression > 0.7f)
                currentNitroStrategy = NitroStrategy.Aggressive;
            else if (personality.nitroDefense > 0.7f)
                currentNitroStrategy = NitroStrategy.Defensive;
            else if (personality.nitroConservation > 0.7f)
                currentNitroStrategy = NitroStrategy.Conservative;
            else
                currentNitroStrategy = NitroStrategy.Opportunistic;
        }
        else
        {
            // Mid-race - blend situation with personality
            if (personality.nitroAggression > 0.6f && hasOpportunityAhead)
                currentNitroStrategy = NitroStrategy.Aggressive;
            else if (personality.nitroDefense > 0.6f)
                currentNitroStrategy = NitroStrategy.Defensive;
            else
                currentNitroStrategy = NitroStrategy.Opportunistic;
        }
    }
    
    private bool MakeStrategicNitroDecision()
    {
        if (!CanUseStrategicNitro()) return false;
        
        bool shouldUseNitro = false;
        string reason = "";
        
        switch (currentNitroStrategy)
        {
            case NitroStrategy.Conservative:
                shouldUseNitro = ShouldUseConservativeNitro(out reason);
                break;
                
            case NitroStrategy.Aggressive:
                shouldUseNitro = ShouldUseAggressiveNitro(out reason);
                break;
                
            case NitroStrategy.Defensive:
                shouldUseNitro = ShouldUseDefensiveNitro(out reason);
                break;
                
            case NitroStrategy.Opportunistic:
                shouldUseNitro = ShouldUseOpportunisticNitro(out reason);
                break;
                
            case NitroStrategy.Desperate:
                shouldUseNitro = ShouldUseDesperateNitro(out reason);
                break;
        }
        
        if (shouldUseNitro)
        {
            lastNitroUseTime = Time.time;
            nitroSlowdownTimer = nitroSlowdownDuration;
            isNitroSlowingDown = true;
            // Nitro usage log removed - too frequent (every nitro use)
        }
        
        return shouldUseNitro;
    }
    
    private bool ShouldUseConservativeNitro(out string reason)
    {
        reason = "";
        
        // Only use nitro in key situations
        if (RaceProgress > 0.8f && currentPosition > 3)
        {
            reason = "Final push for better position";
            return Random.value < 0.6f;
        }
        
        if (hasOpportunityAhead && IsOnStraight() && Random.value < 0.3f)
        {
            reason = "Clear overtaking opportunity on straight";
            return true;
        }
        
        return false;
    }
    
    private bool ShouldUseAggressiveNitro(out string reason)
    {
        reason = "";
        var personality = personalityManager?.GetPersonality();
        
        // Use nitro aggressively for overtaking
        if (hasOpportunityAhead && IsOnStraight())
        {
            reason = "Aggressive overtaking attempt";
            float aggressionBonus = personality?.nitroAggression ?? 0.5f;
            return Random.value < (0.7f + aggressionBonus * 0.3f);
        }
        
        // Use nitro to break away from pack
        if (IsInTrafficPack() && IsOnStraight())
        {
            reason = "Breaking away from traffic pack";
            return Random.value < 0.5f;
        }
        
        return false;
    }
    
    private bool ShouldUseDefensiveNitro(out string reason)
    {
        reason = "";
        var personality = personalityManager?.GetPersonality();
        
        // Use nitro to defend position
        if (IsOnStraight())
        {
            reason = "Defending position";
            float defenseBonus = personality?.nitroDefense ?? 0.5f;
            return Random.value < (0.6f + defenseBonus * 0.4f);
        }
        
        // Use nitro to prevent being overtaken
        if (IsAboutToBeOvertaken())
        {
            reason = "Preventing imminent overtake";
            return Random.value < 0.8f;
        }
        
        return false;
    }
    
    private bool ShouldUseOpportunisticNitro(out string reason)
    {
        reason = "";
        
        // Wait for perfect opportunities
        if (hasOpportunityAhead && IsOnLongStraight())
        {
            reason = "Perfect opportunity on long straight";
            return Random.value < 0.7f;
        }
        
        // Use nitro when multiple cars are close for maximum effect
        if (IsInTrafficPack() && IsOnStraight())
        {
            reason = "Opportunistic move in traffic pack";
            return Random.value < 0.4f;
        }
        
        return false;
    }
    
    private bool ShouldUseDesperateNitro(out string reason)
    {
        reason = "";
        
        // Use nitro more frequently when desperate
        if (IsOnStraight() && Random.value < 0.8f)
        {
            reason = "Desperate attempt to catch up";
            return true;
        }
        
        // Even use nitro in less ideal conditions
        if (hasOpportunityAhead && Random.value < 0.9f)
        {
            reason = "Desperate overtaking attempt";
            return true;
        }
        
        return false;
    }
    
    private bool CanUseStrategicNitro()
    {
        return vehicleController.currentNitro > 10f && 
               !vehicleController.isNitroActive && 
               !vehicleController.isNitroCooldown &&
               Time.time - lastNitroUseTime > 1f; // Minimum 1 second between uses
    }
    
    private bool IsOnStraight()
    {
        // Check if current track section is relatively straight
        float cornerFactor = GetCornerFactor();
        return cornerFactor < 0.2f;
    }
    
    private bool IsOnLongStraight()
    {
        // Check for longer straight sections
        float cornerFactor = GetCornerFactor();
        float lookaheadCorner = GetLookaheadCornerFactor(2f);
        return cornerFactor < 0.1f && lookaheadCorner < 0.1f;
    }
    
    
    private bool HasOvertakingOpportunity()
    {
        // Check if there are cars ahead that can be overtaken
        var nearbyVehicles = GetNearbyVehicles();
        foreach (var vehicle in nearbyVehicles)
        {
            Vector3 relativePos = transform.InverseTransformPoint(vehicle.transform.position);
            if (relativePos.z > 0 && Vector3.Distance(transform.position, vehicle.transform.position) < 25f)
            {
                // Car is ahead and within overtaking range
                return true;
            }
        }
        return false;
    }
    
    private bool IsAboutToBeOvertaken()
    {
        var nearbyVehicles = GetNearbyVehicles();
        foreach (var vehicle in nearbyVehicles)
        {
            Vector3 relativePos = transform.InverseTransformPoint(vehicle.transform.position);
            if (relativePos.z < 0 && Vector3.Distance(transform.position, vehicle.transform.position) < 8f)
            {
                // Car is very close behind
                Rigidbody vehicleRb = vehicleController.GetComponent<Rigidbody>();
                Rigidbody otherRb = vehicle.GetComponent<Rigidbody>();
                if (vehicleRb != null && otherRb != null)
                {
                    Vector3 relativeVelocity = vehicleRb.linearVelocity - otherRb.linearVelocity;
                    if (Vector3.Dot(relativeVelocity, transform.forward) < -2f)
                    {
                        // Car behind is gaining fast
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    private bool IsInTrafficPack()
    {
        // Check if surrounded by multiple cars
        var nearbyVehicles = GetNearbyVehicles();
        int carsNearby = 0;
        foreach (var vehicle in nearbyVehicles)
        {
            if (Vector3.Distance(transform.position, vehicle.transform.position) < 20f)
            {
                carsNearby++;
            }
        }
        return carsNearby >= 2;
    }
    
    private int GetCurrentNitroPosition(AIRaceManager raceManager)
    {
        if (raceManager.SortedRacers.Contains(this))
        {
            return raceManager.SortedRacers.IndexOf(this) + 1;
        }
        return 1;
    }
    
    private List<AIVehicleController> GetNearbyVehicles()
    {
        var nearby = new List<AIVehicleController>();
        if (otherVehicles != null)
        {
            foreach (var vehicle in otherVehicles)
            {
                if (vehicle != null && vehicle != this)
                {
                    nearby.Add(vehicle);
                }
            }
        }
        return nearby;
    }
    
    // === RAMMING SYSTEM METHODS ===
    
    private void FindRammingTarget()
    {
        rammingTarget = null;
        float closestDist = 999f;
        rammingType = "";
        
        foreach (var other in otherVehicles)
        {
            if (other == null || !other.isActiveAndEnabled) continue;
            
            Vector3 toOther = other.transform.position - transform.position;
            float dist = toOther.magnitude;
            
            if (dist > rammingDetectionRange) continue;
            
            float sideDot = Vector3.Dot(transform.right, toOther.normalized);
            float forwardDot = Vector3.Dot(transform.forward, toOther.normalized);
            
            // Priority 1: Side ramming (beside the car)
            if (Mathf.Abs(sideDot) > 0.6f && Mathf.Abs(forwardDot) < 0.6f && dist < 10f && dist < closestDist)
            {
                rammingTarget = other;
                closestDist = dist;
                rammingType = "side";
            }
            // Priority 2: Back ramming (car ahead)
            else if (rammingType != "side" && forwardDot > 0.6f && dist > 3f && dist < closestDist)
            {
                rammingTarget = other;
                closestDist = dist;
                rammingType = "back";
            }
            // Priority 3: Front ramming (car behind, defensive)
            else if (rammingType == "" && forwardDot < -0.6f && dist < 8f && dist < closestDist)
            {
                rammingTarget = other;
                closestDist = dist;
                rammingType = "front";
            }
        }
    }
    
    private void ExecuteRamming()
    {
        if (rammingTarget == null) return;
        
        Vector3 toTarget = rammingTarget.transform.position - transform.position;
        Vector3 localToTarget = transform.InverseTransformDirection(toTarget);
        
        float intensity = rammingAggressiveness;
        
        // Apply personality modifiers
        if (personalityManager != null && personalityManager.GetPersonality() != null)
        {
            var personality = personalityManager.GetPersonality();
            intensity *= personality.aggression;
        }
        
        switch (rammingType)
        {
            case "side":
                // Steer toward the target
                float steerDir = Mathf.Sign(localToTarget.x);
                float targetRamSteer = steerDir * maxSteeringAngle * intensity;
                currentSteer = Mathf.Lerp(currentSteer, targetRamSteer, Time.deltaTime * 4f);
                
                // Accelerate into the ram
                currentAcceleration = Mathf.Lerp(currentAcceleration, 1f + intensity * 0.2f, Time.deltaTime * 3f);
                break;
                
            case "back":
                // Accelerate to catch up and ram from behind
                currentAcceleration = Mathf.Lerp(currentAcceleration, 1f + intensity * 0.3f, Time.deltaTime * 3f);
                
                // Slight steering adjustment to aim at target
                float backSteerDir = Mathf.Sign(localToTarget.x);
                float backSteerAmount = backSteerDir * maxSteeringAngle * 0.3f * intensity;
                currentSteer = Mathf.Lerp(currentSteer, backSteerAmount, Time.deltaTime * 2f);
                break;
                
            case "front":
                // Defensive ramming - brake and block
                currentBrake = Mathf.Lerp(currentBrake, 0.5f * intensity, Time.deltaTime * 2f);
                
                // Weave to block
                float frontSteerDir = Mathf.Sign(localToTarget.x);
                float frontSteerAmount = frontSteerDir * maxSteeringAngle * 0.4f * intensity;
                currentSteer = Mathf.Lerp(currentSteer, frontSteerAmount, Time.deltaTime * 3f);
                break;
        }
    }
}
