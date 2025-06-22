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
    
    [SerializeField, Tooltip("Whether the AI should use nitro on straight sections of the track")]
    private bool useNitroOnStraights = true;
    
    [SerializeField, Range(0f, 1f), Tooltip("Minimum recommended speed threshold for using nitro. Higher values mean nitro is only used on longer straights")]
    private float nitroThreshold = 0.85f;
    
    [SerializeField, Range(0f, 1f), Tooltip("How strongly the AI avoids other vehicles. Higher values cause more aggressive avoidance maneuvers")]
    private float avoidanceStrength = 0.4f;
    
    [SerializeField, Range(1f, 10f), Tooltip("Maximum distance at which the AI will start avoiding other vehicles. Higher values make the AI start avoiding earlier")]
    private float avoidanceDistance = 4f;
    
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
    
    [SerializeField, Range(0.1f, 1f), Tooltip("Corner factor threshold at which the AI will start using handbrake. Higher values mean handbrake is used only in sharper corners")]
    private float handbrakeThreshold = 0.25f;
    
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
    
    // Private variables
    private SimcadeVehicleController vehicleController;
    private RacingLine racingLine;
    private int currentWaypointIndex = 0;
    private float currentSteer = 0f;
    private float currentAcceleration = 0f;
    private float currentBrake = 0f;
    private float currentHandbrake = 0f;
    private bool isUsingNitro = false;
    private List<AIVehicleController> otherVehicles = new List<AIVehicleController>();
    
    private void Awake()
    {
        vehicleController = GetComponent<SimcadeVehicleController>();
        
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
    }
    
    private void Update()
    {
        if (racingLine == null || racingLine.Points.Count == 0) return;
        
        // Get current position in local space of track
        Vector3 localPosition = trackGenerator.transform.InverseTransformPoint(transform.position);
        
        // Find closest point on racing line
        (currentWaypointIndex, _, _) = racingLine.GetClosestPoint(localPosition);
        
        // Get target point ahead on racing line
        int adjustedLookahead = Mathf.RoundToInt(lookaheadPoints * (1f + skillLevel));
        (Vector3 targetPoint, float recommendedSpeed) = racingLine.GetNextTargetPoint(currentWaypointIndex, adjustedLookahead);
        
        // Convert target point to world space
        Vector3 worldTargetPoint = trackGenerator.transform.TransformPoint(targetPoint);
        
        // Calculate steering direction
        Vector3 directionToTarget = worldTargetPoint - transform.position;
        Vector3 localDirection = transform.InverseTransformDirection(directionToTarget);
        float targetSteer = Mathf.Clamp(localDirection.x / localDirection.magnitude, -maxSteeringAngle, maxSteeringAngle);
        
        // Apply skill level to steering precision
        targetSteer *= Mathf.Lerp(1.5f, 1.0f, skillLevel); // Less skilled drivers oversteer
        
        // Adjust for other vehicles (collision avoidance)
        Vector3 avoidanceVector = CalculateAvoidanceVector();
        Vector3 localAvoidance = transform.InverseTransformDirection(avoidanceVector);
        targetSteer += localAvoidance.x * avoidanceStrength;
        
        // Smooth steering
        currentSteer = Mathf.Lerp(currentSteer, targetSteer, Time.deltaTime * steeringSpeed * (1f + skillLevel));
        
        // Detect upcoming corners by analyzing multiple points ahead
        float cornerFactor = DetectUpcomingCorners();
        
        // Add debug log to monitor corner detection
        if (cornerFactor > 0.1f && showDebugInfo)
        {
            Debug.Log($"Car {gameObject.name}: Detected corner with factor {cornerFactor:F2}");
        }
        
        // Calculate target speed based on recommended speed from racing line
        float targetSpeed = recommendedSpeed;
        
        // Apply corner speed reduction if approaching a sharp turn - more aggressive reduction
        if (cornerFactor > 0)
        {
            // Reduce speed based on corner sharpness - more aggressive curve
            float cornerSpeedReduction = Mathf.Lerp(1.0f, cornerSpeedReductionFactor, Mathf.Pow(cornerFactor, 0.7f));
            targetSpeed *= cornerSpeedReduction;
            
            // Add debug visualization for target speed reduction
            if (showDebugInfo)
            {
                Debug.DrawRay(transform.position + Vector3.up * 5f, Vector3.right * cornerSpeedReduction * 3f, Color.magenta);
            }
        }
        
        // Apply skill level to speed management
        float speedMultiplier = Mathf.Lerp(minSpeedMultiplier, maxSpeedMultiplier, skillLevel);
        targetSpeed *= speedMultiplier;
        
        // Adjust for aggressiveness
        targetSpeed *= (1f + aggressiveness * 0.2f); // More aggressive drivers go faster
        
        // Determine acceleration/braking
        float currentSpeed = vehicleController.carVelocity.magnitude / vehicleController.MaxSpeed;
        
        // Debug current speed vs target speed
        if (showDebugInfo)
        {
            Debug.DrawRay(transform.position + Vector3.up * 3f, Vector3.right * currentSpeed * 5f, Color.green);
            Debug.DrawRay(transform.position + Vector3.up * 3.5f, Vector3.right * targetSpeed * 5f, Color.yellow);
        }
        
        // Calculate how much we need to slow down based on corner factor - more gradual curve
        float cornerBrakingFactor = Mathf.Pow(cornerFactor, 0.8f) * 1.2f; // Less aggressive exponential curve
        
        // NEW: Two-phase braking approach for more realistic driving
        // Phase 1: Cut throttle when approaching corners or slightly over target speed
        // Phase 2: Apply brakes only when significantly over target speed or in sharp corners
        
        bool shouldAccelerate = currentSpeed < targetSpeed && cornerFactor < 0.1f; // Lower threshold for acceleration
        bool needsThrottleCut = cornerFactor > 0.05f || currentSpeed > targetSpeed * 1.05f; // Start cutting throttle earlier
        bool needsActiveBraking = currentSpeed > targetSpeed * 1.15f || cornerFactor > 0.3f; // Higher threshold for actual braking
        
        if (shouldAccelerate)
        {
            // Need to accelerate - only when well below target speed and not approaching corners
            float accelerationFactor = 1.0f - (cornerFactor * 3.0f); // More aggressive acceleration reduction near corners
            accelerationFactor = Mathf.Max(0.05f, accelerationFactor); // Lower minimum acceleration
            
            currentAcceleration = Mathf.Lerp(currentAcceleration, accelerationFactor, Time.deltaTime * accelerationSpeed);
            currentBrake = Mathf.Lerp(currentBrake, 0f, Time.deltaTime * accelerationSpeed * 2.0f); // Release brakes quickly
            
            // Use nitro on straights if available and not approaching a corner
            if (useNitroOnStraights && recommendedSpeed > nitroThreshold && cornerFactor < 0.08f && !vehicleController.isNitroCooldown)
            {
                isUsingNitro = true;
            }
            else
            {
                isUsingNitro = false;
            }
        }
        else if (needsThrottleCut)
        {
            // Phase 1: Cut throttle to slow down naturally
            // This creates a more realistic "lift off" behavior before applying brakes
            
            // Cut throttle completely when approaching corners or over target speed
            currentAcceleration = Mathf.Lerp(currentAcceleration, 0f, Time.deltaTime * accelerationSpeed * 1.5f);
            
            // Only apply light braking if significantly over target speed
            float speedDifference = currentSpeed - targetSpeed;
            float lightBrakingIntensity = 0f;
            
            if (speedDifference > 0.1f) // Only brake if 10% over target speed
            {
                lightBrakingIntensity = Mathf.Clamp01(speedDifference * brakingIntensityMultiplier * 0.3f); // Very light braking
            }
            
            currentBrake = Mathf.Lerp(currentBrake, lightBrakingIntensity, Time.deltaTime * accelerationSpeed * 1.5f);
            
            isUsingNitro = false;
            
            // Debug visualization for throttle cut phase
            if (showDebugInfo && cornerFactor > 0.05f)
            {
                Debug.DrawRay(transform.position + Vector3.up * 4.2f, Vector3.left * 2f, Color.orange);
                Debug.Log($"Car {gameObject.name}: Cutting throttle, corner factor {cornerFactor:F2}");
            }
        }
        else if (needsActiveBraking)
        {
            // Phase 2: Apply active braking when necessary
            float speedDifference = currentSpeed - targetSpeed;
            
            // Calculate braking intensity based on speed difference and corner factor
            float brakingIntensity = Mathf.Clamp01(speedDifference * brakingIntensityMultiplier * 0.8f);
            
            // Add corner-based braking for sharp turns
            if (cornerFactor > 0.3f)
            {
                float cornerBraking = cornerBrakingFactor * 0.6f; // Gentler corner braking
                brakingIntensity = Mathf.Max(brakingIntensity, cornerBraking);
            }
            
            // Cut throttle completely when actively braking
            currentAcceleration = Mathf.Lerp(currentAcceleration, 0f, Time.deltaTime * accelerationSpeed * 2.0f);
            
            // Apply brakes gradually
            currentBrake = Mathf.Lerp(currentBrake, brakingIntensity, Time.deltaTime * accelerationSpeed * 1.5f);
            
            isUsingNitro = false;
            
            // Enhanced debug visualization for active braking
            if (showDebugInfo && brakingIntensity > 0.05f)
            {
                Debug.DrawRay(transform.position + Vector3.up * 4f, Vector3.left * brakingIntensity * 5f, Color.red);
                if (brakingIntensity > 0.2f)
                {
                    Debug.Log($"Car {gameObject.name}: Active braking with intensity {brakingIntensity:F2}, corner factor {cornerFactor:F2}");
                }
            }
        }
        else
        {
            // Coasting - gradually reduce inputs
            currentAcceleration = Mathf.Lerp(currentAcceleration, 0f, Time.deltaTime * accelerationSpeed);
            currentBrake = Mathf.Lerp(currentBrake, 0f, Time.deltaTime * accelerationSpeed);
            isUsingNitro = false;
        }
        
        // Use handbrake for sharp corners to prevent loss of control
        currentHandbrake = cornerFactor > handbrakeThreshold ? Mathf.Lerp(currentHandbrake, (cornerFactor - handbrakeThreshold) * 1.5f, Time.deltaTime * accelerationSpeed) : 0f;
        
        // Apply inputs to vehicle controller
        if (vehicleController.inputManager != null)
        {
            vehicleController.inputManager.SetAIInputs(currentSteer, currentAcceleration, currentHandbrake > 0f ? currentHandbrake : currentBrake, isUsingNitro);
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
            
            // Draw avoidance vector
            Debug.DrawRay(transform.position, avoidanceVector * 5f, Color.red);
            
            // Draw handbrake activation
            if (currentHandbrake > 0f)
            {
                Debug.DrawRay(transform.position + Vector3.up * 4.5f, Vector3.right * currentHandbrake * 5f, Color.magenta);
                Debug.Log($"Car {gameObject.name}: Using handbrake with intensity {currentHandbrake:F2}, corner factor {cornerFactor:F2}");
            }
        }
    }
    
    private Vector3 CalculateAvoidanceVector()
    {
        Vector3 avoidanceVector = Vector3.zero;
        
        foreach (var vehicle in otherVehicles)
        {
            if (vehicle == null || !vehicle.isActiveAndEnabled) continue;
            
            Vector3 directionToVehicle = vehicle.transform.position - transform.position;
            float distance = directionToVehicle.magnitude;
            
            // Only avoid if within avoidance distance and in front of us
            if (distance < avoidanceDistance && Vector3.Dot(transform.forward, directionToVehicle.normalized) > 0.5f)
            {
                // Calculate avoidance force (stronger as we get closer)
                float avoidanceForce = 1f - (distance / avoidanceDistance);
                avoidanceVector -= directionToVehicle.normalized * avoidanceForce;
            }
        }
        
        return avoidanceVector;
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
}