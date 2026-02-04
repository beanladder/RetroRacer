using UnityEngine;

/// <summary>
/// Smooths AI steering inputs to prevent sudden unrealistic turns
/// </summary>
public class SmoothSteeringController : MonoBehaviour
{
    [Header("Steering Smoothing")]
    [SerializeField] private float maxSteeringChangeRate = 2f; // Max steering change per second
    [SerializeField] private float emergencySteeringRate = 4f; // Faster rate for emergency avoidance
    [SerializeField] private float steeringStabilityThreshold = 0.1f; // Minimum change to apply
    [SerializeField] private bool enableSteeringLimits = true;
    
    [Header("Lateral Offset Smoothing")]
    [SerializeField] private float maxLateralOffsetChangeRate = 3f; // Max offset change per second
    [SerializeField] private float maxTotalLateralOffset = 4f; // Maximum total lateral offset
    
    [Header("Behavior Modifier Limits")]
    [SerializeField] private float maxBehaviorSteeringModifier = 0.3f; // Limit behavior steering changes
    [SerializeField] private float behaviorModifierSmoothingRate = 1.5f;
    
    [Header("Debug")]
    [SerializeField] private bool showDebugInfo = false;
    
    // Internal state
    private float lastSteeringInput = 0f;
    private float lastLateralOffset = 0f;
    private float smoothedBehaviorModifier = 0f;
    private float lastFrameTime = 0f;
    
    // Emergency detection
    private bool isInEmergencyAvoidance = false;
    private float emergencyAvoidanceTimer = 0f;
    private const float emergencyAvoidanceDuration = 1f;
    
    private AIVehicleController aiController;
    
    private void Start()
    {
        aiController = GetComponent<AIVehicleController>();
        lastFrameTime = Time.time;
    }
    
    /// <summary>
    /// Smooths steering input to prevent sudden changes
    /// </summary>
    public float SmoothSteeringInput(float targetSteering, bool isEmergency = false)
    {
        if (!enableSteeringLimits) return targetSteering;
        
        float deltaTime = Time.time - lastFrameTime;
        if (deltaTime <= 0f) return lastSteeringInput;
        
        // Determine steering change rate
        float steeringRate = isEmergency ? emergencySteeringRate : maxSteeringChangeRate;
        
        // Handle emergency avoidance
        if (isEmergency)
        {
            isInEmergencyAvoidance = true;
            emergencyAvoidanceTimer = emergencyAvoidanceDuration;
        }
        
        if (isInEmergencyAvoidance)
        {
            emergencyAvoidanceTimer -= deltaTime;
            if (emergencyAvoidanceTimer <= 0f)
            {
                isInEmergencyAvoidance = false;
            }
            steeringRate = emergencySteeringRate;
        }
        
        // Calculate maximum allowed change
        float maxChange = steeringRate * deltaTime;
        float steeringDifference = targetSteering - lastSteeringInput;
        
        // Apply stability threshold - ignore very small changes
        if (Mathf.Abs(steeringDifference) < steeringStabilityThreshold && !isEmergency)
        {
            return lastSteeringInput;
        }
        
        // Limit the steering change
        float clampedChange = Mathf.Clamp(steeringDifference, -maxChange, maxChange);
        float smoothedSteering = lastSteeringInput + clampedChange;
        
        // Debug logging
        // Steering adjustment log removed - too frequent
        
        lastSteeringInput = smoothedSteering;
        return smoothedSteering;
    }
    
    /// <summary>
    /// Smooths lateral offset changes to prevent sudden path deviations
    /// </summary>
    public float SmoothLateralOffset(float targetOffset)
    {
        float deltaTime = Time.time - lastFrameTime;
        if (deltaTime <= 0f) return lastLateralOffset;
        
        // Clamp target offset to reasonable limits
        targetOffset = Mathf.Clamp(targetOffset, -maxTotalLateralOffset, maxTotalLateralOffset);
        
        // Calculate maximum allowed change
        float maxChange = maxLateralOffsetChangeRate * deltaTime;
        float offsetDifference = targetOffset - lastLateralOffset;
        
        // Limit the offset change
        float clampedChange = Mathf.Clamp(offsetDifference, -maxChange, maxChange);
        float smoothedOffset = lastLateralOffset + clampedChange;
        
        lastLateralOffset = smoothedOffset;
        return smoothedOffset;
    }
    
    /// <summary>
    /// Smooths behavior modifier steering to prevent sudden personality-based changes
    /// </summary>
    public float SmoothBehaviorModifier(float targetModifier)
    {
        float deltaTime = Time.time - lastFrameTime;
        if (deltaTime <= 0f) return smoothedBehaviorModifier;
        
        // Clamp behavior modifier to reasonable limits
        targetModifier = Mathf.Clamp(targetModifier, -maxBehaviorSteeringModifier, maxBehaviorSteeringModifier);
        
        // Smooth the behavior modifier
        smoothedBehaviorModifier = Mathf.Lerp(smoothedBehaviorModifier, targetModifier, 
            deltaTime * behaviorModifierSmoothingRate);
        
        return smoothedBehaviorModifier;
    }
    
    /// <summary>
    /// Detects if the AI is making an unrealistic steering change
    /// </summary>
    public bool IsSteeringChangeRealistic(float currentSteering, float targetSteering)
    {
        float deltaTime = Time.time - lastFrameTime;
        if (deltaTime <= 0f) return true;
        
        float steeringDifference = Mathf.Abs(targetSteering - currentSteering);
        float maxRealisticChange = maxSteeringChangeRate * deltaTime;
        
        return steeringDifference <= maxRealisticChange * 1.5f; // Allow some tolerance
    }
    
    /// <summary>
    /// Checks if the AI should be in emergency avoidance mode
    /// </summary>
    public bool ShouldUseEmergencyAvoidance(Vector3 threatPosition, float threatDistance)
    {
        // Emergency avoidance for very close threats
        if (threatDistance < 3f) return true;
        
        // Check if threat is directly ahead and approaching fast
        Vector3 toThreat = threatPosition - transform.position;
        float forwardDot = Vector3.Dot(transform.forward, toThreat.normalized);
        
        if (forwardDot > 0.8f && threatDistance < 6f)
        {
            return true;
        }
        
        return false;
    }
    
    /// <summary>
    /// Validates and corrects steering input to prevent track departure
    /// </summary>
    public float ValidateSteeringForTrackBounds(float steering, Vector3 currentPosition, Vector3 trackCenter, float trackWidth)
    {
        // Calculate distance from track center
        Vector3 toTrackCenter = trackCenter - currentPosition;
        toTrackCenter.y = 0f; // Ignore height difference
        float distanceFromCenter = toTrackCenter.magnitude;
        
        // If we're getting close to track edge, limit steering away from center
        float trackEdgeDistance = trackWidth * 0.4f; // 80% of track width
        
        if (distanceFromCenter > trackEdgeDistance)
        {
            // Calculate direction to track center
            Vector3 directionToCenter = toTrackCenter.normalized;
            Vector3 localDirectionToCenter = transform.InverseTransformDirection(directionToCenter);
            
            // If steering would take us further from center, reduce it
            if (Mathf.Sign(steering) != Mathf.Sign(localDirectionToCenter.x))
            {
                float correctionFactor = Mathf.Clamp01(1f - (distanceFromCenter - trackEdgeDistance) / (trackWidth * 0.1f));
                steering *= correctionFactor;
                
                // Track bounds correction log removed - frequent operation
            }
        }
        
        return steering;
    }
    
    private void LateUpdate()
    {
        lastFrameTime = Time.time;
    }
    
    /// <summary>
    /// Reset smoothing state (useful when respawning or teleporting)
    /// </summary>
    public void ResetSmoothingState()
    {
        lastSteeringInput = 0f;
        lastLateralOffset = 0f;
        smoothedBehaviorModifier = 0f;
        isInEmergencyAvoidance = false;
        emergencyAvoidanceTimer = 0f;
        
        // Smoothing reset log removed - frequent operation
    }
    
    // Public getters for debugging
    public float GetLastSteeringInput() => lastSteeringInput;
    public float GetLastLateralOffset() => lastLateralOffset;
    public bool IsInEmergencyAvoidance() => isInEmergencyAvoidance;
}