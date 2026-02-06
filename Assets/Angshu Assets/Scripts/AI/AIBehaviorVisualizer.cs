using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// Visualizes AI behavior state and decision-making in Scene View using Gizmos
/// Shows what the AI is currently doing and thinking above their car
/// </summary>
[RequireComponent(typeof(AIVehicleController))]
public class AIBehaviorVisualizer : MonoBehaviour
{
    [Header("Visualization Settings")]
    [SerializeField] private bool showVisualization = true;
    [SerializeField] private float heightAboveCar = 3f;
    [SerializeField] private float thoughtHeightOffset = 1.5f;
    [SerializeField] private float stateDisplayDuration = 2f; // Longer duration to prevent flickering
    
    private AIVehicleController aiController;
    private AIPersonalityManager personalityManager;
    
    // Behavior tracking
    private string currentBehaviorState = "INITIALIZING";
    private string currentThought = "Waiting...";
    private Color currentStateColor = Color.white;
    private float lastStateChangeTime = 0f;
    private float currentSpeed = 0f;
    
    // State detection
    private bool wasOvertaking = false;
    private bool wasDefending = false;
    private bool wasUsingNitro = false;
    private Vector3 lastPosition;
    private float lastSpeed = 0f;
    
    private void Start()
    {
        aiController = GetComponent<AIVehicleController>();
        personalityManager = GetComponent<AIPersonalityManager>();
        lastPosition = transform.position;
    }
    
    private void Update()
    {
        if (!showVisualization) return;
        
        // Get accurate speed from GearSystem (already in km/h)
        var gearSystem = GetComponent<Ashsvp.GearSystem>();
        if (gearSystem != null)
        {
            currentSpeed = gearSystem.VehicleSpeed;
        }
        
        // Update behavior state
        if (Time.time - lastStateChangeTime > stateDisplayDuration)
        {
            DetectBehaviorState();
        }
    }
    
    private void DetectBehaviorState()
    {
        if (aiController == null) return;
        
        // Get current state info
        bool isRacing = aiController.IsRacing;
        float currentSpeedCalc = (transform.position - lastPosition).magnitude / Time.deltaTime;
        bool isUsingNitro = IsUsingNitro();
        bool isOvertaking = IsOvertaking();
        bool isDefending = IsDefending();
        bool isBraking = IsBraking();
        bool isRamming = IsRamming();
        bool isUnderPressure = personalityManager != null && personalityManager.GetCurrentPressure() > 0.5f;
        bool isInRivalry = personalityManager != null && personalityManager.IsInRivalry();
        
        string newState = currentBehaviorState;
        string newThought = currentThought;
        Color newColor = Color.white;
        
        if (!isRacing)
        {
            newState = "WAITING";
            newThought = "Ready to race...";
            newColor = Color.gray;
        }
        // Ramming takes priority
        else if (isRamming)
        {
            newState = "RAMMING!";
            newThought = "Time to make contact!";
            newColor = new Color(1f, 0.3f, 0f); // Dark orange
        }
        // Check for mistakes (sudden speed loss without braking)
        else if (currentSpeedCalc < lastSpeed * 0.7f && !isBraking && lastSpeed > 10f)
        {
            newState = "MISTAKE!";
            newThought = "Lost control!";
            newColor = Color.red;
        }
        // Nitro usage
        else if (isUsingNitro && isOvertaking)
        {
            newState = "NITRO ATTACK!";
            newThought = "Going for the pass!";
            newColor = new Color(1f, 0.5f, 0f);
        }
        else if (isUsingNitro && isDefending)
        {
            newState = "NITRO DEFENSE!";
            newThought = "Protecting my position!";
            newColor = Color.cyan;
        }
        else if (isUsingNitro)
        {
            newState = "BOOSTING";
            newThought = "Time to go fast!";
            newColor = Color.yellow;
        }
        // Overtaking
        else if (isOvertaking)
        {
            newState = "OVERTAKING";
            newThought = "Looking for an opening...";
            newColor = new Color(1f, 0.65f, 0f);
        }
        // Defending
        else if (isDefending)
        {
            newState = "BLOCKING";
            newThought = "Not letting them through!";
            newColor = Color.blue;
        }
        // Under pressure
        else if (isUnderPressure && isBraking)
        {
            newState = "UNDER PRESSURE";
            newThought = "They're right behind me!";
            newColor = new Color(1f, 0.5f, 0.5f);
        }
        // Rivalry
        else if (isInRivalry)
        {
            newState = "RIVALRY BATTLE";
            newThought = "Time to show them who's boss!";
            newColor = new Color(1f, 0f, 1f);
        }
        // Heavy braking
        else if (isBraking)
        {
            newState = "BRAKING";
            newThought = "Corner ahead, slowing down...";
            newColor = Color.red;
        }
        // Normal racing
        else if (currentSpeedCalc > 5f)
        {
            newState = "RACING";
            newThought = "Following the racing line...";
            newColor = Color.green;
        }
        else
        {
            newState = "CRUISING";
            newThought = "Taking it easy...";
            newColor = Color.white;
        }
        
        // Update state if changed
        if (newState != currentBehaviorState)
        {
            currentBehaviorState = newState;
            currentThought = newThought;
            currentStateColor = newColor;
            lastStateChangeTime = Time.time;
        }
        
        // Update tracking variables
        lastPosition = transform.position;
        lastSpeed = currentSpeedCalc;
        wasOvertaking = isOvertaking;
        wasDefending = isDefending;
        wasUsingNitro = isUsingNitro;
    }
    
    private bool IsUsingNitro()
    {
        var vehicleController = GetComponent<Ashsvp.SimcadeVehicleController>();
        if (vehicleController != null)
        {
            return vehicleController.isNitroActive;
        }
        return false;
    }
    
    private bool IsOvertaking()
    {
        var otherCars = FindObjectsByType<AIVehicleController>(FindObjectsSortMode.None);
        foreach (var other in otherCars)
        {
            if (other == aiController || other == null) continue;
            
            Vector3 relativePos = transform.InverseTransformPoint(other.transform.position);
            float distance = Vector3.Distance(transform.position, other.transform.position);
            
            if (relativePos.z > 0 && distance < 15f && relativePos.z < 20f)
            {
                return true;
            }
        }
        return false;
    }
    
    private bool IsDefending()
    {
        var otherCars = FindObjectsByType<AIVehicleController>(FindObjectsSortMode.None);
        foreach (var other in otherCars)
        {
            if (other == aiController || other == null) continue;
            
            Vector3 relativePos = transform.InverseTransformPoint(other.transform.position);
            float distance = Vector3.Distance(transform.position, other.transform.position);
            
            if (relativePos.z < 0 && distance < 12f && relativePos.z > -15f)
            {
                return true;
            }
        }
        return false;
    }
    
    private bool IsBraking()
    {
        float currentSpeedCalc = (transform.position - lastPosition).magnitude / Time.deltaTime;
        return currentSpeedCalc < lastSpeed * 0.95f && lastSpeed > 5f;
    }
    
    private bool IsRamming()
    {
        // Check if AI is actively ramming (has a ramming target)
        if (aiController == null) return false;
        
        // Use reflection to check private ramming fields
        var rammingTargetField = typeof(AIVehicleController).GetField("rammingTarget", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        
        if (rammingTargetField != null)
        {
            var target = rammingTargetField.GetValue(aiController);
            return target != null;
        }
        
        return false;
    }
    
    /// <summary>
    /// Toggle visualization on/off at runtime
    /// </summary>
    public void ToggleVisualization(bool show)
    {
        showVisualization = show;
    }
    
    /// <summary>
    /// Manually set behavior state (for external systems)
    /// </summary>
    public void SetBehaviorState(string state, string thought, Color color)
    {
        if (!showVisualization) return;
        
        currentBehaviorState = state;
        currentThought = thought;
        currentStateColor = color;
        lastStateChangeTime = Time.time;
    }
    
#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!showVisualization) return;
        
        // Get personality info
        string personalityType = "Unknown";
        if (personalityManager != null && personalityManager.GetPersonality() != null)
        {
            personalityType = personalityManager.GetPersonality().personalityType.ToString();
        }
        
        // Calculate speed in km/h for display (already in km/h from GearSystem)
        float speedKmh = currentSpeed;
        
        // Draw personality type and speed (top)
        Vector3 personalityPos = transform.position + Vector3.up * (heightAboveCar + thoughtHeightOffset);
        GUIStyle personalityStyle = new GUIStyle();
        personalityStyle.normal.textColor = new Color(1f, 1f, 1f, 0.7f);
        personalityStyle.fontSize = 11;
        personalityStyle.fontStyle = FontStyle.Italic;
        personalityStyle.alignment = TextAnchor.MiddleCenter;
        Handles.Label(personalityPos, $"[{personalityType}] {speedKmh:F0} km/h", personalityStyle);
        
        // Draw current thought (middle)
        Vector3 thoughtPos = transform.position + Vector3.up * (heightAboveCar + thoughtHeightOffset * 0.5f);
        GUIStyle thoughtStyle = new GUIStyle();
        thoughtStyle.normal.textColor = new Color(1f, 1f, 0.8f, 0.9f);
        thoughtStyle.fontSize = 10;
        thoughtStyle.fontStyle = FontStyle.Italic;
        thoughtStyle.alignment = TextAnchor.MiddleCenter;
        Handles.Label(thoughtPos, $"\"{currentThought}\"", thoughtStyle);
        
        // Draw behavior state (bottom - larger and colored)
        Vector3 statePos = transform.position + Vector3.up * heightAboveCar;
        GUIStyle stateStyle = new GUIStyle();
        stateStyle.normal.textColor = currentStateColor;
        stateStyle.fontSize = 13;
        stateStyle.fontStyle = FontStyle.Bold;
        stateStyle.alignment = TextAnchor.MiddleCenter;
        Handles.Label(statePos, currentBehaviorState, stateStyle);
    }
#endif
}
