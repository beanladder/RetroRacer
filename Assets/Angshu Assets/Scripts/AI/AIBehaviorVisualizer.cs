using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// Enhanced AI behavior visualizer - shows everything the AI is doing and planning
/// Displays current action, next intention, decision-making, and internal state
/// </summary>
[RequireComponent(typeof(AIVehicleController))]
public class AIBehaviorVisualizer : MonoBehaviour
{
    [Header("Visualization Settings")]
    [SerializeField] private bool showVisualization = true;
    [SerializeField] private float heightAboveCar = 5f;
    [SerializeField] private float lineSpacing = 0.8f;
    [SerializeField] private float stateDisplayDuration = 0.5f; // Update frequently
    
    private AIVehicleController aiController;
    private AIPersonalityManager personalityManager;
    private Ashsvp.SimcadeVehicleController vehicleController;
    
    // Display info
    private string currentAction = "INITIALIZING";
    private string nextIntention = "Waiting...";
    private string decisionReason = "";
    private string internalState = "";
    private Color currentStateColor = Color.white;
    private float lastStateChangeTime = 0f;
    private float currentSpeed = 0f;
    
    // Cached state
    private Vector3 lastPosition;
    private float lastSpeed = 0f;
    
    private void Start()
    {
        aiController = GetComponent<AIVehicleController>();
        personalityManager = GetComponent<AIPersonalityManager>();
        vehicleController = GetComponent<Ashsvp.SimcadeVehicleController>();
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
        
        // Update behavior state frequently
        if (Time.time - lastStateChangeTime > stateDisplayDuration)
        {
            AnalyzeAIBehavior();
            lastStateChangeTime = Time.time;
        }
    }
    
    private void AnalyzeAIBehavior()
    {
        if (aiController == null) return;
        
        // Gather all AI state information
        bool isRacing = aiController.IsRacing;
        bool isUsingNitro = vehicleController != null && vehicleController.isNitroActive;
        bool isOvertaking = IsOvertaking();
        bool isDefending = IsDefending();
        bool isBraking = IsBraking();
        bool isRamming = IsRamming();
        bool isInRivalry = personalityManager != null && personalityManager.IsInRivalry();
        bool isForcedOvertake = GetIsForcedOvertake();
        float cornerFactor = aiController.GetCornerFactor();
        float nitroAmount = vehicleController != null ? vehicleController.currentNitro : 0f;
        
        // Determine current action
        DetermineCurrentAction(isRacing, isUsingNitro, isOvertaking, isDefending, isBraking, isRamming, isInRivalry, isForcedOvertake);
        
        // Determine next intention
        DetermineNextIntention(cornerFactor, nitroAmount, isOvertaking, isDefending);
        
        // Determine decision reason
        DetermineDecisionReason(isUsingNitro, isOvertaking, isRamming, cornerFactor);
        
        // Build internal state string
        BuildInternalState(cornerFactor, nitroAmount);
        
        // Update tracking
        lastPosition = transform.position;
        lastSpeed = currentSpeed;
    }
    
    private void DetermineCurrentAction(bool isRacing, bool isUsingNitro, bool isOvertaking, bool isDefending, bool isBraking, bool isRamming, bool isInRivalry, bool isForcedOvertake)
    {
        if (!isRacing)
        {
            currentAction = "⏸ WAITING";
            currentStateColor = Color.gray;
        }
        else if (isRamming)
        {
            currentAction = "💥 RAMMING";
            currentStateColor = new Color(1f, 0.3f, 0f);
        }
        else if (isUsingNitro && isOvertaking)
        {
            currentAction = isForcedOvertake ? "⚡ FORCED NITRO ATTACK" : "⚡ NITRO ATTACK";
            currentStateColor = new Color(1f, 0.5f, 0f);
        }
        else if (isUsingNitro && isDefending)
        {
            currentAction = "🛡 NITRO DEFENSE";
            currentStateColor = Color.cyan;
        }
        else if (isUsingNitro)
        {
            currentAction = "⚡ BOOSTING";
            currentStateColor = Color.yellow;
        }
        else if (isOvertaking)
        {
            currentAction = isForcedOvertake ? "🎯 FORCED OVERTAKE" : "🏎 OVERTAKING";
            currentStateColor = new Color(1f, 0.65f, 0f);
        }
        else if (isDefending)
        {
            currentAction = "🛡 DEFENDING";
            currentStateColor = Color.blue;
        }
        else if (isInRivalry)
        {
            currentAction = "⚔ RIVALRY BATTLE";
            currentStateColor = new Color(1f, 0f, 1f);
        }
        else if (isBraking)
        {
            currentAction = "🔴 BRAKING";
            currentStateColor = Color.red;
        }
        else if (currentSpeed > 5f)
        {
            currentAction = "🏁 RACING";
            currentStateColor = Color.green;
        }
        else
        {
            currentAction = "🐌 SLOW";
            currentStateColor = Color.white;
        }
    }
    
    private void DetermineNextIntention(float cornerFactor, float nitroAmount, bool isOvertaking, bool isDefending)
    {
        // Look ahead to determine what AI will do next
        if (cornerFactor > 0.3f)
        {
            nextIntention = "→ Will brake for sharp corner";
        }
        else if (cornerFactor > 0.15f)
        {
            nextIntention = "→ Will slow for corner";
        }
        else if (isOvertaking)
        {
            nextIntention = "→ Completing overtake";
        }
        else if (isDefending)
        {
            nextIntention = "→ Holding position";
        }
        else if (nitroAmount > 20f && HasOvertakingOpportunity())
        {
            nextIntention = "→ Looking for nitro opportunity";
        }
        else if (nitroAmount < 10f)
        {
            nextIntention = "→ Conserving nitro";
        }
        else if (IsCarAhead())
        {
            nextIntention = "→ Planning overtake";
        }
        else
        {
            nextIntention = "→ Following racing line";
        }
    }
    
    private void DetermineDecisionReason(bool isUsingNitro, bool isOvertaking, bool isRamming, float cornerFactor)
    {
        if (isRamming)
        {
            decisionReason = GetRammingReason();
        }
        else if (isUsingNitro)
        {
            decisionReason = GetNitroReason();
        }
        else if (isOvertaking)
        {
            decisionReason = "Stuck behind slower car";
        }
        else if (cornerFactor > 0.2f)
        {
            decisionReason = $"Corner detected ({(cornerFactor * 100):F0}% sharp)";
        }
        else
        {
            decisionReason = GetPersonalityReason();
        }
    }
    
    private void BuildInternalState(float cornerFactor, float nitroAmount)
    {
        var personality = personalityManager?.GetPersonality();
        string personalityType = personality != null ? personality.personalityType.ToString() : "Unknown";
        
        int position = GetCurrentPosition();
        float rubberBanding = aiController.RubberBandingFactor;
        
        internalState = $"P{position} | {personalityType} | Nitro:{nitroAmount:F0}% | RB:{rubberBanding:F2}x";
    }
    
    private string GetRammingReason()
    {
        var personality = personalityManager?.GetPersonality();
        if (personality != null)
        {
            float rammingChance = personality.aggression * personality.riskTaking;
            return $"Ramming (Aggro:{(rammingChance * 100):F0}%)";
        }
        return "Ramming opportunity";
    }
    
    private string GetNitroReason()
    {
        // Try to get nitro strategy via reflection
        var strategyField = typeof(AIVehicleController).GetField("currentNitroStrategy", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        
        if (strategyField != null)
        {
            var strategy = strategyField.GetValue(aiController);
            return $"Nitro: {strategy} strategy";
        }
        
        return "Using nitro boost";
    }
    
    private string GetPersonalityReason()
    {
        var personality = personalityManager?.GetPersonality();
        if (personality == null) return "Following AI logic";
        
        if (personality.aggression > 0.7f)
            return "Aggressive driving style";
        else if (personality.skill > 0.8f)
            return "Veteran precision";
        else if (personality.consistency > 0.7f)
            return "Consistent pace";
        else if (personality.riskTaking > 0.7f)
            return "Taking risks";
        else
            return "Conservative approach";
    }
    
    private int GetCurrentPosition()
    {
        var raceManager = FindFirstObjectByType<AIRaceManager>();
        if (raceManager != null && raceManager.CarPositions.TryGetValue(aiController, out int pos))
        {
            return pos;
        }
        return 0;
    }
    
    private bool GetIsForcedOvertake()
    {
        var field = typeof(AIVehicleController).GetField("isForcedOvertake", 
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        
        if (field != null)
        {
            return (bool)field.GetValue(aiController);
        }
        return false;
    }
    
    private bool HasOvertakingOpportunity()
    {
        var otherCars = FindObjectsByType<AIVehicleController>(FindObjectsSortMode.None);
        foreach (var other in otherCars)
        {
            if (other == aiController || other == null) continue;
            
            Vector3 relativePos = transform.InverseTransformPoint(other.transform.position);
            float distance = Vector3.Distance(transform.position, other.transform.position);
            
            if (relativePos.z > 0 && distance < 25f)
            {
                return true;
            }
        }
        return false;
    }
    
    private bool IsCarAhead()
    {
        var otherCars = FindObjectsByType<AIVehicleController>(FindObjectsSortMode.None);
        foreach (var other in otherCars)
        {
            if (other == aiController || other == null) continue;
            
            Vector3 relativePos = transform.InverseTransformPoint(other.transform.position);
            if (relativePos.z > 0 && relativePos.z < 50f)
            {
                return true;
            }
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
        return aiController.GetCurrentBrakeInput() > 0.1f;
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
    
#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!showVisualization) return;
        
        // Calculate speed in km/h for display
        float speedKmh = currentSpeed;
        
        // Line 1: Current Action (largest, colored)
        Vector3 line1Pos = transform.position + Vector3.up * heightAboveCar;
        GUIStyle actionStyle = new GUIStyle();
        actionStyle.normal.textColor = currentStateColor;
        actionStyle.fontSize = 14;
        actionStyle.fontStyle = FontStyle.Bold;
        actionStyle.alignment = TextAnchor.MiddleCenter;
        Handles.Label(line1Pos, currentAction, actionStyle);
        
        // Line 2: Next Intention (medium, white)
        Vector3 line2Pos = transform.position + Vector3.up * (heightAboveCar - lineSpacing);
        GUIStyle intentionStyle = new GUIStyle();
        intentionStyle.normal.textColor = new Color(1f, 1f, 1f, 0.9f);
        intentionStyle.fontSize = 11;
        intentionStyle.fontStyle = FontStyle.Normal;
        intentionStyle.alignment = TextAnchor.MiddleCenter;
        Handles.Label(line2Pos, nextIntention, intentionStyle);
        
        // Line 3: Decision Reason (small, yellow)
        Vector3 line3Pos = transform.position + Vector3.up * (heightAboveCar - lineSpacing * 2);
        GUIStyle reasonStyle = new GUIStyle();
        reasonStyle.normal.textColor = new Color(1f, 1f, 0.6f, 0.8f);
        reasonStyle.fontSize = 10;
        reasonStyle.fontStyle = FontStyle.Italic;
        reasonStyle.alignment = TextAnchor.MiddleCenter;
        Handles.Label(line3Pos, decisionReason, reasonStyle);
        
        // Line 4: Internal State (smallest, gray)
        Vector3 line4Pos = transform.position + Vector3.up * (heightAboveCar - lineSpacing * 3);
        GUIStyle stateStyle = new GUIStyle();
        stateStyle.normal.textColor = new Color(0.8f, 0.8f, 0.8f, 0.7f);
        stateStyle.fontSize = 9;
        stateStyle.fontStyle = FontStyle.Normal;
        stateStyle.alignment = TextAnchor.MiddleCenter;
        Handles.Label(line4Pos, internalState, stateStyle);
        
        // Line 5: Speed (bottom, white)
        Vector3 line5Pos = transform.position + Vector3.up * (heightAboveCar - lineSpacing * 4);
        GUIStyle speedStyle = new GUIStyle();
        speedStyle.normal.textColor = new Color(1f, 1f, 1f, 0.9f);
        speedStyle.fontSize = 12;
        speedStyle.fontStyle = FontStyle.Bold;
        speedStyle.alignment = TextAnchor.MiddleCenter;
        Handles.Label(line5Pos, $"{speedKmh:F0} km/h", speedStyle);
    }
#endif
}
