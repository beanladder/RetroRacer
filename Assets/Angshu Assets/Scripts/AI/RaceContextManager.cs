using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Shared race context manager that provides race data, pressure calculation, and position tracking
/// Used by both AIPersonalityManager and AIVehicleController to avoid duplicate calculations
/// </summary>
public class RaceContextManager : MonoBehaviour
{
    [Header("Race Context Settings")]
    [SerializeField] private float contextUpdateInterval = 0.5f;
    
    // Singleton instance for easy access
    public static RaceContextManager Instance { get; private set; }
    
    // Race context data
    public class RaceContext
    {
        public int position = 1;
        public int totalRacers = 1;
        public float raceProgress = 0f;
        public float pressureLevel = 0f;
        public bool isBeingPressured = false;
        public bool hasOpportunityAhead = false;
        public List<AIVehicleController> nearbyVehicles = new List<AIVehicleController>();
        public AIRaceManager raceManager = null;
    }
    
    // Context cache for each AI car
    private Dictionary<AIVehicleController, RaceContext> contextCache = new Dictionary<AIVehicleController, RaceContext>();
    private float lastUpdateTime = 0f;
    
    private void Awake()
    {
        // Singleton pattern
        if (Instance == null)
        {
            Instance = this;
        }
        else
        {
            Destroy(gameObject);
        }
    }
    
    private void Update()
    {
        // Update context periodically to avoid expensive calculations every frame
        if (Time.time - lastUpdateTime > contextUpdateInterval)
        {
            UpdateAllContexts();
            lastUpdateTime = Time.time;
        }
    }
    
    /// <summary>
    /// Get race context for a specific AI car
    /// </summary>
    public RaceContext GetRaceContext(AIVehicleController aiCar)
    {
        if (!contextCache.ContainsKey(aiCar))
        {
            contextCache[aiCar] = new RaceContext();
            UpdateContextForCar(aiCar);
        }
        
        return contextCache[aiCar];
    }
    
    /// <summary>
    /// Calculate pressure level for a specific AI car based on race situation
    /// </summary>
    public float CalculatePressureLevel(AIVehicleController aiCar, AIPersonalityManager.AIPersonality personality, bool isInRivalry, float rivalryIntensity, int mistakeCount)
    {
        var context = GetRaceContext(aiCar);
        float pressure = 0f;
        
        // Position-based pressure
        if (context.position > 3)
        {
            pressure += 0.2f * (context.position - 3) / Mathf.Max(1f, context.totalRacers - 3);
        }
        
        // Being pressured by cars behind
        if (context.isBeingPressured)
        {
            pressure += 0.3f;
        }
        
        // Late race pressure when behind
        if (context.raceProgress > 0.7f && context.position > context.totalRacers * 0.5f)
        {
            pressure += 0.4f;
        }
        
        // Rivalry pressure
        if (isInRivalry)
        {
            pressure += 0.3f * rivalryIntensity;
        }
        
        // Mistake history pressure
        if (mistakeCount > 2)
        {
            pressure += 0.2f;
        }
        
        // Personality affects pressure resistance
        if (personality != null)
        {
            pressure *= (1f - personality.pressureResistance);
        }
        
        return Mathf.Clamp01(pressure);
    }
    
    /// <summary>
    /// Register an AI car for context tracking
    /// </summary>
    public void RegisterAICar(AIVehicleController aiCar)
    {
        if (!contextCache.ContainsKey(aiCar))
        {
            contextCache[aiCar] = new RaceContext();
        }
    }
    
    /// <summary>
    /// Unregister an AI car from context tracking
    /// </summary>
    public void UnregisterAICar(AIVehicleController aiCar)
    {
        if (contextCache.ContainsKey(aiCar))
        {
            contextCache.Remove(aiCar);
        }
    }
    
    private void UpdateAllContexts()
    {
        foreach (var kvp in contextCache)
        {
            if (kvp.Key != null)
            {
                UpdateContextForCar(kvp.Key);
            }
        }
    }
    
    private void UpdateContextForCar(AIVehicleController aiCar)
    {
        var context = contextCache[aiCar];
        
        // Get race manager
        if (context.raceManager == null)
        {
            context.raceManager = FindFirstObjectByType<AIRaceManager>();
        }
        
        // Update position and race progress
        if (context.raceManager != null)
        {
            context.position = GetCarPosition(aiCar, context.raceManager);
            context.totalRacers = context.raceManager.SortedRacers.Count + 1; // +1 for player
        }
        
        context.raceProgress = aiCar.RaceProgress;
        
        // Update nearby vehicles
        context.nearbyVehicles = GetNearbyVehicles(aiCar);
        
        // Update pressure indicators
        context.isBeingPressured = IsBeingPressuredFromBehind(aiCar, context.nearbyVehicles);
        context.hasOpportunityAhead = HasOvertakingOpportunity(aiCar, context.nearbyVehicles);
    }
    
    private int GetCarPosition(AIVehicleController aiCar, AIRaceManager raceManager)
    {
        if (raceManager.SortedRacers.Contains(aiCar))
        {
            return raceManager.SortedRacers.IndexOf(aiCar) + 1;
        }
        return 1;
    }
    
    private List<AIVehicleController> GetNearbyVehicles(AIVehicleController aiCar)
    {
        var nearby = new List<AIVehicleController>();
        var allAICars = FindObjectsByType<AIVehicleController>(FindObjectsSortMode.None);
        
        foreach (var otherCar in allAICars)
        {
            if (otherCar != aiCar && otherCar != null)
            {
                float distance = Vector3.Distance(aiCar.transform.position, otherCar.transform.position);
                if (distance < 50f) // Within 50 meters
                {
                    nearby.Add(otherCar);
                }
            }
        }
        
        return nearby;
    }
    
    private bool IsBeingPressuredFromBehind(AIVehicleController aiCar, List<AIVehicleController> nearbyVehicles)
    {
        foreach (var vehicle in nearbyVehicles)
        {
            Vector3 relativePos = aiCar.transform.InverseTransformPoint(vehicle.transform.position);
            if (relativePos.z < 0 && Vector3.Distance(aiCar.transform.position, vehicle.transform.position) < 15f)
            {
                // Car is behind and close
                var aiCarRb = aiCar.GetComponent<Rigidbody>();
                var otherRb = vehicle.GetComponent<Rigidbody>();
                if (aiCarRb != null && otherRb != null)
                {
                    Vector3 relativeVelocity = aiCarRb.linearVelocity - otherRb.linearVelocity;
                    if (Vector3.Dot(relativeVelocity, aiCar.transform.forward) < 0)
                    {
                        // Car behind is gaining
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    private bool HasOvertakingOpportunity(AIVehicleController aiCar, List<AIVehicleController> nearbyVehicles)
    {
        foreach (var vehicle in nearbyVehicles)
        {
            Vector3 relativePos = aiCar.transform.InverseTransformPoint(vehicle.transform.position);
            if (relativePos.z > 0 && Vector3.Distance(aiCar.transform.position, vehicle.transform.position) < 25f)
            {
                // Car is ahead and within overtaking range
                return true;
            }
        }
        return false;
    }
    
    private void OnDestroy()
    {
        if (Instance == this)
        {
            Instance = null;
        }
    }
}