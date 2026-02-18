using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Shared race context manager that provides race data and position tracking
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
        
        // Get position directly from race manager (single source of truth)
        if (context.raceManager != null)
        {
            // Use race manager's already-calculated positions
            if (context.raceManager.CarPositions.TryGetValue(aiCar, out int position))
            {
                context.position = position;
            }
            else
            {
                context.position = 1; // Fallback
            }
            
            context.totalRacers = context.raceManager.SortedRacers.Count + 1; // +1 for player
        }
        
        context.raceProgress = aiCar.RaceProgress;
        
        // Update nearby vehicles
        context.nearbyVehicles = GetNearbyVehicles(aiCar);
        
        // Update opportunity indicators
        context.hasOpportunityAhead = HasOvertakingOpportunity(aiCar, context.nearbyVehicles);
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