using UnityEngine;

/// <summary>
/// Preset configurations for AIRaceManager to quickly set up optimal racing experiences
/// </summary>
public static class AIRaceManagerPresets
{
    /// <summary>
    /// Optimal preset for 3 AI racers - creates exciting, competitive races
    /// </summary>
    public static void ApplyThreeRacerPreset(AIRaceManager raceManager)
    {
        // Use reflection to set private fields since they're serialized
        var raceManagerType = typeof(AIRaceManager);
        
        // AI Racers Configuration
        SetField(raceManager, "numberOfAIRacers", 3);
        SetField(raceManager, "startingOffset", 12f);
        
        // AI Difficulty Settings (Balanced Competition)
        SetField(raceManager, "minSkillLevel", 0.4f);
        SetField(raceManager, "maxSkillLevel", 0.85f);
        SetField(raceManager, "minAggressiveness", 0.4f);
        SetField(raceManager, "maxAggressiveness", 0.9f);
        
        // Rubber Banding (Tight Racing)
        SetField(raceManager, "rubberBandingStrength", 0.7f);
        SetField(raceManager, "maxSpeedBoost", 1.25f);
        SetField(raceManager, "maxSpeedPenalty", 0.85f);
        
        // Race Game Mode
        SetField(raceManager, "numberOfLaps", 3);
        SetField(raceManager, "spawnPlayer", true);
        
        // Auto Setup (Optimal Respawn)
        SetField(raceManager, "autoSetupAllVehicles", true);
        SetField(raceManager, "setupPlayerVehicles", true);
        SetField(raceManager, "respawnHeight", 2.5f);
        SetField(raceManager, "respawnForwardOffset", 6f);
        SetField(raceManager, "respawnInvulnerabilityTime", 2.5f);
        
        Debug.Log("[AIRaceManagerPresets] Applied 3-Racer Competitive Preset!");
        // Settings details removed - less console clutter
    }
    
    /// <summary>
    /// Casual preset for 3 AI racers - more forgiving, less aggressive
    /// </summary>
    public static void ApplyThreeRacerCasualPreset(AIRaceManager raceManager)
    {
        // AI Racers Configuration
        SetField(raceManager, "numberOfAIRacers", 3);
        SetField(raceManager, "startingOffset", 15f);
        
        // AI Difficulty Settings (Casual)
        SetField(raceManager, "minSkillLevel", 0.3f);
        SetField(raceManager, "maxSkillLevel", 0.7f);
        SetField(raceManager, "minAggressiveness", 0.2f);
        SetField(raceManager, "maxAggressiveness", 0.6f);
        
        // Rubber Banding (Moderate)
        SetField(raceManager, "rubberBandingStrength", 0.5f);
        SetField(raceManager, "maxSpeedBoost", 1.15f);
        SetField(raceManager, "maxSpeedPenalty", 0.9f);
        
        // Race Game Mode
        SetField(raceManager, "numberOfLaps", 3);
        SetField(raceManager, "spawnPlayer", true);
        
        // Auto Setup
        SetField(raceManager, "autoSetupAllVehicles", true);
        SetField(raceManager, "setupPlayerVehicles", true);
        SetField(raceManager, "respawnHeight", 2f);
        SetField(raceManager, "respawnForwardOffset", 5f);
        SetField(raceManager, "respawnInvulnerabilityTime", 3f);
        
        Debug.Log("[AIRaceManagerPresets] Applied 3-Racer Casual Preset!");
        // Settings details removed - less console clutter
    }
    
    /// <summary>
    /// Hardcore preset for 3 AI racers - very challenging, aggressive AI
    /// </summary>
    public static void ApplyThreeRacerHardcorePreset(AIRaceManager raceManager)
    {
        // AI Racers Configuration
        SetField(raceManager, "numberOfAIRacers", 3);
        SetField(raceManager, "startingOffset", 10f);
        
        // AI Difficulty Settings (Hardcore)
        SetField(raceManager, "minSkillLevel", 0.6f);
        SetField(raceManager, "maxSkillLevel", 0.95f);
        SetField(raceManager, "minAggressiveness", 0.6f);
        SetField(raceManager, "maxAggressiveness", 1.0f);
        
        // Rubber Banding (Minimal - let skill decide)
        SetField(raceManager, "rubberBandingStrength", 0.3f);
        SetField(raceManager, "maxSpeedBoost", 1.1f);
        SetField(raceManager, "maxSpeedPenalty", 0.95f);
        
        // Race Game Mode
        SetField(raceManager, "numberOfLaps", 5);
        SetField(raceManager, "spawnPlayer", true);
        
        // Auto Setup
        SetField(raceManager, "autoSetupAllVehicles", true);
        SetField(raceManager, "setupPlayerVehicles", true);
        SetField(raceManager, "respawnHeight", 2f);
        SetField(raceManager, "respawnForwardOffset", 4f);
        SetField(raceManager, "respawnInvulnerabilityTime", 1.5f);
        
        Debug.Log("[AIRaceManagerPresets] Applied 3-Racer Hardcore Preset!");
        // Settings details removed - less console clutter
    }
    
    private static void SetField(object target, string fieldName, object value)
    {
        var field = target.GetType().GetField(fieldName, 
            System.Reflection.BindingFlags.NonPublic | 
            System.Reflection.BindingFlags.Instance);
        
        if (field != null)
        {
            field.SetValue(target, value);
        }
        else
        {
            Debug.LogWarning($"[AIRaceManagerPresets] Field '{fieldName}' not found!");
        }
    }
}

