using UnityEngine;

/// <summary>
/// ScriptableObject that defines an AI personality profile
/// Create instances via: Assets > Create > AI > Personality
/// </summary>
[CreateAssetMenu(fileName = "New AI Personality", menuName = "AI/Personality", order = 1)]
public class AIPersonalityData : ScriptableObject
{
    [Header("Personality Type")]
    public PersonalityType personalityType;
    
    [Header("Core Traits")]
    [Range(0f, 1f), Tooltip("How aggressive the AI drives - higher values mean more risky overtakes and harder racing")]
    public float aggression = 0.5f;
    
    [Range(0f, 1f), Tooltip("Driving skill level - affects precision, consistency, and racing line following")]
    public float skill = 0.7f;
    
    [Range(0f, 1f), Tooltip("How consistent the AI performs - higher values reduce random variations")]
    public float consistency = 0.6f;
    
    [Range(0f, 1f), Tooltip("Willingness to take risks - affects corner speeds and overtaking attempts")]
    public float riskTaking = 0.5f;
    
    [Range(0f, 1f), Tooltip("Patience level - affects how long AI waits before attempting overtakes")]
    public float patience = 0.5f;
    
    [Header("Nitro Strategy")]
    [Range(0f, 1f), Tooltip("Tendency to use nitro aggressively for attacks")]
    public float nitroAggression = 0.5f;
    
    [Range(0f, 1f), Tooltip("Tendency to use nitro defensively to protect position")]
    public float nitroDefense = 0.3f;
    
    [Range(0f, 1f), Tooltip("Tendency to conserve nitro for strategic moments")]
    public float nitroConservation = 0.4f;
    
    [Header("Racing Behavior")]
    [Range(0f, 1f), Tooltip("How often AI blocks other drivers")]
    public float blockingTendency = 0.3f;
    
    [Range(0f, 1f), Tooltip("Aggression level when attempting overtakes")]
    public float overtakingAggression = 0.5f;
    
    [Range(0f, 1f), Tooltip("How defensively the AI drives when under pressure")]
    public float defensiveDriving = 0.4f;
    
    [Header("Pressure Response")]
    [Range(0f, 1f), Tooltip("Resistance to pressure - higher values mean better performance under stress")]
    public float pressureResistance = 0.6f;
    
    [Range(0f, 1f), Tooltip("Drive to catch up when behind - affects comeback performance")]
    public float comebackDrive = 0.5f;

    public enum PersonalityType
    {
        Aggressive,     // High aggression, low patience, risky
        Conservative,   // Low aggression, high patience, safe
        Opportunist,    // Medium aggression, waits for chances
        Hothead,        // Very aggressive, high pressure under racing
        Veteran,        // High skill, consistent, strategic
        Rookie,         // Low skill, inconsistent, learns during race
        Blocker,        // Defensive, blocks other drivers
        Speedster       // Focuses on pure speed, less tactical
    }
    
    /// <summary>
    /// Get a description of this personality type
    /// </summary>
    public string GetPersonalityDescription()
    {
        switch (personalityType)
        {
            case PersonalityType.Aggressive:
                return "Aggressive driver who takes risks and fights for position";
            case PersonalityType.Conservative:
                return "Careful driver who avoids risks and drives consistently";
            case PersonalityType.Opportunist:
                return "Strategic driver who waits for the right moment to strike";
            case PersonalityType.Hothead:
                return "Hot-tempered driver with high aggression under pressure";
            case PersonalityType.Veteran:
                return "Experienced driver with excellent racecraft and consistency";
            case PersonalityType.Rookie:
                return "Inexperienced driver still learning the ropes";
            case PersonalityType.Blocker:
                return "Defensive driver who excels at blocking opponents";
            case PersonalityType.Speedster:
                return "Speed-focused driver who prioritizes pace over tactics";
            default:
                return "Unknown personality type";
        }
    }
    
    /// <summary>
    /// Create a randomized personality of the specified type
    /// </summary>
    public static AIPersonalityData CreateRandomized(PersonalityType type)
    {
        var personality = CreateInstance<AIPersonalityData>();
        personality.personalityType = type;
        personality.RandomizeForType(type);
        return personality;
    }
    
    /// <summary>
    /// Randomize values based on personality type
    /// </summary>
    public void RandomizeForType(PersonalityType type)
    {
        switch (type)
        {
            case PersonalityType.Aggressive:
                aggression = Random.Range(0.8f, 1.0f);
                skill = Random.Range(0.6f, 0.8f);
                consistency = Random.Range(0.4f, 0.6f);
                riskTaking = Random.Range(0.8f, 1.0f);
                patience = Random.Range(0.1f, 0.3f);
                nitroAggression = Random.Range(0.8f, 1.0f);
                blockingTendency = Random.Range(0.6f, 0.8f);
                overtakingAggression = Random.Range(0.8f, 1.0f);
                pressureResistance = Random.Range(0.4f, 0.6f);
                break;
                
            case PersonalityType.Conservative:
                aggression = Random.Range(0.2f, 0.4f);
                skill = Random.Range(0.7f, 0.9f);
                consistency = Random.Range(0.8f, 1.0f);
                riskTaking = Random.Range(0.1f, 0.3f);
                patience = Random.Range(0.7f, 0.9f);
                nitroConservation = Random.Range(0.7f, 0.9f);
                defensiveDriving = Random.Range(0.7f, 0.9f);
                pressureResistance = Random.Range(0.7f, 0.9f);
                break;
                
            case PersonalityType.Opportunist:
                aggression = Random.Range(0.5f, 0.7f);
                skill = Random.Range(0.6f, 0.8f);
                consistency = Random.Range(0.6f, 0.8f);
                riskTaking = Random.Range(0.4f, 0.6f);
                patience = Random.Range(0.6f, 0.8f);
                nitroAggression = Random.Range(0.4f, 0.6f);
                overtakingAggression = Random.Range(0.6f, 0.8f);
                comebackDrive = Random.Range(0.7f, 0.9f);
                break;
                
            case PersonalityType.Hothead:
                aggression = Random.Range(0.9f, 1.0f);
                skill = Random.Range(0.5f, 0.7f);
                consistency = Random.Range(0.2f, 0.4f);
                riskTaking = Random.Range(0.9f, 1.0f);
                patience = Random.Range(0.0f, 0.2f);
                nitroAggression = Random.Range(0.9f, 1.0f);
                blockingTendency = Random.Range(0.8f, 1.0f);
                pressureResistance = Random.Range(0.1f, 0.3f);
                break;
                
            case PersonalityType.Veteran:
                aggression = Random.Range(0.4f, 0.6f);
                skill = Random.Range(0.8f, 1.0f);
                consistency = Random.Range(0.8f, 1.0f);
                riskTaking = Random.Range(0.3f, 0.5f);
                patience = Random.Range(0.7f, 0.9f);
                nitroConservation = Random.Range(0.6f, 0.8f);
                nitroDefense = Random.Range(0.7f, 0.9f);
                defensiveDriving = Random.Range(0.6f, 0.8f);
                pressureResistance = Random.Range(0.8f, 1.0f);
                break;
                
            case PersonalityType.Rookie:
                aggression = Random.Range(0.3f, 0.5f);
                skill = Random.Range(0.3f, 0.5f);
                consistency = Random.Range(0.2f, 0.4f);
                riskTaking = Random.Range(0.2f, 0.4f);
                patience = Random.Range(0.4f, 0.6f);
                nitroConservation = Random.Range(0.8f, 1.0f);
                pressureResistance = Random.Range(0.2f, 0.4f);
                comebackDrive = Random.Range(0.3f, 0.5f);
                break;
                
            case PersonalityType.Blocker:
                aggression = Random.Range(0.6f, 0.8f);
                skill = Random.Range(0.6f, 0.8f);
                consistency = Random.Range(0.7f, 0.9f);
                riskTaking = Random.Range(0.3f, 0.5f);
                patience = Random.Range(0.5f, 0.7f);
                nitroDefense = Random.Range(0.8f, 1.0f);
                blockingTendency = Random.Range(0.8f, 1.0f);
                defensiveDriving = Random.Range(0.8f, 1.0f);
                break;
                
            case PersonalityType.Speedster:
                aggression = Random.Range(0.7f, 0.9f);
                skill = Random.Range(0.7f, 0.9f);
                consistency = Random.Range(0.6f, 0.8f);
                riskTaking = Random.Range(0.6f, 0.8f);
                patience = Random.Range(0.3f, 0.5f);
                nitroAggression = Random.Range(0.7f, 0.9f);
                overtakingAggression = Random.Range(0.7f, 0.9f);
                blockingTendency = Random.Range(0.2f, 0.4f);
                break;
        }
    }
}
