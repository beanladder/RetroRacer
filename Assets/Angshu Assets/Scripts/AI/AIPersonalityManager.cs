using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class AIPersonalityManager : MonoBehaviour
{
    [Header("Personality Configuration")]
    [SerializeField, Tooltip("Personality assigned by Race Manager - do not set manually")]
    private AIPersonalityData personalityData;
    
    [Header("Rivalry System")]
    [SerializeField] private List<AIPersonalityManager> rivals = new List<AIPersonalityManager>();
    [SerializeField] private float rivalryIntensity = 1f;
    [SerializeField] private bool isInRivalry = false;
    
    // Internal state
    private AIVehicleController aiController;
    private float rivalryTimer = 0f;
    
    private void Start()
    {
        aiController = GetComponent<AIVehicleController>();
        
        // Register with race context manager
        if (RaceContextManager.Instance != null)
        {
            RaceContextManager.Instance.RegisterAICar(aiController);
        }
        
        // Personality will be assigned by AIRaceManager before Start() is called
        // If no personality assigned by race manager, generate a random one as fallback
        if (personalityData == null)
        {
            Debug.LogWarning($"[AIPersonality] {gameObject.name} has no personality assigned by Race Manager. Generating random personality as fallback.");
            personalityData = GenerateRandomPersonality();
        }
        
        // Apply personality to AI controller
        ApplyPersonalityToAI();
        
        // Delay rivalry setup to ensure all AI personalities are initialized
        StartCoroutine(SetupRivalriesDelayed());
        
        Debug.Log($"[AIPersonality] {gameObject.name} personality: {personalityData.personalityType} - {personalityData.GetPersonalityDescription()}");
    }
    
    private void Update()
    {
        if (!aiController.IsRacing || personalityData == null) return;

        // Handle rivalries
        HandleRivalrySystem();
        
        // Apply personality effects
        ApplyPersonalityEffects();
    }
    
    private AIPersonalityData GenerateRandomPersonality()
    {
        // Weighted personality distribution for more interesting races
        float rand = Random.value;
        AIPersonalityData.PersonalityType selectedType;
        
        if (rand < 0.15f) selectedType = AIPersonalityData.PersonalityType.Aggressive;
        else if (rand < 0.25f) selectedType = AIPersonalityData.PersonalityType.Conservative;
        else if (rand < 0.4f) selectedType = AIPersonalityData.PersonalityType.Opportunist;
        else if (rand < 0.5f) selectedType = AIPersonalityData.PersonalityType.Hothead;
        else if (rand < 0.65f) selectedType = AIPersonalityData.PersonalityType.Veteran;
        else if (rand < 0.75f) selectedType = AIPersonalityData.PersonalityType.Rookie;
        else if (rand < 0.85f) selectedType = AIPersonalityData.PersonalityType.Blocker;
        else selectedType = AIPersonalityData.PersonalityType.Speedster;
        
        return AIPersonalityData.CreateRandomized(selectedType);
    }
    
    private void ApplyPersonalityToAI()
    {
        if (aiController == null || personalityData == null) return;
        
        // Apply personality traits to AI controller settings
        float skillMultiplier = personalityData.skill;
        float aggressionMultiplier = personalityData.aggression;
        float riskMultiplier = personalityData.riskTaking;
        
        // Apply these through reflection or public properties
        ApplyPersonalitySettings(skillMultiplier, aggressionMultiplier, riskMultiplier);
    }
    
    private void ApplyPersonalitySettings(float skill, float aggression, float risk)
    {
        // Use public setters instead of reflection - clean and safe
        if (aiController != null)
        {
            aiController.SetSkillLevel(skill);
            aiController.SetAggressiveness(aggression);
            aiController.SetRiskTaking(risk);
        }
    }
    
    private IEnumerator SetupRivalriesDelayed()
    {
        // Wait longer to ensure all AI personalities are initialized
        yield return new WaitForSeconds(2f);
        SetupRivalries();
    }
    
    private void SetupRivalries()
    {
        // Find potential rivals based on personality compatibility
        var allAIManagers = FindObjectsByType<AIPersonalityManager>(FindObjectsSortMode.None);
        
        foreach (var otherManager in allAIManagers)
        {
            // Add null checks to prevent NullReferenceException
            if (otherManager != this && 
                personalityData != null && 
                otherManager.personalityData != null && 
                ShouldBeRivals(personalityData, otherManager.personalityData))
            {
                if (!rivals.Contains(otherManager))
                {
                    rivals.Add(otherManager);
                    otherManager.rivals.Add(this);
                    // Rivalry log kept - important for AI behavior verification
                    Debug.Log($"[Rivalry] {gameObject.name} ({personalityData.personalityType}) vs {otherManager.gameObject.name} ({otherManager.personalityData.personalityType})");
                }
            }
        }
    }
    
    private bool ShouldBeRivals(AIPersonalityData p1, AIPersonalityData p2)
    {
        // Safety check - ensure both personalities are valid
        if (p1 == null || p2 == null) return false;
        
        // Aggressive personalities clash with each other
        if (p1.personalityType == AIPersonalityData.PersonalityType.Aggressive && 
            p2.personalityType == AIPersonalityData.PersonalityType.Aggressive)
            return Random.value < 0.7f;
        
        // Hotheads clash with everyone
        if (p1.personalityType == AIPersonalityData.PersonalityType.Hothead || 
            p2.personalityType == AIPersonalityData.PersonalityType.Hothead)
            return Random.value < 0.5f;
        
        // Blockers vs Speedsters
        if ((p1.personalityType == AIPersonalityData.PersonalityType.Blocker && 
             p2.personalityType == AIPersonalityData.PersonalityType.Speedster) ||
            (p1.personalityType == AIPersonalityData.PersonalityType.Speedster && 
             p2.personalityType == AIPersonalityData.PersonalityType.Blocker))
            return Random.value < 0.6f;
        
        // Veterans vs Rookies (mentorship rivalry)
        if ((p1.personalityType == AIPersonalityData.PersonalityType.Veteran && 
             p2.personalityType == AIPersonalityData.PersonalityType.Rookie) ||
            (p1.personalityType == AIPersonalityData.PersonalityType.Rookie && 
             p2.personalityType == AIPersonalityData.PersonalityType.Veteran))
            return Random.value < 0.4f;
        
        return Random.value < 0.2f; // Small chance for any two drivers to be rivals
    }
    
    
    private void ModifyAIBehavior(string behaviorType, float modifier)
    {
        // Use the behavior modification system in AIVehicleController
        if (aiController != null)
        {
            aiController.ModifyBehavior(behaviorType, modifier);
        }
    }
    
    private void HandleRivalrySystem()
    {
        rivalryTimer -= Time.deltaTime;
        isInRivalry = false;
        
        if (rivalryTimer > 0f) return;
        
        // Check if any rivals are nearby
        foreach (var rival in rivals)
        {
            if (rival == null) continue;
            
            float distance = Vector3.Distance(transform.position, rival.transform.position);
            if (distance < 30f) // Rivalry activates when close
            {
                isInRivalry = true;
                rivalryIntensity = Mathf.Clamp01(1f - (distance / 30f)); // Closer = more intense
                
                // Apply rivalry effects
                ApplyRivalryEffects(rival);
                
                rivalryTimer = 1f; // Check again in 1 second
                break;
            }
        }
    }
    
    private void ApplyRivalryEffects(AIPersonalityManager rival)
    {
        if (personalityData == null) return;
        
        // Increase aggression when near rival
        float rivalryBoost = rivalryIntensity * 0.3f;
        
        // Different personalities react differently to rivalries
        switch (personalityData.personalityType)
        {
            case AIPersonalityData.PersonalityType.Aggressive:
            case AIPersonalityData.PersonalityType.Hothead:
                // Become more aggressive
                ModifyAIBehavior("aggression", rivalryBoost);
                break;
                
            case AIPersonalityData.PersonalityType.Blocker:
                // Increase blocking behavior
                ModifyAIBehavior("blocking", rivalryBoost);
                break;
                
            case AIPersonalityData.PersonalityType.Speedster:
                // Push harder for speed
                ModifyAIBehavior("speed", rivalryBoost);
                break;
                
            case AIPersonalityData.PersonalityType.Veteran:
                // Become more calculated and defensive
                ModifyAIBehavior("defense", rivalryBoost);
                break;
        }
    }
    
    private void ApplyPersonalityEffects()
    {
        if (personalityData == null) return;
        
        // Continuously apply personality effects based on race situation
        
        // Comeback drive - push harder when behind
        if (personalityData.comebackDrive > 0.5f)
        {
            int position = GetCurrentPosition();
            if (position > 3) // Behind
            {
                float comebackBoost = personalityData.comebackDrive * 0.2f;
                ModifyAIBehavior("aggression", comebackBoost);
            }
        }
        
        // Consistency - reduce random variations
        if (personalityData.consistency > 0.7f)
        {
            ModifyAIBehavior("pathRandomness", -personalityData.consistency * 0.5f);
        }
    }
    
    private int GetCurrentPosition()
    {
        // Use shared race context manager for position
        if (RaceContextManager.Instance != null)
        {
            var context = RaceContextManager.Instance.GetRaceContext(aiController);
            return context.position;
        }
        
        // Fallback to direct race manager query
        var raceManager = FindFirstObjectByType<AIRaceManager>();
        if (raceManager != null && raceManager.SortedRacers.Contains(aiController))
        {
            return raceManager.SortedRacers.IndexOf(aiController) + 1;
        }
        return 1;
    }
    
    private void OnDestroy()
    {
        // Unregister from race context manager
        if (RaceContextManager.Instance != null && aiController != null)
        {
            RaceContextManager.Instance.UnregisterAICar(aiController);
        }
    }
    
    public AIPersonalityData GetPersonality() => personalityData;
    public bool IsInRivalry() => isInRivalry;
    
    /// <summary>
    /// Assign a personality to this AI (used by race manager)
    /// </summary>
    public void AssignPersonality(AIPersonalityData personality)
    {
        personalityData = personality;
        ApplyPersonalityToAI();
    }
}