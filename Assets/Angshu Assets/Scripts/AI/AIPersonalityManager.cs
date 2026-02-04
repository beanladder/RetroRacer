using UnityEngine;
using System.Collections.Generic;

public class AIPersonalityManager : MonoBehaviour
{
    // Merged AIPersonality data structure
    [System.Serializable]
    public class AIPersonality
    {
        [Header("Personality Type")]
        public PersonalityType personalityType;
        
        [Header("Core Traits")]
        [Range(0f, 1f)] public float aggression = 0.5f;
        [Range(0f, 1f)] public float skill = 0.7f;
        [Range(0f, 1f)] public float consistency = 0.6f;
        [Range(0f, 1f)] public float riskTaking = 0.5f;
        [Range(0f, 1f)] public float patience = 0.5f;
        
        [Header("Nitro Strategy")]
        [Range(0f, 1f)] public float nitroAggression = 0.5f;
        [Range(0f, 1f)] public float nitroDefense = 0.3f;
        [Range(0f, 1f)] public float nitroConservation = 0.4f;
        
        [Header("Racing Behavior")]
        [Range(0f, 1f)] public float blockingTendency = 0.3f;
        [Range(0f, 1f)] public float overtakingAggression = 0.5f;
        [Range(0f, 1f)] public float defensiveDriving = 0.4f;
        [Range(0f, 1f)] public float mistakeProneness = 0.2f;
        
        [Header("Pressure Response")]
        [Range(0f, 1f)] public float pressureResistance = 0.6f;
        [Range(0f, 1f)] public float comebackDrive = 0.5f;

        public enum PersonalityType
        {
            Aggressive,     // High aggression, low patience, risky
            Conservative,   // Low aggression, high patience, safe
            Opportunist,    // Medium aggression, waits for chances
            Hothead,        // Very aggressive, makes mistakes under pressure
            Veteran,        // High skill, consistent, strategic
            Rookie,         // Low skill, inconsistent, learns during race
            Blocker,        // Defensive, blocks other drivers
            Speedster       // Focuses on pure speed, less tactical
        }
        
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
                    return "Hot-tempered driver prone to mistakes under pressure";
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
    }
    [Header("Personality Configuration")]
    [SerializeField] private AIPersonality personality;
    [SerializeField] private bool randomizePersonality = true;
    [SerializeField] private AIPersonality.PersonalityType forcedPersonalityType;
    
    [Header("Mistake System")]
    [SerializeField] private float baseMistakeChance = 0.1f;
    [SerializeField] private float pressureMistakeMultiplier = 2f;
    [SerializeField] private float mistakeRecoveryTime = 2f;
    
    [Header("Rivalry System")]
    [SerializeField] private List<AIPersonalityManager> rivals = new List<AIPersonalityManager>();
    [SerializeField] private float rivalryIntensity = 1f;
    [SerializeField] private bool isInRivalry = false;
    
    // Internal state
    private AIVehicleController aiController;
    private float currentPressure = 0f;
    private float mistakeTimer = 0f;
    private bool isMakingMistake = false;
    private MistakeType currentMistake = MistakeType.None;
    private float rivalryTimer = 0f;
    
    // Mistake tracking
    private int mistakeCount = 0;
    private float lastMistakeTime = 0f;
    
    public enum MistakeType
    {
        None,
        BrakingTooLate,
        BrakingTooEarly,
        MissedApex,
        Oversteer,
        Understeer,
        ThrottleTooEarly,
        WrongLine
    }
    
    private void Start()
    {
        aiController = GetComponent<AIVehicleController>();
        
        // Register with race context manager
        if (RaceContextManager.Instance != null)
        {
            RaceContextManager.Instance.RegisterAICar(aiController);
        }
        
        // Generate or assign personality
        if (randomizePersonality)
        {
            GenerateRandomPersonality();
        }
        else
        {
            personality = CreatePersonality(forcedPersonalityType);
        }
        
        // Apply personality to AI controller
        ApplyPersonalityToAI();
        
        // Set up rivalries
        SetupRivalries();
        
        Debug.Log($"[AIPersonality] {gameObject.name} personality: {personality.personalityType} - {personality.GetPersonalityDescription()}");
    }
    
    private void Update()
    {
        if (!aiController.IsRacing) return;
        
        // Update pressure level
        UpdatePressureLevel();
        
        // Handle mistakes
        HandleMistakeSystem();
        
        // Handle rivalries
        HandleRivalrySystem();
        
        // Apply personality effects
        ApplyPersonalityEffects();
    }
    
    private void GenerateRandomPersonality()
    {
        // Weighted personality distribution for more interesting races
        float rand = Random.value;
        AIPersonality.PersonalityType selectedType;
        
        if (rand < 0.15f) selectedType = AIPersonality.PersonalityType.Aggressive;
        else if (rand < 0.25f) selectedType = AIPersonality.PersonalityType.Conservative;
        else if (rand < 0.4f) selectedType = AIPersonality.PersonalityType.Opportunist;
        else if (rand < 0.5f) selectedType = AIPersonality.PersonalityType.Hothead;
        else if (rand < 0.65f) selectedType = AIPersonality.PersonalityType.Veteran;
        else if (rand < 0.75f) selectedType = AIPersonality.PersonalityType.Rookie;
        else if (rand < 0.85f) selectedType = AIPersonality.PersonalityType.Blocker;
        else selectedType = AIPersonality.PersonalityType.Speedster;
        
        personality = CreatePersonality(selectedType);
    }
    
    // Merged personality creation factory method
    private AIPersonality CreatePersonality(AIPersonality.PersonalityType type)
    {
        AIPersonality personality = new AIPersonality();
        personality.personalityType = type;
        
        switch (type)
        {
            case AIPersonality.PersonalityType.Aggressive:
                personality.aggression = Random.Range(0.8f, 1.0f);
                personality.skill = Random.Range(0.6f, 0.8f);
                personality.consistency = Random.Range(0.4f, 0.6f);
                personality.riskTaking = Random.Range(0.8f, 1.0f);
                personality.patience = Random.Range(0.1f, 0.3f);
                personality.nitroAggression = Random.Range(0.8f, 1.0f);
                personality.blockingTendency = Random.Range(0.6f, 0.8f);
                personality.overtakingAggression = Random.Range(0.8f, 1.0f);
                personality.mistakeProneness = Random.Range(0.3f, 0.5f);
                personality.pressureResistance = Random.Range(0.4f, 0.6f);
                break;
                
            case AIPersonality.PersonalityType.Conservative:
                personality.aggression = Random.Range(0.2f, 0.4f);
                personality.skill = Random.Range(0.7f, 0.9f);
                personality.consistency = Random.Range(0.8f, 1.0f);
                personality.riskTaking = Random.Range(0.1f, 0.3f);
                personality.patience = Random.Range(0.7f, 0.9f);
                personality.nitroConservation = Random.Range(0.7f, 0.9f);
                personality.defensiveDriving = Random.Range(0.7f, 0.9f);
                personality.mistakeProneness = Random.Range(0.1f, 0.2f);
                personality.pressureResistance = Random.Range(0.7f, 0.9f);
                break;
                
            case AIPersonality.PersonalityType.Opportunist:
                personality.aggression = Random.Range(0.5f, 0.7f);
                personality.skill = Random.Range(0.6f, 0.8f);
                personality.consistency = Random.Range(0.6f, 0.8f);
                personality.riskTaking = Random.Range(0.4f, 0.6f);
                personality.patience = Random.Range(0.6f, 0.8f);
                personality.nitroAggression = Random.Range(0.4f, 0.6f);
                personality.overtakingAggression = Random.Range(0.6f, 0.8f);
                personality.mistakeProneness = Random.Range(0.2f, 0.3f);
                personality.comebackDrive = Random.Range(0.7f, 0.9f);
                break;
                
            case AIPersonality.PersonalityType.Hothead:
                personality.aggression = Random.Range(0.9f, 1.0f);
                personality.skill = Random.Range(0.5f, 0.7f);
                personality.consistency = Random.Range(0.2f, 0.4f);
                personality.riskTaking = Random.Range(0.9f, 1.0f);
                personality.patience = Random.Range(0.0f, 0.2f);
                personality.nitroAggression = Random.Range(0.9f, 1.0f);
                personality.blockingTendency = Random.Range(0.8f, 1.0f);
                personality.mistakeProneness = Random.Range(0.5f, 0.8f);
                personality.pressureResistance = Random.Range(0.1f, 0.3f);
                break;
                
            case AIPersonality.PersonalityType.Veteran:
                personality.aggression = Random.Range(0.4f, 0.6f);
                personality.skill = Random.Range(0.8f, 1.0f);
                personality.consistency = Random.Range(0.8f, 1.0f);
                personality.riskTaking = Random.Range(0.3f, 0.5f);
                personality.patience = Random.Range(0.7f, 0.9f);
                personality.nitroConservation = Random.Range(0.6f, 0.8f);
                personality.nitroDefense = Random.Range(0.7f, 0.9f);
                personality.defensiveDriving = Random.Range(0.6f, 0.8f);
                personality.mistakeProneness = Random.Range(0.0f, 0.1f);
                personality.pressureResistance = Random.Range(0.8f, 1.0f);
                break;
                
            case AIPersonality.PersonalityType.Rookie:
                personality.aggression = Random.Range(0.3f, 0.5f);
                personality.skill = Random.Range(0.3f, 0.5f);
                personality.consistency = Random.Range(0.2f, 0.4f);
                personality.riskTaking = Random.Range(0.2f, 0.4f);
                personality.patience = Random.Range(0.4f, 0.6f);
                personality.nitroConservation = Random.Range(0.8f, 1.0f);
                personality.mistakeProneness = Random.Range(0.4f, 0.7f);
                personality.pressureResistance = Random.Range(0.2f, 0.4f);
                personality.comebackDrive = Random.Range(0.3f, 0.5f);
                break;
                
            case AIPersonality.PersonalityType.Blocker:
                personality.aggression = Random.Range(0.6f, 0.8f);
                personality.skill = Random.Range(0.6f, 0.8f);
                personality.consistency = Random.Range(0.7f, 0.9f);
                personality.riskTaking = Random.Range(0.3f, 0.5f);
                personality.patience = Random.Range(0.5f, 0.7f);
                personality.nitroDefense = Random.Range(0.8f, 1.0f);
                personality.blockingTendency = Random.Range(0.8f, 1.0f);
                personality.defensiveDriving = Random.Range(0.8f, 1.0f);
                personality.mistakeProneness = Random.Range(0.2f, 0.3f);
                break;
                
            case AIPersonality.PersonalityType.Speedster:
                personality.aggression = Random.Range(0.7f, 0.9f);
                personality.skill = Random.Range(0.7f, 0.9f);
                personality.consistency = Random.Range(0.6f, 0.8f);
                personality.riskTaking = Random.Range(0.6f, 0.8f);
                personality.patience = Random.Range(0.3f, 0.5f);
                personality.nitroAggression = Random.Range(0.7f, 0.9f);
                personality.overtakingAggression = Random.Range(0.7f, 0.9f);
                personality.blockingTendency = Random.Range(0.2f, 0.4f);
                personality.mistakeProneness = Random.Range(0.2f, 0.4f);
                break;
        }
        
        return personality;
    }
    
    private void ApplyPersonalityToAI()
    {
        if (aiController == null) return;
        
        // Apply personality traits to AI controller settings
        // These would need to be exposed as public properties in AIVehicleController
        
        // Skill affects precision and consistency
        float skillMultiplier = personality.skill;
        
        // Aggression affects speed targets and overtaking behavior
        float aggressionMultiplier = personality.aggression;
        
        // Risk-taking affects corner speeds and overtaking attempts
        float riskMultiplier = personality.riskTaking;
        
        // Apply these through reflection or public properties
        ApplyPersonalitySettings(skillMultiplier, aggressionMultiplier, riskMultiplier);
    }
    
    private void ApplyPersonalitySettings(float skill, float aggression, float risk)
    {
        // This would require exposing more properties in AIVehicleController
        // For now, we'll modify the existing public properties
        
        var aiType = typeof(AIVehicleController);
        
        // Try to set skill level if property exists
        var skillProperty = aiType.GetField("skillLevel", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (skillProperty != null)
        {
            skillProperty.SetValue(aiController, skill);
        }
        
        // Try to set aggressiveness if property exists
        var aggressivenessProperty = aiType.GetField("aggressiveness", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (aggressivenessProperty != null)
        {
            aggressivenessProperty.SetValue(aiController, aggression);
        }
    }
    
    private void SetupRivalries()
    {
        // Find potential rivals based on personality compatibility
        var allAIManagers = FindObjectsByType<AIPersonalityManager>(FindObjectsSortMode.None);
        
        foreach (var otherManager in allAIManagers)
        {
            if (otherManager != this && ShouldBeRivals(personality, otherManager.personality))
            {
                if (!rivals.Contains(otherManager))
                {
                    rivals.Add(otherManager);
                    otherManager.rivals.Add(this);
                    // Rivalry log kept - important for AI behavior verification
                    Debug.Log($"[Rivalry] {gameObject.name} ({personality.personalityType}) vs {otherManager.gameObject.name} ({otherManager.personality.personalityType})");
                }
            }
        }
    }
    
    private bool ShouldBeRivals(AIPersonality p1, AIPersonality p2)
    {
        // Aggressive personalities clash with each other
        if (p1.personalityType == AIPersonality.PersonalityType.Aggressive && 
            p2.personalityType == AIPersonality.PersonalityType.Aggressive)
            return Random.value < 0.7f;
        
        // Hotheads clash with everyone
        if (p1.personalityType == AIPersonality.PersonalityType.Hothead || 
            p2.personalityType == AIPersonality.PersonalityType.Hothead)
            return Random.value < 0.5f;
        
        // Blockers vs Speedsters
        if ((p1.personalityType == AIPersonality.PersonalityType.Blocker && 
             p2.personalityType == AIPersonality.PersonalityType.Speedster) ||
            (p1.personalityType == AIPersonality.PersonalityType.Speedster && 
             p2.personalityType == AIPersonality.PersonalityType.Blocker))
            return Random.value < 0.6f;
        
        // Veterans vs Rookies (mentorship rivalry)
        if ((p1.personalityType == AIPersonality.PersonalityType.Veteran && 
             p2.personalityType == AIPersonality.PersonalityType.Rookie) ||
            (p1.personalityType == AIPersonality.PersonalityType.Rookie && 
             p2.personalityType == AIPersonality.PersonalityType.Veteran))
            return Random.value < 0.4f;
        
        return Random.value < 0.2f; // Small chance for any two drivers to be rivals
    }
    
    private void UpdatePressureLevel()
    {
        // Use shared race context manager for pressure calculation
        if (RaceContextManager.Instance != null)
        {
            currentPressure = RaceContextManager.Instance.CalculatePressureLevel(
                aiController, 
                personality, 
                isInRivalry, 
                rivalryIntensity, 
                mistakeCount
            );
        }
        else
        {
            // Fallback calculation if no context manager
            currentPressure = 0f;
            
            if (isInRivalry)
            {
                currentPressure += 0.3f * rivalryIntensity;
            }
            
            if (mistakeCount > 2)
            {
                currentPressure += 0.2f;
            }
            
            currentPressure *= (1f - personality.pressureResistance);
            currentPressure = Mathf.Clamp01(currentPressure);
        }
    }
    
    private void HandleMistakeSystem()
    {
        mistakeTimer -= Time.deltaTime;
        
        if (isMakingMistake)
        {
            if (mistakeTimer <= 0f)
            {
                // Recover from mistake
                RecoverFromMistake();
            }
            else
            {
                // Apply mistake effects
                ApplyMistakeEffects();
            }
        }
        else
        {
            // Check if should make a mistake
            float mistakeChance = CalculateMistakeChance();
            if (Random.value < mistakeChance * Time.deltaTime)
            {
                MakeMistake();
            }
        }
    }
    
    private float CalculateMistakeChance()
    {
        float chance = baseMistakeChance * personality.mistakeProneness;
        
        // Pressure increases mistake chance
        chance += currentPressure * pressureMistakeMultiplier * personality.mistakeProneness;
        
        // Rookies make more mistakes early, but learn
        if (personality.personalityType == AIPersonality.PersonalityType.Rookie)
        {
            float learningFactor = Mathf.Clamp01(Time.time / 120f); // Learn over 2 minutes
            chance *= (2f - learningFactor); // Start at 2x mistakes, reduce to 1x
        }
        
        // Veterans make fewer mistakes under pressure
        if (personality.personalityType == AIPersonality.PersonalityType.Veteran)
        {
            chance *= 0.5f;
        }
        
        // Hotheads make more mistakes when pressured
        if (personality.personalityType == AIPersonality.PersonalityType.Hothead)
        {
            chance += currentPressure * 0.5f;
        }
        
        return chance;
    }
    
    private void MakeMistake()
    {
        if (Time.time - lastMistakeTime < 5f) return; // Don't make mistakes too frequently
        
        isMakingMistake = true;
        mistakeTimer = mistakeRecoveryTime * Random.Range(0.8f, 1.2f);
        lastMistakeTime = Time.time;
        mistakeCount++;
        
        // Choose mistake type based on situation
        currentMistake = ChooseMistakeType();
        
        Debug.Log($"[Mistake] {gameObject.name} making mistake: {currentMistake} (Pressure: {currentPressure:F2})");
    }
    
    private MistakeType ChooseMistakeType()
    {
        // Choose mistake based on current situation and personality
        float cornerFactor = aiController.GetCornerFactor();
        
        if (cornerFactor > 0.5f) // In corner
        {
            float rand = Random.value;
            if (rand < 0.3f) return MistakeType.BrakingTooLate;
            else if (rand < 0.5f) return MistakeType.MissedApex;
            else if (rand < 0.7f) return MistakeType.Oversteer;
            else return MistakeType.Understeer;
        }
        else // On straight or light corner
        {
            float rand = Random.value;
            if (rand < 0.4f) return MistakeType.BrakingTooEarly;
            else if (rand < 0.7f) return MistakeType.ThrottleTooEarly;
            else return MistakeType.WrongLine;
        }
    }
    
    private void ApplyMistakeEffects()
    {
        // Apply mistake effects to AI behavior
        // This would require modifying AIVehicleController to accept these inputs
        
        switch (currentMistake)
        {
            case MistakeType.BrakingTooLate:
                // Reduce braking effectiveness
                ModifyAIBehavior("braking", -0.3f);
                break;
                
            case MistakeType.BrakingTooEarly:
                // Increase braking too much
                ModifyAIBehavior("braking", 0.5f);
                break;
                
            case MistakeType.MissedApex:
                // Add steering error
                ModifyAIBehavior("steering", Random.Range(-0.2f, 0.2f));
                break;
                
            case MistakeType.Oversteer:
                // Reduce steering response
                ModifyAIBehavior("steering", -0.4f);
                break;
                
            case MistakeType.Understeer:
                // Increase steering too much
                ModifyAIBehavior("steering", 0.3f);
                break;
                
            case MistakeType.ThrottleTooEarly:
                // Apply throttle too early in corner
                ModifyAIBehavior("throttle", 0.4f);
                break;
                
            case MistakeType.WrongLine:
                // Add path randomness
                ModifyAIBehavior("pathRandomness", 2f);
                break;
        }
    }
    
    private void ModifyAIBehavior(string behaviorType, float modifier)
    {
        // Use the new behavior modification system in AIVehicleController
        if (aiController != null)
        {
            aiController.ModifyBehavior(behaviorType, modifier);
        }
    }
    
    private void RecoverFromMistake()
    {
        isMakingMistake = false;
        currentMistake = MistakeType.None;
        
        // Reset any modified behaviors
        ResetAIBehavior();
        
        Debug.Log($"[Recovery] {gameObject.name} recovered from mistake");
    }
    
    private void ResetAIBehavior()
    {
        // Reset AI behavior to normal
        if (aiController != null)
        {
            aiController.ResetBehaviorModifiers();
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
        // Increase aggression when near rival
        float rivalryBoost = rivalryIntensity * 0.3f;
        
        // Different personalities react differently to rivalries
        switch (personality.personalityType)
        {
            case AIPersonality.PersonalityType.Aggressive:
            case AIPersonality.PersonalityType.Hothead:
                // Become more aggressive and mistake-prone
                ModifyAIBehavior("aggression", rivalryBoost);
                break;
                
            case AIPersonality.PersonalityType.Blocker:
                // Increase blocking behavior
                ModifyAIBehavior("blocking", rivalryBoost);
                break;
                
            case AIPersonality.PersonalityType.Speedster:
                // Push harder for speed
                ModifyAIBehavior("speed", rivalryBoost);
                break;
                
            case AIPersonality.PersonalityType.Veteran:
                // Become more calculated and defensive
                ModifyAIBehavior("defense", rivalryBoost);
                break;
        }
    }
    
    private void ApplyPersonalityEffects()
    {
        // Continuously apply personality effects based on race situation
        
        // Comeback drive - push harder when behind
        if (personality.comebackDrive > 0.5f)
        {
            int position = GetCurrentPosition();
            if (position > 3) // Behind
            {
                float comebackBoost = personality.comebackDrive * 0.2f;
                ModifyAIBehavior("aggression", comebackBoost);
            }
        }
        
        // Consistency - reduce random variations
        if (personality.consistency > 0.7f)
        {
            ModifyAIBehavior("pathRandomness", -personality.consistency * 0.5f);
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
    
    public AIPersonality GetPersonality() => personality;
    public bool IsInRivalry() => isInRivalry;
    public float GetCurrentPressure() => currentPressure;
    public bool IsMakingMistake() => isMakingMistake;
    public MistakeType GetCurrentMistake() => currentMistake;
}