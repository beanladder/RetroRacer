using UnityEngine;
using System.Collections;

/// <summary>
/// Controls race state and manages player input during race phases
/// </summary>
public class RaceController : MonoBehaviour
{
    [Header("Race Control")]
    [SerializeField] private bool enableRaceControl = true;
    
    // Race state
    public static bool IsRaceActive { get; private set; } = false;
    public static bool IsCountdownActive { get; private set; } = false;
    
    // Events
    public static System.Action OnRaceStart;
    public static System.Action OnCountdownStart;
    public static System.Action<int> OnCountdownTick; // Countdown number (3, 2, 1, 0=GO)
    
    private AIRaceManager raceManager;
    
    private void Start()
    {
        if (!enableRaceControl) return;
        
        raceManager = FindFirstObjectByType<AIRaceManager>();
        
        // Reset race state
        IsRaceActive = false;
        IsCountdownActive = false;
        
        // Start countdown after a brief delay
        StartCoroutine(StartRaceSequence());
    }
    
    private IEnumerator StartRaceSequence()
    {
        // Wait for everything to initialize
        yield return new WaitForSeconds(2f);
        
        // Start countdown
        IsCountdownActive = true;
        OnCountdownStart?.Invoke();
        
        Debug.Log("<color=cyan>=== RACE STARTING ===</color>");
        
        // Countdown: 3, 2, 1, GO!
        for (int i = 3; i > 0; i--)
        {
            Debug.Log($"<color=yellow>{i}...</color>");
            OnCountdownTick?.Invoke(i);
            yield return new WaitForSeconds(1f);
        }
        
        // GO!
        Debug.Log("<color=green>GO!</color>");
        OnCountdownTick?.Invoke(0); // 0 = GO
        
        // Enable racing
        IsCountdownActive = false;
        IsRaceActive = true;
        OnRaceStart?.Invoke();
        
        // Enable AI racing
        if (raceManager != null)
        {
            var aiRacers = FindObjectsByType<AIVehicleController>(FindObjectsSortMode.None);
            foreach (var racer in aiRacers)
            {
                if (racer != null)
                {
                    racer.IsRacing = true;
                }
            }
        }
        
        Debug.Log("<color=green>=== RACE ACTIVE ===</color>");
    }
    
    public static void EndRace()
    {
        IsRaceActive = false;
        IsCountdownActive = false;
        Debug.Log("<color=red>=== RACE ENDED ===</color>");
    }
    
    // Method to restart race
    [ContextMenu("Restart Race")]
    public void RestartRace()
    {
        StopAllCoroutines();
        IsRaceActive = false;
        IsCountdownActive = false;
        StartCoroutine(StartRaceSequence());
    }
}