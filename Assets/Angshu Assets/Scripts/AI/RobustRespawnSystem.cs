using UnityEngine;
using System.Collections;
using Ashsvp;

/// <summary>
/// Robust respawn system that completely stops the car, teleports it cleanly, and prevents flickering
/// </summary>
public class RobustRespawnSystem : MonoBehaviour
{
    [Header("Respawn Settings")]
    [SerializeField] private float respawnHeight = 2f;
    [SerializeField] private float respawnForwardOffset = 5f;
    [SerializeField] private float respawnInvulnerabilityTime = 3f;
    [SerializeField] private float initialForwardSpeed = 10f; // Speed to give car after respawn
    
    [Header("Fall Detection")]
    [SerializeField] private float fallThreshold = -50f;
    [SerializeField] private float fallCheckInterval = 0.5f;
    [SerializeField] private bool enableFallDetection = true;
    
    private Rigidbody vehicleRigidbody;
    private SimcadeVehicleController vehicleController;
    private AIVehicleController aiController;
    private SmoothSteeringController smoothSteering;
    
    private float lastRespawnTime = -999f;
    private float lastFallCheck = 0f;
    private bool isRespawning = false;
    
    private AIRaceManager raceManager;
    
    private void Start()
    {
        vehicleRigidbody = GetComponent<Rigidbody>();
        vehicleController = GetComponent<SimcadeVehicleController>();
        aiController = GetComponent<AIVehicleController>();
        smoothSteering = GetComponent<SmoothSteeringController>();
        
        raceManager = FindFirstObjectByType<AIRaceManager>();
        
        if (vehicleRigidbody == null)
        {
            Debug.LogError($"[RobustRespawn] No Rigidbody found on {gameObject.name}");
            enabled = false;
        }
    }
    
    private void Update()
    {
        if (!enableFallDetection || isRespawning) return;
        
        // Periodic fall detection
        if (Time.time - lastFallCheck > fallCheckInterval)
        {
            lastFallCheck = Time.time;
            
            // Skip if in invulnerability
            if (IsInvulnerable()) return;
            
            // Check if fallen below threshold
            if (transform.position.y < fallThreshold)
            {
                // Fall detection log removed - normal operation
                TriggerRespawn();
            }
        }
    }
    
    private void OnTriggerEnter(Collider other)
    {
        if (isRespawning || IsInvulnerable()) return;
        
        // Handle fall trigger
        if (other.CompareTag("Fall"))
        {
            // Fall trigger log removed - normal operation
            TriggerRespawn();
        }
        
        // Handle checkpoint detection
        if (other.CompareTag("Checkpoint") && raceManager != null)
        {
            raceManager.HandleCheckpoint(gameObject, other.transform);
        }
        
        // Handle start/finish line
        if (other.CompareTag("StartFinish") && raceManager != null)
        {
            raceManager.HandleStartFinish(gameObject);
        }
    }
    
    private void TriggerRespawn()
    {
        if (isRespawning) return;
        
        if (raceManager != null)
        {
            raceManager.HandleCarFall(gameObject);
        }
        else
        {
            // Fallback: respawn at current position
            RespawnAt(transform.position, transform.rotation);
        }
    }
    
    /// <summary>
    /// Main respawn method - completely stops the car and teleports it cleanly
    /// </summary>
    public void RespawnAt(Vector3 position, Quaternion rotation)
    {
        if (isRespawning) return;
        
        StartCoroutine(RespawnSequence(position, rotation));
    }
    
    private IEnumerator RespawnSequence(Vector3 targetPosition, Quaternion targetRotation)
    {
        isRespawning = true;
        lastRespawnTime = Time.time;
        
        // Respawn sequence log removed - normal operation
        
        // STEP 1: Completely freeze the car
        FreezeVehicle();
        
        // Wait one physics frame to ensure everything is stopped
        yield return new WaitForFixedUpdate();
        
        // STEP 2: Calculate final respawn position
        Vector3 respawnPosition = targetPosition + Vector3.up * respawnHeight;
        respawnPosition += targetRotation * Vector3.forward * respawnForwardOffset;
        
        // STEP 3: Teleport the car (while still frozen)
        transform.position = respawnPosition;
        transform.rotation = targetRotation;
        
        // Wait another physics frame
        yield return new WaitForFixedUpdate();
        
        // STEP 4: Reset all vehicle systems
        ResetVehicleSystems();
        
        // STEP 5: Unfreeze and apply initial velocity
        UnfreezeVehicle(targetRotation);
        
        // Respawn completion log removed - normal operation
        
        // Wait a bit before allowing another respawn
        yield return new WaitForSeconds(0.5f);
        
        isRespawning = false;
    }
    
    private void FreezeVehicle()
    {
        if (vehicleRigidbody != null)
        {
            // Make kinematic to completely disable physics
            vehicleRigidbody.isKinematic = true;
            vehicleRigidbody.linearVelocity = Vector3.zero;
            vehicleRigidbody.angularVelocity = Vector3.zero;
        }
        
        // Stop all inputs
        if (vehicleController != null && vehicleController.inputManager != null)
        {
            vehicleController.inputManager.SetAIInputs(0f, 0f, 0f, false);
        }
    }
    
    private void UnfreezeVehicle(Quaternion rotation)
    {
        if (vehicleRigidbody != null)
        {
            // Re-enable physics
            vehicleRigidbody.isKinematic = false;
            
            // Clear velocities again just to be sure
            vehicleRigidbody.linearVelocity = Vector3.zero;
            vehicleRigidbody.angularVelocity = Vector3.zero;
            
            // Apply initial forward velocity
            Vector3 forwardVelocity = rotation * Vector3.forward * initialForwardSpeed;
            vehicleRigidbody.linearVelocity = forwardVelocity;
            
            vehicleRigidbody.WakeUp();
        }
    }
    
    private void ResetVehicleSystems()
    {
        // Reset vehicle controller
        if (vehicleController != null)
        {
            // Stop nitro if active
            if (vehicleController.isNitroActive)
            {
                var nitroField = typeof(SimcadeVehicleController).GetField("isNitroActive", 
                    System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                nitroField?.SetValue(vehicleController, false);
            }
        }
        
        // Reset AI controller
        if (aiController != null)
        {
            var resetMethod = typeof(AIVehicleController).GetMethod("ResetBehaviorModifiers", 
                System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance);
            resetMethod?.Invoke(aiController, null);
        }
        
        // Reset smooth steering
        smoothSteering?.ResetSmoothingState();
    }
    
    /// <summary>
    /// Respawn at checkpoint with proper track direction
    /// </summary>
    public void RespawnAtCheckpoint(Transform checkpoint)
    {
        if (checkpoint == null)
        {
            Debug.LogWarning($"[RobustRespawn] Checkpoint is null for {gameObject.name}");
            return;
        }
        
        Vector3 checkpointForward = checkpoint.forward;
        checkpointForward.y = 0f;
        checkpointForward.Normalize();
        
        Quaternion respawnRotation = Quaternion.LookRotation(checkpointForward, Vector3.up);
        
        RespawnAt(checkpoint.position, respawnRotation);
    }
    
    /// <summary>
    /// Respawn at start line
    /// </summary>
    public void RespawnAtStartLine(Transform startLine)
    {
        if (startLine == null)
        {
            Debug.LogWarning($"[RobustRespawn] Start line is null for {gameObject.name}");
            return;
        }
        
        RespawnAtCheckpoint(startLine);
    }
    
    /// <summary>
    /// Check if vehicle is in invulnerability period
    /// </summary>
    public bool IsInvulnerable()
    {
        return Time.time - lastRespawnTime < respawnInvulnerabilityTime;
    }
    
    /// <summary>
    /// Check if currently respawning
    /// </summary>
    public bool IsRespawning()
    {
        return isRespawning;
    }
    
    [ContextMenu("Test Respawn Here")]
    public void TestRespawnHere()
    {
        RespawnAt(transform.position, transform.rotation);
    }
}
