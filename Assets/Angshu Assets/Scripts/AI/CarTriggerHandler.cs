using UnityEngine;

/// <summary>
/// Routes a car's trigger events to the race manager and watches for the cases the triggers miss:
/// falling out of the world, and ending up on the roof where no amount of throttle will help.
/// </summary>
public class CarTriggerHandler : MonoBehaviour
{
    [Header("References")]
    public AIRaceManager raceManager;

    [Header("Recovery")]
    [SerializeField, Tooltip("Height below which the car counts as having fallen out of the world.")]
    private float fallThreshold = -50f;

    [SerializeField, Tooltip("Seconds between recovery checks.")]
    private float checkInterval = 0.5f;

    [SerializeField, Tooltip("Seconds a car may spend upside down before it is put back on the line.")]
    private float overturnedGrace = 3f;

    [SerializeField, Tooltip("Seconds between recoveries, so a bad respawn cannot loop.")]
    private float recoveryCooldown = 3f;

    private float nextCheckTime;
    private float overturnedTime;
    private float nextRecoveryTime;

    private void Start()
    {
        if (raceManager != null) return;

        raceManager = FindFirstObjectByType<AIRaceManager>();
        if (raceManager == null)
        {
            Debug.LogError($"[CarTriggerHandler] No AIRaceManager in the scene; {name} cannot report its progress.");
            enabled = false;
        }
    }

    private void Update()
    {
        if (Time.time < nextCheckTime) return;
        nextCheckTime = Time.time + checkInterval;

        if (Time.time < nextRecoveryTime) return;

        if (transform.position.y < fallThreshold)
        {
            Recover("fell out of the world");
            return;
        }

        // Vector3.up dotted with the car's own up goes negative once it is past horizontal
        if (Vector3.Dot(transform.up, Vector3.up) < -0.1f)
        {
            overturnedTime += checkInterval;
            if (overturnedTime >= overturnedGrace) Recover("came to rest upside down");
            return;
        }

        overturnedTime = 0f;
    }

    private void Recover(string reason)
    {
        overturnedTime = 0f;
        nextRecoveryTime = Time.time + recoveryCooldown;
        Debug.Log($"[CarTriggerHandler] {name} {reason}; returning it to the racing line.");
        raceManager.HandleCarFall(gameObject);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (raceManager == null) return;

        if (other.CompareTag("Fall"))
        {
            if (Time.time < nextRecoveryTime) return;
            nextRecoveryTime = Time.time + recoveryCooldown;
            raceManager.HandleCarFall(gameObject);
        }
        else if (other.CompareTag("Checkpoint"))
        {
            raceManager.HandleCheckpoint(gameObject, other.transform);
        }
        else if (other.CompareTag("StartFinish"))
        {
            raceManager.HandleStartFinish(gameObject);
        }
    }
}
