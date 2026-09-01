using System.Collections;
using System.Collections.Generic;
using Track;
using UnityEngine;

/// <summary>
/// Spawns the grid, runs the race and keeps every driver's picture of the field up to date.
///
/// Position and progress are measured the same way for the player and for the AI: lap number
/// plus how far around the lap the car is on the racing line. Previously the AI reported a
/// lap-less 0..1 figure while the player reported lap plus checkpoint fraction, so an AI on its
/// third lap could sort behind a player on its first.
/// </summary>
public class AIRaceManager : MonoBehaviour
{
    [Header("AI Racers Configuration")]
    [SerializeField] private TrackGenerator trackGenerator;
    [SerializeField] private List<GameObject> aiVehiclePrefabs;
    [SerializeField, Range(1, 10)] private int numberOfAIRacers = 3;
    [SerializeField] private float startingOffset = 15f;

    [Header("AI Difficulty Settings")]
    [SerializeField, Range(0f, 1f)] private float minSkillLevel = 0.5f;
    [SerializeField, Range(0f, 1f)] private float maxSkillLevel = 0.9f;
    [SerializeField, Range(0f, 1f)] private float minAggressiveness = 0.3f;
    [SerializeField, Range(0f, 1f)] private float maxAggressiveness = 0.8f;

    [Header("Rubber Banding")]
    [SerializeField, Range(0f, 1f), Tooltip("How strongly the rubber banding effect pulls cars together.")]
    private float rubberBandingStrength = 0.5f;
    [SerializeField, Tooltip("The max speed boost given to cars that are behind.")]
    private float maxSpeedBoost = 1.2f;
    [SerializeField, Tooltip("The max speed penalty given to the car in the lead.")]
    private float maxSpeedPenalty = 0.9f;

    [Header("Race Game Mode")]
    [SerializeField, Range(1, 20)] private int numberOfLaps = 3;
    [SerializeField] private GameObject playerVehiclePrefab;
    [SerializeField] private bool spawnPlayer = true;
    [SerializeField] private string playerTag = "Player";

    [Header("AI Car Appearance")]
    [SerializeField] private List<Material> aiCarMaterials = new List<Material>();

    [Header("Diagnostics")]
    [SerializeField, Tooltip("Log grid, lap and position changes to the console.")]
    private bool verboseLogging = false;

    private class RaceCarState
    {
        public GameObject car;
        public RaceParticipant participant;
        public AIVehicleController aiController;
        public int currentLap = 1;
        public int lastCheckpoint = -1;
        public bool finished;
        public float finishTime;
        public int position;
        public float progress;
    }

    private readonly List<AIVehicleController> aiRacers = new List<AIVehicleController>();
    private readonly List<RaceCarState> raceCars = new List<RaceCarState>();
    private readonly Dictionary<GameObject, RaceCarState> stateByCar = new Dictionary<GameObject, RaceCarState>();
    private readonly List<RaceParticipant> participants = new List<RaceParticipant>();

    private readonly List<Transform> checkpoints = new List<Transform>();
    private readonly Dictionary<Transform, int> checkpointIndex = new Dictionary<Transform, int>();
    private Transform startFinishLine;

    private AITrackData track;
    private float startLineArcLength;

    private RaceCarState playerState;
    private bool raceIsActive;
    private bool raceFinished;
    private float raceStartTime;

    /// <summary>Cars in running order, best first.</summary>
    public List<AIVehicleController> SortedRacers { get; private set; } = new List<AIVehicleController>();

    /// <summary>Current position of each AI car, counted from 1.</summary>
    public Dictionary<AIVehicleController, int> CarPositions { get; private set; } = new Dictionary<AIVehicleController, int>();

    /// <summary>Every car in the race, AI and player alike.</summary>
    public IReadOnlyList<RaceParticipant> Participants => participants;

    /// <summary>The player's car, or null when the race is all AI.</summary>
    public GameObject PlayerCar => playerState?.car;

    /// <summary>
    /// Index into <see cref="Participants"/> that the debug HUD and spectator camera currently
    /// focus on. Kept here rather than in either of those so both stay in lock step: cycling the
    /// HUD's focus is exactly what the spectator camera orbits.
    /// </summary>
    public int FocusIndex { get; private set; }

    /// <summary>The currently focused participant, or null when the grid has not spawned yet.</summary>
    public RaceParticipant FocusedParticipant()
    {
        if (participants.Count == 0) return null;

        FocusIndex = ((FocusIndex % participants.Count) + participants.Count) % participants.Count;
        return participants[FocusIndex];
    }

    /// <summary>Steps the focus to the next participant and returns it.</summary>
    public RaceParticipant CycleFocus(int direction = 1)
    {
        if (participants.Count == 0) return null;

        FocusIndex += direction;
        return FocusedParticipant();
    }

    private void Start()
    {
        if (trackGenerator == null)
        {
            trackGenerator = FindFirstObjectByType<TrackGenerator>();
            if (trackGenerator == null)
            {
                Debug.LogError("[AIRaceManager] No TrackGenerator in the scene; the race cannot start.");
                enabled = false;
                return;
            }
        }

        StartCoroutine(SpawnAIRacersWhenReady());
        StartCoroutine(UpdatePositionsRoutine());
        StartCoroutine(PeriodicOvertakeOrders());
    }

    // ---------------------------------------------------------------- setup

    private IEnumerator SpawnAIRacersWhenReady()
    {
        // The track generator builds mesh, checkpoints and racing line across several frames
        float timeout = Time.time + 15f;
        while (Time.time < timeout)
        {
            if (trackGenerator.RacingLine != null && trackGenerator.RacingLine.Points.Count > 8) break;
            yield return new WaitForSeconds(0.25f);
        }

        if (trackGenerator.RacingLine == null || trackGenerator.RacingLine.Points.Count <= 8)
        {
            Debug.LogWarning("[AIRaceManager] No racing line on the track; AI racers will not be spawned.");
            yield break;
        }

        // The line may have been regenerated since anything last looked at it
        AITrackData.Invalidate(trackGenerator);
        track = AITrackData.For(trackGenerator);

        CacheTrackFeatures();
        SpawnAIRacers();
        BindParticipants();

        StartCoroutine(StartRaceCountdown());
    }

    private void CacheTrackFeatures()
    {
        checkpoints.Clear();
        checkpointIndex.Clear();
        startFinishLine = null;

        foreach (Transform child in trackGenerator.transform)
        {
            if (child.CompareTag("Checkpoint"))
            {
                checkpointIndex[child] = checkpoints.Count;
                checkpoints.Add(child);
            }
            else if (child.name == "StartFinishLine")
            {
                startFinishLine = child;
            }
        }

        // Measure lap progress from the start line rather than from racing line point zero,
        // otherwise progress jumps backwards somewhere in the middle of the lap.
        startLineArcLength = 0f;
        if (track != null && startFinishLine != null)
        {
            int index = track.Localize(startFinishLine.position, 0, track.Count / 2);
            startLineArcLength = track.ArcLengthAt(index, startFinishLine.position);
        }
    }

    private void SpawnAIRacers()
    {
        if (aiVehiclePrefabs == null || aiVehiclePrefabs.Count == 0)
        {
            Debug.LogError("[AIRaceManager] No AI vehicle prefabs assigned.");
            return;
        }

        foreach (var racer in aiRacers)
        {
            if (racer != null) Destroy(racer.gameObject);
        }

        aiRacers.Clear();
        raceCars.Clear();
        stateByCar.Clear();
        participants.Clear();
        playerState = null;
        raceFinished = false;

        Vector3 startPosition;
        Vector3 startDirection;

        if (startFinishLine != null)
        {
            startPosition = FindGroundPosition(startFinishLine.position);
            startDirection = startFinishLine.forward;
        }
        else
        {
            Debug.LogWarning("[AIRaceManager] No start/finish line found; falling back to the racing line.");
            startPosition = trackGenerator.transform.TransformPoint(trackGenerator.RacingLine.Points[0]);
            startDirection = trackGenerator.transform.TransformPoint(trackGenerator.RacingLine.Points[5]) - startPosition;
        }

        startDirection.y = 0f;
        startDirection = startDirection.sqrMagnitude > 1e-4f ? startDirection.normalized : Vector3.forward;

        SpawnCarsAtPosition(startPosition, startDirection);
    }

    private Vector3 FindGroundPosition(Vector3 startPosition)
    {
        if (Physics.Raycast(startPosition + Vector3.up * 10f, Vector3.down, out RaycastHit hit, 40f, ~0, QueryTriggerInteraction.Ignore))
        {
            return hit.point + Vector3.up * 0.5f;
        }

        return new Vector3(startPosition.x, startPosition.y - 2f, startPosition.z);
    }

    private void SpawnCarsAtPosition(Vector3 startPosition, Vector3 startDirection)
    {
        Vector3 sideDirection = Vector3.Cross(startDirection, Vector3.up);
        const int carsPerRow = 2;
        const float rowSpacing = 12f;
        const float colSpacing = 6f;

        bool wantsPlayer = spawnPlayer && playerVehiclePrefab != null;
        int totalCars = numberOfAIRacers + (wantsPlayer ? 1 : 0);
        int playerGridPosition = wantsPlayer ? Random.Range(0, totalCars) : -1;

        int aiIndex = 0;

        for (int i = 0; i < totalCars; i++)
        {
            int row = i / carsPerRow;
            int col = i % carsPerRow;

            Vector3 rowOffset = -startDirection * (row * rowSpacing + startingOffset);
            Vector3 colOffset = sideDirection * (col - (carsPerRow - 1) * 0.5f) * colSpacing;
            Vector3 position = startPosition + rowOffset + colOffset + Vector3.up * 0.5f;
            Quaternion rotation = Quaternion.LookRotation(startDirection);

            bool isPlayer = i == playerGridPosition;
            GameObject carObj;
            AIVehicleController ai = null;

            if (isPlayer)
            {
                carObj = Instantiate(playerVehiclePrefab, position, rotation);
                carObj.name = "PlayerCar";
                carObj.tag = playerTag;
            }
            else
            {
                GameObject prefab = aiVehiclePrefabs[Random.Range(0, aiVehiclePrefabs.Count)];
                carObj = Instantiate(prefab, position, rotation);
                carObj.name = $"AI_Racer_{aiIndex + 1}";

                ApplyMaterialToAICar(carObj, aiIndex);

                ai = carObj.GetComponent<AIVehicleController>();
                if (ai == null) ai = carObj.AddComponent<AIVehicleController>();

                ai.trackGenerator = trackGenerator;
                ai.RaceManager = this;
                ai.IsRacing = false;
                ai.isMischiefCar = Random.value < 0.25f;
                ConfigureDifficulty(ai, aiIndex);

                aiRacers.Add(ai);
                aiIndex++;
            }

            var participant = new RaceParticipant(carObj, ai, isPlayer);
            var state = new RaceCarState { car = carObj, participant = participant, aiController = ai };

            participants.Add(participant);
            raceCars.Add(state);
            stateByCar[carObj] = state;
            if (isPlayer) playerState = state;

            var triggerHandler = carObj.GetComponent<CarTriggerHandler>();
            if (triggerHandler == null) triggerHandler = carObj.AddComponent<CarTriggerHandler>();
            triggerHandler.raceManager = this;
        }

        if (verboseLogging) Debug.Log($"[AIRaceManager] Grid formed with {raceCars.Count} cars.");
    }

    private void BindParticipants()
    {
        // Every driver shares one list and skips its own entry, so adding the player to the
        // field costs nothing and the AI can finally see the car it is racing against.
        foreach (var state in raceCars)
        {
            state.aiController?.SetParticipants(participants, state.participant);
        }

        RefreshParticipantPositions();

        // Default the debug focus to an AI car rather than whatever landed in slot 0. The grid
        // position is randomised when spawnPlayer is on, so without this the player could easily
        // be first in the list — and every focused-car debug tool (gizmos, F4 sensor lines) would
        // silently have nothing to draw until F2 was pressed enough times to reach an AI car.
        FocusIndex = participants.FindIndex(p => p.Ai != null);
        if (FocusIndex < 0) FocusIndex = 0;
    }

    private void ConfigureDifficulty(AIVehicleController ai, int racerIndex)
    {
        float normalized = numberOfAIRacers > 1 ? racerIndex / (float)(numberOfAIRacers - 1) : 0f;
        float skill = Mathf.Lerp(maxSkillLevel, minSkillLevel, normalized) + Random.Range(-0.08f, 0.08f);
        skill = Mathf.Clamp(skill, Mathf.Min(minSkillLevel, maxSkillLevel), Mathf.Max(minSkillLevel, maxSkillLevel));

        float aggression = Random.Range(Mathf.Min(minAggressiveness, maxAggressiveness), Mathf.Max(minAggressiveness, maxAggressiveness));

        ai.ConfigureDifficulty(skill, aggression);
    }

    // ---------------------------------------------------------------- per-step field state

    private void FixedUpdate()
    {
        if (track == null || participants.Count == 0) return;
        RefreshParticipantPositions();
    }

    /// <summary>
    /// Projects every car onto the racing line once per physics step. Doing it here rather than
    /// inside each driver turns an N-squared problem into a linear one and gives all the AI a
    /// consistent view of the field.
    /// </summary>
    private void RefreshParticipantPositions()
    {
        for (int i = 0; i < participants.Count; i++)
        {
            RaceParticipant participant = participants[i];
            if (!participant.IsAlive) continue;

            // A racing AI localises itself as part of driving and writes the result back here,
            // so only cars that are not doing that for themselves need projecting
            if (participant.Ai != null && participant.Ai.IsRacing) continue;

            Vector3 position = participant.Position;
            int index = track.Localize(position, participant.LineIndex);
            participant.LineIndex = index;
            participant.ArcLength = track.ArcLengthAt(index, position);
            participant.LateralOffset = Vector3.Dot(position - track.Points[index], track.Right[index]);
        }
    }

    /// <summary>Lap plus fraction of a lap, measured from the start line. Comparable across all cars.</summary>
    private float TotalProgress(RaceCarState state)
    {
        float lapFraction = 0f;
        if (track != null && state.participant.IsAlive)
        {
            lapFraction = track.Wrap(state.participant.ArcLength - startLineArcLength) / track.TotalLength;
        }

        float progress = (state.currentLap - 1) + lapFraction;
        return float.IsNaN(progress) ? 0f : progress;
    }

    // ---------------------------------------------------------------- race flow

    private IEnumerator StartRaceCountdown()
    {
        for (int count = 3; count > 0; count--)
        {
            if (verboseLogging) Debug.Log($"<color=yellow>{count}...</color>");
            yield return new WaitForSeconds(1f);
        }

        if (verboseLogging) Debug.Log("<color=green>GO!</color>");

        raceStartTime = Time.time;
        foreach (var racer in aiRacers)
        {
            if (racer != null) racer.IsRacing = true;
        }

        if (!raceIsActive)
        {
            raceIsActive = true;
            StartCoroutine(UpdateRubberBanding());
        }
    }

    private IEnumerator UpdatePositionsRoutine()
    {
        var wait = new WaitForSeconds(0.2f);
        while (true)
        {
            UpdateRacePositions();
            yield return wait;
        }
    }

    private void UpdateRacePositions()
    {
        if (raceCars.Count == 0) return;

        // Snapshot progress first: a comparator that recomputes it is both slower and, if a
        // value ever came back NaN, capable of throwing out of Sort.
        foreach (var state in raceCars) state.progress = TotalProgress(state);

        raceCars.Sort((a, b) =>
        {
            // Finished cars are locked into the order they crossed the line
            if (a.finished && b.finished) return a.finishTime.CompareTo(b.finishTime);
            if (a.finished) return -1;
            if (b.finished) return 1;
            return b.progress.CompareTo(a.progress);
        });

        CarPositions.Clear();
        SortedRacers.Clear();

        for (int i = 0; i < raceCars.Count; i++)
        {
            RaceCarState state = raceCars[i];
            state.position = i + 1;

            if (state.aiController == null) continue;
            CarPositions[state.aiController] = i + 1;
            SortedRacers.Add(state.aiController);
        }
    }

    private IEnumerator UpdateRubberBanding()
    {
        var wait = new WaitForSeconds(1f);

        while (true)
        {
            yield return wait;
            if (aiRacers.Count == 0) continue;

            float leadProgress = float.MinValue;
            float secondProgress = float.MinValue;
            RaceCarState leader = null;

            foreach (var state in raceCars)
            {
                if (state.finished) continue;

                float progress = TotalProgress(state);
                if (progress > leadProgress)
                {
                    secondProgress = leadProgress;
                    leadProgress = progress;
                    leader = state;
                }
                else if (progress > secondProgress)
                {
                    secondProgress = progress;
                }
            }

            if (leader == null) continue;

            foreach (var state in raceCars)
            {
                if (state.aiController == null || state.finished) continue;

                if (state == leader)
                {
                    // Ease off out front, but only once the lead is big enough to be boring
                    float advantage = secondProgress > float.MinValue ? leadProgress - secondProgress : 0f;
                    float penalty = Mathf.InverseLerp(0f, 0.1f, advantage);
                    state.aiController.RubberBandingFactor = Mathf.Lerp(1f, maxSpeedPenalty, penalty * rubberBandingStrength);
                }
                else
                {
                    float deficit = leadProgress - TotalProgress(state);
                    float boost = Mathf.InverseLerp(0f, 0.2f, deficit);
                    state.aiController.RubberBandingFactor = Mathf.Lerp(1f, maxSpeedBoost, boost * rubberBandingStrength);
                }
            }
        }
    }

    private IEnumerator PeriodicOvertakeOrders()
    {
        var wait = new WaitForSeconds(10f);

        while (true)
        {
            yield return wait;
            if (!raceIsActive || raceCars.Count < 2) continue;

            // raceCars is already in running order from UpdateRacePositions
            for (int i = 1; i < raceCars.Count; i++)
            {
                RaceCarState chaser = raceCars[i];
                RaceCarState ahead = raceCars[i - 1];

                if (chaser.finished || chaser.aiController == null || ahead.aiController == null) continue;

                // Only nudge cars that are actually within reach of the one in front
                float gap = track != null
                    ? track.SignedGap(chaser.participant.ArcLength, ahead.participant.ArcLength)
                    : 0f;

                if (gap > 0f && gap < 80f) chaser.aiController.ForceOvertake(ahead.aiController);
            }
        }
    }

    // ---------------------------------------------------------------- triggers

    /// <summary>Called by a car that fell off the world or drove somewhere it cannot recover from.</summary>
    public void HandleCarFall(GameObject car)
    {
        if (!stateByCar.TryGetValue(car, out RaceCarState state) || state.finished) return;

        Vector3 position;
        Quaternion rotation;

        if (track != null)
        {
            // Put the car back on the racing line where it left it, rather than at whichever
            // checkpoint gate happened to be last: much less jarring, and always the right way round.
            track.Sample(state.participant.ArcLength + 5f, state.participant.LineIndex,
                out Vector3 linePoint, out Vector3 lineForward, out _);
            position = linePoint + Vector3.up * 3f;
            rotation = Quaternion.LookRotation(lineForward, Vector3.up);
        }
        else if (state.lastCheckpoint >= 0 && state.lastCheckpoint < checkpoints.Count)
        {
            position = checkpoints[state.lastCheckpoint].position + Vector3.up * 3f;
            rotation = checkpoints[state.lastCheckpoint].rotation;
        }
        else if (startFinishLine != null)
        {
            position = startFinishLine.position + Vector3.up * 3f;
            rotation = startFinishLine.rotation;
        }
        else
        {
            return;
        }

        car.transform.SetPositionAndRotation(position, rotation);

        // Without this the car keeps whatever velocity it had while falling
        if (state.participant.Body != null)
        {
            state.participant.Body.linearVelocity = Vector3.zero;
            state.participant.Body.angularVelocity = Vector3.zero;
        }

        state.aiController?.OnRespawned();

        if (verboseLogging) Debug.Log($"[AIRaceManager] {car.name} recovered to the racing line.");
    }

    /// <summary>Called when a car passes through a checkpoint gate.</summary>
    public void HandleCheckpoint(GameObject car, Transform checkpoint)
    {
        if (!stateByCar.TryGetValue(car, out RaceCarState state) || state.finished) return;
        if (!checkpointIndex.TryGetValue(checkpoint, out int index)) return;

        if (index != state.lastCheckpoint) state.lastCheckpoint = index;
    }

    /// <summary>Called when a car crosses the start/finish line.</summary>
    public void HandleStartFinish(GameObject car)
    {
        if (!stateByCar.TryGetValue(car, out RaceCarState state) || state.finished) return;
        if (checkpoints.Count == 0 || state.lastCheckpoint != checkpoints.Count - 1) return;

        state.currentLap++;
        state.lastCheckpoint = -1;

        if (verboseLogging) Debug.Log($"[AIRaceManager] {car.name} completed lap {state.currentLap - 1}.");
        if (state.currentLap <= numberOfLaps) return;

        state.finished = true;
        state.finishTime = Time.time - raceStartTime;
        Debug.Log($"[AIRaceManager] {car.name} finished in {state.finishTime:F2}s.");

        StopCar(car);
        CheckRaceCompletion();
    }

    private void StopCar(GameObject car)
    {
        var ai = car.GetComponent<AIVehicleController>();
        ai?.StopCar();
    }

    private void CheckRaceCompletion()
    {
        foreach (var state in raceCars)
        {
            if (!state.finished) return;
        }

        if (raceFinished) return;

        raceFinished = true;
        raceIsActive = false;
        Debug.Log("[AIRaceManager] Race complete.");
    }

    // ---------------------------------------------------------------- queries

    /// <summary>Current running order, best first.</summary>
    public List<(string carName, int position, float progress)> GetCurrentPositions()
    {
        var results = new List<(string carName, int position, float progress)>();

        foreach (var state in raceCars)
        {
            if (state.car == null) continue;
            results.Add((state.car.name, state.position, TotalProgress(state)));
        }

        results.Sort((a, b) => a.position.CompareTo(b.position));
        return results;
    }

    /// <summary>Tears down the current grid and starts a fresh race on the current track.</summary>
    public void ResetRace()
    {
        StopAllCoroutines();
        raceIsActive = false;
        raceFinished = false;
        StartCoroutine(SpawnAIRacersWhenReady());
        StartCoroutine(UpdatePositionsRoutine());
        StartCoroutine(PeriodicOvertakeOrders());
    }

    // ---------------------------------------------------------------- appearance

    /// <summary>Paints one AI car in its livery. Picks the largest renderer when there is no Body child.</summary>
    public void ApplyMaterialToAICar(GameObject carObj, int materialIndex = -1)
    {
        if (aiCarMaterials == null || aiCarMaterials.Count == 0 || carObj == null) return;

        if (materialIndex < 0)
        {
            materialIndex = aiRacers.FindIndex(ai => ai != null && ai.gameObject == carObj);
            if (materialIndex < 0) return;
        }

        Material material = aiCarMaterials[materialIndex % aiCarMaterials.Count];
        Renderer target = null;

        Transform body = carObj.transform.Find("Body");
        if (body != null) target = body.GetComponent<Renderer>();

        if (target == null)
        {
            float largest = 0f;
            foreach (var renderer in carObj.GetComponentsInChildren<Renderer>())
            {
                Vector3 size = renderer.bounds.size;
                float volume = size.x * size.y * size.z;
                if (volume <= largest) continue;

                largest = volume;
                target = renderer;
            }
        }

        if (target == null)
        {
            Debug.LogWarning($"[AIRaceManager] No renderer to paint on {carObj.name}.");
            return;
        }

        target.material = material;
    }

    /// <summary>Repaints every AI car, for when the livery list changes at runtime.</summary>
    public void RefreshAllAICarMaterials()
    {
        for (int i = 0; i < aiRacers.Count; i++)
        {
            if (aiRacers[i] != null) ApplyMaterialToAICar(aiRacers[i].gameObject, i);
        }
    }
}
