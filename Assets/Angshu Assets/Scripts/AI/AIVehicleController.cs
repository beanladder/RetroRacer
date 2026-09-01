using System.Collections.Generic;
using Ashsvp;
using Track;
using UnityEngine;

/// <summary>
/// Drives one car around the generated racing line.
///
/// The control stack is the one racing games normally use:
///
///   * a speed profile built from the grip circle and relaxed backwards and forwards, so braking
///     points come out of the car's own limits instead of a hand-tuned "corner factor";
///   * pure pursuit steering with a speed scaled lookahead, mapped through the vehicle's own
///     steering curve so a steer input of 0.4 actually means 0.4 of the available lock;
///   * a lateral offset layer for wander, avoidance and overtaking that is clamped to the track
///     so the car never aims at a point it cannot legally drive to;
///   * an online grip estimate, so a car that is sliding through corners quietly lowers its own
///     limits rather than repeating the same mistake every lap.
///
/// Longitudinal control matters more than it looks here. In this vehicle model the handbrake
/// both slows the car and drops the rear grip to <c>driftFactor</c>, which on these prefabs is
/// negative. Braking therefore goes through negative throttle, which is both stronger and keeps
/// the car pointing where it is going; the handbrake is reserved for hairpins and recovery.
/// </summary>
[RequireComponent(typeof(SimcadeVehicleController))]
public class AIVehicleController : MonoBehaviour
{
    [Header("Track")]
    [Tooltip("Track whose racing line this car follows. Found automatically when left empty.")]
    public TrackGenerator trackGenerator;

    /// <summary>Set by the race manager that spawned this car, so it can check whether it is the one being spectated.</summary>
    public AIRaceManager RaceManager { get; set; }

    /// <summary>
    /// Draws this car's sensor lane, rival links and current decision as real <see cref="LineRenderer"/>
    /// geometry rather than gizmos, so it is visible in the Game view and in builds without the
    /// Scene/Game view Gizmos toggle. Toggled globally by <see cref="AIDebugHud"/> (F4); only the
    /// car <see cref="RaceManager"/> currently has focused actually draws, to keep a full grid readable.
    /// </summary>
    public static bool ShowRuntimeVisuals = false;

    [Header("Driver")]
    [SerializeField, Range(0f, 1f), Tooltip("Overall competence: grip used, lookahead, reaction time and how cleanly the line is held.")]
    private float skillLevel = 0.75f;

    [SerializeField, Range(0f, 1f), Tooltip("Willingness to commit: how soon the driver tries a pass, how little room it accepts and how hard it defends.")]
    private float aggressiveness = 0.6f;

    [SerializeField, Range(0f, 1f), Tooltip("Taste for oversteer. Drivers above zero will hang the tail out through slow corners.")]
    private float driftStyle = 0.25f;

    [SerializeField, Tooltip("Rear grip multiplier while the handbrake is down. The car prefabs ship a negative value, which turns any handbrake input into a spin, so AI cars get a controlled one.")]
    private Vector2 handbrakeRearGrip = new Vector2(0.45f, 0.65f);

    [Header("Grip Model")]
    [SerializeField, Tooltip("Lateral acceleration the car is assumed to hold in a corner, in m/s². The single biggest knob on AI pace.")]
    private float corneringAcceleration = 18f;

    [SerializeField, Tooltip("Deceleration assumed when planning braking points, in m/s².")]
    private float brakingDeceleration = 26f;

    [SerializeField, Tooltip("Acceleration assumed when planning corner exits, in m/s².")]
    private float driveAcceleration = 20f;

    [SerializeField, Tooltip("Let each car learn its real grip from how much it slides, and replan its braking points to match.")]
    private bool adaptGrip = true;

    [SerializeField, Range(0.3f, 1f), Tooltip("Lowest fraction of the nominal cornering grip the learner may settle on.")]
    private float minGripFraction = 0.55f;

    [SerializeField, Range(1f, 2f), Tooltip("Highest fraction of the nominal cornering grip the learner may settle on.")]
    private float maxGripFraction = 1.35f;

    [Header("Steering")]
    [SerializeField, Tooltip("Seconds of travel used to place the steering aim point. Higher is smoother and lazier.")]
    private float lookaheadTime = 1.1f;

    [SerializeField, Tooltip("Shortest steering aim distance, in metres.")]
    private float minLookahead = 12f;

    [SerializeField, Tooltip("Longest steering aim distance, in metres.")]
    private float maxLookahead = 70f;

    [SerializeField, Range(0f, 2f), Tooltip("Extra correction pulling the car back onto its intended line. Too much makes it weave.")]
    private float cornerTrackingGain = 0.25f;

    [SerializeField, Range(0f, 25f), Tooltip("Ceiling on that correction, in degrees of steering.")]
    private float maxTrackingCorrection = 10f;

    [SerializeField, Range(1f, 30f), Tooltip("How fast the steering wheel can be turned, in units of full lock per second.")]
    private float steeringRate = 6f;

    [Header("Pace")]
    [SerializeField, Range(0.5f, 1.2f), Tooltip("Fraction of the planned speed the car actually asks for.")]
    private float paceFactor = 1f;

    [SerializeField, Tooltip("Speed error, in m/s, that produces full throttle.")]
    private float throttleBand = 6f;

    [SerializeField, Tooltip("Speed error, in m/s, that produces full braking.")]
    private float brakeBand = 9f;

    [Header("Line")]
    [SerializeField, Range(0f, 8f), Tooltip("How far an unskilled driver drifts off the ideal line, in metres.")]
    private float lineWander = 3f;

    [SerializeField, Range(0.02f, 1f), Tooltip("How quickly that wander changes.")]
    private float wanderRate = 0.15f;

    [Header("Traffic")]
    [SerializeField, Tooltip("How far ahead, in metres, other cars are taken into account.")]
    private float awarenessDistance = 60f;

    [SerializeField, Tooltip("Gap, in metres, the car tries to keep to the one in front before lifting.")]
    private float followingGap = 9f;

    [SerializeField, Tooltip("Sideways room, in metres, wanted before committing to a pass.")]
    private float sideClearance = 3.5f;

    [SerializeField, Tooltip("Seconds held up behind a slower car before a skilled, patient driver tries a move.")]
    private float basePatience = 2.2f;

    [SerializeField, Tooltip("Seconds a committed overtake runs before it is abandoned.")]
    private float overtakeCommitTime = 6f;

    [SerializeField, Tooltip("Seconds of calm after a pass before another is attempted.")]
    private float overtakeCooldown = 2.5f;

    [Header("Nitro")]
    [SerializeField, Tooltip("Use nitro on straights.")]
    private bool useNitro = true;

    [SerializeField, Tooltip("Metres of clear road wanted ahead before lighting the nitro.")]
    private float nitroClearDistance = 160f;

    [SerializeField, Tooltip("Sharpest bend, as 1/metres of curvature, still counted as a straight.")]
    private float nitroCurvatureLimit = 0.004f;

    [SerializeField, Tooltip("Shortest gap between nitro uses, in seconds.")]
    private Vector2 nitroCooldownRange = new Vector2(14f, 24f);

    [Header("Recovery")]
    [SerializeField, Tooltip("Below this speed, in m/s, with throttle applied, the car counts as stuck.")]
    private float stuckSpeed = 2.5f;

    [SerializeField, Tooltip("Seconds of being stuck before reversing out.")]
    private float stuckTime = 1.6f;

    [SerializeField, Tooltip("Seconds spent reversing during a recovery.")]
    private float reverseTime = 1.4f;

    [Header("Start")]
    [SerializeField, Tooltip("Range of reaction times at the green light. Skill picks a point in it.")]
    private Vector2 startReactionRange = new Vector2(0.12f, 0.7f);

    [Header("Mischief")]
    [Tooltip("This driver will deliberately lean on rivals it is racing alongside.")]
    public bool isMischiefCar = false;

    [Header("Debug")]
    [SerializeField, Tooltip("Which parts of this driver's reasoning to draw. Gizmos must be enabled in the Scene or Game view.")]
    private AIDebugLayers debugLayers = AIDebugLayers.None;

    [SerializeField, Range(50f, 600f), Tooltip("How far ahead, in metres, the speed profile ribbon is drawn.")]
    private float debugProfileSpan = 300f;

    [Header("Runtime Visuals (Game View)")]
    [SerializeField, Range(0.05f, 0.6f), Tooltip("World-space width of the sensor and decision lines drawn when this car is being spectated.")]
    private float runtimeLineWidth = 0.22f;

    [SerializeField, Range(2, 12), Tooltip("Most rivals drawn as links at once, nearest first.")]
    private int maxRivalLines = 8;

    // ---------------------------------------------------------------- public state

    /// <summary>Whether the car is allowed to drive. The race manager releases this at the green light.</summary>
    public bool IsRacing { get; set; }

    /// <summary>How far around the current lap the car is, from 0 to 1.</summary>
    public float LapProgress { get; private set; }

    /// <summary>Distance travelled around the lap, in metres.</summary>
    public float ArcLength { get; private set; }

    /// <summary>Pace multiplier applied by the race manager's rubber banding.</summary>
    public float RubberBandingFactor { get; set; } = 1f;

    /// <summary>The driver's competence, from 0 to 1.</summary>
    public float SkillLevel => skillLevel;

    /// <summary>The driver's willingness to commit, from 0 to 1.</summary>
    public float Aggressiveness => aggressiveness;

    /// <summary>Everything this driver decided on its last physics step, for gizmos and the HUD.</summary>
    public AIDriverTelemetry Telemetry => _telemetry;

    /// <summary>Which parts of this driver's reasoning are drawn. The debug HUD toggles this.</summary>
    public AIDebugLayers DebugLayers
    {
        get => debugLayers;
        set => debugLayers = value;
    }

    // ---------------------------------------------------------------- private state

    private SimcadeVehicleController _vehicle;
    private Rigidbody _body;
    private AITrackData _track;
    private readonly AISpeedProfile _profile = new AISpeedProfile();

    private List<RaceParticipant> _participants;
    private RaceParticipant _self;

    private bool _ready;
    private int _lineIndex;
    private float _crossTrack;
    private float _speed;

    // Latched decisions, refreshed at the driver's reaction rate rather than every step
    private float _plannedSpeed;
    private float _plannedLateral;
    private float _decisionTimer;
    private float _reactionTime = 0.15f;

    private float _steer;
    private float _throttle;
    private float _brake;
    private float _handbrake;
    private float _lateralOffset;

    private float _wanderSeed;
    private float _gripFraction = 1f;
    private float _gripSampleTimer;
    private float _slipAccumulator;
    private float _slipSamples;

    private RaceParticipant _blockingCar;
    private float _blockedTimer;
    private float _overtakeTimer;
    private float _cooldownTimer;
    private int _overtakeSide;

    private bool _defending;
    private float _defendTimer;

    private float _stuckTimer;
    private float _reverseTimer;

    private float _nitroTimer;
    private bool _nitroLatched;
    private float _startDelay;
    private bool _wasRacing;

    private float _baseAcceleration;
    private float _accelerationBoost = 1f;

    private AIDriverTelemetry _telemetry;

    private LineRenderer _aimLine;
    private LineRenderer _sensorLeft;
    private LineRenderer _sensorRight;
    private readonly List<LineRenderer> _rivalLines = new List<LineRenderer>();
    private bool _runtimeVisualsVisible;
    private static Material _runtimeLineMaterial;
    private const int SensorLaneSamples = 10;

    // Cached limits, all scaled by skill so one dial moves the whole driver
    private float _effectiveLookaheadTime;
    private float _effectivePace;
    private float _patience;
    private float _wanderAmount;

    // ---------------------------------------------------------------- lifecycle

    private void Awake()
    {
        _vehicle = GetComponent<SimcadeVehicleController>();
        _body = GetComponent<Rigidbody>();

        // The chase camera belongs to the player's car, not to twelve opponents
        if (_vehicle.cinemachineCamera != null) _vehicle.cinemachineCamera.gameObject.SetActive(false);
        if (_vehicle.alternativeCamera != null) _vehicle.alternativeCamera.gameObject.SetActive(false);

        _vehicle.AutoCounterSteer = true;
        _baseAcceleration = _vehicle.Acceleration;

        // A negative driftFactor inverts rear lateral friction rather than reducing it, so a
        // driver that touches the handbrake on a hairpin simply spins. Give each car its own
        // sane value instead, which also makes the drifters look slightly different from
        // one another.
        _vehicle.driftFactor = Random.Range(handbrakeRearGrip.x, handbrakeRearGrip.y);
        _wanderSeed = Random.Range(0f, 1000f);
        _nitroTimer = Random.Range(nitroCooldownRange.x, nitroCooldownRange.y) * 0.5f;

        ApplyDifficulty();
    }

    private void Start()
    {
        if (trackGenerator == null) trackGenerator = FindFirstObjectByType<TrackGenerator>();

        if (_vehicle.inputManager != null)
        {
            _vehicle.inputManager.enabled = true;
            _vehicle.inputManager.SetAIInputs(0f, 0f, 1f, false);
        }

        TryBind();
    }

    private void OnDisable()
    {
        // Never leave a boosted engine behind on a disabled car
        _vehicle.Acceleration = _baseAcceleration;
        UpdateRuntimeVisuals(false, Vector3.zero);
    }

    /// <summary>
    /// Resolves the racing line and builds this car's speed profile. Safe to call repeatedly;
    /// it does nothing until the track has finished generating.
    /// </summary>
    private bool TryBind()
    {
        if (_ready) return true;
        if (trackGenerator == null) return false;

        _track = AITrackData.For(trackGenerator);
        if (_track == null) return false;

        RebuildProfile();
        _lineIndex = _track.Localize(transform.position, 0, _track.Count / 2);
        ArcLength = _track.ArcLengthAt(_lineIndex, transform.position);
        _plannedSpeed = _profile.Evaluate(_track, ArcLength, _lineIndex);
        _ready = true;
        return true;
    }

    private void RebuildProfile()
    {
        _profile.Build(
            _track,
            _vehicle.MaxSpeed,
            corneringAcceleration * _gripFraction,
            brakingDeceleration * Mathf.Lerp(0.75f, 1f, skillLevel),
            driveAcceleration);
    }

    /// <summary>Sets the driver's competence and appetite for risk, and re-derives everything from them.</summary>
    public void ConfigureDifficulty(float skill, float aggression)
    {
        skillLevel = Mathf.Clamp01(skill);
        aggressiveness = Mathf.Clamp01(aggression);
        ApplyDifficulty();
        if (_ready) RebuildProfile();
    }

    private void ApplyDifficulty()
    {
        // One dial, many consequences: a weaker driver looks less far ahead, reacts later,
        // holds a scruffier line and leaves more of the car's grip unused.
        _effectiveLookaheadTime = lookaheadTime * Mathf.Lerp(0.8f, 1.25f, skillLevel);
        _effectivePace = paceFactor * Mathf.Lerp(0.86f, 1.02f, skillLevel) * Mathf.Lerp(0.97f, 1.05f, aggressiveness);
        _reactionTime = Mathf.Lerp(0.30f, 0.06f, skillLevel);
        _wanderAmount = lineWander * Mathf.Lerp(1f, 0.12f, skillLevel);
        _patience = basePatience * Mathf.Lerp(1.8f, 0.45f, aggressiveness);
        _gripFraction = Mathf.Clamp(Mathf.Lerp(0.8f, 1.05f, skillLevel), minGripFraction, maxGripFraction);
    }

    /// <summary>Hands the driver the field it is racing against, including the player.</summary>
    public void SetParticipants(List<RaceParticipant> participants, RaceParticipant self)
    {
        _participants = participants;
        _self = self;
    }

    // ---------------------------------------------------------------- driving

    private void FixedUpdate()
    {
        if (!_ready && !TryBind()) return;

        float dt = Time.fixedDeltaTime;

        if (!IsRacing)
        {
            HoldOnGrid();
            _wasRacing = false;
            return;
        }

        if (!_wasRacing)
        {
            // Everyone gets away at a slightly different moment, and the better drivers go first
            _startDelay = Mathf.Lerp(startReactionRange.y, startReactionRange.x, skillLevel) * Random.Range(0.7f, 1.3f);
            _handbrake = 0f;
            _wasRacing = true;
        }

        UpdateLocalisation();
        UpdateGripEstimate(dt);

        _decisionTimer -= dt;
        if (_decisionTimer <= 0f)
        {
            _decisionTimer = _reactionTime;
            Plan();
        }

        Drive(dt);
    }

    private void HoldOnGrid()
    {
        _steer = 0f;
        _throttle = 0f;
        _brake = 0f;
        _handbrake = 1f;
        if (_vehicle.inputManager != null) _vehicle.inputManager.SetAIInputs(0f, 0f, 1f, false);
    }

    private void UpdateLocalisation()
    {
        Vector3 position = transform.position;

        _lineIndex = _track.Localize(position, _lineIndex);
        ArcLength = _track.ArcLengthAt(_lineIndex, position);
        LapProgress = ArcLength / _track.TotalLength;

        _crossTrack = Vector3.Dot(position - _track.Points[_lineIndex], _track.Right[_lineIndex]);
        _speed = _body != null ? _body.linearVelocity.magnitude : _vehicle.carVelocity.magnitude;

        if (_self != null)
        {
            _self.ArcLength = ArcLength;
            _self.LateralOffset = _crossTrack;
        }
    }

    /// <summary>
    /// Watches how much the car slides in corners and nudges its assumed grip to match, then
    /// replans. A driver that keeps washing out will brake earlier next lap; one with margin
    /// in hand will gradually lean on the car harder.
    /// </summary>
    private void UpdateGripEstimate(float dt)
    {
        if (!adaptGrip) return;

        Vector3 local = _vehicle.localVehicleVelocity;
        if (local.z > 5f)
        {
            _slipAccumulator += Mathf.Abs(local.x) / local.z;
            _slipSamples += 1f;
        }

        _gripSampleTimer += dt;
        if (_gripSampleTimer < 2.5f || _slipSamples < 20f) return;

        float averageSlip = _slipAccumulator / _slipSamples;
        _gripSampleTimer = 0f;
        _slipAccumulator = 0f;
        _slipSamples = 0f;

        // Around 0.12 of sideways to forward speed is a car working hard but still tracking
        const float slipTarget = 0.12f;
        float previous = _gripFraction;
        float correction = Mathf.Clamp((slipTarget - averageSlip) * 0.6f, -0.08f, 0.05f);
        _gripFraction = Mathf.Clamp(_gripFraction + correction, minGripFraction, maxGripFraction);

        if (Mathf.Abs(_gripFraction - previous) > 0.01f) RebuildProfile();
    }

    /// <summary>Latched decisions: where on the track to aim and how fast to be going there.</summary>
    private void Plan()
    {
        float lookahead = Mathf.Clamp(_speed * _effectiveLookaheadTime, minLookahead, maxLookahead);

        // Speed comes from the profile, but the car also has to respect whatever is coming in
        // the next braking zone rather than only the point under its wheels.
        float planned = Mathf.Min(
            _profile.Evaluate(_track, ArcLength, _lineIndex),
            _profile.MinimumAhead(_track, ArcLength, lookahead * 0.9f, _lineIndex));

        planned *= _effectivePace * Mathf.Clamp(RubberBandingFactor, 0.5f, 2f);

        float lateral = EvaluateWander();
        lateral += EvaluateTraffic(ref planned);

        if (_defending)
        {
            _defendTimer -= _reactionTime;
            if (_defendTimer <= 0f) _defending = false;
            else lateral += DefensiveOffset();
        }

        // Never aim at a point the car cannot legally reach. The racing line already runs up to
        // roughly 45% of the half width off centre, so the same again is the safe ceiling.
        float limit = _track.HalfWidth * 0.45f;
        _plannedLateral = Mathf.Clamp(lateral, -limit, limit);

        // Nitro raises the car's top speed, so without this the driver would brake against its
        // own boost on a straight. It may only lift the target where the profile already had
        // the car flat out: corner and braking limits come from grip and nitro does not help.
        float ceiling = _vehicle.MaxSpeed;
        if (_vehicle.isNitroActive)
        {
            ceiling *= Mathf.Max(1f, _vehicle.nitroMaxSpeedMultiplier);
            if (planned > _vehicle.MaxSpeed * 0.98f) planned = ceiling;
        }

        _plannedSpeed = Mathf.Clamp(planned, 0f, ceiling);
    }

    private float EvaluateWander()
    {
        if (_wanderAmount <= 0.01f) return 0f;
        return (Mathf.PerlinNoise(Time.time * wanderRate, _wanderSeed) * 2f - 1f) * _wanderAmount;
    }

    private float DefensiveOffset()
    {
        // Move to the inside of the corner ahead, which is the line an attacker wants
        float curvature = _track.CurvatureAt(ArcLength + 30f, _lineIndex);
        float side = Mathf.Abs(curvature) > 1e-4f ? Mathf.Sign(curvature) : Mathf.Sign(_plannedLateral);
        return side * _track.HalfWidth * 0.3f * Mathf.Lerp(0.4f, 1f, aggressiveness);
    }

    // ---------------------------------------------------------------- traffic

    /// <summary>
    /// Looks at every other car in racing line coordinates, which is far more useful than world
    /// space dot products: "40 metres up the road and 3 metres to my left" answers both whether
    /// the car is in the way and which side there is room to pass on.
    /// Returns the lateral offset traffic calls for and may lower <paramref name="planned"/>.
    /// </summary>
    private float EvaluateTraffic(ref float planned)
    {
        _cooldownTimer = Mathf.Max(0f, _cooldownTimer - _reactionTime);

        if (_participants == null || _participants.Count < 2)
        {
            _blockingCar = null;
            _blockedTimer = 0f;
            return 0f;
        }

        RaceParticipant closestAhead = null;
        float closestGap = float.MaxValue;
        float avoidance = 0f;

        float myLateral = _crossTrack;
        float carLength = 5f;

        for (int i = 0; i < _participants.Count; i++)
        {
            RaceParticipant other = _participants[i];
            if (other == null || other == _self || !other.IsAlive) continue;

            float gap = _track.SignedGap(ArcLength, other.ArcLength);
            if (gap > awarenessDistance || gap < -carLength * 2f) continue;

            float lateralGap = other.LateralOffset - myLateral;

            if (gap > 0f)
            {
                // Ahead of us. Steer around it, and note the nearest one for overtake logic.
                float urgency = Mathf.InverseLerp(awarenessDistance, followingGap, gap);
                float overlap = Mathf.InverseLerp(sideClearance * 2f, 0f, Mathf.Abs(lateralGap));

                if (overlap > 0f && urgency > 0f)
                {
                    float side = Mathf.Abs(lateralGap) < 0.1f ? PreferredSide() : -Mathf.Sign(lateralGap);
                    avoidance += side * sideClearance * overlap * urgency;
                }

                if (gap < closestGap)
                {
                    closestGap = gap;
                    closestAhead = other;
                }
            }
            else if (Mathf.Abs(lateralGap) < sideClearance * 1.6f)
            {
                // Alongside. Hold station rather than turning into a car we cannot see.
                avoidance += Mathf.Sign(lateralGap != 0f ? -lateralGap : 1f) * sideClearance * 0.6f;
            }
        }

        UpdateOvertakeState(closestAhead, closestGap);

        if (_overtakeTimer > 0f && _overtakeSide != 0)
        {
            avoidance += _overtakeSide * (sideClearance + _track.HalfWidth * 0.15f);
        }
        else if (closestAhead != null && closestGap < followingGap)
        {
            // Tucked in with nowhere to go: match the car in front instead of driving into it
            float theirSpeed = closestAhead.Speed;
            float closing = Mathf.InverseLerp(followingGap, followingGap * 0.35f, closestGap);
            planned = Mathf.Min(planned, Mathf.Lerp(planned, theirSpeed * 0.96f, closing));
        }

        if (isMischiefCar && closestAhead != null && closestGap < carLength * 2.5f)
        {
            // Lean on the car in front rather than settling in behind it
            avoidance = Mathf.Lerp(avoidance, closestAhead.LateralOffset - myLateral, 0.6f);
            planned *= 1.05f;
        }

        return avoidance;
    }

    private int PreferredSide()
    {
        // Prefer the side with more room left on the track, breaking ties toward the outside
        float room = _track.HalfWidth * 0.45f;
        return _crossTrack > 0f ? (_crossTrack > room * 0.5f ? -1 : 1) : (_crossTrack < -room * 0.5f ? 1 : -1);
    }

    private void UpdateOvertakeState(RaceParticipant ahead, float gap)
    {
        if (_overtakeTimer > 0f)
        {
            _overtakeTimer -= _reactionTime;

            bool cleared = _blockingCar == null || !_blockingCar.IsAlive ||
                           _track.SignedGap(ArcLength, _blockingCar.ArcLength) < -3f;

            if (cleared || _overtakeTimer <= 0f) EndOvertake();
            return;
        }

        if (_cooldownTimer > 0f || ahead == null || gap > awarenessDistance * 0.6f)
        {
            _blockedTimer = 0f;
            _blockingCar = null;
            return;
        }

        // Only worth a move if we would genuinely be quicker with clear road
        bool quicker = _plannedSpeed > ahead.Speed * 1.04f;
        if (!quicker)
        {
            _blockedTimer = Mathf.Max(0f, _blockedTimer - _reactionTime);
            return;
        }

        _blockingCar = ahead;
        _blockedTimer += _reactionTime;
        if (_blockedTimer < _patience) return;

        // Commit to the side with room, checking nobody is already sitting there
        int side = ahead.LateralOffset > 0f ? -1 : 1;
        if (!SideIsClear(side)) side = -side;
        if (!SideIsClear(side)) return;

        _overtakeSide = side;
        _overtakeTimer = overtakeCommitTime;
        _blockedTimer = 0f;
        _accelerationBoost = Mathf.Lerp(1.05f, 1.2f, aggressiveness);
        _vehicle.Acceleration = _baseAcceleration * _accelerationBoost;

        if (ahead.Ai != null) ahead.Ai.TryStartDefense(aggressiveness);
    }

    private bool SideIsClear(int side)
    {
        float target = _crossTrack + side * sideClearance * 1.5f;
        if (Mathf.Abs(target) > _track.HalfWidth * 0.5f) return false;

        for (int i = 0; i < _participants.Count; i++)
        {
            RaceParticipant other = _participants[i];
            if (other == null || other == _self || !other.IsAlive) continue;

            float gap = _track.SignedGap(ArcLength, other.ArcLength);
            if (gap < -8f || gap > 25f) continue;
            if (Mathf.Abs(other.LateralOffset - target) < sideClearance) return false;
        }

        return true;
    }

    private void EndOvertake()
    {
        _overtakeTimer = 0f;
        _overtakeSide = 0;
        _blockingCar = null;
        _blockedTimer = 0f;
        _cooldownTimer = overtakeCooldown;
        _accelerationBoost = 1f;
        _vehicle.Acceleration = _baseAcceleration;
    }

    /// <summary>Asks this driver to make life difficult for someone trying to come past.</summary>
    public void TryStartDefense(float attackerAggression)
    {
        if (_defending) return;

        // A stubborn driver defends often; a hard charging attacker is harder to hold off
        float willingness = Mathf.Lerp(0.2f, 0.75f, aggressiveness) * Mathf.Lerp(1.1f, 0.7f, attackerAggression);
        if (Random.value > willingness) return;

        _defending = true;
        _defendTimer = Random.Range(2.5f, 5f) * Mathf.Lerp(0.7f, 1.3f, aggressiveness);
    }

    /// <summary>Ordered by the race manager to have a go at a specific car.</summary>
    public void ForceOvertake(AIVehicleController target)
    {
        if (target == null || _overtakeTimer > 0f || _participants == null) return;

        RaceParticipant entry = _participants.Find(p => p != null && p.Ai == target);
        if (entry == null || !entry.IsAlive) return;

        int side = entry.LateralOffset > 0f ? -1 : 1;
        _blockingCar = entry;
        _overtakeSide = side;
        _overtakeTimer = overtakeCommitTime;
        _blockedTimer = 0f;
        _accelerationBoost = Mathf.Lerp(1.05f, 1.2f, aggressiveness);
        _vehicle.Acceleration = _baseAcceleration * _accelerationBoost;
        target.TryStartDefense(aggressiveness);
    }

    // ---------------------------------------------------------------- actuation

    private void Drive(float dt)
    {
        float lookahead = Mathf.Clamp(_speed * _effectiveLookaheadTime, minLookahead, maxLookahead);

        // Ease onto the planned offset instead of snapping, so avoidance reads as a lane change
        _lateralOffset = Damp(_lateralOffset, _plannedLateral, 2.5f, dt);

        _track.Sample(ArcLength + lookahead, _lineIndex, out Vector3 linePoint, out _, out Vector3 lineRight);
        Vector3 aimPoint = linePoint + lineRight * _lateralOffset;

        float targetSteer = ComputeSteering(aimPoint);
        ComputeLongitudinal(dt, out float targetThrottle, out float targetBrake);
        float targetHandbrake = ComputeHandbrake();

        if (UpdateRecovery(dt, ref targetSteer, ref targetThrottle, ref targetBrake, ref targetHandbrake))
        {
            _decisionTimer = 0f;
        }

        // Rate limited so the AI cannot do anything a player's hands could not
        _steer = Mathf.MoveTowards(_steer, targetSteer, steeringRate * dt);
        _throttle = Damp(_throttle, targetThrottle, 12f, dt);
        _brake = Damp(_brake, targetBrake, 16f, dt);
        _handbrake = Damp(_handbrake, targetHandbrake, 10f, dt);

        bool nitro = ChooseNitro(dt);

        // The handbrake blocks the throttle in this vehicle model, so never ask for both
        float acceleration = _handbrake > 0.1f ? 0f : Mathf.Clamp(_throttle - _brake, -1f, 1f);

        if (_vehicle.inputManager != null)
        {
            _vehicle.inputManager.SetAIInputs(Mathf.Clamp(_steer, -1f, 1f), acceleration, _handbrake, nitro);
        }

        RecordTelemetry(aimPoint, lookahead, nitro);

        bool spectated = ShowRuntimeVisuals && RaceManager != null && RaceManager.FocusedParticipant()?.Ai == this;
        UpdateRuntimeVisuals(spectated, aimPoint);
    }

    private void RecordTelemetry(Vector3 aimPoint, float lookahead, bool nitro)
    {
        _telemetry.Valid = true;
        _telemetry.State = ResolveState();

        _telemetry.ArcLength = ArcLength;
        _telemetry.LapProgress = LapProgress;
        _telemetry.CrossTrack = _crossTrack;

        _telemetry.Speed = _speed;
        _telemetry.TargetSpeed = _plannedSpeed;
        _telemetry.MaxSpeed = _vehicle.MaxSpeed;

        _telemetry.AimPoint = aimPoint;
        _telemetry.Lookahead = lookahead;
        _telemetry.PlannedLateral = _plannedLateral;
        _telemetry.AppliedLateral = _lateralOffset;

        _telemetry.Steer = _steer;
        _telemetry.Throttle = _throttle;
        _telemetry.Brake = _brake;
        _telemetry.Handbrake = _handbrake;
        _telemetry.Nitro = nitro;

        _telemetry.GripFraction = _gripFraction;
        _telemetry.RubberBanding = RubberBandingFactor;
        _telemetry.ReactionTime = _reactionTime;
        _telemetry.BlockedTimer = _blockedTimer;
        _telemetry.Patience = _patience;
        _telemetry.OvertakeSide = _overtakeSide;
        _telemetry.OvertakeTimer = _overtakeTimer;
        _telemetry.Defending = _defending;
        _telemetry.StuckTimer = _stuckTimer;
        _telemetry.ReverseTimer = _reverseTimer;

        bool hasBlocker = _blockingCar != null && _blockingCar.IsAlive;
        _telemetry.BlockingCar = hasBlocker ? _blockingCar.Transform.name : null;
        _telemetry.BlockingGap = hasBlocker ? _track.SignedGap(ArcLength, _blockingCar.ArcLength) : 0f;
    }

    private AIDriverState ResolveState()
    {
        if (!IsRacing) return AIDriverState.Waiting;
        if (_reverseTimer > 0f) return AIDriverState.Recovering;
        if (_overtakeTimer > 0f) return AIDriverState.Overtaking;
        if (_defending) return AIDriverState.Defending;
        if (_brake > 0.15f) return AIDriverState.Braking;
        if (_throttle < 0.25f && _speed > _plannedSpeed) return AIDriverState.Lifting;
        return AIDriverState.Cruising;
    }

    // ---------------------------------------------------------------- runtime visuals (Game view)

    private static readonly Color SensorLaneColour = new Color(0.3f, 0.62f, 1f, 0.4f);
    private static readonly Color TrackedClearColour = new Color(0.45f, 0.6f, 0.85f, 0.35f);
    private static readonly Color AlongsideColour = new Color(1f, 0.85f, 0.15f, 0.85f);
    private static readonly Color ThreatColour = new Color(1f, 0.35f, 0.25f, 0.85f);
    private static readonly Color OvertakingColour = new Color(0.85f, 0.35f, 1f, 0.9f);

    /// <summary>
    /// The Game-view equivalent of the gizmo layers: real <see cref="LineRenderer"/> geometry, so
    /// it renders whether or not Gizmos are toggled on and would still render in a build. Kept to
    /// one spectated car at a time, both to keep a nine-car grid readable and because this is the
    /// same car the spectator camera orbits.
    /// </summary>
    private void UpdateRuntimeVisuals(bool active, Vector3 aimPoint)
    {
        if (!active)
        {
            if (_runtimeVisualsVisible) HideRuntimeVisuals();
            return;
        }

        EnsureRuntimeVisualObjects();
        _runtimeVisualsVisible = true;

        DrawAimLine(aimPoint);
        DrawSensorLane();
        DrawRivalLinks();
    }

    private void HideRuntimeVisuals()
    {
        _runtimeVisualsVisible = false;

        if (_aimLine != null) _aimLine.enabled = false;
        if (_sensorLeft != null) _sensorLeft.enabled = false;
        if (_sensorRight != null) _sensorRight.enabled = false;

        for (int i = 0; i < _rivalLines.Count; i++)
        {
            _rivalLines[i].enabled = false;
        }
    }

    private void EnsureRuntimeVisualObjects()
    {
        if (_aimLine == null) _aimLine = CreateDebugLine("Sensor_Aim");
        if (_sensorLeft == null) _sensorLeft = CreateDebugLine("Sensor_LaneLeft");
        if (_sensorRight == null) _sensorRight = CreateDebugLine("Sensor_LaneRight");
    }

    /// <summary>
    /// The single line every other layer here explains: where the driver decided to aim, coloured
    /// by what it decided to do about it. This is the "decision tree" made visible.
    /// </summary>
    private void DrawAimLine(Vector3 aimPoint)
    {
        _aimLine.enabled = true;
        _aimLine.widthMultiplier = runtimeLineWidth;
        Color colour = _telemetry.StateColor;
        _aimLine.startColor = colour;
        _aimLine.endColor = new Color(colour.r, colour.g, colour.b, colour.a * 0.35f);
        _aimLine.positionCount = 2;
        _aimLine.SetPosition(0, transform.position + Vector3.up * 0.6f);
        _aimLine.SetPosition(1, aimPoint + Vector3.up * 0.6f);
    }

    /// <summary>
    /// The car's actual awareness zone drawn as a curving lane rather than a straight world-space
    /// line, so it reads correctly through corners: the width traffic logic treats as "alongside"
    /// (<see cref="sideClearance"/>), extruded out to <see cref="awarenessDistance"/>.
    /// </summary>
    private void DrawSensorLane()
    {
        _sensorLeft.enabled = true;
        _sensorRight.enabled = true;
        _sensorLeft.widthMultiplier = runtimeLineWidth * 0.7f;
        _sensorRight.widthMultiplier = runtimeLineWidth * 0.7f;
        _sensorLeft.startColor = _sensorLeft.endColor = SensorLaneColour;
        _sensorRight.startColor = _sensorRight.endColor = SensorLaneColour;
        _sensorLeft.positionCount = SensorLaneSamples;
        _sensorRight.positionCount = SensorLaneSamples;

        for (int i = 0; i < SensorLaneSamples; i++)
        {
            float t = i / (float)(SensorLaneSamples - 1);
            _track.Sample(ArcLength + t * awarenessDistance, _lineIndex, out Vector3 point, out _, out Vector3 right);
            Vector3 lift = point + Vector3.up * 0.35f;
            _sensorLeft.SetPosition(i, lift + right * sideClearance);
            _sensorRight.SetPosition(i, lift - right * sideClearance);
        }
    }

    /// <summary>
    /// A line to every rival the traffic model is currently weighing up, coloured by the same
    /// classification <see cref="EvaluateTraffic"/> uses: red for the car being avoided or passed,
    /// magenta once a pass is committed, amber alongside, dim blue for everything else in range.
    /// </summary>
    private void DrawRivalLinks()
    {
        Vector3 origin = transform.position + Vector3.up * 1.4f;
        int used = 0;

        if (_participants != null)
        {
            for (int i = 0; i < _participants.Count && used < maxRivalLines; i++)
            {
                RaceParticipant other = _participants[i];
                if (other == null || other == _self || !other.IsAlive) continue;

                float gap = _track.SignedGap(ArcLength, other.ArcLength);
                if (gap > awarenessDistance || gap < -12f) continue;

                Color colour;
                if (other == _blockingCar) colour = _overtakeTimer > 0f ? OvertakingColour : ThreatColour;
                else if (gap < 0f && Mathf.Abs(other.LateralOffset - _crossTrack) < sideClearance * 1.6f) colour = AlongsideColour;
                else if (gap >= 0f && Mathf.Abs(other.LateralOffset - _crossTrack) < sideClearance * 2f) colour = ThreatColour;
                else colour = TrackedClearColour;

                LineRenderer line = RivalLine(used);
                line.enabled = true;
                line.widthMultiplier = runtimeLineWidth * 0.8f;
                line.startColor = colour;
                line.endColor = new Color(colour.r, colour.g, colour.b, colour.a * 0.5f);
                line.positionCount = 2;
                line.SetPosition(0, origin);
                line.SetPosition(1, other.Position + Vector3.up * 1.4f);
                used++;
            }
        }

        for (int i = used; i < _rivalLines.Count; i++)
        {
            _rivalLines[i].enabled = false;
        }
    }

    private LineRenderer RivalLine(int index)
    {
        while (_rivalLines.Count <= index)
        {
            _rivalLines.Add(CreateDebugLine("Sensor_Rival" + _rivalLines.Count));
        }

        return _rivalLines[index];
    }

    private LineRenderer CreateDebugLine(string debugName)
    {
        GameObject host = new GameObject(debugName) { hideFlags = HideFlags.DontSave };
        host.transform.SetParent(transform, false);

        LineRenderer line = host.AddComponent<LineRenderer>();
        line.material = RuntimeLineMaterial();
        line.useWorldSpace = true;
        line.numCapVertices = 2;
        line.widthMultiplier = runtimeLineWidth;
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        line.receiveShadows = false;
        line.generateLightingData = false;
        line.enabled = false;
        return line;
    }

    /// <summary>
    /// Sprites/Default is used deliberately: it is a built-in shader present under every render
    /// pipeline including URP, and it is one of the few unlit shaders that multiplies in the
    /// LineRenderer's per-vertex colour, which is how each line gets its own colour from one
    /// shared material instead of needing one material per line.
    /// </summary>
    private static Material RuntimeLineMaterial()
    {
        if (_runtimeLineMaterial != null) return _runtimeLineMaterial;

        Shader shader = Shader.Find("Sprites/Default")
            ?? Shader.Find("Universal Render Pipeline/Unlit")
            ?? Shader.Find("Unlit/Color");

        _runtimeLineMaterial = new Material(shader) { hideFlags = HideFlags.DontSave };
        return _runtimeLineMaterial;
    }

    /// <summary>
    /// Pure pursuit: the steering angle that puts the car on a circular arc through the aim
    /// point, then mapped back through the vehicle's own steering curve so the input means
    /// what the physics thinks it means.
    /// </summary>
    private float ComputeSteering(Vector3 aimPoint)
    {
        Vector3 toAim = aimPoint - transform.position;
        toAim.y = 0f;

        Vector3 local = transform.InverseTransformDirection(toAim);
        float distance = Mathf.Max(1f, new Vector2(local.x, local.z).magnitude);
        float alpha = Mathf.Atan2(local.x, Mathf.Max(0.5f, local.z));

        float wheelBase = Mathf.Max(0.5f, _vehicle.wheelBase);
        float pathCurvature = 2f * Mathf.Sin(alpha) / distance;
        float wheelAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase * pathCurvature);

        // Nudge back toward the intended line; pure pursuit alone leaves a steady offset
        // through long constant corners. Dividing by speed keeps the correction gentle when it
        // matters and the cap stops it turning into a weave at walking pace.
        float lineError = _lateralOffset - _crossTrack;
        float correction = Mathf.Rad2Deg * Mathf.Atan(cornerTrackingGain * lineError / Mathf.Max(12f, _speed));
        wheelAngle += Mathf.Clamp(correction, -maxTrackingCorrection, maxTrackingCorrection);

        // AckermannSteering scales the commanded lock by turnCurve(speed), so undo that here
        float speedScale = 1f;
        if (_vehicle.turnCurve != null && _vehicle.turnCurve.length > 0)
        {
            speedScale = _vehicle.turnCurve.Evaluate(Mathf.Abs(_vehicle.localVehicleVelocity.z) / Mathf.Max(1f, _vehicle.MaxSpeed));
        }

        float available = Mathf.Max(4f, _vehicle.MaxTurnAngle * Mathf.Max(0.15f, speedScale));
        return Mathf.Clamp(wheelAngle / available, -1f, 1f);
    }

    private void ComputeLongitudinal(float dt, out float throttle, out float brake)
    {
        float error = _plannedSpeed - _speed;

        if (error >= 0f)
        {
            throttle = Mathf.Clamp01(error / Mathf.Max(0.5f, throttleBand));
            brake = 0f;

            // Keep some drive on out of slow corners rather than coasting to the apex
            if (_plannedSpeed > 1f) throttle = Mathf.Max(throttle, 0.15f);
        }
        else
        {
            throttle = 0f;
            brake = Mathf.Clamp01(-error / Mathf.Max(0.5f, brakeBand));
        }

        // Still reacting to the lights
        if (_startDelay > 0f)
        {
            _startDelay -= dt;
            throttle = 0f;
            brake = 0.4f;
        }
    }

    private float ComputeHandbrake()
    {
        if (driftStyle <= 0.01f) return 0f;

        // Only for genuine hairpins, and only when the car is arriving too quickly for the
        // front axle to rotate it on its own.
        float curvature = _track.MaxCurvatureAhead(ArcLength, Mathf.Max(20f, _speed * 0.7f), _lineIndex);
        float radius = curvature > 1e-5f ? 1f / curvature : float.MaxValue;
        bool hairpin = radius < _track.HalfWidth * 2.5f;
        bool tooFast = _speed > _plannedSpeed * 1.25f;

        return hairpin && tooFast ? Mathf.Lerp(0.35f, 0.9f, driftStyle) : 0f;
    }

    /// <summary>
    /// Notices a car that is beached or facing the wrong way and reverses it out. Without this the
    /// only way back onto the track is the fall trigger, which teleports and looks terrible.
    /// </summary>
    private bool UpdateRecovery(float dt, ref float steer, ref float throttle, ref float brake, ref float handbrake)
    {
        if (_reverseTimer > 0f)
        {
            _reverseTimer -= dt;

            // Reverse away from the line, steering so the nose swings back toward it
            steer = Mathf.Clamp(_crossTrack * 0.25f, -1f, 1f);
            throttle = 0f;
            brake = 1f;
            handbrake = 0f;
            return true;
        }

        bool wantsToMove = _throttle > 0.3f || _plannedSpeed > 5f;
        if (wantsToMove && _speed < stuckSpeed) _stuckTimer += dt;
        else _stuckTimer = 0f;

        if (_stuckTimer > stuckTime)
        {
            _stuckTimer = 0f;
            _reverseTimer = reverseTime;
        }

        return false;
    }

    private bool ChooseNitro(float dt)
    {
        if (!useNitro) return false;

        _nitroTimer -= dt;

        bool available = _vehicle.currentNitro > 15f && !_vehicle.isNitroCooldown;
        if (!available)
        {
            _nitroLatched = false;
            return false;
        }

        if (_vehicle.isNitroActive) return true;
        if (_nitroTimer > 0f) return false;

        float curvature = _track.MaxCurvatureAhead(ArcLength, nitroClearDistance, _lineIndex);
        bool clearRoad = curvature < nitroCurvatureLimit;

        if (!clearRoad)
        {
            // Decide once per straight, not once per frame; the old code rolled the dice every
            // update, which made a 30% chance fire almost immediately every single time.
            _nitroLatched = false;
            return false;
        }

        if (_nitroLatched) return false;
        _nitroLatched = true;

        if (Random.value > Mathf.Lerp(0.45f, 0.95f, aggressiveness)) return false;

        _nitroTimer = Random.Range(nitroCooldownRange.x, nitroCooldownRange.y);
        return true;
    }

    /// <summary>Brings the car to a controlled stop, used when it crosses the line for the last time.</summary>
    public void StopCar()
    {
        IsRacing = false;
        _vehicle.Acceleration = _baseAcceleration;
        if (_vehicle.inputManager != null) _vehicle.inputManager.SetAIInputs(0f, -1f, 0f, false);
    }

    /// <summary>Puts the driver back on the line after a respawn so it does not fight a stale hint.</summary>
    public void OnRespawned()
    {
        _lineIndex = _track != null ? _track.Localize(transform.position, 0, _track.Count / 2) : 0;
        _stuckTimer = 0f;
        _reverseTimer = 0f;
        _steer = 0f;
        _throttle = 0f;
        _brake = 0f;
        _handbrake = 0f;
        EndOvertake();
    }

    // ---------------------------------------------------------------- helpers

    /// <summary>
    /// Frame rate independent smoothing. <c>Lerp(a, b, dt * k)</c> is the classic version of this
    /// and quietly changes behaviour with frame rate; this does not.
    /// </summary>
    private static float Damp(float current, float target, float sharpness, float dt)
    {
        return Mathf.Lerp(current, target, 1f - Mathf.Exp(-sharpness * dt));
    }

    // ---------------------------------------------------------------- debug drawing

    private static readonly Color SlowColour = new Color(1f, 0.2f, 0.15f);
    private static readonly Color FastColour = new Color(0.25f, 1f, 0.35f);

    private void OnDrawGizmos()
    {
        if (!Application.isPlaying || debugLayers == AIDebugLayers.None) return;
        if (!_ready || _track == null || _profile.Speed == null) return;

        Vector3 roof = transform.position + Vector3.up * 2.5f;

        if ((debugLayers & AIDebugLayers.SpeedProfile) != 0) DrawSpeedProfileGizmo();
        if ((debugLayers & AIDebugLayers.PlannedLine) != 0) DrawPlannedLineGizmo();
        if ((debugLayers & AIDebugLayers.Steering) != 0) DrawSteeringGizmo();
        if ((debugLayers & AIDebugLayers.Traffic) != 0) DrawTrafficGizmo();
        if ((debugLayers & AIDebugLayers.Inputs) != 0) DrawInputGauges(roof);
        if ((debugLayers & AIDebugLayers.Readout) != 0) DrawReadout(roof);
    }

    /// <summary>
    /// The plan itself, drawn as a ribbon: colour and rib height are the target speed at that
    /// point. Braking zones show up as the ribbon fading to red well before the corner, which is
    /// the whole point of the backward relaxation pass.
    /// </summary>
    private void DrawSpeedProfileGizmo()
    {
        float maxSpeed = Mathf.Max(1f, _vehicle.MaxSpeed);
        Vector3 lift = Vector3.up * 1.2f;

        int index = _lineIndex;
        float travelled = 0f;
        bool foundLift = false;
        Vector3 liftPoint = Vector3.zero;
        float slowest = float.MaxValue;
        Vector3 apex = Vector3.zero;

        for (int step = 0; step < _track.Count && travelled < debugProfileSpan; step++)
        {
            int next = (index + 1) % _track.Count;
            float target = _profile.Speed[index];
            float fraction = Mathf.Clamp01(target / maxSpeed);

            Vector3 a = _track.Points[index] + lift;
            Vector3 b = _track.Points[next] + lift;

            Gizmos.color = Color.Lerp(SlowColour, FastColour, fraction);
            Gizmos.DrawLine(a, b);

            // A rib every few points turns the ribbon into a readable speed graph
            if (step % 6 == 0) Gizmos.DrawLine(a, a + Vector3.up * fraction * 14f);

            // The first point the car is already travelling too fast for: its lift point
            if (!foundLift && target < _speed - 1f)
            {
                foundLift = true;
                liftPoint = a;
            }

            if (target < slowest)
            {
                slowest = target;
                apex = a;
            }

            travelled += _track.SegmentLength[index];
            index = next;
        }

        // Current speed drawn on the same scale, so it can be read straight off the ribbon
        Vector3 here = transform.position + lift;
        Gizmos.color = Color.white;
        Gizmos.DrawLine(here, here + Vector3.up * Mathf.Clamp01(_speed / maxSpeed) * 14f);

        if (foundLift)
        {
            Gizmos.color = new Color(1f, 0.75f, 0.05f);
            Gizmos.DrawWireSphere(liftPoint, 2.5f);
            Gizmos.DrawLine(liftPoint, liftPoint + Vector3.up * 12f);
        }

        if (slowest < float.MaxValue)
        {
            Gizmos.color = SlowColour;
            Gizmos.DrawWireSphere(apex, 2f);
        }
    }

    /// <summary>
    /// Three lines that together explain the lateral decision: where the racing line runs, the
    /// offset line the driver has chosen, and how far it currently is from that choice.
    /// </summary>
    private void DrawPlannedLineGizmo()
    {
        Gizmos.color = new Color(1f, 1f, 1f, 0.35f);
        int index = _lineIndex;
        for (int i = 0; i < 80; i++)
        {
            int next = (index + 1) % _track.Count;
            Gizmos.DrawLine(_track.Points[index], _track.Points[next]);
            index = next;
        }

        Gizmos.color = new Color(0.3f, 0.9f, 1f);
        Vector3 previous = Vector3.zero;
        for (int i = 0; i <= 24; i++)
        {
            _track.Sample(ArcLength + i * 8f, _lineIndex, out Vector3 point, out _, out Vector3 right);
            Vector3 offsetPoint = point + right * _lateralOffset + Vector3.up * 0.6f;
            if (i > 0) Gizmos.DrawLine(previous, offsetPoint);
            previous = offsetPoint;
        }

        _track.Sample(ArcLength, _lineIndex, out Vector3 origin, out _, out Vector3 sideways);
        Vector3 wanted = origin + sideways * _lateralOffset + Vector3.up;

        // Magenta is the offset it has chosen; yellow is the error it still has to steer out
        Gizmos.color = Color.magenta;
        Gizmos.DrawLine(origin + Vector3.up, wanted);
        Gizmos.DrawWireSphere(wanted, 0.6f);

        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(wanted, transform.position + Vector3.up);

        // Where the offset is allowed to reach at all
        float limit = _track.HalfWidth * 0.45f;
        Gizmos.color = new Color(1f, 1f, 1f, 0.25f);
        Gizmos.DrawLine(origin + sideways * limit, origin - sideways * limit);
    }

    /// <summary>
    /// The aim point pure pursuit is chasing, and the arc the car will actually follow at its
    /// current steering angle. When the two diverge, the car is understeering.
    /// </summary>
    private void DrawSteeringGizmo()
    {
        Vector3 eye = transform.position + Vector3.up;

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(eye, _telemetry.AimPoint + Vector3.up);
        Gizmos.DrawWireSphere(_telemetry.AimPoint + Vector3.up, 1.6f);

        float speedScale = 1f;
        if (_vehicle.turnCurve != null && _vehicle.turnCurve.length > 0)
        {
            speedScale = _vehicle.turnCurve.Evaluate(Mathf.Abs(_vehicle.localVehicleVelocity.z) / Mathf.Max(1f, _vehicle.MaxSpeed));
        }

        float wheelAngle = _steer * _vehicle.MaxTurnAngle * Mathf.Max(0.05f, speedScale);
        float radius = Mathf.Abs(wheelAngle) > 0.5f
            ? Mathf.Max(0.5f, _vehicle.wheelBase) / Mathf.Tan(Mathf.Abs(wheelAngle) * Mathf.Deg2Rad)
            : float.MaxValue;

        Gizmos.color = new Color(0.4f, 0.85f, 1f, 0.9f);
        Vector3 point = transform.position + Vector3.up * 0.4f;
        Vector3 heading = transform.forward;
        const float stepLength = 3f;

        for (int i = 0; i < 24; i++)
        {
            Vector3 next = point + heading * stepLength;
            Gizmos.DrawLine(point, next);
            point = next;

            if (radius >= 1e5f) continue;
            float turn = Mathf.Sign(wheelAngle) * stepLength / radius * Mathf.Rad2Deg;
            heading = Quaternion.Euler(0f, turn, 0f) * heading;
        }
    }

    /// <summary>Every rival the driver is weighing up, and what it decided about the nearest one.</summary>
    private void DrawTrafficGizmo()
    {
        if (_participants == null) return;

        Vector3 eye = transform.position + Vector3.up * 1.5f;

        for (int i = 0; i < _participants.Count; i++)
        {
            RaceParticipant other = _participants[i];
            if (other == null || other == _self || !other.IsAlive) continue;

            float gap = _track.SignedGap(ArcLength, other.ArcLength);
            if (gap > awarenessDistance || gap < -12f) continue;

            // Red for a car close ahead fading to grey at the edge of awareness; amber alongside
            Gizmos.color = gap >= 0f
                ? Color.Lerp(SlowColour, new Color(0.5f, 0.5f, 0.5f, 0.4f), Mathf.Clamp01(gap / awarenessDistance))
                : new Color(1f, 0.6f, 0.1f, 0.7f);

            Gizmos.DrawLine(eye, other.Position + Vector3.up * 1.5f);
        }

        if (_blockingCar != null && _blockingCar.IsAlive)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawWireSphere(_blockingCar.Position + Vector3.up * 1.5f, 3f);

            // How close it is to running out of patience with this car
            float patience = Mathf.Max(0.01f, _patience);
            Vector3 bar = eye + Vector3.up * 3.5f;
            Gizmos.color = new Color(1f, 1f, 1f, 0.3f);
            Gizmos.DrawLine(bar, bar + transform.right * 2f);
            Gizmos.color = Color.magenta;
            Gizmos.DrawLine(bar, bar + transform.right * 2f * Mathf.Clamp01(_blockedTimer / patience));
        }

        if (_overtakeSide != 0)
        {
            Gizmos.color = Color.magenta;
            Vector3 arrow = transform.position + Vector3.up * 1.5f;
            Gizmos.DrawLine(arrow, arrow + transform.right * _overtakeSide * 6f);
            Gizmos.DrawWireSphere(arrow + transform.right * _overtakeSide * 6f, 0.8f);
        }
    }

    /// <summary>Pedal and wheel positions as gauges, so inputs can be read without the profiler.</summary>
    private void DrawInputGauges(Vector3 roof)
    {
        Vector3 right = transform.right;

        DrawGauge(roof - right * 1.2f, _telemetry.Throttle, FastColour);
        DrawGauge(roof - right * 0.6f, _telemetry.Brake, SlowColour);
        DrawGauge(roof, _telemetry.Handbrake, new Color(1f, 0.4f, 1f));
        DrawGauge(roof + right * 0.6f, _telemetry.Nitro ? 1f : 0f, Color.cyan);
        DrawGauge(roof + right * 1.2f, _telemetry.SpeedFraction, Color.white);

        // Target speed marked on the speed gauge, so over and under speed are obvious
        Vector3 speedBase = roof + right * 1.2f;
        Gizmos.color = Color.yellow;
        Vector3 tick = speedBase + Vector3.up * 3f * _telemetry.TargetFraction;
        Gizmos.DrawLine(tick - right * 0.25f, tick + right * 0.25f);

        // Steering as a needle on a centred track
        Vector3 wheel = roof + Vector3.up * 3.6f;
        Gizmos.color = new Color(1f, 1f, 1f, 0.35f);
        Gizmos.DrawLine(wheel - right * 1.5f, wheel + right * 1.5f);
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(wheel, wheel + right * _telemetry.Steer * 1.5f);
        Gizmos.DrawWireSphere(wheel + right * _telemetry.Steer * 1.5f, 0.18f);
    }

    private static void DrawGauge(Vector3 baseline, float value, Color colour)
    {
        Gizmos.color = new Color(colour.r, colour.g, colour.b, 0.2f);
        Gizmos.DrawLine(baseline, baseline + Vector3.up * 3f);
        Gizmos.color = colour;
        Gizmos.DrawLine(baseline, baseline + Vector3.up * 3f * Mathf.Clamp01(value));
    }

#if UNITY_EDITOR
    private static GUIStyle _readoutStyle;

    private void DrawReadout(Vector3 roof)
    {
        if (_readoutStyle == null)
        {
            // Built from scratch rather than from GUI.skin, which Unity only guarantees inside
            // OnGUI and this is OnDrawGizmos
            _readoutStyle = new GUIStyle
            {
                fontSize = 11,
                richText = true,
                alignment = TextAnchor.UpperLeft
            };

            _readoutStyle.normal.textColor = Color.white;
        }

        AIDriverTelemetry t = _telemetry;
        Color colour = t.StateColor;
        string hex = ColorUtility.ToHtmlStringRGB(colour);

        string pass = t.OvertakeSide == 0
            ? (t.BlockingCar != null ? $"held up by {t.BlockingCar} ({t.BlockingGap:F0}m), patience {t.BlockedTimer:F1}/{t.Patience:F1}s" : "clear road")
            : $"passing {(t.OvertakeSide > 0 ? "right" : "left")} of {t.BlockingCar ?? "?"}, {t.OvertakeTimer:F1}s left";

        string text =
            $"<color=#{hex}><b>{name} - {t.State}</b></color>\n" +
            $"speed {t.Speed:F1} / {t.TargetSpeed:F1} m/s  ({t.SpeedError:+0.0;-0.0} m/s)\n" +
            $"T {t.Throttle:F2}  B {t.Brake:F2}  H {t.Handbrake:F2}  steer {t.Steer:+0.00;-0.00}{(t.Nitro ? "  NITRO" : string.Empty)}\n" +
            $"line {t.CrossTrack:+0.0;-0.0}m, aiming {t.AppliedLateral:+0.0;-0.0}m (want {t.PlannedLateral:+0.0;-0.0}m)\n" +
            $"grip {t.GripFraction:F2}  band {t.RubberBanding:F2}  react {t.ReactionTime * 1000f:F0}ms  look {t.Lookahead:F0}m\n" +
            pass;

        if (t.ReverseTimer > 0f) text += $"\nrecovering, {t.ReverseTimer:F1}s";
        else if (t.StuckTimer > 0.2f) text += $"\nstuck {t.StuckTimer:F1}s";

        UnityEditor.Handles.Label(roof + Vector3.up * 4.5f, text, _readoutStyle);
    }
#else
    private void DrawReadout(Vector3 roof)
    {
    }
#endif
}
