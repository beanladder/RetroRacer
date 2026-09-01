using Unity.Cinemachine;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// A free camera for watching AI cars, independent of whatever is racing.
///
/// Rather than spawning a second camera and juggling audio listeners, this takes over the same
/// camera Cinemachine already drives: it disables the <see cref="CinemachineBrain"/> on
/// <see cref="Camera.main"/> for as long as spectating is active, drives that camera's transform
/// directly, then re-enables the brain on the way out so Cinemachine resumes from whichever
/// vcam has priority.
///
/// Two modes, cycled with one key:
///   <b>Orbit</b> — orbits the car <see cref="AIRaceManager"/> currently has focused (the same
///   focus the debug HUD cycles), so watching a car's sensors light up and flying around it use
///   the same target.
///   <b>Free fly</b> — detaches entirely for a flythrough of the track.
/// Hold the right mouse button to look around in either mode, matching the Scene view's own
/// fly-camera convention so it needs no explanation.
/// </summary>
[RequireComponent(typeof(AIRaceManager))]
public class AISpectatorCamera : MonoBehaviour
{
    private enum Mode
    {
        Off,
        Orbit,
        FreeFly
    }

    [Header("Orbit")]
    [SerializeField, Tooltip("Starting distance from the focused car, in metres.")]
    private float orbitDistance = 14f;

    [SerializeField, Tooltip("Closest the orbit is allowed to zoom in, in metres.")]
    private float minOrbitDistance = 5f;

    [SerializeField, Tooltip("Furthest the orbit is allowed to zoom out, in metres.")]
    private float maxOrbitDistance = 70f;

    [SerializeField, Range(0.5f, 12f), Tooltip("Mouse look speed while orbiting.")]
    private float orbitSensitivity = 3.5f;

    [SerializeField, Range(0.5f, 12f), Tooltip("Scroll wheel zoom speed while orbiting.")]
    private float orbitZoomSpeed = 6f;

    [SerializeField, Tooltip("Lowest and highest pitch the orbit camera can look at, in degrees.")]
    private Vector2 pitchLimits = new Vector2(-20f, 75f);

    [SerializeField, Range(1f, 20f), Tooltip("How tightly the camera chases the car. Higher is stiffer.")]
    private float followSharpness = 6f;

    [Header("Free Fly")]
    [SerializeField, Tooltip("Cruise speed, in metres per second.")]
    private float flySpeed = 35f;

    [SerializeField, Tooltip("Speed multiplier while holding Shift.")]
    private float flyBoost = 3f;

    [SerializeField, Range(0.5f, 12f), Tooltip("Mouse look speed while flying.")]
    private float flyLookSensitivity = 3.5f;

    private AIRaceManager _manager;
    private Mode _mode = Mode.Off;
    private Camera _camera;
    private CinemachineBrain _brain;
    private bool _brainWasEnabled;
    private float _yaw;
    private float _pitch;
    private float _currentDistance;

    /// <summary>Human-readable mode, for the debug HUD.</summary>
    public string ModeName => _mode switch
    {
        Mode.Orbit => "orbit",
        Mode.FreeFly => "free fly",
        _ => "off"
    };

    private void Awake()
    {
        _manager = GetComponent<AIRaceManager>();
    }

    private void OnDisable()
    {
        SetMode(Mode.Off);
    }

    private void Update()
    {
        if (_mode == Mode.Off || _camera == null) return;

        if (_mode == Mode.Orbit) UpdateOrbit();
        else UpdateFreeFly();
    }

    /// <summary>Steps Off &#8594; Orbit &#8594; Free fly &#8594; Off. Bound to F5 by the debug HUD.</summary>
    public void CycleMode()
    {
        Mode next = _mode switch
        {
            Mode.Off => Mode.Orbit,
            Mode.Orbit => Mode.FreeFly,
            _ => Mode.Off
        };

        SetMode(next);
    }

    private void SetMode(Mode mode)
    {
        if (mode == _mode) return;

        if (_mode == Mode.Off && mode != Mode.Off) AcquireCamera();

        if (_camera != null)
        {
            if (mode == Mode.Orbit) SeedOrbitFromCurrentView();
            else if (mode == Mode.FreeFly) SeedFreeFlyFromCurrentView();
        }

        if (mode == Mode.Off && _mode != Mode.Off) ReleaseCamera();

        _mode = mode;
    }

    private void AcquireCamera()
    {
        _camera = Camera.main;
        if (_camera == null) return;

        // Disabling the brain, rather than the whole camera, keeps the AudioListener alive and
        // leaves every Cinemachine vcam's priority state untouched for when it resumes.
        _brain = _camera.GetComponent<CinemachineBrain>();
        if (_brain == null) return;

        _brainWasEnabled = _brain.enabled;
        _brain.enabled = false;
    }

    private void ReleaseCamera()
    {
        if (_brain != null) _brain.enabled = _brainWasEnabled;
        _brain = null;
        _camera = null;
    }

    private void SeedOrbitFromCurrentView()
    {
        RaceParticipant target = _manager.FocusedParticipant();
        if (target == null || !target.IsAlive)
        {
            _currentDistance = orbitDistance;
            _yaw = 0f;
            _pitch = 15f;
            return;
        }

        Vector3 offset = _camera.transform.position - target.Position;
        float distance = offset.magnitude;

        if (distance < 0.5f)
        {
            // Coming from Off with the camera already near the target: start from a clean
            // three-quarter view rather than a degenerate near-zero offset.
            _currentDistance = orbitDistance;
            _yaw = target.Transform != null ? target.Transform.eulerAngles.y + 150f : 0f;
            _pitch = 18f;
            return;
        }

        _currentDistance = Mathf.Clamp(distance, minOrbitDistance, maxOrbitDistance);
        Vector3 flat = new Vector3(offset.x, 0f, offset.z);
        _yaw = flat.sqrMagnitude > 0.001f ? Mathf.Atan2(flat.x, flat.z) * Mathf.Rad2Deg : 0f;

        float verticalRatio = Mathf.Clamp(offset.y / Mathf.Max(0.01f, distance), -1f, 1f);
        _pitch = Mathf.Clamp(Mathf.Asin(verticalRatio) * Mathf.Rad2Deg, pitchLimits.x, pitchLimits.y);
    }

    private void SeedFreeFlyFromCurrentView()
    {
        Vector3 euler = _camera.transform.eulerAngles;
        _pitch = euler.x > 180f ? euler.x - 360f : euler.x;
        _yaw = euler.y;
    }

    private void UpdateOrbit()
    {
        RaceParticipant target = _manager.FocusedParticipant();
        if (target == null || !target.IsAlive) return;

        Mouse mouse = Mouse.current;
        if (mouse != null)
        {
            if (mouse.rightButton.isPressed)
            {
                Vector2 delta = mouse.delta.ReadValue();
                _yaw += delta.x * orbitSensitivity * 0.1f;
                _pitch = Mathf.Clamp(_pitch - delta.y * orbitSensitivity * 0.1f, pitchLimits.x, pitchLimits.y);
            }

            float scroll = mouse.scroll.ReadValue().y;
            if (Mathf.Abs(scroll) > 0.01f)
            {
                _currentDistance = Mathf.Clamp(_currentDistance - scroll * orbitZoomSpeed * 0.02f, minOrbitDistance, maxOrbitDistance);
            }
        }

        Quaternion lookRotation = Quaternion.Euler(_pitch, _yaw, 0f);
        Vector3 desiredPosition = target.Position + Vector3.up * 1.4f - lookRotation * Vector3.forward * _currentDistance;

        float t = 1f - Mathf.Exp(-followSharpness * Time.deltaTime);
        _camera.transform.position = Vector3.Lerp(_camera.transform.position, desiredPosition, t);
        _camera.transform.rotation = Quaternion.Slerp(_camera.transform.rotation, lookRotation, t);
    }

    private void UpdateFreeFly()
    {
        Mouse mouse = Mouse.current;
        if (mouse != null && mouse.rightButton.isPressed)
        {
            Vector2 delta = mouse.delta.ReadValue();
            _yaw += delta.x * flyLookSensitivity * 0.1f;
            _pitch = Mathf.Clamp(_pitch - delta.y * flyLookSensitivity * 0.1f, -89f, 89f);
        }

        _camera.transform.rotation = Quaternion.Euler(_pitch, _yaw, 0f);

        Keyboard keyboard = Keyboard.current;
        if (keyboard == null) return;

        Vector3 move = Vector3.zero;
        if (keyboard[Key.W].isPressed) move += Vector3.forward;
        if (keyboard[Key.S].isPressed) move += Vector3.back;
        if (keyboard[Key.A].isPressed) move += Vector3.left;
        if (keyboard[Key.D].isPressed) move += Vector3.right;
        if (keyboard[Key.E].isPressed || keyboard[Key.Space].isPressed) move += Vector3.up;
        if (keyboard[Key.Q].isPressed || keyboard[Key.LeftCtrl].isPressed) move += Vector3.down;

        if (move.sqrMagnitude < 0.001f) return;

        float speed = flySpeed * (keyboard[Key.LeftShift].isPressed ? flyBoost : 1f);
        _camera.transform.position += _camera.transform.TransformDirection(move.normalized) * speed * Time.deltaTime;
    }
}
