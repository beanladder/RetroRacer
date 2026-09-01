using System.Text;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// An on-screen readout of what every driver in the race is thinking.
///
/// Gizmos explain one car in the world; this explains the whole field at a glance, which is what
/// you want when tuning pace or working out why one car keeps losing time. Drop it on the same
/// object as the <see cref="AIRaceManager"/>.
///
/// The project runs the new Input System exclusively, so the hotkeys read <c>Keyboard.current</c>
/// rather than the legacy <c>Input</c> class, which would throw here.
/// </summary>
[RequireComponent(typeof(AIRaceManager))]
public class AIDebugHud : MonoBehaviour
{
    [Header("Display")]
    [SerializeField, Tooltip("Show the field table.")]
    private bool showHud = true;

    [SerializeField, Tooltip("Show a detailed panel for the focused car.")]
    private bool showDetail = true;

    [SerializeField, Tooltip("Corner offset of the panel, in pixels.")]
    private Vector2 origin = new Vector2(12f, 12f);

    [SerializeField, Range(9, 24), Tooltip("Font size of the readout.")]
    private int fontSize = 12;

    [Header("Hotkeys")]
    [SerializeField, Tooltip("Toggles the whole overlay.")]
    private Key toggleKey = Key.F1;

    [SerializeField, Tooltip("Steps the detail panel to the next car.")]
    private Key cycleKey = Key.F2;

    [SerializeField, Tooltip("Turns every debug gizmo layer on the focused car on or off.")]
    private Key gizmoKey = Key.F3;

    [SerializeField, Tooltip("Turns the Game-view sensor and decision lines on the focused car on or off.")]
    private Key runtimeVisualsKey = Key.F4;

    [SerializeField, Tooltip("Cycles the spectator camera: off, orbit, free fly.")]
    private Key spectatorKey = Key.F5;

    private AIRaceManager _manager;
    private AISpectatorCamera _spectator;
    private GUIStyle _style;
    private Texture2D _background;
    private readonly StringBuilder _builder = new StringBuilder(1024);

    private void Awake()
    {
        _manager = GetComponent<AIRaceManager>();
        _spectator = GetComponent<AISpectatorCamera>();

        // The hotkeys below do nothing if this component was never added to a GameObject in the
        // first place — a MonoBehaviour that only exists as a .cs file never runs. This line is
        // the fastest way to tell those two situations apart: if it never appears in the Console,
        // AIDebugHud is missing from the scene, not broken.
        Debug.Log($"[AIDebugHud] Ready on '{name}'. F1 hide, F2 focus, F3 gizmos, F4 sensors, F5 spectate" +
                  (_spectator == null ? " (no AISpectatorCamera on this object — F5 will do nothing until one is added)." : "."));
    }

    private void OnDestroy()
    {
        if (_background != null) Destroy(_background);
    }

    private void Update()
    {
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null) return;

        if (keyboard[toggleKey].wasPressedThisFrame) showHud = !showHud;
        if (keyboard[cycleKey].wasPressedThisFrame) _manager.CycleFocus();
        if (keyboard[gizmoKey].wasPressedThisFrame) ToggleFocusedGizmos();

        if (keyboard[runtimeVisualsKey].wasPressedThisFrame)
        {
            AIVehicleController.ShowRuntimeVisuals = !AIVehicleController.ShowRuntimeVisuals;
            AIVehicleController focused = FocusedDriver();
            string state = AIVehicleController.ShowRuntimeVisuals ? "ON" : "OFF";
            string on = focused != null ? focused.name : "no AI car focused yet (grid may not have spawned)";
            Debug.Log($"[AIDebugHud] Sensor lines {state} — drawing on {on}.");
        }

        if (keyboard[spectatorKey].wasPressedThisFrame)
        {
            if (_spectator == null)
            {
                Debug.LogWarning("[AIDebugHud] F5 pressed, but no AISpectatorCamera is attached here. Add one next to AIRaceManager.");
            }
            else
            {
                _spectator.CycleMode();
                Debug.Log($"[AIDebugHud] Spectator camera: {_spectator.ModeName}.");
            }
        }
    }

    private void ToggleFocusedGizmos()
    {
        AIVehicleController driver = FocusedDriver();
        if (driver == null) return;

        driver.DebugLayers = driver.DebugLayers == AIDebugLayers.None
            ? AIDebugLayers.Everything
            : AIDebugLayers.None;
    }

    private AIVehicleController FocusedDriver()
    {
        return _manager.FocusedParticipant()?.Ai;
    }

    private void OnGUI()
    {
        if (!showHud) return;

        var participants = _manager.Participants;
        if (participants.Count == 0) return;

        EnsureStyle();

        float width = 560f;
        float lineHeight = fontSize + 5f;
        float tableHeight = lineHeight * (participants.Count + 3);

        GUI.DrawTexture(new Rect(origin.x, origin.y, width, tableHeight), _background);
        GUI.Label(new Rect(origin.x + 8f, origin.y + 4f, width, tableHeight), BuildTable(), _style);

        if (!showDetail) return;

        AIVehicleController driver = FocusedDriver();
        if (driver == null) return;

        float detailY = origin.y + tableHeight + 8f;
        float detailHeight = lineHeight * 11f;

        GUI.DrawTexture(new Rect(origin.x, detailY, width, detailHeight), _background);
        GUI.Label(new Rect(origin.x + 8f, detailY + 4f, width, detailHeight), BuildDetail(driver), _style);
    }

    private string BuildTable()
    {
        _builder.Clear();
        _builder.AppendLine("<b>AI FIELD</b>   <color=#888888>F1 hide   F2 focus   F3 gizmos   F4 sensors   F5 spectate</color>");
        _builder.AppendLine("<color=#888888>car                 state       speed / target      T    B    grip</color>");

        var participants = _manager.Participants;
        int focusIndex = _manager.FocusIndex;
        for (int i = 0; i < participants.Count; i++)
        {
            RaceParticipant participant = participants[i];
            if (!participant.IsAlive) continue;

            string marker = i == focusIndex ? "> " : "  ";
            string carName = Trim(participant.Transform.name, 16);

            if (participant.Ai == null)
            {
                _builder.AppendLine($"{marker}{carName,-16}  <color=#88ccff>PLAYER</color>      {participant.Speed,5:F1}");
                continue;
            }

            AIDriverTelemetry t = participant.Ai.Telemetry;
            string hex = ColorUtility.ToHtmlStringRGB(t.StateColor);

            _builder.AppendLine(
                $"{marker}{carName,-16}  <color=#{hex}>{t.State,-10}</color>  {t.Speed,5:F1} /{t.TargetSpeed,6:F1}  {t.Throttle,4:F2} {t.Brake,4:F2}  {t.GripFraction,4:F2}");
        }

        return _builder.ToString();
    }

    private string BuildDetail(AIVehicleController driver)
    {
        AIDriverTelemetry t = driver.Telemetry;
        string hex = ColorUtility.ToHtmlStringRGB(t.StateColor);

        _builder.Clear();
        _builder.AppendLine($"<b>{driver.name}</b>   <color=#{hex}>{t.State}</color>");
        _builder.AppendLine($"skill {driver.SkillLevel:F2}   aggression {driver.Aggressiveness:F2}   reaction {t.ReactionTime * 1000f:F0}ms");
        _builder.AppendLine($"speed      {t.Speed,6:F1} m/s      target {t.TargetSpeed,6:F1}      error {t.SpeedError,6:+0.0;-0.0}");
        _builder.AppendLine($"inputs     throttle {t.Throttle:F2}   brake {t.Brake:F2}   handbrake {t.Handbrake:F2}   steer {t.Steer:+0.00;-0.00}");
        _builder.AppendLine($"line       at {t.CrossTrack:+0.0;-0.0}m   aiming {t.AppliedLateral:+0.0;-0.0}m   want {t.PlannedLateral:+0.0;-0.0}m");
        _builder.AppendLine($"planning   lookahead {t.Lookahead:F0}m   grip {t.GripFraction:F2}   rubber band {t.RubberBanding:F2}");
        _builder.AppendLine($"lap        {t.LapProgress * 100f:F1}%   arc {t.ArcLength:F0}m");

        bool sensorsOn = AIVehicleController.ShowRuntimeVisuals;
        string spectatorState = _spectator != null ? _spectator.ModeName : "n/a";
        _builder.AppendLine($"<color=#888888>sensors {(sensorsOn ? "on" : "off")}   spectator {spectatorState}</color>");

        if (t.OvertakeSide != 0)
        {
            _builder.AppendLine($"<color=#ff66ff>passing {(t.OvertakeSide > 0 ? "right" : "left")} of {t.BlockingCar ?? "?"}, {t.OvertakeTimer:F1}s remaining</color>");
        }
        else if (t.BlockingCar != null)
        {
            _builder.AppendLine($"held up by {t.BlockingCar} at {t.BlockingGap:F0}m, patience {t.BlockedTimer:F1}/{t.Patience:F1}s");
        }
        else
        {
            _builder.AppendLine("<color=#888888>clear road</color>");
        }

        if (t.Defending) _builder.AppendLine("<color=#66aaff>defending its position</color>");
        if (t.ReverseTimer > 0f) _builder.AppendLine($"<color=#ff8800>recovering, {t.ReverseTimer:F1}s</color>");
        else if (t.StuckTimer > 0.2f) _builder.AppendLine($"<color=#ff8800>stuck for {t.StuckTimer:F1}s</color>");
        if (t.Nitro) _builder.AppendLine("<color=#00ffff>nitro</color>");

        return _builder.ToString();
    }

    private static string Trim(string value, int length)
    {
        return value.Length <= length ? value : value.Substring(0, length);
    }

    private void EnsureStyle()
    {
        if (_background == null)
        {
            _background = new Texture2D(1, 1);
            _background.SetPixel(0, 0, new Color(0f, 0f, 0f, 0.72f));
            _background.Apply();
        }

        if (_style != null && _style.fontSize == fontSize) return;

        _style = new GUIStyle(GUI.skin.label)
        {
            fontSize = fontSize,
            richText = true,
            alignment = TextAnchor.UpperLeft,
            wordWrap = false
        };

        _style.normal.textColor = Color.white;
    }
}
