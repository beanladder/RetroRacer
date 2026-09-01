using System;
using UnityEngine;

/// <summary>Which parts of a driver's reasoning to draw. Toggle these per car in the inspector.</summary>
[Flags]
public enum AIDebugLayers
{
    None = 0,

    /// <summary>The racing line ahead, the offset line the car intends to drive, and its cross track error.</summary>
    PlannedLine = 1 << 0,

    /// <summary>The speed profile ahead as a coloured ribbon, plus the lift point and the apex it is planning for.</summary>
    SpeedProfile = 1 << 1,

    /// <summary>The pure pursuit aim point and the arc the current steering angle will actually follow.</summary>
    Steering = 1 << 2,

    /// <summary>Throttle, brake, handbrake, nitro and steering as gauges above the car.</summary>
    Inputs = 1 << 3,

    /// <summary>Links to every rival being considered, the car being followed and the side chosen to pass on.</summary>
    Traffic = 1 << 4,

    /// <summary>A text readout of the driver's state above the roof. Editor only.</summary>
    Readout = 1 << 5,

    Everything = PlannedLine | SpeedProfile | Steering | Inputs | Traffic | Readout
}

/// <summary>What the driver is currently doing, in one word.</summary>
public enum AIDriverState
{
    Waiting,
    Cruising,
    Lifting,
    Braking,
    Overtaking,
    Defending,
    Recovering
}

/// <summary>
/// A snapshot of one driver's decision making, refreshed every physics step.
///
/// The gizmos and the on-screen HUD both read this rather than reaching into the controller, so
/// the driving code stays free of presentation concerns and the HUD can show any car it likes.
/// </summary>
public struct AIDriverTelemetry
{
    public bool Valid;
    public AIDriverState State;

    // Where it is
    public float ArcLength;
    public float LapProgress;
    public float CrossTrack;

    // How fast it is going and how fast it means to be going
    public float Speed;
    public float TargetSpeed;
    public float MaxSpeed;

    // Where it is aiming
    public Vector3 AimPoint;
    public float Lookahead;
    public float PlannedLateral;
    public float AppliedLateral;

    // What it is asking the car for
    public float Steer;
    public float Throttle;
    public float Brake;
    public float Handbrake;
    public bool Nitro;

    // Why
    public float GripFraction;
    public float RubberBanding;
    public float ReactionTime;
    public float BlockedTimer;
    public float Patience;
    public int OvertakeSide;
    public float OvertakeTimer;
    public bool Defending;
    public float StuckTimer;
    public float ReverseTimer;
    public string BlockingCar;
    public float BlockingGap;

    /// <summary>Speed as a fraction of the car's maximum, for gauges.</summary>
    public float SpeedFraction => MaxSpeed > 1f ? Mathf.Clamp01(Speed / MaxSpeed) : 0f;

    /// <summary>Target speed as a fraction of the car's maximum, for gauges.</summary>
    public float TargetFraction => MaxSpeed > 1f ? Mathf.Clamp01(TargetSpeed / MaxSpeed) : 0f;

    /// <summary>How far off its planned speed the car is, in m/s. Negative means it is too fast.</summary>
    public float SpeedError => TargetSpeed - Speed;

    public Color StateColor
    {
        get
        {
            switch (State)
            {
                case AIDriverState.Braking: return new Color(1f, 0.25f, 0.2f);
                case AIDriverState.Lifting: return new Color(1f, 0.75f, 0.1f);
                case AIDriverState.Overtaking: return new Color(1f, 0.4f, 1f);
                case AIDriverState.Defending: return new Color(0.4f, 0.7f, 1f);
                case AIDriverState.Recovering: return new Color(1f, 0.5f, 0f);
                case AIDriverState.Waiting: return new Color(0.6f, 0.6f, 0.6f);
                default: return new Color(0.3f, 1f, 0.4f);
            }
        }
    }
}
