using Ashsvp;
using UnityEngine;

/// <summary>
/// One car in the race, AI or player.
///
/// The AI used to keep a list of other <see cref="AIVehicleController"/>s, which meant it could
/// not see the player at all: the one car you actually race against was invisible to every
/// opponent. Tracking participants instead of controllers fixes that, and gives the traffic code
/// a rigidbody to read real velocities from.
///
/// The race manager refreshes <see cref="ArcLength"/> and <see cref="LateralOffset"/> once per
/// physics step for everyone, so each driver gets the whole field in racing line coordinates
/// without every car localising every other car.
/// </summary>
public sealed class RaceParticipant
{
    public Transform Transform { get; }
    public Rigidbody Body { get; }
    public SimcadeVehicleController Vehicle { get; }
    public AIVehicleController Ai { get; }
    public bool IsPlayer { get; }

    /// <summary>Distance travelled around the current lap, in metres.</summary>
    public float ArcLength { get; set; }

    /// <summary>Offset from the racing line, in metres. Positive is to the right of travel.</summary>
    public float LateralOffset { get; set; }

    /// <summary>Cached racing line index, used as the search hint for the next lookup.</summary>
    public int LineIndex { get; set; }

    public RaceParticipant(GameObject car, AIVehicleController ai, bool isPlayer)
    {
        Transform = car != null ? car.transform : null;
        Body = car != null ? car.GetComponent<Rigidbody>() : null;
        Vehicle = car != null ? car.GetComponent<SimcadeVehicleController>() : null;
        Ai = ai;
        IsPlayer = isPlayer;
    }

    public bool IsAlive => Transform != null;

    public Vector3 Position => Transform != null ? Transform.position : Vector3.zero;

    public Vector3 Velocity => Body != null ? Body.linearVelocity : Vector3.zero;

    public float Speed => Body != null ? Body.linearVelocity.magnitude : 0f;
}
