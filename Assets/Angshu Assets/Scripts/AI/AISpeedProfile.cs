using UnityEngine;

/// <summary>
/// The speed a particular car can carry through every point of the racing line.
///
/// Corner speed comes from the grip circle, v = sqrt(a / k), where k is curvature. On its own
/// that is useless: it tells a car how fast it may go at the apex, not when to stop accelerating.
/// The two relaxation passes fix that. The backward pass walks the lap in reverse and caps each
/// point at the speed from which the car can still brake down to the next one, which is what puts
/// the braking point in the right place. The forward pass caps each point at the speed the car
/// could actually have reached by accelerating out of the previous one.
///
/// Each car builds its own profile, so a low grip or timid driver naturally brakes earlier and
/// carries less speed through the same corner.
/// </summary>
public sealed class AISpeedProfile
{
    /// <summary>Target speed in metres per second at each racing line point.</summary>
    public float[] Speed { get; private set; }

    /// <summary>Rebuilds the profile for a set of car limits. Cheap enough to redo when grip drifts.</summary>
    public void Build(AITrackData track, float maxSpeed, float lateralAcceleration,
        float brakingDeceleration, float driveAcceleration)
    {
        if (track == null || track.Count < 4) return;

        int count = track.Count;
        if (Speed == null || Speed.Length != count) Speed = new float[count];

        lateralAcceleration = Mathf.Max(0.5f, lateralAcceleration);
        brakingDeceleration = Mathf.Max(0.5f, brakingDeceleration);
        driveAcceleration = Mathf.Max(0.5f, driveAcceleration);

        for (int i = 0; i < count; i++)
        {
            float curvature = Mathf.Abs(track.Curvature[i]);
            float cornerSpeed = curvature > 1e-5f
                ? Mathf.Sqrt(lateralAcceleration / curvature)
                : maxSpeed;

            Speed[i] = Mathf.Min(maxSpeed, cornerSpeed);
        }

        // Two laps of each pass is enough for a closed loop to settle
        for (int pass = 0; pass < 2; pass++)
        {
            for (int i = count - 1; i >= 0; i--)
            {
                int next = (i + 1) % count;
                float reachable = Mathf.Sqrt(
                    Speed[next] * Speed[next] + 2f * brakingDeceleration * track.SegmentLength[i]);
                Speed[i] = Mathf.Min(Speed[i], reachable);
            }

            for (int i = 0; i < count; i++)
            {
                int previous = (i - 1 + count) % count;
                float reachable = Mathf.Sqrt(
                    Speed[previous] * Speed[previous] + 2f * driveAcceleration * track.SegmentLength[previous]);
                Speed[i] = Mathf.Min(Speed[i], reachable);
            }
        }
    }

    /// <summary>Target speed at an arc length.</summary>
    public float Evaluate(AITrackData track, float arcLength, int hint)
    {
        if (Speed == null || track == null) return 0f;

        int index = track.IndexAtDistance(arcLength, hint);
        int next = (index + 1) % track.Count;
        float t = Mathf.Clamp01((track.Wrap(arcLength) - track.Distance[index]) / Mathf.Max(0.001f, track.SegmentLength[index]));
        return Mathf.Lerp(Speed[index], Speed[next], t);
    }

    /// <summary>
    /// Slowest point within a distance ahead. The profile already encodes braking points, but
    /// looking ahead as well keeps the car honest when it is running off-line or behind traffic.
    /// </summary>
    public float MinimumAhead(AITrackData track, float arcLength, float distance, int hint)
    {
        if (Speed == null || track == null) return 0f;

        int index = track.IndexAtDistance(arcLength, hint);
        float travelled = 0f;
        float slowest = float.MaxValue;

        for (int step = 0; step < track.Count && travelled < distance; step++)
        {
            slowest = Mathf.Min(slowest, Speed[index]);
            travelled += track.SegmentLength[index];
            index = (index + 1) % track.Count;
        }

        return slowest == float.MaxValue ? Speed[track.IndexAtDistance(arcLength, hint)] : slowest;
    }
}
