using System;
using System.Collections.Generic;
using Track;
using UnityEngine;

/// <summary>
/// World space geometry of a track's racing line, shared by every car racing on it.
///
/// The racing line itself is just a list of track-local points. Driving code needs rather more
/// than that: arc length, heading, side normals and signed curvature. Building those once per
/// track and sharing the result keeps every AI car's per-frame work down to a short local search.
/// </summary>
public sealed class AITrackData
{
    private static readonly Dictionary<TrackGenerator, AITrackData> Cache =
        new Dictionary<TrackGenerator, AITrackData>();

    /// <summary>Number of points on the line.</summary>
    public int Count { get; private set; }

    /// <summary>Racing line points in world space.</summary>
    public Vector3[] Points { get; private set; }

    /// <summary>Horizontal unit heading at each point.</summary>
    public Vector3[] Forward { get; private set; }

    /// <summary>Horizontal unit normal at each point, pointing to the right of travel.</summary>
    public Vector3[] Right { get; private set; }

    /// <summary>Signed curvature in 1/metres. Positive bends to the right.</summary>
    public float[] Curvature { get; private set; }

    /// <summary>Distance from each point to the next one.</summary>
    public float[] SegmentLength { get; private set; }

    /// <summary>Arc length from the start of the lap to each point.</summary>
    public float[] Distance { get; private set; }

    /// <summary>Length of one lap.</summary>
    public float TotalLength { get; private set; }

    /// <summary>Distance from the centre line to the edge of the track mesh.</summary>
    public float HalfWidth { get; private set; }

    private RacingLine _source;
    private int _sourceCount;

    /// <summary>
    /// Returns the shared data for a track, building it the first time and whenever the track
    /// has been regenerated underneath us. Returns null while the racing line is not ready yet.
    /// </summary>
    public static AITrackData For(TrackGenerator generator)
    {
        if (generator == null) return null;

        RacingLine line = generator.RacingLine;
        if (line == null || line.Points.Count < 8) return null;

        if (Cache.TryGetValue(generator, out AITrackData cached) && cached.Matches(line))
            return cached;

        AITrackData data = new AITrackData();
        data.Build(generator, line);
        Cache[generator] = data;
        return data;
    }

    /// <summary>Drops the cached data for a track, forcing a rebuild on the next request.</summary>
    public static void Invalidate(TrackGenerator generator)
    {
        if (generator != null) Cache.Remove(generator);
    }

    // Statics survive a play mode restart when domain reloading is turned off
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void ClearCache() => Cache.Clear();

    private bool Matches(RacingLine line) => ReferenceEquals(_source, line) && _sourceCount == line.Points.Count;

    private void Build(TrackGenerator generator, RacingLine line)
    {
        _source = line;
        _sourceCount = line.Points.Count;

        Transform space = generator.transform;
        Count = _sourceCount;
        Points = new Vector3[Count];
        Forward = new Vector3[Count];
        Right = new Vector3[Count];
        Curvature = new float[Count];
        SegmentLength = new float[Count];
        Distance = new float[Count];
        HalfWidth = Mathf.Max(1f, generator.Width);

        for (int i = 0; i < Count; i++)
        {
            Points[i] = space.TransformPoint(line.Points[i]);
        }

        float travelled = 0f;
        for (int i = 0; i < Count; i++)
        {
            Distance[i] = travelled;
            SegmentLength[i] = Vector3.Distance(Points[i], Points[(i + 1) % Count]);
            travelled += SegmentLength[i];
        }

        TotalLength = Mathf.Max(0.001f, travelled);

        for (int i = 0; i < Count; i++)
        {
            // Central difference: less jittery than using the next point alone
            Vector3 heading = Points[(i + 1) % Count] - Points[(i - 1 + Count) % Count];
            heading.y = 0f;
            Forward[i] = heading.sqrMagnitude > 1e-6f ? heading.normalized : Vector3.forward;
            Right[i] = Vector3.Cross(Vector3.up, Forward[i]).normalized;
        }

        BuildCurvature();
    }

    private void BuildCurvature()
    {
        float averageSegment = TotalLength / Count;

        // Measure the bend over roughly a track width. Adjacent points are only a couple of
        // metres apart, so a tight stencil reads spline jitter as corners that are not there.
        int stencil = Mathf.Clamp(
            Mathf.RoundToInt(HalfWidth / Mathf.Max(0.01f, averageSegment)),
            2,
            Mathf.Max(2, Count / 8));

        for (int i = 0; i < Count; i++)
        {
            Vector3 previous = Points[(i - stencil + Count) % Count];
            Vector3 current = Points[i];
            Vector3 next = Points[(i + stencil) % Count];

            Vector3 incoming = current - previous;
            Vector3 outgoing = next - current;
            Vector3 chord = next - previous;
            incoming.y = 0f;
            outgoing.y = 0f;
            chord.y = 0f;

            float denominator = incoming.magnitude * outgoing.magnitude * chord.magnitude;
            if (denominator < 1e-5f)
            {
                Curvature[i] = 0f;
                continue;
            }

            // Menger curvature (4 * triangle area / product of side lengths), signed by the
            // turn direction so positive means the line bends right.
            float turn = incoming.z * outgoing.x - incoming.x * outgoing.z;
            Curvature[i] = 2f * turn / denominator;
        }

        Smooth(Curvature, 2);
    }

    private static void Smooth(float[] values, int passes)
    {
        int count = values.Length;
        float[] scratch = new float[count];

        for (int pass = 0; pass < passes; pass++)
        {
            for (int i = 0; i < count; i++)
            {
                scratch[i] = (values[(i - 1 + count) % count] + 2f * values[i] + values[(i + 1) % count]) * 0.25f;
            }

            Array.Copy(scratch, values, count);
        }
    }

    /// <summary>
    /// Finds the racing line point closest to a world position, searching outward from a hint.
    /// Falls back to a full scan when the hint turns out to be stale, which happens on the first
    /// frame and after a respawn.
    /// </summary>
    public int Localize(Vector3 worldPosition, int hint, int window = 24)
    {
        int best = -1;
        float bestSqr = float.MaxValue;

        for (int offset = -window; offset <= window; offset++)
        {
            int index = ((hint + offset) % Count + Count) % Count;
            float sqr = (Points[index] - worldPosition).sqrMagnitude;
            if (sqr >= bestSqr) continue;

            bestSqr = sqr;
            best = index;
        }

        // Further from the line than the track is wide means the hint cannot be trusted
        float reacquireRange = HalfWidth * 4f;
        if (best < 0 || bestSqr > reacquireRange * reacquireRange)
            return FullScan(worldPosition);

        return best;
    }

    private int FullScan(Vector3 worldPosition)
    {
        int best = 0;
        float bestSqr = float.MaxValue;

        for (int i = 0; i < Count; i++)
        {
            float sqr = (Points[i] - worldPosition).sqrMagnitude;
            if (sqr >= bestSqr) continue;

            bestSqr = sqr;
            best = i;
        }

        return best;
    }

    /// <summary>
    /// Arc length of the position projected onto the line, interpolated within the segment rather
    /// than snapped to the nearest point.
    /// </summary>
    public float ArcLengthAt(int index, Vector3 worldPosition)
    {
        int previous = (index - 1 + Count) % Count;
        int next = (index + 1) % Count;

        float forwardT = Mathf.Clamp01(ProjectOnSegment(Points[index], Points[next], worldPosition));
        float backT = Mathf.Clamp01(ProjectOnSegment(Points[previous], Points[index], worldPosition));

        Vector3 forwardPoint = Vector3.Lerp(Points[index], Points[next], forwardT);
        Vector3 backPoint = Vector3.Lerp(Points[previous], Points[index], backT);

        return (forwardPoint - worldPosition).sqrMagnitude <= (backPoint - worldPosition).sqrMagnitude
            ? Distance[index] + forwardT * SegmentLength[index]
            : Distance[previous] + backT * SegmentLength[previous];
    }

    private static float ProjectOnSegment(Vector3 a, Vector3 b, Vector3 point)
    {
        Vector3 ab = b - a;
        float lengthSqr = ab.sqrMagnitude;
        return lengthSqr < 1e-6f ? 0f : Vector3.Dot(point - a, ab) / lengthSqr;
    }

    /// <summary>Index of the segment containing an arc length, walking forward from a hint.</summary>
    public int IndexAtDistance(float arcLength, int hint)
    {
        arcLength = Wrap(arcLength);
        int index = ((hint % Count) + Count) % Count;

        for (int step = 0; step < Count; step++)
        {
            float start = Distance[index];
            int next = (index + 1) % Count;
            float end = next == 0 ? TotalLength : Distance[next];

            if (arcLength >= start && arcLength < end) return index;
            index = next;
        }

        return index;
    }

    /// <summary>Position and frame of the racing line at an arc length.</summary>
    public void Sample(float arcLength, int hint, out Vector3 position, out Vector3 forward, out Vector3 right)
    {
        int index = IndexAtDistance(arcLength, hint);
        int next = (index + 1) % Count;
        float t = Mathf.Clamp01((Wrap(arcLength) - Distance[index]) / Mathf.Max(0.001f, SegmentLength[index]));

        position = Vector3.Lerp(Points[index], Points[next], t);
        forward = Vector3.Slerp(Forward[index], Forward[next], t);
        right = Vector3.Cross(Vector3.up, forward).normalized;
    }

    /// <summary>Sharpest bend found within a distance ahead. Used to decide what is coming.</summary>
    public float MaxCurvatureAhead(float arcLength, float distance, int hint)
    {
        int index = IndexAtDistance(arcLength, hint);
        float travelled = 0f;
        float worst = 0f;

        for (int step = 0; step < Count && travelled < distance; step++)
        {
            worst = Mathf.Max(worst, Mathf.Abs(Curvature[index]));
            travelled += SegmentLength[index];
            index = (index + 1) % Count;
        }

        return worst;
    }

    /// <summary>Signed curvature interpolated at an arc length.</summary>
    public float CurvatureAt(float arcLength, int hint)
    {
        int index = IndexAtDistance(arcLength, hint);
        int next = (index + 1) % Count;
        float t = Mathf.Clamp01((Wrap(arcLength) - Distance[index]) / Mathf.Max(0.001f, SegmentLength[index]));
        return Mathf.Lerp(Curvature[index], Curvature[next], t);
    }

    /// <summary>Folds an arc length into a single lap.</summary>
    public float Wrap(float arcLength)
    {
        arcLength %= TotalLength;
        return arcLength < 0f ? arcLength + TotalLength : arcLength;
    }

    /// <summary>
    /// Shortest signed gap from one arc length to another around the loop.
    /// Positive means <paramref name="to"/> is ahead.
    /// </summary>
    public float SignedGap(float from, float to)
    {
        float gap = Wrap(to - from);
        return gap > TotalLength * 0.5f ? gap - TotalLength : gap;
    }
}
