using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using UnityEngine.Splines;

namespace Track
{
    /// <summary>
    /// Generates and stores an optimal racing line for AI vehicles to follow
    /// </summary>
    public class RacingLine
    {
        private List<Vector3> _racingLinePoints = new List<Vector3>();
        private List<float> _recommendedSpeeds = new List<float>(); // Speed values at each point (0-1 normalized)
        
        public List<Vector3> Points => _racingLinePoints;
        public List<float> RecommendedSpeeds => _recommendedSpeeds;
        
        /// <summary>
        /// Generates an optimal racing line based on a central spline.
        /// </summary>
        /// <param name="spline">The central spline of the track.</param>
        /// <param name="trackWidth">The width of the track.</param>
        /// <param name="resolution">Number of points to generate for the racing line.</param>
        /// <param name="minDeviationFactor">The deviation factor for straight sections.</param>
        /// <param name="maxDeviationFactor">The deviation factor for corners.</param>
        /// <param name="deviationScale">The scale of the Perlin noise for deviation, affects "waviness".</param>
        public void Generate(Spline spline, float trackWidth, int resolution, float minDeviationFactor, float maxDeviationFactor, float deviationScale = 8f)
        {
            if (spline == null || spline.Count < 2)
            {
                Debug.LogWarning("Racing Line generation failed: Spline is invalid.");
                return;
            }

            _racingLinePoints.Clear();
            _recommendedSpeeds.Clear();

            float perlinSeed = UnityEngine.Random.Range(0f, 1000f);
            
            // Step 1: Generate points by sampling the spline and adding natural deviation.
            for (int i = 0; i <= resolution; i++)
            {
                float t = (float)i / resolution;

                // Position, tangent, and normal from the central spline
                Vector3 pointOnSpline = spline.EvaluatePosition(t);
                Vector3 tangent = math.normalize(spline.EvaluateTangent(t));
                Vector3 normal = Vector3.Cross(tangent, Vector3.up).normalized;

                // --- NEW: Calculate curvature to vary the deviation ---
                float lookbehind = Mathf.Clamp01(t - 0.02f);
                float lookahead = Mathf.Clamp01(t + 0.02f);
                Vector3 tangentBehind = math.normalize(spline.EvaluateTangent(lookbehind));
                Vector3 tangentAhead = math.normalize(spline.EvaluateTangent(lookahead));
                
                // Angle between tangents gives us a measure of curvature
                float angle = Vector3.Angle(tangentBehind, tangentAhead);
                // Normalize curvature, clamping at 45 degrees as a "very sharp" turn
                float normalizedCurvature = Mathf.InverseLerp(0f, 45f, angle);

                // Determine current deviation factor based on curvature
                float currentDeviationFactor = Mathf.Lerp(minDeviationFactor, maxDeviationFactor, normalizedCurvature);

                // Perlin noise creates a smooth, random value between -1 and 1
                float noise = (Mathf.PerlinNoise(t * deviationScale, perlinSeed) * 2f) - 1f;

                // Calculate the maximum possible deviation from the center line
                float maxDeviation = trackWidth * 0.45f * currentDeviationFactor;

                // Apply the deviation
                float deviation = noise * maxDeviation;
                _racingLinePoints.Add(pointOnSpline + normal * deviation);
            }

            // Step 1.5: Smooth the generated racing line to remove high-frequency noise.
            // This is crucial for the AI's corner detection to work correctly.
            List<Vector3> rawPoints = new List<Vector3>(_racingLinePoints);
            
            // We do two passes of smoothing for a better result.
            for (int pass = 0; pass < 2; pass++)
            {
                _racingLinePoints.Clear();
                for (int i = 0; i < rawPoints.Count; i++)
                {
                    // Use a 5-point weighted average for smoothing.
                    Vector3 p0 = rawPoints[(i - 2 + rawPoints.Count) % rawPoints.Count];
                    Vector3 p1 = rawPoints[(i - 1 + rawPoints.Count) % rawPoints.Count];
                    Vector3 p2 = rawPoints[i];
                    Vector3 p3 = rawPoints[(i + 1) % rawPoints.Count];
                    Vector3 p4 = rawPoints[(i + 2) % rawPoints.Count];

                    Vector3 smoothedPoint = p0 * 0.1f + p1 * 0.2f + p2 * 0.4f + p3 * 0.2f + p4 * 0.1f;
                    _racingLinePoints.Add(smoothedPoint);
                }
                // Use the smoothed points as input for the next pass.
                rawPoints = new List<Vector3>(_racingLinePoints);
            }

            // Step 2: Recalculate recommended speeds based on the final line's curvature.
            _recommendedSpeeds.Clear();

            for (int i = 0; i < _racingLinePoints.Count; i++)
            {
                int prevIndex = (i - 1 + _racingLinePoints.Count) % _racingLinePoints.Count;
                int nextIndex = (i + 1) % _racingLinePoints.Count;

                Vector3 toPrev = (_racingLinePoints[prevIndex] - _racingLinePoints[i]).normalized;
                Vector3 toNext = (_racingLinePoints[nextIndex] - _racingLinePoints[i]).normalized;

                // Curvature is calculated from the angle between segments.
                float curvature = 1.0f - (Vector3.Dot(toPrev, toNext) + 1.0f) / 2.0f;

                // Recommended speed is inversely related to curvature.
                float speedFactor = Mathf.Lerp(0.3f, 1.0f, Mathf.Pow(1.0f - curvature, 0.7f));

                // Smooth speed transitions between points.
                if (i > 0)
                {
                    float prevSpeed = _recommendedSpeeds[i - 1];
                    float maxChange = 0.1f; // Max speed change between adjacent points
                    speedFactor = Mathf.Clamp(speedFactor, prevSpeed - maxChange, prevSpeed + maxChange);
                }

                _recommendedSpeeds.Add(speedFactor);
            }

            // Step 3: Apply a final smoothing pass to the speed profile.
            List<float> smoothedSpeeds = new List<float>(_recommendedSpeeds);
            _recommendedSpeeds.Clear();

            for (int i = 0; i < smoothedSpeeds.Count; i++)
            {
                int prev = (i - 1 + smoothedSpeeds.Count) % smoothedSpeeds.Count;
                int next = (i + 1) % smoothedSpeeds.Count;

                // Weighted average for smoother speed transitions
                float smoothedSpeed = smoothedSpeeds[prev] * 0.25f +
                                     smoothedSpeeds[i] * 0.5f +
                                     smoothedSpeeds[next] * 0.25f;

                _recommendedSpeeds.Add(smoothedSpeed);
            }
        }
        
        /// <summary>
        /// Gets the closest point on the racing line to the given position
        /// </summary>
        public (int index, Vector3 point, float speed) GetClosestPoint(Vector3 position)
        {
            if (_racingLinePoints.Count == 0)
                return (-1, Vector3.zero, 1.0f);
                
            int closestIndex = 0;
            float closestDistance = float.MaxValue;
            
            for (int i = 0; i < _racingLinePoints.Count; i++)
            {
                float distance = Vector3.Distance(position, _racingLinePoints[i]);
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestIndex = i;
                }
            }
            
            return (closestIndex, _racingLinePoints[closestIndex], _recommendedSpeeds[closestIndex]);
        }
        
        /// <summary>
        /// Gets the next target point on the racing line
        /// </summary>
        public (Vector3 point, float speed) GetNextTargetPoint(int currentIndex, int lookaheadPoints = 5)
        {
            if (_racingLinePoints.Count == 0)
                return (Vector3.zero, 1.0f);
                
            int targetIndex = (currentIndex + lookaheadPoints) % _racingLinePoints.Count;
            return (_racingLinePoints[targetIndex], _recommendedSpeeds[targetIndex]);
        }
    }
}