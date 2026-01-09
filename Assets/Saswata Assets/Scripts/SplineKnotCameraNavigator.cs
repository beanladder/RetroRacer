using UnityEngine;
using Unity.Cinemachine;
using UnityEngine.Splines;
using UnityEngine.InputSystem;

public class SplineKnotCameraNavigator : MonoBehaviour
{
    [SerializeField] private CinemachineSplineDolly dolly;
    [SerializeField] private SplineContainer splineContainer;
    [SerializeField] private float moveSpeed = 5f;

    private int currentKnotIndex = 0;
    private float targetPosition;
    private int knotCount;

    private void Start()
    {
        if (dolly == null || splineContainer == null)
        {
            Debug.LogError("Dolly or SplineContainer is not assigned.");
            enabled = false;
            return;
        }
    
        dolly.PositionUnits = PathIndexUnit.Knot;
        knotCount = splineContainer.Spline.Count;
    
        currentKnotIndex = 3; // ✅ Start at 3rd knot
        MoveToKnot(currentKnotIndex);
        dolly.CameraPosition = targetPosition;
    }

    private void Update()
    {
        if (Keyboard.current != null)
        {
            if (Keyboard.current.rightArrowKey.wasPressedThisFrame)
            {
                // Move right (decrement index)
                currentKnotIndex = (currentKnotIndex - 1 + knotCount) % knotCount;
                MoveToKnot(currentKnotIndex);
            }
            else if (Keyboard.current.leftArrowKey.wasPressedThisFrame)
            {
                // Move left (increment index)
                currentKnotIndex = (currentKnotIndex + 1) % knotCount;
                MoveToKnot(currentKnotIndex);
            }
        }

        float current = dolly.CameraPosition;
        float target = targetPosition;

        // Handle wrap-around movement (shortest path)
        float directDist = Mathf.Abs(target - current);
        float wrappedDist = knotCount - directDist;

        if (directDist <= wrappedDist)
        {
            // Move directly
            dolly.CameraPosition = Mathf.MoveTowards(current, target, moveSpeed * Time.deltaTime);
        }
        else
        {
            // Move the wrapped way
            float direction = Mathf.Sign(target - current);
            direction = -direction; // reverse for shortest route
            dolly.CameraPosition = (dolly.CameraPosition + direction * moveSpeed * Time.deltaTime + knotCount) % knotCount;
        }
    }

    private void MoveToKnot(int knotIndex)
    {
        targetPosition = knotIndex;
    }
}
