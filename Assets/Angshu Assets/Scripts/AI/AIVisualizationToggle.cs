using UnityEngine;

/// <summary>
/// Global toggle for AI behavior visualization
/// Press a key to toggle all visualizations on/off during gameplay
/// </summary>
public class AIVisualizationToggle : MonoBehaviour
{
    [Header("Toggle Settings")]
    [SerializeField] private KeyCode toggleKey = KeyCode.F3;
    [SerializeField] private bool startEnabled = true;
    
    private bool isEnabled;
    
    private void Start()
    {
        isEnabled = startEnabled;
        UpdateAllVisualizers();
    }
    
    private void Update()
    {
        if (Input.GetKeyDown(toggleKey))
        {
            isEnabled = !isEnabled;
            UpdateAllVisualizers();
            Debug.Log($"[AI Visualization] {(isEnabled ? "ENABLED" : "DISABLED")} - Press {toggleKey} to toggle");
        }
    }
    
    private void UpdateAllVisualizers()
    {
        var visualizers = FindObjectsByType<AIBehaviorVisualizer>(FindObjectsSortMode.None);
        foreach (var visualizer in visualizers)
        {
            if (visualizer != null)
            {
                visualizer.ToggleVisualization(isEnabled);
            }
        }
    }
}
