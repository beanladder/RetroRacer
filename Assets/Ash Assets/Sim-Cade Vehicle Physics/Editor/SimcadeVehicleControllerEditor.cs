using Ashsvp;
using UnityEditor;

[CustomEditor(typeof(SimcadeVehicleController))]
public class SimcadeVehicleControllerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        // Just display the default inspector
        DrawDefaultInspector();
    }
}
