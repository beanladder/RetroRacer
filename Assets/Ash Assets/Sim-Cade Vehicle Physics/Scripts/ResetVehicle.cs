using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.InputSystem;

namespace Ashsvp
{
    public class ResetVehicle : MonoBehaviour
    {
        private InputSystem_Actions inputActions;

        private void Awake()
        {
            inputActions = new InputSystem_Actions();
        }

        private void OnEnable()
        {
            inputActions.Enable();
        }

        private void OnDisable()
        {
            inputActions.Disable();
        }

        public void resetVehicle()
        {
            var pos = transform.position;
            pos.y += 1;
            transform.position = pos;
            transform.rotation = Quaternion.identity;
        }
        
        public void Quit()
        {
            Application.Quit();
        }
        
        public void ResetScene()
        {
            Scene scene = SceneManager.GetActiveScene();
            SceneManager.LoadScene(scene.name);
        }

        private void Update()
        {
            // Check for R key using new input system
            if (Keyboard.current != null && Keyboard.current.rKey.wasPressedThisFrame)
            {
                resetVehicle();
            }
        }
    }
}
