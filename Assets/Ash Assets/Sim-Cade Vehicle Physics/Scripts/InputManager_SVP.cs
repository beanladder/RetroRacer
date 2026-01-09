using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.InputSystem;
using static Ashsvp.InputManager_SVP;

namespace Ashsvp
{
    public class InputManager_SVP : MonoBehaviour
    {
        // AI control flags
        private bool isAIControlled = false;
        private float aiSteerInput = 0f;
        private float aiAccelerationInput = 0f;
        private float aiHandbrakeInput = 0f;
        private bool aiNitroInput = false;
        
        // New Input System
        private InputSystem_Actions inputActions;
        private Vector2 moveInput;
        private bool sprintInput;
        private bool cameraSwitchPressed;

        [Serializable]
        public class MobileInput
        {
            public UiButton_SVP steerLeft;
            public UiButton_SVP steerRight;
            public UiButton_SVP accelerate;
            public UiButton_SVP decelerate;
            public UiButton_SVP handBrake;
            public UiButton_SVP cameraSwitch;
        }

        public bool useMobileInput = false;
        public MobileInput mobileInput = new MobileInput();

        public float SteerInput { get; private set; }
        public float AccelerationInput { get; private set; }
        public float HandbrakeInput { get; private set; }
        public bool NitroInput { get; private set; }
        public bool CameraSwitchInput { get; private set; }

        private void Awake()
        {
            inputActions = new InputSystem_Actions();
        }

        private void OnEnable()
        {
            inputActions.Enable();
            inputActions.Player.Move.performed += OnMove;
            inputActions.Player.Move.canceled += OnMove;
            inputActions.Player.Sprint.performed += OnSprint;
            inputActions.Player.Sprint.canceled += OnSprint;
            inputActions.Player.Jump.performed += OnJump;
            inputActions.Player.Jump.canceled += OnJump;
        }

        private void OnDisable()
        {
            inputActions.Player.Move.performed -= OnMove;
            inputActions.Player.Move.canceled -= OnMove;
            inputActions.Player.Sprint.performed -= OnSprint;
            inputActions.Player.Sprint.canceled -= OnSprint;
            inputActions.Player.Jump.performed -= OnJump;
            inputActions.Player.Jump.canceled -= OnJump;
            inputActions.Disable();
        }

        private void OnMove(InputAction.CallbackContext context)
        {
            moveInput = context.ReadValue<Vector2>();
        }

        private void OnSprint(InputAction.CallbackContext context)
        {
            sprintInput = context.ReadValueAsButton();
        }

        private void OnJump(InputAction.CallbackContext context)
        {
            if (context.performed)
            {
                cameraSwitchPressed = true;
            }
        }
        
        
        // Method for AI to set inputs
        public void SetAIInputs(float steer, float acceleration, float handbrake, bool nitro)
        {
            isAIControlled = true;
            aiSteerInput = steer;
            aiAccelerationInput = acceleration;
            aiHandbrakeInput = handbrake;
            aiNitroInput = nitro;
        }


        private void Update()
        {
            // If AI controlled, use AI inputs
            if (isAIControlled)
            {
                SteerInput = aiSteerInput;
                AccelerationInput = aiAccelerationInput;
                HandbrakeInput = aiHandbrakeInput;
                NitroInput = aiNitroInput;
                return;
            }
            
            // Otherwise use player inputs
            float tempSteerInput = GetNewInputSteerInput();
            float tempAccelerationInput = GetNewInputAccelerationInput();
            float tempHandbrakeInput = GetNewInputHandbrakeInput();

            if (useMobileInput)
            {
                tempSteerInput = GetMobileSteerInput();
                tempAccelerationInput = GetMobileAccelerationInput();
                tempHandbrakeInput = GetMobileHandbrakeInput();
            }

            AccelerationInput = Mathf.Abs(tempAccelerationInput) > 0 ? Mathf.Lerp(AccelerationInput, tempAccelerationInput, 15 * Time.deltaTime) : 0;
            SteerInput = Mathf.Abs(tempSteerInput) > 0 ? Mathf.Lerp(SteerInput, tempSteerInput, 15 * Time.deltaTime)
                : Mathf.Lerp(SteerInput, tempSteerInput, 25 * Time.deltaTime);
            HandbrakeInput = tempHandbrakeInput;
            NitroInput = sprintInput;
            CameraSwitchInput = useMobileInput ? mobileInput.cameraSwitch.isPressed : cameraSwitchPressed;
            
            // Reset camera switch after reading
            if (cameraSwitchPressed)
                cameraSwitchPressed = false;
        }

        private float GetNewInputSteerInput()
        {
            return moveInput.x;
        }

        private float GetNewInputAccelerationInput()
        {
            return moveInput.y;
        }

        private float GetNewInputHandbrakeInput()
        {
            return inputActions.Player.Jump.IsPressed() ? 1f : 0f;
        }


        private float GetMobileSteerInput()
        {
            float steerInput = 0f;
            if (mobileInput.steerLeft.isPressed)
                steerInput -= 1f;
            if (mobileInput.steerRight.isPressed)
                steerInput += 1f;
            return steerInput;
        }

        private float GetMobileAccelerationInput()
        {
            float accelInput = 0f;
            if (mobileInput.accelerate.isPressed)
                accelInput += 1f;
            if (mobileInput.decelerate.isPressed)
                accelInput -= 1f;
            return accelInput;
        }

        private float GetMobileHandbrakeInput()
        {
            return  mobileInput.handBrake.isPressed ? 1f : 0f;
        }

    }
}
