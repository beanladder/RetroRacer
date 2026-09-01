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
        [Serializable]
        public class KeyboardInput
        {
            public KeyCode steerLeft = KeyCode.A;
            public KeyCode steerRight = KeyCode.D;
            public KeyCode accelerate = KeyCode.W;
            public KeyCode decelerate = KeyCode.S;
            public KeyCode handBrake = KeyCode.Space;
            public KeyCode cameraSwitch = KeyCode.C;
        }

        public KeyboardInput keyboardInput = new KeyboardInput();

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
            float tempSteerInput = GetKeyboardSteerInput();
            float tempAccelerationInput = GetKeyboardAccelerationInput();
            float tempHandbrakeInput = GetKeyboardHandbrakeInput();

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
            NitroInput = IsKeyPressed(KeyCode.LeftShift);
            CameraSwitchInput = useMobileInput ? mobileInput.cameraSwitch.isPressed : IsKeyPressed(keyboardInput.cameraSwitch);
        }

        private float GetKeyboardSteerInput()
        {
            float steerInput = 0f;
            if (IsKeyPressed(keyboardInput.steerLeft))
                steerInput -= 1f;
            if (IsKeyPressed(keyboardInput.steerRight))
                steerInput += 1f;
            return steerInput;
        }

        private float GetKeyboardAccelerationInput()
        {
            float accelInput = 0f;
            if (IsKeyPressed(keyboardInput.accelerate))
                accelInput += 1f;
            if (IsKeyPressed(keyboardInput.decelerate))
                accelInput -= 1f;
            return accelInput;
        }

        private float GetKeyboardHandbrakeInput()
        {
            return IsKeyPressed(keyboardInput.handBrake) ? 1f : 0f;
        }

        private bool IsKeyPressed(KeyCode keyCode)
        {
            if (Keyboard.current == null) return false;
            
            Key key = ConvertKeyCodeToKey(keyCode);
            if (key == Key.None) return false;
            
            return Keyboard.current[key].isPressed;
        }

        private Key ConvertKeyCodeToKey(KeyCode keyCode)
        {
            switch (keyCode)
            {
                case KeyCode.A: return Key.A;
                case KeyCode.B: return Key.B;
                case KeyCode.C: return Key.C;
                case KeyCode.D: return Key.D;
                case KeyCode.E: return Key.E;
                case KeyCode.F: return Key.F;
                case KeyCode.G: return Key.G;
                case KeyCode.H: return Key.H;
                case KeyCode.I: return Key.I;
                case KeyCode.J: return Key.J;
                case KeyCode.K: return Key.K;
                case KeyCode.L: return Key.L;
                case KeyCode.M: return Key.M;
                case KeyCode.N: return Key.N;
                case KeyCode.O: return Key.O;
                case KeyCode.P: return Key.P;
                case KeyCode.Q: return Key.Q;
                case KeyCode.R: return Key.R;
                case KeyCode.S: return Key.S;
                case KeyCode.T: return Key.T;
                case KeyCode.U: return Key.U;
                case KeyCode.V: return Key.V;
                case KeyCode.W: return Key.W;
                case KeyCode.X: return Key.X;
                case KeyCode.Y: return Key.Y;
                case KeyCode.Z: return Key.Z;
                case KeyCode.Space: return Key.Space;
                case KeyCode.LeftShift: return Key.LeftShift;
                case KeyCode.RightShift: return Key.RightShift;
                case KeyCode.LeftControl: return Key.LeftCtrl;
                case KeyCode.RightControl: return Key.RightCtrl;
                case KeyCode.LeftAlt: return Key.LeftAlt;
                case KeyCode.RightAlt: return Key.RightAlt;
                case KeyCode.UpArrow: return Key.UpArrow;
                case KeyCode.DownArrow: return Key.DownArrow;
                case KeyCode.LeftArrow: return Key.LeftArrow;
                case KeyCode.RightArrow: return Key.RightArrow;
                default: return Key.None;
            }
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
