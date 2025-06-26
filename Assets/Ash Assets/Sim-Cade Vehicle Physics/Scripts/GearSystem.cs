using System.Collections;
using UnityEngine;
using Unity;
using Unity.Cinemachine;

namespace Ashsvp
{
    public class GearSystem : MonoBehaviour
    {
        public float VehicleSpeed;
        public int currentGear;
        private SimcadeVehicleController vehicleController;
        public int[] gearSpeeds = new int[] { 40, 80, 120, 160, 220 };
        private CinemachineBasicMultiChannelPerlin noise;
        public float maxShakeSpeed = 200f; // Max speed at which shake reaches full effect
        public float maxShakeAmplitude = 1.2f; // Max shake strength (tweak for subtlety)
        public float baseFrequency = 1.0f; // Base frequency for noise
        public float shakeStartSpeed = 60f; // Speed at which shake begins


        [Header("Camera Settings")]
        public CinemachineCamera virtualCamera;
        public AnimationCurve fovCurve = AnimationCurve.Linear(0, 60, 1, 75);
        public float minFOV = 60f;
        public float maxFOV = 75f;
        private float maxSpeed;

        public AudioSystem AudioSystem;
        private int currentGearTemp;
        
        void Awake()
        {
            virtualCamera = GetComponentInChildren<CinemachineCamera>();
        }

        

        void Start()
        {
            vehicleController = GetComponent<SimcadeVehicleController>();
            currentGear = 1;
            maxSpeed = gearSpeeds[gearSpeeds.Length - 1];

            // Get Perlin Noise from camera
            if (virtualCamera != null)
            {
                noise = virtualCamera.GetComponent<CinemachineBasicMultiChannelPerlin>();

            }
        }


        void Update()
        {
            float velocityMag = Vector3.ProjectOnPlane(vehicleController.localVehicleVelocity, transform.up).magnitude;
            if (vehicleController.vehicleIsGrounded)
            {
                velocityMag = vehicleController.localVehicleVelocity.magnitude;
            }

            VehicleSpeed = Mathf.RoundToInt(velocityMag * 3.6f);
            gearShift();
            UpdateCameraFOV();
            // 🔥 Subtle shake based on speed
            // 🔥 Subtle shake based on speed
            if (noise != null)
            {
                if (VehicleSpeed >= shakeStartSpeed)
                {
                    float effectiveRange = maxShakeSpeed - shakeStartSpeed;
                    float clampedSpeed = Mathf.Clamp(VehicleSpeed, shakeStartSpeed, maxShakeSpeed);
                    float speedPercent = (clampedSpeed - shakeStartSpeed) / effectiveRange;
            
                    noise.AmplitudeGain = Mathf.Lerp(0f, maxShakeAmplitude, speedPercent);
                    noise.FrequencyGain = baseFrequency;
                }
                else
                {
                    noise.AmplitudeGain = 0f; // No shake below threshold
                    noise.FrequencyGain = baseFrequency;
                }
            }


        }

        void UpdateCameraFOV()
        {
            if (virtualCamera == null) return;

            float speedMultiplier = vehicleController.isNitroActive ? vehicleController.nitroMaxSpeedMultiplier : 1f;
            float normalizedSpeed = Mathf.Clamp01(VehicleSpeed / (maxSpeed * speedMultiplier));
            float targetFOV = Mathf.Lerp(minFOV, maxFOV * speedMultiplier, fovCurve.Evaluate(normalizedSpeed));

            virtualCamera.Lens.FieldOfView = Mathf.Lerp(
                virtualCamera.Lens.FieldOfView,
                targetFOV,
                Time.deltaTime * (vehicleController.isNitroActive ? 5f : 3f)
            );
        }

        void gearShift()
        {
            // Add a buffer to prevent rapid gear switching
            float upShiftBuffer = 5f; // Speed buffer to shift up
            float downShiftBuffer = 5f; // Speed buffer to shift down

            // Shift up
            if (currentGear < gearSpeeds.Length && VehicleSpeed > gearSpeeds[currentGear - 1] + upShiftBuffer)
            {
                currentGear++;
                if (CurrentGearProperty != currentGear)
                {
                    CurrentGearProperty = currentGear;
                }
            }
            // Shift down
            else if (currentGear > 1 && VehicleSpeed < gearSpeeds[currentGear - 2] - downShiftBuffer)
            {
                currentGear--;
                if (CurrentGearProperty != currentGear)
                {
                    CurrentGearProperty = currentGear;
                }
            }
        }


        public int CurrentGearProperty
        {
            get { return currentGearTemp; }
            set
            {
                currentGearTemp = value;
                if (vehicleController.accelerationInput > 0 && vehicleController.localVehicleVelocity.z > 0 && 
                    !AudioSystem.GearSound.isPlaying && vehicleController.vehicleIsGrounded)
                {
                    vehicleController.VehicleEvents.OnGearChange.Invoke();
                    AudioSystem.GearSound.Play();
                    StartCoroutine(shiftingGear());
                }
                AudioSystem.engineSound.volume = 0.5f;
            }
        }

        IEnumerator shiftingGear()
        {
            vehicleController.CanAccelerate = false;
            yield return new WaitForSeconds(0.3f);
            vehicleController.CanAccelerate = true;
        }
    }
}