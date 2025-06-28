using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using Unity.Cinemachine;
namespace Ashsvp
{
    public class SimcadeVehicleController : MonoBehaviour
    {
        public InputManager_SVP inputManager;

        [Header("Suspension")]
        [Space(10)]
        public float springForce = 30000f;
        public float springDamper = 200f;
        private float MaxSpringDistance;
        private float[] suspensionForce = new float[4];
        [Header("Camera Settings")]
        [Space(10)]
        public CinemachineCamera cinemachineCamera;
        public CinemachineCamera alternativeCamera;
        public float normalCameraHeight = 1.51f;
        public float nitroCameraHeight = 0.9f;
        public float cameraTransitionSpeed = 2.0f;
        private bool isUsingAlternativeCamera = false;

        [Header("Car Stats")]
        [Space(10)]
        public float MaxSpeed = 200f;
        public float Acceleration;
        public AnimationCurve AccelerationCurve;
        public AnimationCurve InclineAccelerationCurve = AnimationCurve.Constant(0, 1, 1);
        public float MaxTurnAngle = 30f;
        public AnimationCurve turnCurve;
        public float brakeAcceleration = 50f;
        public float RollingResistance = 2f;
        public float driftFactor = 0.2f;
        [Range(0, 90)] public float maxDriftAngle = 60f;
        private float driftAngle;
        public float FrictionCoefficient = 1f;
        public float slopeSlideAngle = 30f;
        public AnimationCurve sideFrictionCurve;
        public AnimationCurve forwardFrictionCurve;
        public Transform CenterOfMass_air;
        private Vector3 CentreOfMass_ground;
        public bool AutoCounterSteer = false;
        public float DownForce = 5;
        public float airAngularDrag = 0.2f;

        [Header("Nitro System")]
        public float nitroAccelerationMultiplier = 2f;
        public float nitroMaxSpeedMultiplier = 1.5f;
        public float nitroDuration = 3f;
        public float nitroCooldown = 5f;
        public float nitroDepletionRate = 0.3f;
        public float nitroRefillRate = 0.1f;
        [Range(0, 100)] public float currentNitro = 100f;
        public bool isNitroActive;
        public bool isNitroCooldown;
        public List<ParticleSystem> nitroParticles = new List<ParticleSystem>();
        public AudioSource nitroSound;
        private float originalAcceleration;
        private float originalMaxSpeed;

        [Header("Visuals")]
        [Space(10)]
        public Transform VehicleBody;
        [Range(0, 10)] public float forwardBodyTilt = 3f;
        [Range(0, 10)] public float sidewaysBodyTilt = 3f;
        public GameObject WheelSkid;
        public GameObject SkidMarkController;
        public float wheelRadius;
        public float maxWheelTravel = 0.2f;
        public float skidmarkWidth;
        public Transform[] HardPoints = new Transform[4];
        public Transform[] Wheels;

        [HideInInspector] public Vector3 carVelocity;

        [Header("Events")]
        public Vehicle_Events VehicleEvents;

        [Serializable]
        public class Vehicle_Events
        {
            public UnityEvent OnTakeOff;
            public UnityEvent OnGrounded;
            public UnityEvent OnGearChange;
            public UnityEvent OnNitroStart;
            public UnityEvent OnNitroEnd;
        }

        private bool tempGroundedProperty;
        private RaycastHit[] wheelHits = new RaycastHit[4];
        [HideInInspector] public float steerInput, accelerationInput, handbrakeInput, rearTrack, wheelBase, ackermennLeftAngle, ackermennRightAngle;
        private Rigidbody rb;
        [HideInInspector] public Vector3 localVehicleVelocity;
        private Vector3 lastVelocity;
        private int NumberOfGroundedWheels;
        [HideInInspector] public bool vehicleIsGrounded;
        private float[] offset_Prev = new float[4];
        [HideInInspector] public bool CanDrive, CanAccelerate;
        private GearSystem GearSystem;
        [HideInInspector] public float[] forwardSlip = new float[4], slipCoeff = new float[4], skidTotal = new float[4];
        private WheelSkid[] wheelSkids = new WheelSkid[4];

        void Awake()
        {
            inputManager = inputManager ?? GetComponent<InputManager_SVP>() ?? throw new MissingComponentException("Input Manager is missing.");
            if (nitroParticles.Count == 0)
            {
                ParticleSystem[] foundParticles = GetComponentsInChildren<ParticleSystem>();
                foreach (var ps in foundParticles)
                {
                    nitroParticles.Add(ps);
                }


                if (nitroParticles.Count == 0)
                    Debug.LogWarning("No Nitro Particle Systems found on " + gameObject.name);
            }


            GameObject SkidMarkController_Self = Instantiate(SkidMarkController);
            SkidMarkController_Self.GetComponent<Skidmarks>().SkidmarkWidth = skidmarkWidth;

            CanDrive = true;
            CanAccelerate = true;

            rb = GetComponent<Rigidbody>();
            lastVelocity = Vector3.zero;

            originalAcceleration = Acceleration;
            originalMaxSpeed = MaxSpeed;

            for (int i = 0; i < Wheels.Length; i++)
            {
                HardPoints[i].localPosition = new Vector3(Wheels[i].localPosition.x, 0, Wheels[i].localPosition.z);
                wheelSkids[i] = Instantiate(WheelSkid, Wheels[i].GetChild(0)).GetComponent<WheelSkid>();
                setWheelSkidvalues_Start(i, SkidMarkController_Self.GetComponent<Skidmarks>(), wheelRadius);
            }
            MaxSpringDistance = Mathf.Abs(Wheels[0].localPosition.y - HardPoints[0].localPosition.y) + 0.1f + wheelRadius;

            wheelBase = Vector3.Distance(Wheels[0].position, Wheels[2].position);
            rearTrack = Vector3.Distance(Wheels[0].position, Wheels[1].position);

            GearSystem = GetComponent<GearSystem>();
            // Find Cinemachine camera if not assigned
if (cinemachineCamera == null)
{
    cinemachineCamera = FindFirstObjectByType<CinemachineCamera>();
    if (cinemachineCamera == null)
        Debug.LogWarning("No CinemachineCamera found in scene. Camera effects will not work.");
}
        }

        private void Start()
        {
            CentreOfMass_ground = (HardPoints[0].localPosition + HardPoints[1].localPosition + HardPoints[2].localPosition + HardPoints[3].localPosition) / 4;
            rb.centerOfMass = CentreOfMass_ground;
        }

        private void Update()
        {
            if (CanDrive && CanAccelerate)
            {
                accelerationInput = inputManager.AccelerationInput;
                steerInput = inputManager.SteerInput;
                handbrakeInput = inputManager.HandbrakeInput;
            }
            else if (CanDrive && !CanAccelerate)
            {
                accelerationInput = 0;
                steerInput = inputManager.SteerInput;
                handbrakeInput = inputManager.HandbrakeInput;
            }
            else
            {
                accelerationInput = 0;
                steerInput = 0;
                handbrakeInput = 1;
            }

            HandleNitroInput();
            RefillNitro();
            UpdateCameraPosition();
            HandleCameraSwitch();
        }

        void FixedUpdate()
        {
            localVehicleVelocity = transform.InverseTransformDirection(rb.linearVelocity);
            driftAngle = Mathf.Abs(Vector3.Angle(transform.forward, Vector3.ProjectOnPlane(rb.linearVelocity, transform.up)));

            AckermannSteering(steerInput);

            for (int i = 0; i < 4; i++) suspensionForce[i] = 0;

            for (int i = 0; i < Wheels.Length; i++)
            {
                bool wheelIsGrounded = false;
                AddSuspensionForce_2(HardPoints[i].position, Wheels[i], MaxSpringDistance, out wheelHits[i], out wheelIsGrounded, out suspensionForce[i], i);
                GroundedCheckPerWheel(wheelIsGrounded);
                tireVisual(wheelIsGrounded, Wheels[i], HardPoints[i], wheelHits[i].distance, i);
                setWheelSkidvalues_Update(i, skidTotal[i], wheelHits[i].point, wheelHits[i].normal);
            }

            float suspensionForce_hackSum = (suspensionForce[0] + suspensionForce[1] + suspensionForce[2] + suspensionForce[3]) / 4;
            for (int i = 0; i < 4; i++) suspensionForce[i] = suspensionForce_hackSum;

            vehicleIsGrounded = (NumberOfGroundedWheels > 1);

            if (vehicleIsGrounded)
            {
                AddAcceleration(accelerationInput);
                AddRollingResistance();
                brakeLogic(handbrakeInput);
                bodyAnimation();

                if (rb.centerOfMass != CentreOfMass_ground) rb.centerOfMass = CentreOfMass_ground;
                rb.angularDamping = 1;
                rb.AddForce(-transform.up * DownForce * rb.mass);
            }
            else
            {
                if (rb.centerOfMass != CenterOfMass_air.localPosition) rb.centerOfMass = CenterOfMass_air.localPosition;
                rb.angularDamping = airAngularDrag;
            }

            for (int i = 0; i < Wheels.Length; i++)
            {
                float factor = (i < 2) ? 1f : (handbrakeInput > 0.1f && driftAngle < maxDriftAngle) ? driftFactor : 1f;
                AddLateralFriction_2(HardPoints[i].position, Wheels[i], wheelHits[i], vehicleIsGrounded, factor, suspensionForce[i], i);
            }

            NumberOfGroundedWheels = 0;
            if (GroundedProperty != vehicleIsGrounded) GroundedProperty = vehicleIsGrounded;
        }
        void UpdateCameraPosition()
{
    if (cinemachineCamera == null) return;
    
    var targetOffset = cinemachineCamera.GetComponent<CinemachinePositionComposer>();
    if (targetOffset == null) return;
    
    float targetHeight = isNitroActive ? nitroCameraHeight : normalCameraHeight;
    Vector3 currentOffset = targetOffset.TargetOffset;
    Vector3 newOffset = new Vector3(
        currentOffset.x,
        Mathf.Lerp(currentOffset.y, targetHeight, Time.deltaTime * cameraTransitionSpeed),
        currentOffset.z
    );
    
    targetOffset.TargetOffset = newOffset;
}

void HandleCameraSwitch()
{
    if (inputManager.CameraSwitchInput && !isUsingAlternativeCamera && alternativeCamera != null)
    {
        cinemachineCamera.Priority = 0;
        alternativeCamera.Priority = 10;
        isUsingAlternativeCamera = true;
    }
    else if (!inputManager.CameraSwitchInput && isUsingAlternativeCamera)
    {
        cinemachineCamera.Priority = 10;
        alternativeCamera.Priority = 0;
        isUsingAlternativeCamera = false;
    }
}

        void HandleNitroInput()
        {
            if (inputManager.NitroInput && currentNitro > 0 && !isNitroCooldown && vehicleIsGrounded && !isNitroActive)
            {
                StartCoroutine(ActivateNitro());
            }
        }

        void RefillNitro()
        {
            if (!isNitroActive && currentNitro < 100f && !isNitroCooldown)
            {
                currentNitro = Mathf.Clamp(currentNitro + (nitroRefillRate * Time.deltaTime), 0, 100f);
            }
        }

        IEnumerator ActivateNitro()
        {
            isNitroActive = true;
            VehicleEvents.OnNitroStart.Invoke();

            foreach (var ps in nitroParticles)
            {
                if (ps != null && !ps.isPlaying) ps.Play();
            }

            if (nitroSound != null) nitroSound.Play();
            
            float originalDrag = rb.linearDamping;
            Acceleration *= nitroAccelerationMultiplier;
            MaxSpeed *= nitroMaxSpeedMultiplier;
            rb.linearDamping *= 0.5f;

            float timer = nitroDuration;
            while (timer > 0 && currentNitro > 0)
            {
                currentNitro -= nitroDepletionRate * Time.deltaTime;
                timer -= Time.deltaTime;
                yield return null;
            }

            Acceleration = originalAcceleration;
            MaxSpeed = originalMaxSpeed;
            rb.linearDamping = originalDrag;

            foreach (var ps in nitroParticles)
            {
                if (ps != null && ps.isPlaying) ps.Stop();
            }


            isNitroActive = false;
            isNitroCooldown = true;
            VehicleEvents.OnNitroEnd.Invoke();
            yield return new WaitForSeconds(nitroCooldown);
            isNitroCooldown = false;
        }

        void AddAcceleration(float accelerationInput)
        {
            float angle = Vector3.Angle(transform.up, Vector3.up);
            float accelerationModifier = InclineAccelerationCurve.Evaluate(angle / 180f);
            float adjustedAccelerationInput = accelerationInput * accelerationModifier;

            float deltaSpeed = Acceleration * adjustedAccelerationInput * Time.fixedDeltaTime;
            deltaSpeed = Mathf.Clamp(deltaSpeed, -MaxSpeed, MaxSpeed) * AccelerationCurve.Evaluate(Mathf.Abs(localVehicleVelocity.z / MaxSpeed));

            if ((adjustedAccelerationInput > 0 && localVehicleVelocity.z < 0) || (adjustedAccelerationInput < 0 && localVehicleVelocity.z > 0))
            {
                deltaSpeed = (1 + Mathf.Abs(localVehicleVelocity.z / MaxSpeed)) * Acceleration * adjustedAccelerationInput * Time.fixedDeltaTime;
            }
            if (handbrakeInput < 0.1f && localVehicleVelocity.z < MaxSpeed)
            {
                rb.linearVelocity += transform.forward * deltaSpeed;
            }
        }

        void AddRollingResistance()
        {
            float localSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
            float deltaSpeed = RollingResistance * Time.fixedDeltaTime * Mathf.Clamp01(Mathf.Abs(localSpeed));
            deltaSpeed = Mathf.Clamp(deltaSpeed, -MaxSpeed, MaxSpeed);
            
            if (accelerationInput == 0)
            {
                if (localSpeed > 0) rb.linearVelocity -= transform.forward * deltaSpeed;
                else rb.linearVelocity += transform.forward * deltaSpeed;
            }
        }

        void brakeLogic(float brakeInput)
        {
            float localSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
            float deltaSpeed = brakeAcceleration * brakeInput * Time.fixedDeltaTime * Mathf.Clamp01(Mathf.Abs(localSpeed));
            deltaSpeed = Mathf.Clamp(deltaSpeed, -MaxSpeed, MaxSpeed);
            
            if (localSpeed > 0) rb.linearVelocity -= transform.forward * deltaSpeed;
            else rb.linearVelocity += transform.forward * deltaSpeed;
        }

        void AddSuspensionForce_2(Vector3 hardPoint, Transform wheel, float MaxSpringDistance, out RaycastHit wheelHit, out bool WheelIsGrounded, out float SuspensionForce, int WheelNum)
        {
            var direction = -transform.up;
            WheelIsGrounded = Physics.SphereCast(hardPoint + (transform.up * wheelRadius), wheelRadius, direction, out wheelHit, MaxSpringDistance, ~0, QueryTriggerInteraction.Ignore);

            if (WheelIsGrounded)
            {
                Vector3 springDir = wheelHit.normal;
                float offset = (MaxSpringDistance + 0.1f - wheelHit.distance) / (MaxSpringDistance - wheelRadius - 0.1f);
                offset = Mathf.Clamp01(offset);
                float vel = -((offset - offset_Prev[WheelNum]) / Time.fixedDeltaTime);
                Vector3 wheelWorldVel = rb.GetPointVelocity(wheelHit.point);
                float WheelVel = Vector3.Dot(transform.up, wheelWorldVel);

                offset_Prev[WheelNum] = offset;
                if (offset < 0.3f) vel = 0;
                else if (vel < 0 && offset > 0.6f && WheelVel < 10) vel *= 10;

                float TotalSpringForce = offset * offset * springForce;
                float totalDampingForce = Mathf.Clamp(-(vel * springDamper), -0.25f * rb.mass * Mathf.Abs(WheelVel) / Time.fixedDeltaTime, 0.25f * rb.mass * Mathf.Abs(WheelVel) / Time.fixedDeltaTime);
                if ((MaxSpringDistance + 0.1f - wheelHit.distance) < 0.1f) totalDampingForce = 0;
                float force = TotalSpringForce + totalDampingForce;
                SuspensionForce = force;
                Vector3 suspensionForce_vector = Vector3.Project(springDir, transform.up) * force;
                rb.AddForceAtPosition(suspensionForce_vector, hardPoint);
            }
            else
            {
                SuspensionForce = 0;
            }
        }

        public void AddLateralFriction_2(Vector3 hardPoint, Transform wheel, RaycastHit wheelHit, bool wheelIsGrounded, float factor, float suspensionForce, int wheelNum)
        {
            if (wheelIsGrounded)
            {
                Vector3 SurfaceNormal = wheelHit.normal;
                Vector3 sideVelocity = (wheel.InverseTransformDirection(rb.GetPointVelocity(hardPoint)).x) * wheel.right;
                Vector3 forwardVelocity = (wheel.InverseTransformDirection(rb.GetPointVelocity(hardPoint)).z) * wheel.forward;

                slipCoeff[wheelNum] = sideVelocity.magnitude / (sideVelocity.magnitude + Mathf.Clamp(forwardVelocity.magnitude, 0.1f, forwardVelocity.magnitude));
                Vector3 contactDesiredAccel = -Vector3.ProjectOnPlane(sideVelocity, SurfaceNormal) / Time.fixedDeltaTime;

                Vector3 frictionForce = suspensionForce * FrictionCoefficient * -sideVelocity.normalized * sideFrictionCurve.Evaluate(slipCoeff[wheelNum]);
                float clampedFrictionForce = Mathf.Min(rb.mass / 4 * contactDesiredAccel.magnitude, -Physics.gravity.y * rb.mass);
                frictionForce = Vector3.ClampMagnitude(frictionForce * forwardFrictionCurve.Evaluate(forwardVelocity.magnitude / MaxSpeed), clampedFrictionForce);

                float slopeAngle = Vector3.Angle(transform.up, Vector3.up);
                Vector3 gravityForce = Physics.gravity.y * (rb.mass / 4) * Vector3.up;
                Vector3 gravitySideFriction = -Vector3.Project(gravityForce, transform.right);
                Vector3 gravityForwardFriction = -Vector3.Project(gravityForce, transform.forward);

                if (slopeAngle > slopeSlideAngle)
                {
                    gravitySideFriction = Vector3.zero;
                    gravityForwardFriction = Vector3.zero;
                }

                rb.AddForceAtPosition((frictionForce * factor) + gravitySideFriction, hardPoint);
                
                if(handbrakeInput > 0 || localVehicleVelocity.magnitude < 0.1f)
                {
                    rb.AddForce(gravityForwardFriction);
                }
            }
        }

        void AckermannSteering(float steerInput)
        {
            float turnRadius = wheelBase / Mathf.Tan(MaxTurnAngle / Mathf.Rad2Deg);
            if (steerInput > 0)
            {
                ackermennLeftAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius + (rearTrack / 2))) * steerInput;
                ackermennRightAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius - (rearTrack / 2))) * steerInput;
            }
            else if (steerInput < 0)
            {
                ackermennLeftAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius - (rearTrack / 2))) * steerInput;
                ackermennRightAngle = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turnRadius + (rearTrack / 2))) * steerInput;
            }
            else
            {
                ackermennLeftAngle = 0;
                ackermennRightAngle = 0;
            }

            if (localVehicleVelocity.z > 0 && AutoCounterSteer && Mathf.Abs(localVehicleVelocity.x) > 1f)
            {
                ackermennLeftAngle += Vector3.SignedAngle(transform.forward, rb.linearVelocity + transform.forward, transform.up);
                ackermennLeftAngle = Mathf.Clamp(ackermennLeftAngle, -70, 70);
                ackermennRightAngle += Vector3.SignedAngle(transform.forward, rb.linearVelocity + transform.forward, transform.up);
                ackermennRightAngle = Mathf.Clamp(ackermennRightAngle, -70, 70);
            }

            Wheels[0].localRotation = Quaternion.Euler(0, ackermennLeftAngle * turnCurve.Evaluate(localVehicleVelocity.z / MaxSpeed), 0);
            Wheels[1].localRotation = Quaternion.Euler(0, ackermennRightAngle * turnCurve.Evaluate(localVehicleVelocity.z / MaxSpeed), 0);
        }

        void tireVisual(bool WheelIsGrounded, Transform wheel, Transform hardPoint, float hitDistance, int tireNum)
        {
            if (WheelIsGrounded)
            {
                Vector3 wheelPos = wheel.localPosition;
                if (offset_Prev[tireNum] > 0.3f)
                {
                    wheelPos = hardPoint.localPosition + (Vector3.up * wheelRadius) - Vector3.up * (hitDistance);
                }
                else
                {
                    wheelPos = Vector3.Lerp(new Vector3(hardPoint.localPosition.x, wheel.localPosition.y, hardPoint.localPosition.z), 
                                          hardPoint.localPosition + (Vector3.up * wheelRadius) - Vector3.up * (hitDistance), 0.1f);
                }

                if (wheelPos.y > hardPoint.localPosition.y + wheelRadius + maxWheelTravel - MaxSpringDistance)
                {
                    wheelPos.y = hardPoint.localPosition.y + wheelRadius + maxWheelTravel - MaxSpringDistance;
                }
                wheel.localPosition = wheelPos;
            }
            else
            {
                wheel.localPosition = Vector3.Lerp(new Vector3(hardPoint.localPosition.x, wheel.localPosition.y, hardPoint.localPosition.z), 
                                                  hardPoint.localPosition + (Vector3.up * wheelRadius) - Vector3.up * MaxSpringDistance, 0.05f);
            }

            Vector3 wheelVelocity = rb.GetPointVelocity(hardPoint.position);
            float minRotation = (Vector3.Dot(wheelVelocity, wheel.forward) / wheelRadius) * Time.fixedDeltaTime * Mathf.Rad2Deg;
            float maxRotation = (Mathf.Sign(Vector3.Dot(wheelVelocity, wheel.forward)) * MaxSpeed / wheelRadius) * Time.fixedDeltaTime * Mathf.Rad2Deg;
            float wheelRotation = 0;

            if (handbrakeInput > 0.1f)
            {
                wheelRotation = 0;
            }
            else if (Mathf.Abs(accelerationInput) > 0.1f)
            {
                wheel.GetChild(0).RotateAround(wheel.position, wheel.right, maxRotation / 2);
                wheelRotation = maxRotation;
            }
            else
            {
                wheel.GetChild(0).RotateAround(wheel.position, wheel.right, minRotation);
                wheelRotation = minRotation;
            }
            wheel.GetChild(0).localPosition = Vector3.zero;
            var rot = wheel.GetChild(0).localRotation;
            rot.y = 0;
            rot.z = 0;
            wheel.GetChild(0).localRotation = rot;

            forwardSlip[tireNum] = Mathf.Abs(Mathf.Clamp((wheelRotation - minRotation) / (maxRotation), -1, 1));
            skidTotal[tireNum] = WheelIsGrounded ? Mathf.MoveTowards(skidTotal[tireNum], (forwardSlip[tireNum] + slipCoeff[tireNum]) / 2, 0.05f) : 0;
        }

        void setWheelSkidvalues_Start(int wheelNum, Skidmarks skidmarks, float radius)
        {
            wheelSkids[wheelNum].skidmarks = skidmarks;
            wheelSkids[wheelNum].radius = wheelRadius;
        }

        void setWheelSkidvalues_Update(int wheelNum, float skidTotal, Vector3 skidPoint, Vector3 normal)
        {
            wheelSkids[wheelNum].skidTotal = skidTotal;
            wheelSkids[wheelNum].skidPoint = skidPoint;
            wheelSkids[wheelNum].normal = normal;
        }

        void bodyAnimation()
        {
            Vector3 accel = Vector3.ProjectOnPlane((rb.linearVelocity - lastVelocity) / Time.fixedDeltaTime, transform.up);
            accel = transform.InverseTransformDirection(accel);
            lastVelocity = rb.linearVelocity;

            Quaternion targetRotation = Quaternion.Euler(
                Mathf.Clamp(-accel.z / 10, -forwardBodyTilt, forwardBodyTilt), 
                0, 
                Mathf.Clamp(accel.x / 5, -sidewaysBodyTilt, sidewaysBodyTilt)
            );

            if(isNitroActive) targetRotation *= Quaternion.Euler(-5, 0, 0);

            VehicleBody.localRotation = Quaternion.Lerp(VehicleBody.localRotation, targetRotation, 0.1f);
        }

        void GroundedCheckPerWheel(bool wheelIsGrounded)
        {
            if (wheelIsGrounded) NumberOfGroundedWheels += 1;
        }

        public bool GroundedProperty
        {
            get { return tempGroundedProperty; }
            set
            {
                if (value != tempGroundedProperty)
                {
                    tempGroundedProperty = value;
                    if (tempGroundedProperty) VehicleEvents.OnGrounded.Invoke();
                    else VehicleEvents.OnTakeOff.Invoke();
                }
            }
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            for (int i = 0; i < Wheels.Length; i++)
            {
                Gizmos.DrawLine(HardPoints[i].position + (transform.up * wheelRadius), Wheels[i].position);
                Gizmos.DrawWireSphere(Wheels[i].position, wheelRadius);
                Gizmos.DrawSphere(HardPoints[i].position + (transform.up * wheelRadius), 0.05f);
                UnityEditor.Handles.color = Color.red;
                UnityEditor.Handles.ArrowHandleCap(0, Wheels[i].position + transform.up * wheelRadius, Wheels[i].rotation * Quaternion.LookRotation(Vector3.up), maxWheelTravel, EventType.Repaint);
            }
        }
#endif

        private void OnCollisionEnter(Collision collision)
        {
            Vector3 wantedImpulseY = Vector3.Dot(collision.impulse, transform.up) * transform.up;
            Vector3 wantedImpulseZ = Vector3.Dot(collision.impulse, transform.forward) * transform.forward;
        }
    }
}