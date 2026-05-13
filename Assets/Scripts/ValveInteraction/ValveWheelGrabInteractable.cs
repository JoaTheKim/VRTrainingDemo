using System;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

namespace VRTraining.ValveInteraction
{
    /// <summary>
    /// Controller-driven valve wheel using XR Grab Interactable without physics joints.
    /// Rotation is derived from the interactor attach pose projected onto the wheel plane (same idea as the VR Template XRKnob).
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    [DisallowMultipleComponent]
    public class ValveWheelGrabInteractable : XRGrabInteractable
    {
        const float k_DefaultLimitSnapThresholdDegrees = 4.0f;

        [Serializable]
        public class AngleChangedEvent : UnityEvent<float> { }

        [Header("Visual target")]
        [SerializeField]
        [Tooltip("Transform that receives the final local Euler rotation on the chosen axis. Defaults to this object if unset.")]
        Transform m_WheelTransform;

        [SerializeField]
        [Tooltip("Local axis of the wheel that spins.")]
        ValveRotationAxis m_RotationAxis = ValveRotationAxis.Y;

        [Header("Angle limits")]
        [SerializeField]
        float m_MinAngle = -120.0f;

        [SerializeField]
        float m_MaxAngle = 120.0f;

        [SerializeField]
        [Tooltip("Multiplies the angular delta from hand motion each frame. Values below 1 reduce travel; above 1 amplify it.")]
        float m_RotationSensitivity = 1.0f;

        [Header("Optional detents")]
        [SerializeField]
        [Tooltip("If greater than zero, the angle snaps to the nearest multiple of this value (degrees).")]
        float m_DetentStepDegrees = 0.0f;

        [Header("Limit snap and feedback")]
        [SerializeField]
        [Tooltip("When enabled, angles within the threshold of a limit snap to that limit.")]
        bool m_SnapToLimits = true;

        [SerializeField]
        [Tooltip("Distance from min or max angle (degrees) at which snapping applies.")]
        float m_LimitSnapThresholdDegrees = k_DefaultLimitSnapThresholdDegrees;

        [SerializeField]
        [Tooltip("Short haptic pulse on the selecting controller when snapping to a limit.")]
        bool m_HapticOnLimitSnap = true;

        [SerializeField]
        float m_LimitSnapHapticAmplitude = 0.35f;

        [SerializeField]
        float m_LimitSnapHapticDuration = 0.08f;

        [Header("Events")]
        [SerializeField]
        AngleChangedEvent m_OnAngleChanged = new AngleChangedEvent();

        float m_CurrentAngle;
        float m_PreviousPlaneAngle;
        bool m_HasPreviousPlaneAngle;
        bool m_WasAtMinLimit;
        bool m_WasAtMaxLimit;

        /// <summary>Current signed angle in degrees after clamping and optional detents.</summary>
        public float CurrentAngle => m_CurrentAngle;

        /// <summary>Normalized 0–1 between min and max when max &gt; min; otherwise 0.</summary>
        public float NormalizedOpen =>
            Mathf.Approximately(m_MaxAngle, m_MinAngle) ? 0.0f : Mathf.InverseLerp(m_MinAngle, m_MaxAngle, m_CurrentAngle);

        public AngleChangedEvent OnAngleChanged => m_OnAngleChanged;

        /// <inheritdoc />
        protected override void Awake()
        {
            addDefaultGrabTransformers = false;
            trackPosition = false;
            trackRotation = false;
            throwOnDetach = false;

            base.Awake();

            ClearSingleGrabTransformers();
            ClearMultipleGrabTransformers();

            if (m_WheelTransform == null)
                m_WheelTransform = transform;

            ConfigureRigidbodyForValve();
        }

        void Start()
        {
            if (m_WheelTransform == null)
                m_WheelTransform = transform;

            m_CurrentAngle = ReadAngleFromWheelLocalEuler();
            ApplyWheelLocalRotation();
            m_OnAngleChanged.Invoke(m_CurrentAngle);
        }

        protected override void Reset()
        {
            base.Reset();

            addDefaultGrabTransformers = false;
            trackPosition = false;
            trackRotation = false;
            throwOnDetach = false;
            movementType = MovementType.Kinematic;

            if (!TryGetComponent(out Rigidbody body))
                body = gameObject.AddComponent<Rigidbody>();

            body.useGravity = false;
            ClearRigidbodyVelocitiesIfDynamic(body);
            body.isKinematic = true;
        }

        /// <inheritdoc />
        public override void ProcessInteractable(XRInteractionUpdateOrder.UpdatePhase updatePhase)
        {
            base.ProcessInteractable(updatePhase);

            if (updatePhase != XRInteractionUpdateOrder.UpdatePhase.Dynamic)
                return;

            if (!isSelected || interactorsSelecting.Count == 0)
                return;

            var attachTransform = interactorsSelecting[0].GetAttachTransform(this);

            var planeAngle = ComputePlaneAngleDegrees(attachTransform.position);
            if (!m_HasPreviousPlaneAngle)
            {
                m_PreviousPlaneAngle = planeAngle;
                m_HasPreviousPlaneAngle = true;
                return;
            }

            var delta = Mathf.DeltaAngle(m_PreviousPlaneAngle, planeAngle) * m_RotationSensitivity;
            m_PreviousPlaneAngle = planeAngle;

            m_CurrentAngle += delta;
            m_CurrentAngle = Mathf.Clamp(m_CurrentAngle, m_MinAngle, m_MaxAngle);

            if (m_SnapToLimits && m_LimitSnapThresholdDegrees > 0.0f)
            {
                if (m_CurrentAngle - m_MinAngle <= m_LimitSnapThresholdDegrees)
                {
                    TrySnapToLimit(ref m_CurrentAngle, m_MinAngle, ref m_WasAtMinLimit);
                    m_WasAtMaxLimit = false;
                }
                else if (m_MaxAngle - m_CurrentAngle <= m_LimitSnapThresholdDegrees)
                {
                    TrySnapToLimit(ref m_CurrentAngle, m_MaxAngle, ref m_WasAtMaxLimit);
                    m_WasAtMinLimit = false;
                }
                else
                {
                    m_WasAtMinLimit = false;
                    m_WasAtMaxLimit = false;
                }
            }

            if (m_DetentStepDegrees > Mathf.Epsilon)
                m_CurrentAngle = Mathf.Round(m_CurrentAngle / m_DetentStepDegrees) * m_DetentStepDegrees;

            m_CurrentAngle = Mathf.Clamp(m_CurrentAngle, m_MinAngle, m_MaxAngle);

            ApplyWheelLocalRotation();
            m_OnAngleChanged.Invoke(m_CurrentAngle);
        }

        protected override void OnSelectEntered(SelectEnterEventArgs args)
        {
            base.OnSelectEntered(args);

            m_WasAtMinLimit = false;
            m_WasAtMaxLimit = false;

            var attachTransform = args.interactorObject.GetAttachTransform(this);
            m_PreviousPlaneAngle = ComputePlaneAngleDegrees(attachTransform.position);
            m_HasPreviousPlaneAngle = true;
        }

        protected override void OnSelectExited(SelectExitEventArgs args)
        {
            base.OnSelectExited(args);

            m_HasPreviousPlaneAngle = false;
        }

        void TrySnapToLimit(ref float angle, float limitValue, ref bool engagementLatch)
        {
            if (engagementLatch)
            {
                angle = limitValue;
                return;
            }

            var beforeSnap = angle;
            angle = limitValue;
            engagementLatch = true;

            if (!m_HapticOnLimitSnap || Mathf.Approximately(beforeSnap, limitValue))
                return;

            if (interactorsSelecting.Count > 0 &&
                interactorsSelecting[0] is XRBaseInputInteractor inputInteractor)
            {
                inputInteractor.SendHapticImpulse(m_LimitSnapHapticAmplitude, m_LimitSnapHapticDuration);
            }
        }

        void ConfigureRigidbodyForValve()
        {
            if (!TryGetComponent(out Rigidbody body))
                return;

            body.useGravity = false;
            ClearRigidbodyVelocitiesIfDynamic(body);
            body.isKinematic = true;
        }

        static void ClearRigidbodyVelocitiesIfDynamic(Rigidbody body)
        {
            if (body.isKinematic)
                return;

#if UNITY_2023_3_OR_NEWER
            body.linearVelocity = Vector3.zero;
#else
            body.velocity = Vector3.zero;
#endif
            body.angularVelocity = Vector3.zero;
        }

        float ComputePlaneAngleDegrees(Vector3 attachWorldPosition)
        {
            var pivot = m_WheelTransform.position;
            var worldOffset = attachWorldPosition - pivot;
            var planeNormal = GetRotationAxisWorld();
            var projected = Vector3.ProjectOnPlane(worldOffset, planeNormal);
            if (projected.sqrMagnitude < 1e-8f)
                return m_HasPreviousPlaneAngle ? m_PreviousPlaneAngle : 0.0f;

            projected.Normalize();
            var localFlat = m_WheelTransform.InverseTransformDirection(projected);

            switch (m_RotationAxis)
            {
                case ValveRotationAxis.X:
                    localFlat.x = 0.0f;
                    break;
                case ValveRotationAxis.Y:
                    localFlat.y = 0.0f;
                    break;
                case ValveRotationAxis.Z:
                    localFlat.z = 0.0f;
                    break;
            }

            return PlaneAngleFromLocalDirection(localFlat, m_RotationAxis);
        }

        Vector3 GetRotationAxisWorld()
        {
            switch (m_RotationAxis)
            {
                case ValveRotationAxis.X:
                    return m_WheelTransform.right;
                case ValveRotationAxis.Y:
                    return m_WheelTransform.up;
                default:
                    return m_WheelTransform.forward;
            }
        }

        static float PlaneAngleFromLocalDirection(Vector3 localFlat, ValveRotationAxis axis)
        {
            float componentA;
            float componentB;
            switch (axis)
            {
                case ValveRotationAxis.X:
                    componentA = localFlat.y;
                    componentB = localFlat.z;
                    break;
                case ValveRotationAxis.Y:
                    componentA = localFlat.x;
                    componentB = localFlat.z;
                    break;
                default:
                    componentA = localFlat.x;
                    componentB = localFlat.y;
                    break;
            }

            return Mathf.Atan2(componentB, componentA) * Mathf.Rad2Deg;
        }

        float ReadAngleFromWheelLocalEuler()
        {
            var euler = m_WheelTransform.localEulerAngles;
            switch (m_RotationAxis)
            {
                case ValveRotationAxis.X:
                    return NormalizeSignedAngle(euler.x);
                case ValveRotationAxis.Y:
                    return NormalizeSignedAngle(euler.y);
                default:
                    return NormalizeSignedAngle(euler.z);
            }
        }

        static float NormalizeSignedAngle(float eulerDegrees)
        {
            if (eulerDegrees > 180.0f)
                eulerDegrees -= 360.0f;

            return eulerDegrees;
        }

        void ApplyWheelLocalRotation()
        {
            var euler = m_WheelTransform.localEulerAngles;
            switch (m_RotationAxis)
            {
                case ValveRotationAxis.X:
                    m_WheelTransform.localRotation = Quaternion.Euler(m_CurrentAngle, euler.y, euler.z);
                    break;
                case ValveRotationAxis.Y:
                    m_WheelTransform.localRotation = Quaternion.Euler(euler.x, m_CurrentAngle, euler.z);
                    break;
                default:
                    m_WheelTransform.localRotation = Quaternion.Euler(euler.x, euler.y, m_CurrentAngle);
                    break;
            }
        }

        void OnValidate()
        {
            if (m_MinAngle > m_MaxAngle)
                m_MinAngle = m_MaxAngle;

            if (m_LimitSnapThresholdDegrees < 0.0f)
                m_LimitSnapThresholdDegrees = 0.0f;

            if (m_WheelTransform == null)
                m_WheelTransform = transform;
        }
    }
}
