using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Valve.VR;

namespace Valve.VR
{
    public class Custom_Tracked_Object : MonoBehaviour
    {
        public GameObject fusion;
        IMU_Sensor_Object fusion_script;
        Vector3 vivePrevPos;
        public Vector3 viveVelocity { get; private set; }
        float vivePrevTime;
        float viveDeltaTime;
        public enum EIndex
        {
            None = -1,
            Hmd = (int)OpenVR.k_unTrackedDeviceIndex_Hmd,
            Device1,
            Device2,
            Device3,
            Device4,
            Device5,
            Device6,
            Device7,
            Device8,
            Device9,
            Device10,
            Device11,
            Device12,
            Device13,
            Device14,
            Device15,
            Device16
        }

        public EIndex index;

        public bool isValid { get; private set; }
        void Start()
        {
            fusion_script = fusion.GetComponent<IMU_Sensor_Object>();
        }
            private void OnNewPoses(TrackedDevicePose_t[] poses)
        {
            if (index == EIndex.None)
                return;

            var i = (int)index;

            isValid = false;
            if (poses.Length <= i)
                return;

            if (!poses[i].bDeviceIsConnected)
                return;

            if (!poses[i].bPoseIsValid)
                return;

            isValid = true;

            var pose = new SteamVR_Utils.RigidTransform(poses[i].mDeviceToAbsoluteTracking);

            transform.localPosition = pose.pos;
            transform.localRotation = pose.rot;


            //Debug.Log("Event position : " + transform.localPosition.ToString());
            //Debug.Log("detaTime : " + deltaTime);
        }

        SteamVR_Events.Action newPosesAction;

        Custom_Tracked_Object()
        {
            newPosesAction = SteamVR_Events.NewPosesAction(OnNewPoses);
        }

        private void Awake()
        {
            OnEnable();
        }

        void OnEnable()
        {
            var render = SteamVR_Render.instance;
            if (render == null)
            {
                enabled = false;
                return;
            }
            vivePrevTime = Time.realtimeSinceStartup;
            viveDeltaTime = 100;
            viveVelocity = Vector3.zero;
            vivePrevPos = Vector3.zero;
            newPosesAction.enabled = true;
        }

        void OnDisable()
        {
            newPosesAction.enabled = false;
            isValid = false;
        }

        private void FixedUpdate()
        {
            //viveDeltaTime = Time.realtimeSinceStartup - vivePrevTime;
            //viveVelocity = (transform.localPosition - vivePrevPos) / viveDeltaTime;
            //vivePrevPos = transform.localPosition;
            //vivePrevTime = Time.realtimeSinceStartup;
            //Debug.Log("Velocity : " + viveVelocity);
        }

        public void SetDeviceIndex(int index)
        {
            if (System.Enum.IsDefined(typeof(EIndex), index))
                this.index = (EIndex)index;
        }
    }
}