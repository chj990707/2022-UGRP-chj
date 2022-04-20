using UnityEngine;
using System;
using System.IO.Ports;

public class IMU_Sensor_Object : MonoBehaviour
{
    SerialPort m_SerialPort = new SerialPort("COM6", 9600, Parity.None, 8, StopBits.One);
    string m_Data = null;
    float GyroPrevTime = 0;
    float GyroDeltaTime = 0;
    float AccPrevTime = 0;
    float AccDeltaTime = 0;

    void Start()
    {
        m_SerialPort.Open();
        GyroPrevTime = Time.realtimeSinceStartup;
        AccPrevTime = Time.realtimeSinceStartup;
    }

    private void FixedUpdate()
    {
        if (m_SerialPort.IsOpen)
        {
            try
            {
                m_Data = m_SerialPort.ReadLine();
                string[] datas = m_Data.Split('/');
                if (datas[0].Equals("GYRO"))
                {
                    GyroDeltaTime = Time.realtimeSinceStartup - GyroPrevTime;
                    Quaternion rot = Quaternion.Euler(transform.rotation.eulerAngles
                        + new Vector3(float.Parse(datas[1]) * GyroDeltaTime, float.Parse(datas[2]) * GyroDeltaTime, float.Parse(datas[3]) * GyroDeltaTime));
                    //Debug.Log("rotation : " + rot.eulerAngles);
                    transform.rotation = rot;
                    GyroPrevTime = Time.realtimeSinceStartup;
                }
                if (datas[0].Equals("ACC"))
                {
                    AccDeltaTime = Time.realtimeSinceStartup - AccPrevTime;
                    Debug.Log("Acceleration : "+ (transform.rotation * ((new Vector3(float.Parse(datas[1]), float.Parse(datas[2]), float.Parse(datas[3]))))+new Vector3(0,1,0)).ToString() + "g");
                    AccPrevTime = Time.realtimeSinceStartup;
                }
            }
            catch(Exception e) { }
            m_SerialPort.ReadTimeout = 20;
        }
    }

    void OnApplicationQuit()
    {
        m_SerialPort.Close();
    }
}