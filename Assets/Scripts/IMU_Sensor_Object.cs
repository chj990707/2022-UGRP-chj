using UnityEngine;
using System;
using System.IO.Ports;

public class IMU_Sensor_Object : MonoBehaviour
{
    SerialPort m_SerialPort = new SerialPort("COM6", 38400, Parity.None, 8, StopBits.One);
    string m_Data = null;
    float imuPrevTime = 0;
    float imuDeltaTime = 0;
    int readLine = 0;

    void Start()
    {
        m_SerialPort.Open();
        imuPrevTime = Time.realtimeSinceStartup;
    }

    private void FixedUpdate()
    {
        readLine = 0;
        if (m_SerialPort.IsOpen)
        {
            m_SerialPort.ReadTimeout = 10;
            try
            {
                while (true)
                {
                    m_Data = m_SerialPort.ReadLine();
                    readLine++;
                }
            }
            catch(Exception e) { }
            if(readLine == 0) { return; }
            string[] datas = m_Data.Split('/');
            imuDeltaTime = Time.realtimeSinceStartup - imuPrevTime;
            //Debug.Log("delta Time : " + DeltaTime);
            Quaternion rot = Quaternion.Euler(transform.rotation.eulerAngles
                + new Vector3(float.Parse(datas[0]) * imuDeltaTime, float.Parse(datas[1]) * imuDeltaTime, float.Parse(datas[2]) * imuDeltaTime));
            Debug.Log("rotation : " + rot.eulerAngles);
            transform.rotation = rot;
            Debug.Log("Acceleration : "+ (transform.rotation * ((new Vector3(float.Parse(datas[3]), float.Parse(datas[4]), float.Parse(datas[5]))))+new Vector3(0,1,0)).ToString() + "g");
            imuPrevTime = Time.realtimeSinceStartup;
        }
    }

    void OnApplicationQuit()
    {
        m_SerialPort.Close();
    }
}