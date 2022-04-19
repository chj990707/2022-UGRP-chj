using UnityEngine;
using System;
using System.IO.Ports;

public class IMU_Sensor_Object : MonoBehaviour
{
    SerialPort m_SerialPort = new SerialPort("COM6", 9600, Parity.None, 8, StopBits.One);
    string m_Data = null;

    void Start()
    {
        m_SerialPort.Open();
    }

    private void Update()
    {
        try
        {
            if (m_SerialPort.IsOpen)
            {
                m_Data = m_SerialPort.ReadLine();
                string[] datas = m_Data.Split('/');
                if (datas[0].Equals("GYRO")){
                    Quaternion rot = Quaternion.Euler(transform.rotation.eulerAngles
                        + new Vector3(float.Parse(datas[1]) * Time.deltaTime, float.Parse(datas[2]) * Time.deltaTime, float.Parse(datas[3]) * Time.deltaTime));
                    Debug.Log("rotation : " + rot.eulerAngles);
                    transform.rotation = rot;
                }
                m_SerialPort.ReadTimeout = 5;

            }
        }

        catch (Exception e)
        {
        }
    }

    void OnApplicationQuit()
    {
        m_SerialPort.Close();
    }
}