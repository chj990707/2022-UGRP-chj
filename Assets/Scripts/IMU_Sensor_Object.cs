using UnityEngine;
using System;
using System.IO.Ports;
using CSML;
using Valve.VR;

public class IMU_Sensor_Object : MonoBehaviour
{
    SerialPort m_SerialPort = new SerialPort("COM6", 38400, Parity.None, 8, StopBits.One);
    string m_Data = null;
    float imuPrevTime = 0;
    float imuDeltaTime = 0;
    int readLine = 0;
    
    Matrix rot_A = new Matrix(4, 4);
    Matrix rot_H = new Matrix(4, 4);
    Matrix rot_K = new Matrix(4, 4);
    Matrix rot_P = new Matrix(4, 4);
    Matrix rot_P_p = new Matrix(4, 4);
    Matrix rot_Q = new Matrix(4, 4);
    Matrix rot_R = new Matrix(4, 4);

    Matrix rot_z = new Matrix(4, 1);
    Matrix rot_x = new Matrix(4, 1);
    Matrix rot_x_p = new Matrix(4, 1);

    Matrix vel_A = new Matrix(3,3);
    Matrix vel_H = new Matrix(3,3);
    Matrix vel_K = new Matrix(3,3);
    Matrix vel_P = new Matrix(3,3);
    Matrix vel_P_p = new Matrix(3,3);
    Matrix vel_Q = new Matrix(3,3);
    Matrix vel_R = new Matrix(3,3);

    Matrix vel_z = new Matrix(3, 1);
    Matrix vel_x = new Matrix(3, 1);
    Matrix vel_x_p = new Matrix(3, 1);

    GameObject tracker;
    Custom_Tracked_Object tracker_script;

    void Start()
    {
        m_SerialPort.Open();
        imuPrevTime = Time.realtimeSinceStartup;
        tracker = GameObject.Find("Capsule");
        tracker_script = tracker.GetComponent<Custom_Tracked_Object>();
        rot_H = Matrix.Identity(4);
        rot_Q = Matrix.Identity(4) * 0.1;
        rot_R = Matrix.Identity(4) * 0.1;
        rot_P = new Matrix(4,4);

        vel_H = Matrix.Identity(3);
        vel_Q = Matrix.Identity(3) * 0.1;
        vel_R = Matrix.Identity(3) * 0.1;
        vel_P = new Matrix(3, 3);

        rot_x = new Matrix(new double[,] { { transform.rotation.x }, { transform.rotation.y }, { transform.rotation.z }, { transform.rotation.w } });
        vel_x = new Matrix(new double[,] { { 0 },{ 0 },{ 0 } });
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
            Debug.Log(m_Data);
            string[] datas = m_Data.Split('/');
            imuDeltaTime = Time.realtimeSinceStartup - imuPrevTime;
            Debug.Log("delta Time : " + imuDeltaTime);
            float gyro_x = float.Parse(datas[0]) * 250f / 32768f * imuDeltaTime;
            float gyro_y = float.Parse(datas[1]) * 250 / 32768f * imuDeltaTime;
            float gyro_z = float.Parse(datas[2]) * 250f / 32768f * imuDeltaTime;
            float accel_x = float.Parse(datas[3]) / 16384f;
            float accel_y = float.Parse(datas[4]) / 16384f;
            float accel_z = float.Parse(datas[5]) / 16384f;
            Vector3 accel = new Vector3(accel_x, accel_y, accel_z);

            rot_A = new Matrix(new double[,]{ { 1,          - gyro_x / 2, -gyro_y / 2, -gyro_z / 2},
                                              { gyro_x / 2, 1,             gyro_z / 2, -gyro_y / 2},
                                              { gyro_y / 2, -gyro_z / 2, 1,            gyro_x / 2},
                                              { gyro_z / 2,  gyro_y / 2,  -gyro_x / 2 , 1        } });

            vel_A = new Matrix(new double[,] { { 1, 0, 0},
                                               { 0, 1, 0},
                                               { 0, 0, 1}});

            //Debug.Log("Kalman rotation : " + rotation_Kalman(rot_A, tracker.transform.rotation));
            Debug.Log("Kalman Velocity : " + velocity_Kalman(vel_A, tracker_script.viveVelocity, accel, imuDeltaTime));
            Quaternion rot = Quaternion.Euler(transform.rotation.eulerAngles
                + new Vector3(gyro_x, gyro_y, gyro_z));
            Debug.Log("rotation : " + rot.eulerAngles);
            transform.rotation = rot;
            Debug.Log("Acceleration : "+ (transform.rotation * (new Vector3(accel_x, accel_y, accel_z))).ToString() + "g");
            imuPrevTime = Time.realtimeSinceStartup;
        }
    }

    void OnApplicationQuit()
    {
        m_SerialPort.Close();
    }

    Quaternion rotation_Kalman(Matrix A, Quaternion input)
    {
        rot_z = new Matrix(new double[,] { { input.x },{ input.y },{ input.z },{ input.w } });
        //예측값 계산
        rot_x_p = A * rot_x;
        rot_P_p = A * rot_P * A.Transpose() + rot_Q;

        //칼만 이득
        rot_K = rot_P_p * rot_H.Transpose() * (rot_H * rot_P_p * rot_H.Transpose() + rot_R).Inverse();

        //추정값
        rot_x = rot_x_p + rot_K * (rot_z - rot_H * rot_x_p);

        //오차 공분산
        rot_P = rot_P_p - rot_K * rot_H * rot_P_p;

        //반환
        return new Quaternion((float)rot_x[1, 1].Re, (float)rot_x[2, 1].Re, (float)rot_x[3, 1].Re, (float)rot_x[4, 1].Re);
    }

    Vector3 velocity_Kalman(Matrix A, Vector3 velocity, Vector3 accel, float T)
    {
        vel_z = new Matrix(new double[,] { { velocity.x }, { velocity.y }, { velocity.z } });
        Matrix acc_z = new Matrix(new double[,] { { accel.x * T }, { accel.y * T }, { accel.z * T } });
        //예측값 계산
        //vel_x_p = A * vel_x + acc_z;
        //vel_P_p = A * vel_P * A.Transpose() + vel_Q;
        //현재 모델에서 A가 Identity이기 때문에 생략함
        vel_x_p = vel_x + acc_z;
        vel_P_p = vel_P + vel_Q;

        //칼만 이득
        vel_K = vel_P_p * vel_H.Transpose() * (vel_H * vel_P_p * vel_H.Transpose() + vel_R).Inverse();

        //추정값
        vel_x = vel_x_p + vel_K * (vel_z - vel_H * vel_x_p);

        //오차 공분산
        vel_P = vel_P_p - vel_K * vel_H * vel_P_p;

        //반환
        return new Vector3((float)vel_x[1, 1].Re, (float)vel_x[2, 1].Re, (float)vel_x[3, 1].Re);
    }
}