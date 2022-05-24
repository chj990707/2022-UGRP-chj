using CSML;
using System;
using System.Collections;
using System.IO.Ports;
using System.IO;
using UnityEngine;
using Valve.VR;

public class IMU_Sensor_Object : MonoBehaviour
{
    string path = @"C:\Users\JYP\Desktop\Calibration\Text.txt";
    public bool syncRotWithTracker = false;
    public bool use_LPF_gyro = false;
    public bool use_kalman_only = false;
    public bool use_kalman_with_LPF = true;

    SerialPort m_SerialPort = new SerialPort("COM6", 38400, Parity.None, 8, StopBits.One);
    string m_Data = null;
    float rotPrevTime = 0;
    float rotDeltaTime = 0;
    float posPrevTime = 0;
    float posDeltaTime = 0;
    int readLine = 0;

    Vector3 rot_LPF_x = new Vector3(0,0,0);
    Vector3 acc_LPF_x = new Vector3(0, 0, 0);
    float LPF_rot_alpha = 0.7f;
    float LPF_acc_alpha = 0.7f;

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

    Matrix pos_A = new Matrix(9,9);
    Matrix pos_H_acc = new Matrix(3, 9);
    Matrix pos_Q_acc = new Matrix(9, 9);
    Matrix pos_R_acc = new Matrix(3, 3);
    Matrix pos_K_acc = new Matrix(9, 9);
    Matrix pos_P_acc = new Matrix(9, 9);
    Matrix pos_P_p_acc = new Matrix(9, 9);

    Matrix pos_H_trk = new Matrix(3, 9);
    Matrix pos_Q_trk = new Matrix(9, 9);
    Matrix pos_R_trk = new Matrix(3, 3);
    Matrix pos_K_trk = new Matrix(9, 9);
    Matrix pos_P_trk = new Matrix(9, 9);
    Matrix pos_P_p_trk = new Matrix(9, 9);

    Matrix acc_z = new Matrix(3, 1);
    Matrix pos_x = new Matrix(9, 1);
    Matrix pos_x_p = new Matrix(9, 1);

    public GameObject tracker;
    Transform accel_gauge;
    Custom_Tracked_Object tracker_script;

    void Start()
    {
        accel_gauge = transform.Find("Cube");
        m_SerialPort.Open();
        StartCoroutine("OnIMUReceived");
        rotPrevTime = Time.realtimeSinceStartup;
        posPrevTime = Time.realtimeSinceStartup;
        tracker_script = tracker.GetComponent<Custom_Tracked_Object>();
        rot_H = Matrix.Identity(4);
        rot_Q = Matrix.Identity(4) * 10;
        rot_R = Matrix.Identity(4) * 0.1;
        rot_P = new Matrix(4,4);

        pos_H_acc = new Matrix(new double[,]{ { 0, 0, 0, 0, 0, 0, 1, 0, 0},
                                              { 0, 0, 0, 0, 0, 0, 0, 1, 0},
                                              { 0, 0, 0, 0, 0, 0, 0, 0, 1}});
        pos_Q_acc = Matrix.Identity(9) * 0.1;
        pos_R_acc = Matrix.Identity(3) * 0.1;
        pos_P_acc = new Matrix(9, 9);

        pos_H_trk = new Matrix(new double[,]{ { 1, 0, 0, 0, 0, 0, 0, 0, 0},
                                              { 0, 1, 0, 0, 0, 0, 0, 0, 0},
                                              { 0, 0, 1, 0, 0, 0, 0, 0, 0}});
        pos_Q_trk = Matrix.Identity(9) * 0.1;
        pos_R_trk = Matrix.Identity(3) * 0.1;
        pos_P_trk = new Matrix(9, 9);

        rot_LPF_x = transform.rotation.eulerAngles;
        rot_x = new Matrix(new double[,] { { transform.rotation.x }, { transform.rotation.y }, { transform.rotation.z }, { transform.rotation.w } });
        pos_x = new Matrix(new double[,] { { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 } });
    }

    IEnumerator OnIMUReceived()
    {
        while (true)
        {
            yield return new WaitUntil(() => { return m_SerialPort.BytesToRead > 0; });
            String m_Data;
            m_Data = m_SerialPort.ReadLine();
            //Debug.Log(m_Data);
            try
            {
                string[] datas = m_Data.Split('/');
                if (datas.Length != 6) continue;
                rotDeltaTime = Time.realtimeSinceStartup - rotPrevTime;
                posDeltaTime = Time.realtimeSinceStartup - posPrevTime;
                //Debug.Log("delta Time : " + rotDeltaTime);
                float gyro_pitch = float.Parse(datas[1]) * 250f / 32768f * rotDeltaTime;
                float gyro_roll = float.Parse(datas[0]) * 250f / 32768f * rotDeltaTime;
                float gyro_yaw = -float.Parse(datas[2]) * 250f / 32768f * rotDeltaTime; // vive와 imu의 축 일치시키기 위해 순서 변경.
                float accel_x = 1/1.00041f * (float.Parse(datas[3]) / 16384f - 0.00272f);
                float accel_y = 1/0.99970f * (float.Parse(datas[4]) / 16384f - 0.01545f);
                float accel_z = 1/1.01362f * (float.Parse(datas[5]) / 16384f - 0.00097f);


                Vector3 acc_LPF = LPF_accel_IMU(new Vector3(-accel_y, -accel_x, accel_z));

                //IMU의 가속도 센서와 자이로 센서로부터 현재 각도를 추정해야 함.
                //VR 트래커는 손에 들고 사용할 것을 가정하기 때문에 움직임에서 큰 가속도가 발생할 것으로 예상됨.
                //1. low-pass filter로 잡음을 제거한 자이로 센서만 사용하기
                //2. kalman fusion을 사용해 자이로와 가속도계 융합하기(단, 가속도 벡터를 normalize)


                Vector3 normalized_acc = Vector3.Normalize(acc_LPF);
                Vector3 acc_euler_without_roll = Quaternion.FromToRotation(normalized_acc, Vector3.up).eulerAngles;
                Vector3 acc_euler = new Vector3(acc_euler_without_roll.x, transform.rotation.eulerAngles.y, acc_euler_without_roll.z);
                Vector3 gyro_LPF = LPF_rotation_IMU(new Vector3(gyro_pitch, gyro_roll, gyro_yaw));
                
                //Debug.Log("Normalized Acceleration : " + normalized_acc);
                //Debug.Log("acceleration orientation without roll: " + acc_euler_without_roll);
                //Debug.Log("acceleration orientation: " + acc_euler);

                if (syncRotWithTracker)
                {
                    transform.rotation = tracker.transform.rotation;
                }
                else if (use_LPF_gyro)
                {
                    transform.rotation = transform.rotation * Quaternion.Euler(gyro_LPF);
                }
                else if (use_kalman_only)
                {
                    rot_A = new Matrix(new double[,]{ { 1,          - gyro_pitch / 2, -gyro_roll / 2, -gyro_yaw / 2},
                                                  { gyro_pitch / 2, 1,             gyro_yaw / 2, -gyro_roll / 2},
                                                  { gyro_roll / 2, -gyro_yaw / 2, 1,            gyro_pitch / 2},
                                                  { gyro_yaw / 2,  gyro_roll / 2,  -gyro_pitch / 2 , 1        } });
                    Quaternion acc_quaternion = Quaternion.Euler(acc_euler);

                    Quaternion kalman_rot = Kalman_rotation_IMU(rot_A, acc_quaternion);
                    transform.rotation = kalman_rot;
                }
                else if (use_kalman_with_LPF)
                {
                    rot_A = new Matrix(new double[,]{ { 1,          - gyro_LPF.x / 2, -gyro_LPF.y / 2, -gyro_LPF.z / 2},
                                                  { gyro_LPF.x / 2, 1,             gyro_LPF.z / 2, -gyro_LPF.y / 2},
                                                  { gyro_LPF.y / 2, -gyro_LPF.z / 2, 1,            gyro_LPF.x / 2},
                                                  { gyro_LPF.z / 2,  gyro_LPF.y / 2,  -gyro_LPF.x / 2 , 1        } });
                    Quaternion acc_quaternion = Quaternion.Euler(acc_euler);

                    Quaternion kalman_rot = Kalman_rotation_IMU(rot_A, acc_quaternion);
                    transform.rotation = kalman_rot;
                }
                else
                {
                    transform.rotation = transform.rotation * Quaternion.Euler(new Vector3(gyro_pitch, gyro_roll, gyro_yaw));
                }
                //Debug.Log("Current orientation : " + transform.rotation.eulerAngles);

                accel_gauge.localPosition = (acc_LPF + Quaternion.Inverse(transform.rotation) * new Vector3(0, -1, 0)) * 9.8f;

                //방향 맞는지 다시 확인 필요
                Vector3 accel = (transform.rotation * (acc_LPF) + new Vector3(0, -1, 0)) * 9.8f;

                Debug.Log("global 가속도(g 보상) : " + accel.ToString() + "m/s^2");
                Debug.Log("global 가속도 크기 : " + (acc_LPF.magnitude * 9.8f) + "m/s^2");


                pos_A = new Matrix(new double[,]{ { 1, 0, 0, posDeltaTime, 0, 0, 0.5f*posDeltaTime*posDeltaTime, 0, 0},
                                              { 0, 1, 0, 0, posDeltaTime, 0, 0, 0.5f*posDeltaTime*posDeltaTime, 0},
                                              { 0, 0, 1, 0, 0, posDeltaTime, 0, 0, 0.5f*posDeltaTime*posDeltaTime},
                                              { 0, 0, 0, 1, 0, 0, posDeltaTime, 0, 0},
                                              { 0, 0, 0, 0, 1, 0, 0, posDeltaTime, 0},
                                              { 0, 0, 0, 0, 0, 1, 0, 0, posDeltaTime},
                                              { 0, 0, 0, 0, 0, 0, 1, 0, 0},
                                              { 0, 0, 0, 0, 0, 0, 0, 1, 0},
                                              { 0, 0, 0, 0, 0, 0, 0, 0, 1}});
                
                Vector3 kalman_pos = Kalman_position_IMU(pos_A, accel);

                if (syncRotWithTracker)
                {
                    transform.position = tracker.transform.position;
                }
                else transform.position = kalman_pos;

                rotPrevTime = Time.realtimeSinceStartup;
                posPrevTime = Time.realtimeSinceStartup;
            }
            catch(Exception e)
            {
                continue;
            }
        }
    }

    void OnApplicationQuit()
    {
        m_SerialPort.Close();
    }

    Quaternion Kalman_rotation_IMU(Matrix A, Quaternion input)
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

        Quaternion kalman_rot = new Quaternion((float)rot_x[1, 1].Re, (float)rot_x[2, 1].Re, (float)rot_x[3, 1].Re, (float)rot_x[4, 1].Re);

        //반환
        return kalman_rot;
    }

    public Quaternion Change_rotation_TRK(Quaternion input)
    {
        transform.rotation = input;
        rot_x = new Matrix(new double[,] { { input.x }, { input.y }, { input.z }, { input.w } });
        rotPrevTime = Time.realtimeSinceStartup;
        return transform.rotation;
    }

    Vector3 LPF_rotation_IMU(Vector3 Input)
    {
        rot_LPF_x = LPF_acc_alpha * rot_LPF_x + (1 - LPF_acc_alpha) * Input;
        return rot_LPF_x;
    }
    Vector3 LPF_accel_IMU(Vector3 Input)
    {
        acc_LPF_x = LPF_rot_alpha * acc_LPF_x + (1 - LPF_rot_alpha) * Input;
        return acc_LPF_x;
    }

    Vector3 Kalman_position_IMU(Matrix A, Vector3 accel)
    {
        Matrix acc_z = new Matrix(new double[,] { {accel.x, accel.y, accel.z } }).Transpose();
        //예측값 계산
        //vel_x_p = A * vel_x + acc_z;
        //vel_P_p = A * vel_P * A.Transpose() + vel_Q;
        pos_x_p = A * pos_x;
        pos_P_p_acc = A * pos_P_acc * A.Transpose() + pos_Q_acc;

        //칼만 이득
        pos_K_acc = pos_P_p_acc * pos_H_acc.Transpose() * (pos_H_acc * pos_P_p_acc * pos_H_acc.Transpose() + pos_R_acc).Inverse();

        //Debug.Log("pos_H * pos_x_p : " + (pos_H_acc * pos_x_p).Transpose());

        //추정값
        pos_x = pos_x_p + pos_K_acc * (acc_z - pos_H_acc * pos_x_p);
        Debug.Log("pos_x from acc : "+ pos_x.Transpose());

        //오차 공분산
        pos_P_acc = pos_P_p_acc - pos_K_acc * pos_H_acc * pos_P_p_acc;

        //반환
        return new Vector3((float)pos_x[1, 1].Re, (float)pos_x[2, 1].Re, (float)pos_x[3, 1].Re);
    }

    public Vector3 Change_position_LPF_TRK(Vector3 input)
    {
        transform.position = input;
        posDeltaTime = Time.realtimeSinceStartup - posPrevTime;
        pos_x[4, 1].Re = (pos_x[1, 1].Re - input.x) / posDeltaTime * 0.7f + pos_x[4, 1].Re * 0.3f;
        pos_x[5, 1].Re = (pos_x[2, 1].Re - input.y) / posDeltaTime * 0.7f + pos_x[5, 1].Re * 0.3f;
        pos_x[6, 1].Re = (pos_x[3, 1].Re - input.z) / posDeltaTime * 0.7f + pos_x[6, 1].Re * 0.3f;
        pos_x[1, 1].Re = input.x;
        pos_x[2, 1].Re = input.y;
        pos_x[3, 1].Re = input.z;
        posPrevTime = Time.realtimeSinceStartup;
        Debug.Log("Changed position, pos_x : " + pos_x.Transpose());
        return transform.position;
    }
    public Vector3 Kalman_position_TRK(Vector3 position)
    {
        posDeltaTime = Time.realtimeSinceStartup - posPrevTime;

        if (posDeltaTime < 0.01f) return transform.position;

        Matrix A = new Matrix(new double[,]{ { 1, 0, 0, posDeltaTime, 0, 0, 0.5f*posDeltaTime*posDeltaTime, 0, 0},
                                              { 0, 1, 0, 0, posDeltaTime, 0, 0, 0.5f*posDeltaTime*posDeltaTime, 0},
                                              { 0, 0, 1, 0, 0, posDeltaTime, 0, 0, 0.5f*posDeltaTime*posDeltaTime},
                                              { 0, 0, 0, 1, 0, 0, posDeltaTime, 0, 0},
                                              { 0, 0, 0, 0, 1, 0, 0, posDeltaTime, 0},
                                              { 0, 0, 0, 0, 0, 1, 0, 0, posDeltaTime},
                                              { 0, 0, 0, 0, 0, 0, 1, 0, 0},
                                              { 0, 0, 0, 0, 0, 0, 0, 1, 0},
                                              { 0, 0, 0, 0, 0, 0, 0, 0, 1}});
        Matrix pos_z = new Matrix(new double[,] { { position.x, position.y, position.z } }).Transpose();
        //예측값 계산
        pos_x_p = A * pos_x;
        pos_P_p_trk = A * pos_P_trk * A.Transpose() + pos_Q_trk;

        //칼만 이득
        pos_K_trk = pos_P_p_trk * pos_H_trk.Transpose() * (pos_H_trk * pos_P_p_trk * pos_H_trk.Transpose() + pos_R_trk).Inverse();

        //추정값
        pos_x = pos_x_p + pos_K_trk * (pos_z - pos_H_trk * pos_x_p);
        Debug.Log("pos_x from tracker : " + pos_x.Transpose());

        //오차 공분산
        pos_P_trk = pos_P_p_trk - pos_K_trk * pos_H_trk * pos_P_p_trk;

        posPrevTime = Time.realtimeSinceStartup;
        transform.position = new Vector3((float)pos_x[1, 1].Re, (float)pos_x[2, 1].Re, (float)pos_x[3, 1].Re);
        //반환
        return new Vector3((float)pos_x[1, 1].Re, (float)pos_x[2, 1].Re, (float)pos_x[3, 1].Re);
    }

}