using UnityEngine;
using System;
using System.IO.Ports;
using CSML;
using Valve.VR;
using System.Collections;

public class IMU_Sensor_Object : MonoBehaviour
{
    public bool syncRotWithTracker = false;
    public bool use_LPF_gyro = false;

    SerialPort m_SerialPort = new SerialPort("COM6", 38400, Parity.None, 8, StopBits.One);
    string m_Data = null;
    float rotPrevTime = 0;
    float rotDeltaTime = 0;
    float posPrevTime = 0;
    float posDeltaTime = 0;
    int readLine = 0;

    Vector3 rot_LPF_x = new Vector3(0,0,0);
    float LPF_alpha = 0.7f;

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

    GameObject tracker;
    Custom_Tracked_Object tracker_script;

    void Start()
    {
        m_SerialPort.DataReceived += OnIMUReceived;
        m_SerialPort.Open();
        rotPrevTime = Time.realtimeSinceStartup;
        posPrevTime = Time.realtimeSinceStartup;
        tracker = GameObject.Find("Capsule");
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
    void OnIMUReceived(object sender, SerialDataReceivedEventArgs e)
    {
        while (((SerialPort)sender).BytesToRead > 0)
        {
            String m_Data = ((SerialPort)sender).ReadLine();
        }
        //Debug.Log(m_Data);
        string[] datas = m_Data.Split('/');
        rotDeltaTime = Time.realtimeSinceStartup - rotPrevTime;
        posDeltaTime = Time.realtimeSinceStartup - posPrevTime;
        Debug.Log("delta Time : " + rotDeltaTime);
        float gyro_pitch = float.Parse(datas[1]) * 250f / 32768f * rotDeltaTime;
        float gyro_roll = float.Parse(datas[0]) * 250f / 32768f * rotDeltaTime;
        float gyro_yaw = -float.Parse(datas[2]) * 250f / 32768f * rotDeltaTime; // vive�� imu�� �� ��ġ��Ű�� ���� ���� ����.
        float accel_x = float.Parse(datas[3]) / 16384f;
        float accel_y = float.Parse(datas[4]) / 16384f;
        float accel_z = float.Parse(datas[5]) / 16384f;
        //IMU�� ���ӵ� ������ ���̷� �����κ��� ���� ������ �����ؾ� ��.
        //VR Ʈ��Ŀ�� �տ� ��� ����� ���� �����ϱ� ������ �����ӿ��� ū ���ӵ��� �߻��� ������ �����.
        //1. low-pass filter�� ������ ������ ���̷� ������ ����ϱ�
        //2. kalman fusion�� ����� ���̷ο� ���ӵ��� �����ϱ�(��, ���ӵ� ���͸� normalize)

        if (syncRotWithTracker)
        {
            transform.rotation = tracker.transform.rotation;
        }
        else if (use_LPF_gyro)
        {
            transform.rotation = transform.rotation * Quaternion.Euler(LPF_rotation_IMU(new Vector3(gyro_pitch, gyro_roll, gyro_yaw)));
        }
        else 
        {
            rot_A = new Matrix(new double[,]{ { 1,          - gyro_pitch / 2, -gyro_roll / 2, -gyro_yaw / 2},
                                              { gyro_pitch / 2, 1,             gyro_yaw / 2, -gyro_roll / 2},
                                              { gyro_roll / 2, -gyro_yaw / 2, 1,            gyro_pitch / 2},
                                              { gyro_yaw / 2,  gyro_roll / 2,  -gyro_pitch / 2 , 1        } });
            Vector3 normalized_acc = Vector3.Normalize(new Vector3(-accel_y, -accel_x, accel_z));
            double acc_pitch = Math.Asin(normalized_acc.x);
            double acc_roll = Math.Asin(-normalized_acc.y / Math.Cos(acc_pitch));
            Quaternion acc_quaternion = Quaternion.Euler((float)acc_pitch, (float)acc_roll, transform.rotation.eulerAngles.z);

            Quaternion kalman_rot = Kalman_rotation_IMU(rot_A, acc_quaternion);
            transform.rotation = kalman_rot; 
        }
        //Debug.Log("Kalman rotation : " + kalman_rot.eulerAngles);
        //Debug.Log("rotation : " + rot.eulerAngles);
        //Debug.Log("Kalman Velocity : " + );

        //Debug.Log("left handed ���ӵ� : " + ((-new Vector3(-accel_y, -accel_x, accel_z))).ToString() + "g");
        //Debug.Log("global ���ӵ� : " + (transform.rotation * (-new Vector3(-accel_y, -accel_x, accel_z))).ToString() + "g"); // �� inverse�� �ƴѰ�?
        //Debug.Log("Global ���� IMU�� y�� : " + (transform.rotation * (new Vector3(0, 1, 0))).ToString());
        pos_A = new Matrix(new double[,] { { 1, 0, 0},
                                               { 0, 1, 0},
                                               { 0, 0, 1}});

        //���� �´��� �ٽ� Ȯ�� �ʿ�
        throw new NotImplementedException();
        Vector3 accel = (transform.rotation * (-new Vector3(-accel_y, -accel_x, accel_z)) + new Vector3(0, 1, 0)) * 9.8f;

        Debug.Log("global ���ӵ�(g ����) : " + accel.ToString() + "m/s^2");


        pos_A = new Matrix(new double[,]{ { 1, 0, 0, posDeltaTime, 0, 0, 0.5f*posDeltaTime*posDeltaTime, 0, 0},
                                          { 0, 1, 0, 0, posDeltaTime, 0, 0, 0.5f*posDeltaTime*posDeltaTime, 0},
                                          { 0, 0, 1, 0, 0, posDeltaTime, 0, 0, 0.5f*posDeltaTime*posDeltaTime},
                                          { 0, 0, 0, 1, 0, 0, posDeltaTime, 0, 0},
                                          { 0, 0, 0, 0, 1, 0, 0, posDeltaTime, 0},
                                          { 0, 0, 0, 0, 0, 1, 0, 0, posDeltaTime},
                                          { 0, 0, 0, 0, 0, 0, 1, 0, 0},
                                          { 0, 0, 0, 0, 0, 0, 0, 1, 0},
                                          { 0, 0, 0, 0, 0, 0, 0, 0, 1}});

        Vector3 kalman_vel = Kalman_position_IMU(pos_A, accel);

        if (syncRotWithTracker)
        {
            transform.position = tracker.transform.position;
        }
        else transform.position = transform.position + kalman_vel * rotDeltaTime;

        rotPrevTime = Time.realtimeSinceStartup;
        posPrevTime = Time.realtimeSinceStartup;
    }

    void OnApplicationQuit()
    {
        m_SerialPort.Close();
    }

    Quaternion Kalman_rotation_IMU(Matrix A, Quaternion input)
    {
        rot_z = new Matrix(new double[,] { { input.x },{ input.y },{ input.z },{ input.w } });
        //������ ���
        rot_x_p = A * rot_x;
        rot_P_p = A * rot_P * A.Transpose() + rot_Q;
        //Į�� �̵�
        rot_K = rot_P_p * rot_H.Transpose() * (rot_H * rot_P_p * rot_H.Transpose() + rot_R).Inverse();

        //������
        rot_x = rot_x_p + rot_K * (rot_z - rot_H * rot_x_p);

        //���� ���л�
        rot_P = rot_P_p - rot_K * rot_H * rot_P_p;

        Quaternion kalman_rot = new Quaternion((float)rot_x[1, 1].Re, (float)rot_x[2, 1].Re, (float)rot_x[3, 1].Re, (float)rot_x[4, 1].Re);

        //��ȯ
        return kalman_rot;
    }

    Vector3 LPF_rotation_IMU(Vector3 Input)
    {
        rot_LPF_x = LPF_alpha * rot_LPF_x + (1 - LPF_alpha) * Input;
        return rot_LPF_x;
    }

    Vector3 Kalman_position_IMU(Matrix A, Vector3 accel)
    {
        Matrix acc_z = new Matrix(new double[,] { { 0, 0, 0, 0, 0, 0, accel.x, accel.y, accel.z } }).Transpose();
        //������ ���
        //vel_x_p = A * vel_x + acc_z;
        //vel_P_p = A * vel_P * A.Transpose() + vel_Q;
        pos_x_p = A * pos_x;
        pos_P_p_acc = A * pos_P_acc * A.Transpose() + pos_Q_acc;

        //Į�� �̵�
        pos_K_acc = pos_P_p_acc * pos_H_acc.Transpose() * (pos_H_acc * pos_P_p_acc * pos_H_acc.Transpose() + pos_R_acc).Inverse();

        //������
        pos_x = pos_x_p + pos_K_acc * (this.acc_z - pos_H_acc * pos_x_p);

        //���� ���л�
        pos_P_acc = pos_P_p_acc - pos_K_acc * pos_H_acc * pos_P_p_acc;

        //��ȯ
        return new Vector3((float)pos_x[1, 1].Re, (float)pos_x[2, 1].Re, (float)pos_x[3, 1].Re);
    }
    public Vector3 Kalman_position_TRK(Matrix A, Vector3 position)
    {
        Matrix acc_z = new Matrix(new double[,] { { position.x, position.y, position.z, 0, 0, 0, 0, 0, 0 } }).Transpose();
        //������ ���
        pos_x_p = A * pos_x;
        pos_P_p_trk = A * pos_P_trk * A.Transpose() + pos_Q_trk;

        //Į�� �̵�
        pos_K_trk = pos_P_p_trk * pos_H_trk.Transpose() * (pos_H_trk * pos_P_p_trk * pos_H_trk.Transpose() + pos_R_trk).Inverse();

        //������
        pos_x = pos_x_p + pos_K_trk * (this.acc_z - pos_H_trk * pos_x_p);

        //���� ���л�
        pos_P_trk = pos_P_p_trk - pos_K_trk * pos_H_trk * pos_P_p_trk;

        //��ȯ
        return new Vector3((float)pos_x[1, 1].Re, (float)pos_x[2, 1].Re, (float)pos_x[3, 1].Re);
    }
}