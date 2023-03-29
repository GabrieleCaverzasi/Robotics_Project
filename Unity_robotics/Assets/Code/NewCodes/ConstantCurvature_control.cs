using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem; // da aggiungere come libreria da packege manager
using MathNet.Numerics; //NuGet, libreria da aggiungere scaricandola da gitHub
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;


public class ConstantCurvature_control : MonoBehaviour
{
    Chateter controls;

    Vector2 move;
    Vector2 rotate;
    float ML_bending, AP_bending, kappa, phi;
    public float tendon_offset;
    float L = 0f , unit = 0.1f;
    public int nodes; 
    float[] q = new float[4];

    public void InitializeCatheter()
    {
        kappa = 0;
        phi = 0;
        TransfMatrix();

    }

    private void Awake()
    {
        controls = new Chateter();

        /*
        controls.Gameplay.M_bending.performed += ctx => M_bending();
        controls.Gameplay.L_bending.performed += ctx => L_bending();

        controls.Gameplay.ML_bending.performed += ctx => ML_bending = ctx.ReadValue<float>();
        controls.Gameplay.ML_bending.canceled += ctx => ML_bending = 0f;
        
        controls.Gameplay.AP_bending.performed += ctx => AP_bending = ctx.ReadValue<float>();
        controls.Gameplay.AP_bending.canceled += ctx => AP_bending = 0f;
        */
        controls.Gameplay.Forward.performed += ctx => Forward();
        controls.Gameplay.Backward.performed += ctx => Backward();

        /*controls.Gameplay.Move.performed += ctx => move = ctx.ReadValue<Vector2>(); //we don't have to trigger a function, we only have to set the value of move to the value of the thumbstick
        controls.Gameplay.Move.canceled += ctx => move = Vector2.zero;

        controls.Gameplay.Rotate.performed += ctx => rotate = ctx.ReadValue<Vector2>(); //we don't have to trigger a function, we only have to set the value of move to the value of the thumbstick
        controls.Gameplay.Rotate.canceled += ctx => rotate = Vector2.zero;
        */
        controls.Gameplay.Grow.performed += ctx => Grow();
    }

    void M_bending()
    {
        float unit1 = 0.01f;
        q[2] = q[2] - unit1; //ogni volta che premiamo la freccia a dx vogliamo tirare il cavo ML
        q[0] = -q[2];
    }

    void L_bending()
    {
        float unit1 = 0.01f;
        q[0] = q[0] - unit1; //ogni volta che premiamo la freccia a sx vogliamo rilasciare il cavo ML
        q[2] = -q[0];
    }

    void Forward()
    {
        L = L + unit;
    }

    void Backward()
    {
        L = L - unit;
    }



    void Grow()
    {
        transform.localScale *= 1.1f;
    }

    void Update()
    {
        /*Vector2 m = new Vector2(move.x, move.y) * Time.deltaTime;
        transform.Translate(m, Space.World);

        Vector2 r = new Vector2(-rotate.y, rotate.x) * 100f * Time.deltaTime;
        transform.Rotate(r, Space.World);*/

        Kinematic_parameters();
        TransfMatrix();

        //Debug.Log("kappa value: " + kappa);
        //Debug.Log("phi value: " + phi);
    }

    void Kinematic_parameters()
    {
        float disp_ML_1 = (ML_bending * L / 2);
        float disp_AP_2 = (AP_bending * L / 2);
        float disp_ML_3 = -(ML_bending * L / 2);
        float disp_AP_4 = -(AP_bending * L / 2);

        float[] l = new float[] { L + q[0], L + q[1], L + q[2], L + q [3]};

        float l1 = l[0];
        float l2 = l[1];
        float l3 = l[2];
        float l4 = l[3];

        //Debug.Log("lenght 1: " + l1 + " length 2: " +l2);

        float k_num = (l1 - 3*l2 + l3 + l4) * Mathf.Sqrt((Mathf.Pow((l4-l2), 2) + Mathf.Pow((l3 - l1), 2)));
        float k_den = tendon_offset * (l1 + l2 + l3 + l4) * (l4 - l2);

        //Debug.Log("disp_ML: " + disp_ML + " disp_AP: " + disp_AP + " l1: " +l1 + " l2: " + l2 + " l3: " + l3 + " l4: " + l4 );

        if (k_den == 0)
        {
            float k_num_1 = (l2 - 3 * l1 + l3 + l4) * Mathf.Sqrt((Mathf.Pow((l4 - l2), 2) + Mathf.Pow((l3 - l1), 2)));
            float k_den_1 = tendon_offset * (l1 + l2 + l3 + l4) * (l3 - l1);
            if (k_den_1 == 0)
            {
                kappa = 0;
                phi = 0;  
            }
            else
            {
                kappa = k_num_1 / k_den_1;
            }
        }

        else
        {
            kappa = k_num / k_den; 
        }

        phi = Mathf.Atan2(l4 - l2, l3 - l1);

        if (k_num ==0 && k_den != 0)
        {
            kappa = 0;
            phi = 0;
        }

        kappa = kappa * 10;
    }

    void TransfMatrix()
    {
        Vector3[] points = new Vector3[nodes];
        for (int g = 0; g < nodes; g++)
        {
            points[g] = new Vector3();
        }

        double [] si = Generate.LinearSpaced(nodes, 0, L);
        for (int g = 0; g < nodes; g++)
        {
            //kappa = 3.32f;
            //phi = 1.57f;
            float s = (float)si[g];
            float theta = kappa * s;
            if (kappa < Mathf.Pow(3, -17))
            {
                points[g] = new Vector3(0.0f, 0f, s);
                GameObject.Find("node" + g).transform.localPosition = points[g];
            }
            else
            {
                Vector3 pi = new Vector3((float)((1 - Mathf.Cos(theta)) * Mathf.Cos(phi) / kappa),
                                        (float)((1 - Mathf.Cos(theta)) * Mathf.Sin(phi) / kappa),
                                        (float)(Mathf.Sin(theta) / kappa));

                GameObject.Find("node" + g).transform.localPosition = pi;


                Matrix <float> Rz = Matrix<float>.Build.DenseOfColumnArrays(
                                new [] {Mathf.Cos(phi), -Mathf.Sin(phi), 0f}, 
                                new [] {Mathf.Sin(phi), Mathf.Cos(phi), 0f}, 
                                new [] { 0f, 0f, 1f});

                Matrix<float> Ry = Matrix<float>.Build.DenseOfColumnArrays(
                                new [] {Mathf.Cos(theta), 0f, Mathf.Sin(theta)},
                                new [] {0f, 1f, 0f},
                                new [] {-Mathf.Sin(theta), 0f, Mathf.Cos(theta)});

                Matrix<float> Rz2 = Matrix<float>.Build.DenseOfColumnArrays(
                                new[] { Mathf.Cos(-phi), -Mathf.Sin(-phi), 0f },
                                new[] { Mathf.Sin(-phi), Mathf.Cos(-phi), 0f },
                                new[] { 0f, 0f, 1f });

                Matrix<float> Rot = Rz * Ry * Rz2;
                //Matrix<float> Rot = Rot * Rz2;
            }

            
        }
    }

    void OnEnable()
    {
        controls.Gameplay.Enable();
    }

    void OnDisable()
    {
        controls.Gameplay.Disable();
    }

}
