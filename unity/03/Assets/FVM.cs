using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;


public class FVM : MonoBehaviour
{
    float dt = 0.003f;
    float mass = 1;

    // https://web.cse.ohio-state.edu/~wang.3602/Wang-2016-DME/Wang-2016-DME.pdf 
    // 论文里面带有ppt
    // where s0 and s1 are the two elastic moduli controlling the resistance to deformation, also known as the Lame parameters
    float stiffness_0 = 10000;
    float stiffness_1 = 5000;

    float damp = 0.999f;

    //tetrahedron 四面体信息
    int[] Tet;
    int tet_number;         //The number of tetrahedra

    Vector3[] Force;
    Vector3[] V;
    Vector3[] X;
    int number;             //The number of vertices

    Matrix4x4[] inv_Dm;

    //For Laplacian smoothing.
    Vector3[] V_sum;
    int[] V_num;

    SVD svd = new SVD();

    Vector3 m_Grivarty = Vector3.down * 9.8f;
    // Start is called before the first frame update
    void Start()
    {
        // FILO IO: Read the house model from files.
        // The model is from Jonathan Schewchuk's Stellar lib.
        {
            string fileContent = File.ReadAllText("Assets/house2.ele");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

            tet_number = int.Parse(Strings[0]);
            Tet = new int[tet_number * 4]; //每一个四面体有四个顶点，依次存储 -> 顶点

            for (int tet = 0; tet < tet_number; tet++)
            {
                Tet[tet * 4 + 0] = int.Parse(Strings[tet * 5 + 4]) - 1;
                Tet[tet * 4 + 1] = int.Parse(Strings[tet * 5 + 5]) - 1;
                Tet[tet * 4 + 2] = int.Parse(Strings[tet * 5 + 6]) - 1;
                Tet[tet * 4 + 3] = int.Parse(Strings[tet * 5 + 7]) - 1;
            }
        }
        {
            string fileContent = File.ReadAllText("Assets/house2.node");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            number = int.Parse(Strings[0]);
            X = new Vector3[number];
            for (int i = 0; i < number; i++)
            {
                X[i].x = float.Parse(Strings[i * 5 + 5]) * 0.4f;
                X[i].y = float.Parse(Strings[i * 5 + 6]) * 0.4f;
                X[i].z = float.Parse(Strings[i * 5 + 7]) * 0.4f;
            }
            //Centralize the model.
            Vector3 center = Vector3.zero;
            for (int i = 0; i < number; i++)
            {
                center += X[i];
            }

            center = center / number;
            for (int i = 0; i < number; i++)
            {
                X[i] -= center;
                float temp = X[i].y;
                X[i].y = X[i].z;
                X[i].z = temp;
            }
        }

        // 测试
        // tet_number=1;
        // Tet = new int[tet_number*4];
        // Tet[0]=0;
        // Tet[1]=1;
        // Tet[2]=2;
        // Tet[3]=3;
        //
        // number=4;
        // X = new Vector3[number];
        // V = new Vector3[number];
        // Force = new Vector3[number];
        // X[0]= new Vector3(0, 0, 0);
        // X[1]= new Vector3(1, 0, 0);
        // X[2]= new Vector3(0, 1, 0);
        // X[3]= new Vector3(0, 0, 1);


        //Create triangle mesh.
        //总数等于四面体的格式 
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            //一个四面体有四个三角形 
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }

        //四面体个数4 *  每个三角形顶点数 3 = 12
        int[] triangles = new int[tet_number * 12];
        for (int t = 0; t < tet_number * 4; t++)
        {
            triangles[t * 3 + 0] = t * 3 + 0;
            triangles[t * 3 + 1] = t * 3 + 1;
            triangles[t * 3 + 2] = t * 3 + 2;
            //由一堆四边形组成的房子
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.triangles = triangles; //onl index
        mesh.RecalculateNormals();


        V = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        //TODO: Need to allocate and assign inv_Dm

        //构造四面体的受力情况
        //reference state tet
        inv_Dm = new Matrix4x4[tet_number];
        for (int tet = 0; tet < tet_number; tet++)
        {

            var Dm = Build_Edge_Matrix(tet);
            //有限元这里选择四面体，实际也是一种优化吧（因为比三角形少很多）


            Matrix4x4 invDm = Dm.inverse;
            inv_Dm[tet] = invDm;
        }

    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
        //TODO: Need to build edge matrix here.

        Vector3 V0 = X[Tet[tet * 4 + 0]];
        Vector3 V1 = X[Tet[tet * 4 + 1]];
        Vector3 V2 = X[Tet[tet * 4 + 2]];
        Vector3 V3 = X[Tet[tet * 4 + 3]];

        //每一个四面体有四个顶点
        Vector4 X10 = V1 - V0;
        Vector4 X20 = V2 - V0;
        Vector4 X30 = V3 - V0;

        Matrix4x4 Dm = new Matrix4x4(X10, X20, X30, Vector4.zero);
        Dm[3, 3] = 1.0f;

        return Dm;
    }

    Matrix4x4 float_muti_Matrix(float a, Matrix4x4 mat)
    {
        Matrix4x4 ret = new Matrix4x4()
        {
            m00 = mat.m00 * a,
            m01 = mat.m01 * a,
            m02 = mat.m02 * a,
            m03 = mat.m03 * a,

            m10 = mat.m10 * a,
            m11 = mat.m11 * a,
            m12 = mat.m12 * a,
            m13 = mat.m13 * a,

            m20 = mat.m20 * a,
            m21 = mat.m21 * a,
            m22 = mat.m22 * a,
            m23 = mat.m23 * a,


            m30 = mat.m30 * a,
            m31 = mat.m31 * a,
            m32 = mat.m32 * a,
            //
            m33 = 1,
        };

        return ret;
    }


    Matrix4x4 Matrix_add_Matrix(Matrix4x4 lMatrix, Matrix4x4 rMatrix)
    {
        return new Matrix4x4()
        {
            m00 = lMatrix.m00 + rMatrix.m00,
            m01 = lMatrix.m01 + rMatrix.m01,
            m02 = lMatrix.m02 + rMatrix.m02,
            m03 = lMatrix.m03 + rMatrix.m03,

            m10 = lMatrix.m10 + rMatrix.m10,
            m11 = lMatrix.m11 + rMatrix.m11,
            m12 = lMatrix.m12 + rMatrix.m12,
            m13 = lMatrix.m13 + rMatrix.m13,

            m20 = lMatrix.m20 + rMatrix.m20,
            m21 = lMatrix.m21 + rMatrix.m21,
            m22 = lMatrix.m22 + rMatrix.m22,
            m23 = lMatrix.m23 + rMatrix.m23,

            m30 = lMatrix.m30 + rMatrix.m30,
            m31 = lMatrix.m31 + rMatrix.m31,
            m32 = lMatrix.m32 + rMatrix.m32,
            //
            m33 = 1,

        };
    }


    void _Update()
    {
        // Jump up.
        if (Input.GetKeyDown(KeyCode.Space))
        {
            //模拟大小步长的效果
            for (int i = 0; i < number; i++)
                V[i].y += 0.2f;
        }

        for (int i = 0; i < number; i++)
        {
            //TODO: Add gravity to Force.
            //质量是1 但是也要意思一下 
            //注意这里是赋值，而不是刚开始就设置
            Force[i] = m_Grivarty * mass;
        }

        for (int tet = 0; tet < tet_number; tet++)
        {
            //TODO: Deformation Gradient
            //很简单啊，形变梯度，直接计算即可

            int index_0 = Tet[tet * 4 + 0];
            int index_1 = Tet[tet * 4 + 1];
            int index_2 = Tet[tet * 4 + 2];
            int index_3 = Tet[tet * 4 + 3];


            var dm = Build_Edge_Matrix(tet);

            //求出了形变量 deformation gradient （包含了旋转和拉伸信息，旋转不会产生力，所以过滤掉）
            Matrix4x4 F = dm * inv_Dm[tet];
            //TODO: Green Strain
            //减去单位矩阵
            Matrix4x4 tmp = F.transpose * F;
            tmp.m00 -= 1.0f;
            tmp.m11 -= 1.0f;
            tmp.m22 -= 1.0f;


            //没有重载浮点数乘以矩阵
            Matrix4x4 GreenStrain = float_muti_Matrix(0.5f, tmp);

            //TODO: Second PK Stress
            float GreenTrace = GreenStrain.m00 + GreenStrain.m11 + GreenStrain.m22;


            Matrix4x4 two_miu_G = float_muti_Matrix(2.0f * stiffness_1, GreenStrain);
            Matrix4x4 lamda_trace_I = float_muti_Matrix(stiffness_0 * GreenTrace, Matrix4x4.identity);

            Matrix4x4 S = Matrix_add_Matrix(two_miu_G, lamda_trace_I);
            Matrix4x4 P = F * S;


            //TODO: Elastic Force
            float determinantD = inv_Dm[tet].determinant;
            var tmpMatrix = P * (inv_Dm[tet].transpose);
            Matrix4x4 forceMatrix = float_muti_Matrix(-1.0f / (6 * determinantD), tmpMatrix);

            Vector3 force_1 = forceMatrix.GetColumn(0);
            Vector3 force_2 = forceMatrix.GetColumn(1);
            Vector3 force_3 = forceMatrix.GetColumn(2);

            Vector3 force_0 = -(force_1 + force_2 + force_3);

            Force[index_0] += force_0;
            Force[index_1] += force_1;
            Force[index_2] += force_2;
            Force[index_3] += force_3;

        }

        for (int i = 0; i < number; i++)
        {
            //TODO: Update X and V here.
            Vector3 acceleration = Force[i] / mass;
            Vector3 delta_V = acceleration * dt;
            //为保持稳定，先更新是速度

            V[i] += delta_V;
            X[i] += V[i] * dt;


            //TODO: (Particle) collision with floor.

            //这里偷懒了下 从shape matching  那里copy了下

            Vector3 outplane_point = X[i] - new Vector3(0, -3f, 0);
            Vector3 Normal = new Vector3(0, 1, 0);

            //法线方向上的投影
            float signDistance = Vector3.Dot(Normal, outplane_point);
            if (signDistance < 0)
            {
                Vector3 vi = V[i];

                float vi_normal_length = Vector3.Dot(vi, Normal);
                if (vi_normal_length < 0)
                {
                    Vector3 vi_normal = vi_normal_length * Normal;
                    Vector3 vi_tangent = vi - vi_normal;

                    vi_normal = -vi_normal * 0.95f;
                    float a = Mathf.Max(1 - (0.90f * (1 + 0.98f) * vi_normal.magnitude / vi_tangent.magnitude), 0);
                    vi_tangent *= a;

                    //各自模拟各自的速度
                    V[i] = vi_tangent + vi_normal;
                }
            }
        }
    }



    // Update is called once per frame
    void Update()
    {
        for (int l = 0; l < 10; l++)
            _Update();

        // Dump the vertex array for rendering.
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
    }
}
