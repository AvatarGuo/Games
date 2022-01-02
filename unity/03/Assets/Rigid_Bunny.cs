using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices;
using UnityEngine.Assertions;
using System.Collections.Generic;

// 两个比较好的开源资料
// https://www.toptal.com/game/video-game-physics-part-i-an-introduction-to-rigid-body-dynamics
// https://graphics.pixar.com/pbm2001/

public class Rigid_Bunny : MonoBehaviour
{
    bool launched = false;
    const float dt = 0.015f;						//可以用帧率来替换delta time . 如 30帧/s = 1s/30 = 0.033s =  33毫秒
    private Vector3 G = Vector3.down * 9.8f;							//重力的G ，表示 9.8N / kg

    public Vector3 v = new Vector3(0, 0, 0);    // velocity
    public Vector3 w = new Vector3(0, 0, 0);    // angular velocity

    float mass;                                 // mass		//torque -力矩

    private MeshFilter meshfilter;
    private Mesh mesh;
    private Vector3[] vertics;

    Matrix4x4 I_ref;                            // reference inertia(惯性) // g = 9.8N / KG 参照状态叫refrence inertia 来表示 表示

    float linear_decay = 0.999f;                // for velocity decay
    float angular_decay = 0.98f;                //模拟空气阻力
    float restitution = 0.98f;					// for collision
    float restitution_tangent = 0.98f;           //切线方向滑动摩擦

    void Start()
    {
        CalculateInertiaTensorAndMass();
    }

    void CalculateInertiaTensorAndMass()
    {
        meshfilter = this.gameObject.GetComponent<MeshFilter>();
        Matrix4x4 _I_ref = Matrix4x4.identity; // ref inertia tensor

        int m = 1;
        if (meshfilter != null)
        {
            mesh = meshfilter.mesh;
            vertics = mesh.vertices;

            for (int i = 0; i < vertics.Length; i++)
            {
                mass += m;

                var vertic = vertics[i];

                //Transpose * self = dot product = sqrMagnitude
                var sqrLenth = vertic.sqrMagnitude;

                //先计算对角线
                _I_ref.m00 += sqrLenth * 1 * m;
                _I_ref.m11 += sqrLenth * 1 * m;
                _I_ref.m22 += sqrLenth * 1 * m;

                //注意转置的先后顺序，如前面是数字后面是矩阵
                _I_ref.m00 -= vertic.x * vertic.x * m;
                _I_ref.m01 -= vertic.x * vertic.y * m;
                _I_ref.m02 -= vertic.x * vertic.z * m;

                //第二行
                _I_ref.m10 -= vertic.x * vertic.y * m;
                _I_ref.m11 -= vertic.y * vertic.y * m;
                _I_ref.m12 -= vertic.y * vertic.z * m;

                //第三行
                _I_ref.m20 -= vertic.x * vertic.z * m;
                _I_ref.m21 -= vertic.z * vertic.y * m;
                _I_ref.m22 -= vertic.z * vertic.z * m;
            }
        }
        I_ref = _I_ref;
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    Quaternion Quaternion_Add(Quaternion a, Quaternion b)
    {
        Quaternion r = new Quaternion()
        {
            x = a.x + b.x,
            y = a.y + b.y,
            z = a.z + b.z,
            w = a.w + b.w,
        };
        return r;
    }

    Matrix4x4 Matrix_reduce(Matrix4x4 a, Matrix4x4 b)
    {
        Matrix4x4 result = new Matrix4x4()
        {
            m00 = a.m00 - b.m00,
            m01 = a.m01 - b.m01,
            m02 = a.m02 - b.m02,
            m03 = a.m03 - b.m03,

            m10 = a.m10 - b.m10,
            m11 = a.m11 - b.m11,
            m12 = a.m12 - b.m12,
            m13 = a.m13 - b.m13,

            m20 = a.m20 - b.m20,
            m21 = a.m21 - b.m21,
            m22 = a.m22 - b.m22,
            m23 = a.m23 - b.m23,

            m30 = a.m30 - b.m30,
            m31 = a.m31 - b.m31,
            m32 = a.m32 - b.m32,
            m33 = 1.0f,

        };
        return result;
    }


    // In this function, update v and w by the impulse due to the collision with
    //a plane <P, N>
    //impulse 翻译等于 冲量
    void Collision_Impulse(Vector3 P, Vector3 N)
    {
        // point + normal 决定了一个平面

        int total_size = 0;
        Vector3 sumofImpulse = Vector3.zero;
        Vector3 sumofRi = Vector3.zero;


        //1 检测是否发生碰撞
        Matrix4x4 localToWorld = transform.localToWorldMatrix;
        Vector3 center_position = meshfilter.mesh.bounds.center;

        //世界坐标系旋转矩阵
        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(transform.rotation);

        //太多了 可以考虑加个bounding box 
        for (int i = 0; i < vertics.Length; i++)
        {

            //这里面计算的全部用point 因为计算的是平面相关
            //点全部转到世界坐标系去计算
            Vector3 worldCenter = localToWorld.MultiplyPoint(center_position);
            Vector3 worldVertic = localToWorld.MultiplyPoint(vertics[i]);


            Vector3 ri = worldVertic - worldCenter;
            Vector3 Rri = rotationMatrix.MultiplyVector(ri); //Rotation Matrix 

            //世界坐标的位置
            Vector3 xi_position = worldCenter + Rri;  //旋转矩阵，没有平移矩阵，所以没毛病 

            Vector3 in_vector = xi_position - P;
            float sign_distance = Vector3.Dot(in_vector, N);
            if (sign_distance <= 0)
            {

                //求出第i点的速度 ，然后做速度的分解
                Vector3 vi = v + Vector3.Cross(w, Rri);

                //在法线方向上的投影
                float vi_normal_length = Vector3.Dot(vi, N);
                //速度还在向下陷 ，这样采样改变速度
                if (vi_normal_length < 0)
                {
                    //将速度做一个水平和竖直方向的拆分
                    //切线，副切线，法线三组坐标系，这里做下切线和法线拆分
                    Vector3 vi_normal = vi_normal_length * N;	//法线方向上的速度
                    Vector3 vi_tangent = vi - vi_normal;        //切线方向上的速度

                    //法线方向的直接向外 这里没有考虑摩擦系数，
                    // The quantity ε is called the coefficient of restitution and must satisfy 0 ≤ ε ≤ 1.If ε = 1,
                    //εin particular, no kinetic energy is lost
                    vi_normal = -vi_normal * restitution; //同论文中写的 ε
                    //静摩擦系数
                    float a = Mathf.Max(1 - (restitution_tangent * (1 + restitution) * vi_normal.magnitude / vi_tangent.magnitude), 0);

                    vi_tangent *= a;



                    Vector3 vi_new = vi_normal + vi_tangent;



                    //计算出冲量的值
                    Matrix4x4 massMatrix = new Matrix4x4(
                                            new Vector4(1.0f, 0, 0, 0) / mass,
                                            new Vector4(0, 1.0f, 0, 0) / mass,
                                            new Vector4(0, 0, 1.0f, 0) / mass,
                                            new Vector4(0, 0, 0, 1.0f) / mass
                                        );


                    //这里把shapematching 还有动作模拟
                    Matrix4x4 K = Matrix_reduce(massMatrix, Get_Cross_Matrix(Rri) * I_ref.inverse * Get_Cross_Matrix(Rri));

                    //
                    Vector3 j_impuse = K.inverse * (vi_new - vi);

                    ++total_size;
                    sumofRi += Rri;
                    sumofImpulse += j_impuse;
                }
            }
        }

        if (total_size > 0)
        {

            sumofImpulse /= total_size;
            sumofRi /= total_size;


            //最后更新力和力矩
            //算出总动量，通过总动量计算速度的该变量
            //冲量是一个一瞬间的力，所以没有delta t 时间分量的该变量
            v += sumofImpulse / mass;
            //力矩也要平均下

            Vector3 torque = Vector3.Cross(sumofRi, sumofImpulse);

            Vector3 delta_w = I_ref.inverse * torque;
            w += delta_w;
        }
    }

    Vector3 applyTorqueW(float deltaTime, Vector3 force)
    {
        //在某个顶点施加一个合力

        Matrix4x4 rotationMatrix = Matrix4x4.Rotate(transform.rotation);

        //找到任意一个顶点，计算下位置偏移
        MeshFilter meshfilter = this.gameObject.GetComponent<MeshFilter>();

        Mesh mesh = meshfilter.mesh;

        Vector3[] vertics = mesh.vertices;
        Vector3 center = mesh.bounds.center;

        //要换成世界坐标

        Vector3 r0 = vertics[0] - center;//算出杠杆的长度
        Vector3 torque = Vector3.Cross(rotationMatrix * r0, force);

        //inertia
        // Matrix4x4 rotationI = rotationMatrix * I_ref * rotationMatrix.transpose;

        Vector3 deltaW = I_ref.inverse * torque * deltaTime;
        return deltaW;
    }


    Vector3 get_gravity()
    {
        //模拟的时候只有重力,而重力是产生力矩的
        //Given the gravitation being the only force ,
        //you don’t need tocal culatetorque or to update the angular velocity.
        //Now after you launch the bunny, you should see the bunny flying and spinning (with some initial angular velocity)!
        return G * mass;
    }

    /// <summary>
    /// transform.pos 和 两个matrix相乘 算出来的值是一样的测试。
    /// 父->子
    /// 子->父
    /// 横纵坐标的变换和切换
    /// 测试代码
    /// </summary>
    void updatePosMatrix()
    {
        //local vector
        //本质transform.position 和 矩阵相乘得到的值是相同的。
        Vector4 colum0 = new Vector4(1, 0, 0, 0);
        Vector4 colum1 = new Vector4(0, 1, 0, 0);
        Vector4 colum2 = new Vector4(0, 0, 1, 0);
        //最后一列表示平移
        Vector4 colum3 = new Vector4(transform.localPosition.x, transform.localPosition.y, transform.localPosition.z, 1);
        Matrix4x4 localMatrix = new Matrix4x4(colum0, colum1, colum2, colum3);

        //目前只考虑了旋转
        //未考虑旋转，旋转矩阵，绕着x轴旋转和y轴旋转还有z轴旋转，x,y,z
        //目前只考虑下x旋转

        float angle = transform.parent.localRotation.eulerAngles.x;

        float randis = angle * Mathf.Deg2Rad;
        //
        float cosTheta = Mathf.Cos(randis);
        float sinTheta = Mathf.Sin(randis);

        //旋转矩阵
        Vector4 p_colum0 = new Vector4(1.0f, 0, 0, 0);
        Vector4 p_colum1 = new Vector4(0.0f, cosTheta, sinTheta, 0);
        Vector4 p_colum2 = new Vector4(0.0f, -sinTheta, cosTheta, 0);

        //平移矩阵
        Vector4 p_colum3 = new Vector4(transform.parent.localPosition.x, transform.parent.localPosition.y,
            transform.localPosition.z, 1.0f);
        //
        Matrix4x4 parentMatrix = new Matrix4x4(p_colum0, p_colum1, p_colum2, p_colum3);
        //最终计算出结果
        Matrix4x4 result = parentMatrix * localMatrix;
        Vector3 pos = new Vector3(result.m03, result.m13, result.m23);

    }

    // Update is called once per frame
    void Update()
    {

        float deltaTime = dt;
        //Game Control
        if (Input.GetKey("r"))
        {
            transform.position = new Vector3(0, 0.6f, 0);
            restitution = 0.5f;
            launched = false;
        }

        if (Input.GetKey("l"))
        {
            v = new Vector3(8, 2, 0);
            //可以刚开始给个速度，然后加上冲量弄下
            //gravity will not produce torque
            // w = new Vector3(0, 2, 0);
            // w += applyTorqueW(deltaTime , new Vector3(-500, 0, 0));
            launched = true;
        }

        if (!launched)
        {
            return;
        }

        //
        if (w.magnitude < 0.1f || v.magnitude < 0.1f)
        {
            if (restitution >= 0)
            {
                restitution -= 0.02f;
            }
        }

        // Part I: Update velocities

        //simulate linear velocity
        Vector3 force = get_gravity(); //合力， 所有顶点的合力 第一步只考虑重力
        Vector3 acceleration = force / mass;		 	// 加速度=  力/m
        v += acceleration * deltaTime;

        //simulate quation velocity(模拟角速度)
        //since R(t) is a matrix, so use w 

        // Part II: Collision Impulse
        Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
        Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

        Vector3 x = transform.position;

        //Update angular status
        Quaternion q = transform.rotation;

        x += v * deltaTime;

        //通过四元组模拟旋转更新量
        Vector3 ww = w * deltaTime / 2;
        Quaternion wq = new Quaternion(ww.x, ww.y, ww.z, 0);

        q = Quaternion_Add(q, wq * q);

        w *= linear_decay;
        v *= angular_decay;


        // Part IV: Assign to the object
        transform.position = x;
        transform.rotation = q;
    }
}
