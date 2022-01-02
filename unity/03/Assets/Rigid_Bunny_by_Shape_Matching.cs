using System.Collections;
using System.Collections.Generic;
using UnityEngine;



public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{
	/// <summary>
	/// 初始化速度
	/// </summary>
	public const float INIT_SPEED = 5.0f;

	public bool launched = false;
	Vector3[] X;    //记录所有顶点位置
	Vector3[] Q;    //记录每个顶点距离原点的偏移值
	Vector3[] V;    //所有的速度
					//Q Q transpose
	Matrix4x4 QQt = Matrix4x4.zero;

	Matrix4x4 QQT_Inv = Matrix4x4.zero;


	//用下impluse的方法调试
	float linear_decay = 0.999f;                // for velocity decay
	float restitution = 0.98f;                  // for collision
	float restitution_tangent = 0.98f;           //切线方向滑动摩擦


	Matrix4x4 Matrix_add(Matrix4x4 a, Matrix4x4 b)
	{
		Matrix4x4 result = new Matrix4x4()
		{
			m00 = a.m00 + b.m00,
			m01 = a.m01 + b.m01,
			m02 = a.m02 + b.m02,
			m03 = a.m03 + b.m03,

			m10 = a.m10 + b.m10,
			m11 = a.m11 + b.m11,
			m12 = a.m12 + b.m12,
			m13 = a.m13 + b.m13,

			m20 = a.m20 + b.m20,
			m21 = a.m21 + b.m21,
			m22 = a.m22 + b.m22,
			m23 = a.m23 + b.m23,

			m30 = a.m30 + b.m30,
			m31 = a.m31 + b.m31,
			m32 = a.m32 + b.m32,
			m33 = 1.0f,

		};
		return result;
	}



	// Start is called before the first frame update
	void Start()
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		V = new Vector3[mesh.vertices.Length];
		X = mesh.vertices;
		Q = mesh.vertices;

		//Centerizing Q.
		Vector3 c = Vector3.zero;
		for (int i = 0; i < Q.Length; i++)
			c += Q[i];
		c /= Q.Length;
		for (int i = 0; i < Q.Length; i++)
			Q[i] -= c;

		//Get QQ^t ready.
		for (int i = 0; i < Q.Length; i++)
		{
			QQt[0, 0] += Q[i][0] * Q[i][0];
			QQt[0, 1] += Q[i][0] * Q[i][1];
			QQt[0, 2] += Q[i][0] * Q[i][2];
			QQt[1, 0] += Q[i][1] * Q[i][0];
			QQt[1, 1] += Q[i][1] * Q[i][1];
			QQt[1, 2] += Q[i][1] * Q[i][2];
			QQt[2, 0] += Q[i][2] * Q[i][0];
			QQt[2, 1] += Q[i][2] * Q[i][1];
			QQt[2, 2] += Q[i][2] * Q[i][2];
		}
		QQt[3, 3] = 1;

		QQT_Inv = QQt.inverse;

        for (int i = 0; i < X.Length; i++)
        {
            V[i][0] = INIT_SPEED;
            V[i][1] = 2;
        }

        //坐标全部变成了世界坐标空间了
		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position = Vector3.zero;
		transform.rotation = Quaternion.identity;
	}

	// Polar Decomposition that returns the rotation from F.
	Matrix4x4 Get_Rotation(Matrix4x4 F)
	{
		Matrix4x4 C = Matrix4x4.zero;
		for (int ii = 0; ii < 3; ii++)
			for (int jj = 0; jj < 3; jj++)
				for (int kk = 0; kk < 3; kk++)
					C[ii, jj] += F[kk, ii] * F[kk, jj];

		Matrix4x4 C2 = Matrix4x4.zero;
		for (int ii = 0; ii < 3; ii++)
			for (int jj = 0; jj < 3; jj++)
				for (int kk = 0; kk < 3; kk++)
					C2[ii, jj] += C[ii, kk] * C[jj, kk];

		float det = F[0, 0] * F[1, 1] * F[2, 2] +
						F[0, 1] * F[1, 2] * F[2, 0] +
						F[1, 0] * F[2, 1] * F[0, 2] -
						F[0, 2] * F[1, 1] * F[2, 0] -
						F[0, 1] * F[1, 0] * F[2, 2] -
						F[0, 0] * F[1, 2] * F[2, 1];

		float I_c = C[0, 0] + C[1, 1] + C[2, 2];
		float I_c2 = I_c * I_c;
		float II_c = 0.5f * (I_c2 - C2[0, 0] - C2[1, 1] - C2[2, 2]);
		float III_c = det * det;
		float k = I_c2 - 3 * II_c;

		Matrix4x4 inv_U = Matrix4x4.zero;
		if (k < 1e-10f)
		{
			float inv_lambda = 1 / Mathf.Sqrt(I_c / 3);
			inv_U[0, 0] = inv_lambda;
			inv_U[1, 1] = inv_lambda;
			inv_U[2, 2] = inv_lambda;
		}
		else
		{
			float l = I_c * (I_c * I_c - 4.5f * II_c) + 13.5f * III_c;
			float k_root = Mathf.Sqrt(k);
			float value = l / (k * k_root);
			if (value < -1.0f) value = -1.0f;
			if (value > 1.0f) value = 1.0f;
			float phi = Mathf.Acos(value);
			float lambda2 = (I_c + 2 * k_root * Mathf.Cos(phi / 3)) / 3.0f;
			float lambda = Mathf.Sqrt(lambda2);

			float III_u = Mathf.Sqrt(III_c);
			if (det < 0) III_u = -III_u;
			float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2 * III_u / lambda);
			float II_u = (I_u * I_u - I_c) * 0.5f;


			float inv_rate, factor;
			inv_rate = 1 / (I_u * II_u - III_u);
			factor = I_u * III_u * inv_rate;

			Matrix4x4 U = Matrix4x4.zero;
			U[0, 0] = factor;
			U[1, 1] = factor;
			U[2, 2] = factor;

			factor = (I_u * I_u - II_u) * inv_rate;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					U[i, j] += factor * C[i, j] - inv_rate * C2[i, j];

			inv_rate = 1 / III_u;
			factor = II_u * inv_rate;
			inv_U[0, 0] = factor;
			inv_U[1, 1] = factor;
			inv_U[2, 2] = factor;

			factor = -I_u * inv_rate;
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					inv_U[i, j] += factor * U[i, j] + inv_rate * C[i, j];
		}

		Matrix4x4 R = Matrix4x4.zero;
		for (int ii = 0; ii < 3; ii++)
			for (int jj = 0; jj < 3; jj++)
				for (int kk = 0; kk < 3; kk++)
					R[ii, jj] += F[ii, kk] * inv_U[kk, jj];
		R[3, 3] = 1;
		return R;
	}

	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
	{
		for (int i = 0; i < Q.Length; i++)
		{
			Vector3 x = (Vector3)(R * Q[i]) + c;


			V[i] += (x - X[i]) * inv_dt;
			X[i] = x;
		}
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices = X;
	}

	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		for (int i = 0; i < V.Length; i++)
		{
			Vector3 outplane_point = X[i] - P;

			//法线方向上的投影
			float signDistance = Vector3.Dot(N, outplane_point);
			if (signDistance < 0)
			{
				Vector3 vi = V[i];

				float vi_normal_length = Vector3.Dot(vi, N);
				if (vi_normal_length < 0)
				{
					Vector3 vi_normal = vi_normal_length * N;
					Vector3 vi_tangent = vi - vi_normal;

					vi_normal = -vi_normal * restitution;
					float a = Mathf.Max(1 - (restitution_tangent * (1 + restitution) * vi_normal.magnitude / vi_tangent.magnitude), 0);
					vi_tangent *= a;

					//各自模拟各自的速度
					V[i] = vi_tangent + vi_normal;
				}
			}

		}

	}


	void Collision(float inv_dt)
	{
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

	}

	Vector3 sum_of_force()
	{
		//g = 9.8N/KG
		return Vector3.down * 9.8f;
	}

	// Update is called once per frame
	void Update()
	{

		float dt = 0.015f;

		//Step 1: run a simple particle system.
		for (int i = 0; i < V.Length; i++)
		{
			int mass = 1;
			Vector3 force = sum_of_force();
			Vector3 acceleration = force / mass;

			//更新速度 & 位置 position
			V[i] += acceleration * dt;
			X[i] += V[i] * dt;
		}


		//Step 2: Perform simple particle collision.
		//impulse
		Collision(1 / dt);

		// Step 3: Use shape matching to get new translation c and 
		// new rotation R. Update the mesh by c and R.

		Vector3 c = Vector3.zero;
		for (int i = 0; i < X.Length; i++)
			c += X[i];
		c /= X.Length;


		Matrix4x4 xTotal = Matrix4x4.zero;
		for (int i = 0; i < X.Length; i++)
		{

			//
			// xTotal += (X[i] - c )  ;

			//3x1 和 1x3 = 3x3
			//两者比较简单直接手撸写吧

			Vector3 offset = X[i] - c;
			Vector3 ri = Q[i];

			//手撸了 3x1 ,1x3 = 3x3矩阵
			Matrix4x4 tmp = new Matrix4x4()
			{
				m00 = offset.x * ri.x,
				m01 = offset.x * ri.y,
				m02 = offset.x * ri.z,
				m03 = 0,

				m10 = offset.y * ri.x,
				m11 = offset.y * ri.y,
				m12 = offset.y * ri.z,
				m13 = 0,

				m20 = offset.z * ri.x,
				m21 = offset.z * ri.y,
				m22 = offset.z * ri.z,
				m23 = 0,

				m30 = 0,
				m31 = 0,
				m32 = 0,
				m33 = 1
			};

			xTotal = Matrix_add(xTotal, tmp);
		}

		xTotal[3, 3] = 1.0f;



		Matrix4x4 A = xTotal * QQT_Inv;
		Matrix4x4 R = Get_Rotation(A);

		//Shape Matching (translation)
		//Shape Matching (rotation)
		Update_Mesh(c, R, 1 / dt);


	}
}
