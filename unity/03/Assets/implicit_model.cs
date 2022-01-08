using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class implicit_model : MonoBehaviour
{
	float t = 0.0333f;
	float mass = 1.0f;  // mass of i
	float damping = 0.99f;
	float rho = 0.995f;
	float spring_k = 8000;
	int[] E;
	float[] L;
	Vector3[] V;


	private GameObject sphere;

	private Mesh mesh;


	//静态mesh 动态mesh

	// Start is called before the first frame update
	void Start()
	{
		mesh = GetComponent<MeshFilter>().mesh;

		//Resize the mesh.
		int n = 21;
		Vector3[] X = new Vector3[n * n];       // 21*21
		Vector2[] UV = new Vector2[n * n];      // 21*21 
		int[] triangles = new int[(n - 1) * (n - 1) * 6];
		for (int j = 0; j < n; j++)
		{
			for (int i = 0; i < n; i++)
			{
				// mesh的uv相关
				X[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
				UV[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
			}
		}


		//四边形设置编号索引相关
		int t = 0;
		for (int j = 0; j < n - 1; j++)
		{
			for (int i = 0; i < n - 1; i++)
			{
				triangles[t * 6 + 0] = j * n + i;
				triangles[t * 6 + 1] = j * n + i + 1;
				triangles[t * 6 + 2] = (j + 1) * n + i + 1;
				triangles[t * 6 + 3] = j * n + i;
				triangles[t * 6 + 4] = (j + 1) * n + i + 1;
				triangles[t * 6 + 5] = (j + 1) * n + i;
				t++;
			}
		}

		//一个是动态创建的mesh 一个是静态创建的mesh
		mesh.vertices = X;
		mesh.triangles = triangles;
		mesh.uv = UV;
		//因为材质没变，所以可以重新计算uv
		mesh.RecalculateNormals();


		//Construct the original E
		int[] _E = new int[triangles.Length * 2];
		for (int i = 0; i < triangles.Length; i += 3)
		{
			_E[i * 2 + 0] = triangles[i + 0];
			_E[i * 2 + 1] = triangles[i + 1];
			_E[i * 2 + 2] = triangles[i + 1];
			_E[i * 2 + 3] = triangles[i + 2];
			_E[i * 2 + 4] = triangles[i + 2];
			_E[i * 2 + 5] = triangles[i + 0];
		}
		//Reorder the original edge list
		for (int i = 0; i < _E.Length; i += 2)
			if (_E[i] > _E[i + 1])
				Swap(ref _E[i], ref _E[i + 1]);
		//Sort the original edge list using quicksort
		Quick_Sort(ref _E, 0, _E.Length / 2 - 1);

		int e_number = 0;
		for (int i = 0; i < _E.Length; i += 2)
			if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
				e_number++;

		//初始化所有的边 ，对边设置弹簧，当前没有弄（这里比较简单，因为正方形的原因，所以可以主动设置长宽等信息）
		E = new int[e_number * 2];
		for (int i = 0, e = 0; i < _E.Length; i += 2)
			if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
			{
				E[e * 2 + 0] = _E[i + 0];
				E[e * 2 + 1] = _E[i + 1];
				e++;
			}

		//初始长度(约束条件)
		L = new float[E.Length / 2];
		for (int e = 0; e < E.Length / 2; e++)
		{
			int v0 = E[e * 2 + 0];
			int v1 = E[e * 2 + 1];
			L[e] = (X[v0] - X[v1]).magnitude;
		}

		//给每个顶点赋值初始化速度
		V = new Vector3[X.Length];
		for (int i = 0; i < V.Length; i++)
			V[i] = new Vector3(0, 0, 0);

	}

	void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if (l < r)
		{
			j = Quick_Sort_Partition(ref a, l, r);
			Quick_Sort(ref a, l, j - 1);
			Quick_Sort(ref a, j + 1, r);
		}
	}

	int Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a[l * 2 + 0];
		pivot_1 = a[l * 2 + 1];
		i = l;
		j = r + 1;
		while (true)
		{
			do ++i; while (i <= r && (a[i * 2] < pivot_0 || a[i * 2] == pivot_0 && a[i * 2 + 1] <= pivot_1));
			do --j; while (a[j * 2] > pivot_0 || a[j * 2] == pivot_0 && a[j * 2 + 1] > pivot_1);
			if (i >= j) break;
			Swap(ref a[i * 2], ref a[j * 2]);
			Swap(ref a[i * 2 + 1], ref a[j * 2 + 1]);
		}
		Swap(ref a[l * 2 + 0], ref a[j * 2 + 0]);
		Swap(ref a[l * 2 + 1], ref a[j * 2 + 1]);
		return j;
	}

	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Collision_Handling()
	{
		if (sphere == null)
		{
			sphere = GameObject.Find("Sphere").gameObject;
		}
		// Mesh mesh = GetComponent<MeshFilter> ().mesh;
		//还要把局部坐标全部
		Vector3[] X = mesh.vertices;


		Vector3 c = sphere.transform.position;
		float radius = 2.7f;
		// float sqrRadius = radius * radius;

		//不过也可以不用平均冲量，因为是对单个点来计算碰撞的
		for (int i = 0; i < X.Length; i++)
		{
			Vector3 world_xi = transform.TransformPoint(X[i]);
			Vector3 ci = world_xi - c;
			float ci_magnitude = ci.magnitude;

			if (ci_magnitude <= radius)
			{
				//这个是公式给的
				//没有对速度做分解，直接加了个冲量上去
				V[i] += 1 / t * (c + radius * (ci) / ci.magnitude - X[i]);

				//不改x位置其实也可以，但是容易穿帮
				X[i] = transform.InverseTransformPoint(c + radius * ci / ci.magnitude);
			}
		}

		//Handle colllision.

		mesh.vertices = X;
	}

	void Get_Gradient(Vector3[] X, Vector3[] X_hat, float t, Vector3[] G)
	{
		//Momentum and Gravity.

		//把X_hat 设置成中间变量 
		for (int i = 0; i < X.Length; i++)
		{
			if (i == 0 || i == 20) continue;

			Vector3 worldXi = transform.TransformPoint(X[i]);

			//算一阶导数 ,第 k 时刻，算第 k+1 时刻
			G[i] = mass / (t * t) * (worldXi - X_hat[i]) - Vector3.down * mass * 9.8f;
		}

		//弹簧的边对两个顶点的力 ，依次对边的顶点做处理
		for (int j = 0; j < E.Length; j += 2)
		{
			//边的两个点计算下
			int edge_i = E[j];

			//省去一个+2的操作，分别对应了两个边
			int edge_j = E[j + 1];

			//弹簧弹力 不能过滤，布料本身的物理模拟
			//if (edage_i == 0 || edage_i == 20 || edage_j == 0 || edage_j == 20)
			//{
			// continue;
			//}

			//在世界空间内计算 ，统一坐标空间，否则容易出问题
			Vector3 vector_edage = X[edge_i] - X[edge_j];
			Vector3 world_vector_edage = transform.TransformVector(vector_edage);
			int L_index = j / 2;


			//力的分解，只需要计算弹力即可
			G[edge_i] += spring_k * (1 - L[L_index] / world_vector_edage.magnitude) * world_vector_edage;
			G[edge_j] -= spring_k * (1 - L[L_index] / world_vector_edage.magnitude) * world_vector_edage;
		}
	}

	// Update is called once per frame
	void Update()
	{
		// Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;            //	每一帧copy一份数组 ,另外需要坐标系的变换，都是局部的坐标系
		Vector3[] last_X = new Vector3[X.Length];   //	最后一次的该变量
		Vector3[] X_hat = new Vector3[X.Length];
		Vector3[] G = new Vector3[X.Length];  //Gradient 计算的梯度中间值

		//Initial Setup.
		//可以用帧率去计算

		float deltaTime = t;


		//第一步初始化 x_hat
		for (int i = 0; i < X.Length; i++)
		{
			if (i == 0 || i == 20)
				continue;

			Vector3 Xi_WorldP = transform.TransformPoint(X[i]);

			V[i] *= damping;
			X_hat[i] = Xi_WorldP + V[i] * deltaTime;

			//刚开始初始化X[i] 直接用了V[0]的速度，隐试积分应该用的是V1的速度算，这里有点显示积分的感觉
			X[i] = transform.InverseTransformPoint(X_hat[i]);

		}

		//一旦 delta X 变化量小于某个值，即不需要模拟了，
		//否则每次做一次模拟
		for (int k = 0; k < 32; k++)
		{
			//jacobi method with chebyshev acceleration

			// 一阶导数 + 二阶导数用隐式积分的方法求值
			Get_Gradient(X, X_hat, t, G);

			//Update X by gradient.
			for (int i = 0; i < X.Length; i++)
			{
				if (i == 0 || i == 20) continue;

				//梯度，一阶导数除以二阶导数
				//
				Vector3 deltaX_world = (1.0f / (1.0f / (t * t) * mass + 4 * spring_k)) * G[i];
				Vector3 delteX_local = transform.InverseTransformVector(deltaX_world);
				X[i] -= delteX_local;

				//不能直接改x 因为获取梯度的时候 还是需要这些变量的
				// last_X[i] += deltaX;
			}
		}

		for (int i = 0; i < X.Length; i++)
		{
			if (i == 0 || i == 20) continue;

			Vector3 xi_world = transform.TransformPoint(X[i]);
			V[i] += 1.0f / deltaTime * (xi_world - X_hat[i]);
		}

		//Finishing.

		mesh.vertices = X;

		Collision_Handling();

		//修改了mesh后 需要重新计算下法线相关
		mesh.RecalculateNormals();
	}
}
