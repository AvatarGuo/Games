using UnityEngine;
using System.Collections;

public class PBD_model : MonoBehaviour
{

	float t = 0.0333f;
	float damping = 0.99f;
	int[] E;
	float[] L;
	Vector3[] V;
	Mesh mesh;
	private float mass_i = 1.0f;
	private GameObject sphere;


	//jacobi方法计数相关
	private Vector3[] sumofX;
	private int[] sumofXCount;

	// Use this for initialization
	void Start()
	{
		mesh = GetComponent<MeshFilter>().mesh;

		//Resize the mesh.
		int n = 21;
		Vector3[] X = new Vector3[n * n];
		Vector2[] UV = new Vector2[n * n];
		int[] T = new int[(n - 1) * (n - 1) * 6];
		for (int j = 0; j < n; j++)
			for (int i = 0; i < n; i++)
			{
				X[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
				UV[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
			}
		int t = 0;
		for (int j = 0; j < n - 1; j++)
			for (int i = 0; i < n - 1; i++)
			{
				T[t * 6 + 0] = j * n + i;
				T[t * 6 + 1] = j * n + i + 1;
				T[t * 6 + 2] = (j + 1) * n + i + 1;
				T[t * 6 + 3] = j * n + i;
				T[t * 6 + 4] = (j + 1) * n + i + 1;
				T[t * 6 + 5] = (j + 1) * n + i;
				t++;
			}
		mesh.vertices = X;
		mesh.triangles = T;
		mesh.uv = UV;
		mesh.RecalculateNormals();

		//Construct the original edge list
		int[] _E = new int[T.Length * 2];
		for (int i = 0; i < T.Length; i += 3)
		{
			_E[i * 2 + 0] = T[i + 0];
			_E[i * 2 + 1] = T[i + 1];
			_E[i * 2 + 2] = T[i + 1];
			_E[i * 2 + 3] = T[i + 2];
			_E[i * 2 + 4] = T[i + 2];
			_E[i * 2 + 5] = T[i + 0];
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

		E = new int[e_number * 2];
		for (int i = 0, e = 0; i < _E.Length; i += 2)
			if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
			{
				E[e * 2 + 0] = _E[i + 0];
				E[e * 2 + 1] = _E[i + 1];
				e++;
			}

		L = new float[E.Length / 2];
		for (int e = 0; e < E.Length / 2; e++)
		{
			int i = E[e * 2 + 0];
			int j = E[e * 2 + 1];
			L[e] = (X[i] - X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i = 0; i < X.Length; i++)
			V[i] = new Vector3(0, 0, 0);


		//总共次数
		sumofX = new Vector3[X.Length];
		sumofXCount = new int[X.Length];
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

	void Strain_Limiting()
	{
		// Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		//Apply PBD here.
		//质心位置是不变的，模拟原则.
		//多个弹簧的话 ，质心不变，依次递归 ，实际顶点已经排序好了  所以直接从 i = 0 开始到N即可。
		//机器学习里面的一种方法
		//两种方法

		//这里面也有一个jacobi 方法，并行 + 和顺序无关， 比较好的方法.容易并行。  取平均值总更新。
		//迭代更多能减少迭代次数

		//jacobi方法可以忽略顺序，方便并行
		for (int i = 0; i < X.Length; i++)
		{
			sumofX[i] = Vector3.zero;
			sumofXCount[i] = 0;
		}


		for (int j = 0; j < E.Length; j += 2)
		{
			//边的两个点计算下
			int edge_i = E[j];
			//省去一个+2的操作，分别对应了两个边
			int edge_j = E[j + 1];

			//当前边的编号
			int L_index = j / 2;

			Vector3 edage = X[edge_i] - X[edge_j];

			float magnitude = edage.magnitude;
			Vector3 normal_edage = edage.normalized;

			//因为做过排序了，所以0的拉力是向上,1的拉力是向下
			sumofX[edge_i] += X[edge_i] - 1.0f / 2 * (magnitude - L[L_index]) * normal_edage;
			sumofX[edge_j] += X[edge_j] + 1.0f / 2 * (magnitude - L[L_index]) * normal_edage;

			sumofXCount[edge_i] += 1;
			sumofXCount[edge_j] += 1;
		}

		for (int i = 0; i < X.Length; i++)
		{
			if (i == 0 || i == 20) continue;


			Vector3 tmpXi = (sumofX[i] + 0.2f * X[i]) / (sumofXCount[i] + 0.2f);
			V[i] += (tmpXi - X[i]) / t;
			X[i] = tmpXi;
		}


		mesh.vertices = X;
	}

	void Collision_Handling()
	{
		// Mesh mesh = GetComponent<MeshFilter> ().mesh;
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
		mesh.vertices = X;
	}

	// Update is called once per frame
	void Update()
	{
		// Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;  //a copy of vertics

		for (int i = 0; i < X.Length; i++)
		{
			if (i == 0 || i == 20) continue;

			//Initial Setup
			//首先计算每个顶点受到的力，然后计算加速度
			Vector3 force = mass_i * Vector3.down * 9.8f;
			Vector3 acceleration = force / mass_i;

			//Vt = V0 + at;
			V[i] += acceleration * t;
			V[i] *= damping;

			//这里相当于显示积分了，先用显示积分算出第一帧的值
			X[i] += V[i] * t;
		}
		mesh.vertices = X;

		for (int l = 0; l < 32; l++)
			Strain_Limiting();

		Collision_Handling();

		mesh.RecalculateNormals();

	}


}

