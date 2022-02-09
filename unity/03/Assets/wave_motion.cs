using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.PlayerLoop;

public struct T_HitInfo
{
	public Vector3 hitPoint;
	
	//存储当前的顶点编号做个记录
	public int i;
	public int j;
}

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;  
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	private float damping =  0.991f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	private float rho = 0.997f;//水的密度

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;



	//计算浮力的力矩，存储下hitpoint
	private List<T_HitInfo> hitpoints_v = new List<T_HitInfo>(12);
	private List<T_HitInfo> hitpoints_w = new List<T_HitInfo>(12);

	//计算下旋转的力矩

	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			
			//
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for (int i = li; i <= ui; i++)
		for (int j = lj; j <= uj; j++)
			if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
			{
				Ax[i, j] = 0;
				if (i != 0) Ax[i, j] -= x[i - 1, j] - x[i, j];
				if (i != size - 1) Ax[i, j] -= x[i + 1, j] - x[i, j];
				if (j != 0) Ax[i, j] -= x[i, j - 1] - x[i, j];
				if (j != size - 1) Ax[i, j] -= x[i, j + 1] - x[i, j];
			}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{
		hitpoints_v.Clear();
		hitpoints_w.Clear();
		 

		 //Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				new_h[i, j] = h[i, j] + ( h[i, j] - old_h[i, j] ) * damping;

				//
				if (i > 0)
				{
					new_h[i, j] += (h[i - 1, j] - h[i, j]) * rate;
				}
				if (i < size - 1)
				{
					new_h[i, j] += (h[i + 1, j] - h[i, j]) * rate;
				}
				
				//
				if (j > 0)
				{
					new_h[i, j] += (h[i, j - 1] - h[i, j]) * rate;
				}
				if (j < size - 1)
				{
					new_h[i, j] += (h[i, j + 1] - h[i, j]) * rate;
				}
			}
		}
		
		
		//Step 2: Block->Water coupling
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		//TODO: for block 1, calculate low_h.
		// cube_v
		
		var cube_motion_v = Transform.FindObjectOfType<cube_motion>();
		Transform trans_v = cube_motion_v.transform;
		Matrix4x4 rotationMatrix_v = Matrix4x4.Rotate(trans_v.rotation);
		
		cube_v = trans_v.position;
		int li = size-1 ;
		int ui = 0;
		int lj = size -1;
		int uj = 0;//限制下最小范围
		for ( int i = 0; i < size; i++ )
		{
			for ( int j = 0; j < size; j++ )
			{
				Vector3 pos_x  = X[i * size + j];
				Vector3 offset = pos_x - cube_v;
				if (offset.magnitude < 0.8f)
				{
					//为了更精确，底部发射一条射线，判断是否相交
					Vector3 x_bottom_pos = pos_x + Vector3.down;
					RaycastHit hit;
					// Does the ray intersect any objects excluding the player layer
					if (Physics.Raycast(x_bottom_pos, Vector3.up, out hit ,2.0f))
					{
						li = i < li ? i : li;
						ui = i > ui ? i : ui;

						lj = j < lj ? j : lj;
						uj = j > uj ? j : uj;

						low_h[i, j] = hit.point.y; //cube_v.y - 0.3f; //cube下面要低0.4f
						b[i,j] = 1.0f / rate * (new_h[i,j] - low_h[i,j]);
						cg_mask[i, j] = true;
						
							
						//计算下此时滑块受到的浮力
						T_HitInfo hitInfo = new T_HitInfo()
						{
							i = i,
							j = j,
							hitPoint = hit.point
						};
							
						hitpoints_v.Add(hitInfo);
						continue;
					}
				}
				
				//else
				vh[i, j] = 0;
				cg_mask[i, j] = false;
			}
		}
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		//限制一个最小矩形范围，然后在矩形范围内做共轭梯度法，这里简化先简单写下
		Conjugate_Gradient( cg_mask , b , vh , li ,ui ,lj , uj );
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				if (i > 0)
				{
					new_h[i, j] += (vh[i-1,j] - vh[i,j]) * rate * gamma;
				}
				if (i < size - 1)
				{
					new_h[i, j] += (vh[i+1,j] - vh[i, j]) * rate * gamma;
				}
				
				//
				if (j > 0)
				{
					new_h[i, j] += (vh[i, j - 1] - vh[i, j]) * rate * gamma;
				}
				if (j < size - 1)
				{
					new_h[i, j] += (vh[i, j + 1] - vh[i, j]) * rate * gamma;
				}
			}
		}


		//TODO: for block 2, calculate low_h.
		var cube_motion_w = Transform.FindObjectOfType<block_motion>();
		Transform trans_w = cube_motion_w.transform;
		Matrix4x4 rotationMatrix_w = Matrix4x4.Rotate(trans_w.rotation);
		cube_w = trans_w.position;
		int w_li = size-1 ;
		int w_ui = 0;
		int w_lj = size -1;
		int w_uj = 0;//限制下最小范围
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				
				Vector3 pos_w  = X[i * size + j]; 
				Vector3 offset_w = pos_w - cube_w;
				if (offset_w.magnitude < 0.8f)
				{
					//在使用射线检测 精确下距离
					RaycastHit hit;
					Vector3 x_bottom_pos = pos_w + Vector3.down;
					if (Physics.Raycast(x_bottom_pos, Vector3.up, out hit, 2.0f))
					{
						w_li = i < w_li ? i : w_li;
						w_ui = i > w_ui ? i : w_ui;

						w_lj = j < w_lj ? j : w_lj;
						w_uj = j > w_uj ? j : w_uj;

						low_h[i, j] = hit.point.y;//pos_w.y - 0.3f; //cube下面要低0.4f
						b[i, j] = 1.0f / rate * (new_h[i, j] - low_h[i, j]);
						cg_mask[i, j] = true;

						T_HitInfo hitInfo = new T_HitInfo()
						{
							i = i,
							j = j,
							hitPoint = hit.point,
						};
						
						hitpoints_w.Add(hitInfo);
						continue;
					}
				}
				
				// else
				vh[i, j] = 0;
				cg_mask[i, j] = false;
			}
		}
		
		Conjugate_Gradient( cg_mask , b , vh , w_li ,w_ui ,w_lj , w_uj );
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				//
				if (i > 0)
				{
					new_h[i, j] += (vh[i-1,j] - vh[i,j]) * rate * gamma;
				}
				if (i < size - 1)
				{
					new_h[i, j] += (vh[i+1,j] - vh[i, j]) * rate * gamma;
				}
				
				//
				if (j > 0)
				{
					new_h[i, j] += (vh[i, j - 1] - vh[i, j]) * rate * gamma;
				}
				if (j < size - 1)
				{
					new_h[i, j] += (vh[i, j + 1] - vh[i, j]) * rate * gamma;
				}
			}
		}
		
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
	
		//TODO: Diminish vh.
		//TODO: Update new_h by vh.

		//Step 3
		//TODO: old_h <- h; h <- new_h;
		
		//set height value back
		// for (int i = 0; i < size; i++)
		// {
		// 	for (int j = 0; j < size; j++)
		// 	{
		// 		old_h[i, j] = h[i, j];
		// 		//为了规避 限制下范围,水的高度不能超过cube
		// 		if ((i > li && i < ui && j > lj && j < uj) || (w_li > li && i < w_ui && j > w_lj && j < w_uj ) )
		// 		{
		// 			float height = Mathf.Min(0.55f, new_h[i, j]);
		// 			// h[i, j] = height;
		// 			new_h[i,j] = height;
		//
		// 		}
		// 		// else
		// 			h[i, j] = new_h[i, j];
		// 	}
		// }
		
		//Step 4: Water->Block coupling.
		//More TODO here.
		
		//计算下li的合力
		
		Vector3 sum_of_torque_v = Vector3.zero;
		for (int i = 0; i < hitpoints_v.Count ; i++)
		{
			
			Vector3 hitPoint = hitpoints_v[i].hitPoint; 
			Vector3 ri  = hitPoint - trans_v.position;
			Vector3 Rri = rotationMatrix_v.MultiplyVector(ri);

			//设置i,j编号
			int ii = hitpoints_v[i].i;
			int jj = hitpoints_v[i].j;
			
			//计算受到的浮力大小
			float now_height = h[ii,jj];
			float new_height = new_h[ii,jj];
			Vector3 force_buoyancy = Vector3.up * 9.8f * ( now_height - new_height);
			//计算下力矩的大小
			
			Vector3 torque = Vector3.Cross( Rri , force_buoyancy ); 
			
			sum_of_torque_v += torque;
		}

		cube_motion_v.AffectRotationbyTorque( sum_of_torque_v , rotationMatrix_v , 0.1f );
		
		//在算下另一个立方体
		Vector3 sum_of_torque_w = Vector3.zero;
		for (int i = 0; i < hitpoints_w.Count ; i++)
		{
			
			Vector3 hitPoint = hitpoints_w[i].hitPoint; 
			Vector3 ri  = hitPoint - trans_v.position;
			Vector3 Rri = rotationMatrix_w.MultiplyVector(ri);

			//设置i,j编号
			int ii = hitpoints_w[i].i;
			int jj = hitpoints_w[i].j;
			
			//计算受到的浮力大小
			float now_height_w = h[ii,jj];
			float new_height_w = new_h[ii,jj];
			Vector3 force_buoyancy_w = Vector3.up * 9.8f * ( now_height_w - new_height_w);
			//计算下力矩的大小
			
			Vector3 torque_w = Vector3.Cross( Rri , force_buoyancy_w ); 
			
			sum_of_torque_w += torque_w;
		}

		cube_motion_w.AffectRotationbyTorque( sum_of_torque_v , rotationMatrix_w , 0.1f );

		//Step 3
		//TODO: old_h <- h; h <- new_h;
		//set height value back
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				old_h[i, j] = h[i, j];
				//为了规避 限制下范围,水的高度不能超过cube
				if ((i > li && i < ui && j > lj && j < uj) || (w_li > li && i < w_ui && j > w_lj && j < w_uj ) )
				{
					float height = Mathf.Min(0.55f, new_h[i, j]);
					// h[i, j] = height;
					new_h[i,j] = height;

				}
				// else
				h[i, j] = new_h[i, j];
			}
		}
		
	}
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				h[i,j] = X[i*size+j].y;
			}
		}

		if (Input.GetKeyDown ("r")) 
		{
			//TODO: Add random water.
			int i_index = Random.Range(2,size-2);
			int j_index = Random.Range(2,size-2);

			float r = Random.Range(0.05f,0.5f);
			h[i_index, j_index] += r;

			h[i_index - 1, j_index] -= 0.25f * r;
			h[i_index + 1, j_index] -= 0.25f * r;

			h[i_index, j_index + 1] -= 0.25f * r;
			h[i_index, j_index - 1] -= 0.25f * r;
		}
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		//TODO: Store h back into X.y and recalculate normal.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				X[i * size + j].y = h[i, j];
			}
		}

		mesh.vertices = X;
		mesh.RecalculateNormals();

	}
}
