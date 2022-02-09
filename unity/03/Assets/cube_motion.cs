using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class cube_motion : MonoBehaviour
{
	bool pressed  = false;
	public bool cube_move = false;
	Vector3 offset;
	

	public Vector3 w = new Vector3(0, 0, 0);    // angular velocity
	Matrix4x4 I_ref;

	private int mass = 0;


    // Start is called before the first frame update
    void Start()
    {
	    CalculateInertiaTensorAndMass();
    }
    
    void CalculateInertiaTensorAndMass()
    {
		
	    MeshFilter meshfilter = this.gameObject.GetComponent<MeshFilter>();
	    Matrix4x4 _I_ref = Matrix4x4.identity; // ref inertia tensor

	    int m = 1;
	    if (meshfilter != null)
	    {
		    Mesh mesh = meshfilter.mesh;
		    var vertics = mesh.vertices;

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

	
    // Update is called once per frame
	void Update () 
	{
		if (Input.GetMouseButtonDown(0))
		{
			pressed = true;
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
			if (Vector3.Cross(ray.direction, transform.position - ray.origin).magnitude < 0.8f) 
				cube_move = true;
			else 
				cube_move = false;
			
			offset = Input.mousePosition - Camera.main.WorldToScreenPoint(transform.position);
		}

		if (Input.GetMouseButtonUp (0))
		{
			pressed = false;
			cube_move = false;
		}

		if(pressed)
		{
			if (cube_move)
			{
				Vector3 mouse = Input.mousePosition;
				mouse -= offset;
				mouse.z = Camera.main.WorldToScreenPoint(transform.position).z;
				Vector3 p = Camera.main.ScreenToWorldPoint(mouse);
				p.y = transform.position.y;
				transform.position = p;
			}
			else
			{
				//float h = 2.0f * Input.GetAxis("Mouse X");
				//Camera.main.transform.RotateAround(new Vector3(0, 0, 0), Vector3.up, h);
			}
		}
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

	public void AffectRotationbyTorque(Vector3 torque , Matrix4x4 R ,float dt)
	{	
		Matrix4x4 I = R * I_ref * R.transpose;
		Vector3 delta_w = I.inverse * torque;
		
  		Quaternion q = transform.rotation;
        w = delta_w * 0.98f ;
        //通过四元组模拟旋转更新量
        Vector3 ww = w * dt / 2;
        Quaternion wq = new Quaternion(ww.x, ww.y, ww.z, 0);
		
        q = Quaternion_Add(q, wq * q);
		transform.rotation = q;

	}
}
