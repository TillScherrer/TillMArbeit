using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;

public class SplineFlatExtruder : MonoBehaviour
{
    [SerializeField]
    int segments = 30;

    [SerializeField]
    float width = 1;
    // Start is called before the first frame update
    void Start()
    {
        SplineContainer spline = GetComponent<SplineContainer>();

        Mesh mesh = new Mesh();
        Vector3[] vertices = new Vector3[segments*2 + 2];
        Vector3[] normals = new Vector3[segments * 2 + 2];
        int[] triangles = new int[segments * 6];


        for (int i = 0; i <= segments; i++)
        {
            float tInSpline = (float)i / (float)(segments+1);
            float3 pos;
            float3 foward;
            float3 up;
            spline.Evaluate(tInSpline, out pos, out foward, out up);
            Vector3 right = Vector3.Cross(up, foward).normalized;

            

            vertices[i * 2] =     (Vector3)pos + (width * 0.5f * right); //right vertex of current segment
            vertices[i * 2 + 1] = (Vector3)pos + (width * 0.5f * -right); //left vertex of current segment

            normals[i * 2] = up;
            normals[i * 2 + 1] = up;

            if (i == 0) continue;
            //meshes are formed clockwise. Two triangles together connect the previous segment to the current
            //first triangle from prev to current section 
            triangles[i * 6 - 6] = i * 2 - 2; //right previous
            triangles[i * 6 - 5] = i * 2 - 1; //left previous
            triangles[i * 6 - 4] = i * 2; //right current
            //second triangle form prev to current section
            triangles[i * 6 - 3] = i * 2; //right current
            triangles[i * 6 - 2] = i * 2 - 1; //left previous
            triangles[i * 6 - 1] = i * 2 + 1; //left current
        }

        //from global to local space. Otherwise the transform would be applied twice, once from the object to the mesh and again in the mesh itself
        transform.InverseTransformPoints(vertices);
        transform.InverseTransformDirections(normals);

        mesh.SetVertices(vertices);
        mesh.SetNormals(normals);
        mesh.SetTriangles(triangles,0);
        
        GetComponent<MeshFilter>().mesh = mesh;
        GetComponent<MeshCollider>().sharedMesh = mesh;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
