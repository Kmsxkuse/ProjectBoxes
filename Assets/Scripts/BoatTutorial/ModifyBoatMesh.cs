using System.Collections.Generic;
using UnityEngine;

namespace BoatTutorial
{
    //Generates the mesh that's below the water
    public class ModifyBoatMesh
    {
        //Find all the distances to water once because some triangles share vertices, so reuse
        private readonly float[] allDistancesToWater;

        //The boat transform needed to get the global position of a vertice
        private readonly Transform boatTrans;

        //Positions in allVerticesArray, such as 0, 3, 5, to build triangles
        private readonly int[] boatTriangles;

        //Coordinates of all vertices in the original boat
        private readonly Vector3[] boatVertices;

        //So we only need to make the transformation from local to global once
        public Vector3[] boatVerticesGlobal;

        //The triangles belonging to the part of the boat that's under water
        public List<TriangleData> underWaterTriangleData = new List<TriangleData>();

        public ModifyBoatMesh(GameObject boatObj)
        {
            //Get the transform
            boatTrans = boatObj.transform;

            //Init the arrays and lists
            boatVertices = boatObj.GetComponent<MeshFilter>().mesh.vertices;
            boatTriangles = boatObj.GetComponent<MeshFilter>().mesh.triangles;

            //The boat vertices in global position
            boatVerticesGlobal = new Vector3[boatVertices.Length];
            //Find all the distances to water once because some triangles share vertices, so reuse
            allDistancesToWater = new float[boatVertices.Length];
        }

        //Generate the underwater mesh
        public void GenerateUnderwaterMesh()
        {
            //Reset
            underWaterTriangleData.Clear();

            //Find all the distances to water once because some triangles share vertices, so reuse
            for (var j = 0; j < boatVertices.Length; j++)
            {
                //The coordinate should be in global position
                var globalPos = boatTrans.TransformPoint(boatVertices[j]);

                //Save the global position so we only need to calculate it once here
                //And if we want to debug we can convert it back to local
                boatVerticesGlobal[j] = globalPos;

                allDistancesToWater[j] = WaterController.current.DistanceToWater(globalPos, Time.time);
            }

            //Add the triangles that are below the water
            AddTriangles();
        }

        //Add all the triangles that's part of the underwater mesh
        private void AddTriangles()
        {
            //List that will store the data we need to sort the vertices based on distance to water
            var vertexData = new List<VertexData>();

            //Add init data that will be replaced
            vertexData.Add(new VertexData());
            vertexData.Add(new VertexData());
            vertexData.Add(new VertexData());


            //Loop through all the triangles (3 vertices at a time = 1 triangle)
            var i = 0;
            while (i < boatTriangles.Length)
            {
                //Loop through the 3 vertices
                for (var x = 0; x < 3; x++)
                {
                    //Save the data we need
                    vertexData[x].distance = allDistancesToWater[boatTriangles[i]];

                    vertexData[x].index = x;

                    vertexData[x].globalVertexPos = boatVerticesGlobal[boatTriangles[i]];

                    i++;
                }


                //All vertices are above the water
                if (vertexData[0].distance > 0f && vertexData[1].distance > 0f && vertexData[2].distance > 0f) continue;


                //Create the triangles that are below the waterline

                //All vertices are underwater
                if (vertexData[0].distance < 0f && vertexData[1].distance < 0f && vertexData[2].distance < 0f)
                {
                    var p1 = vertexData[0].globalVertexPos;
                    var p2 = vertexData[1].globalVertexPos;
                    var p3 = vertexData[2].globalVertexPos;

                    //Save the triangle
                    underWaterTriangleData.Add(new TriangleData(p1, p2, p3));
                }
                //1 or 2 vertices are below the water
                else
                {
                    //Sort the vertices
                    vertexData.Sort((x, y) => x.distance.CompareTo(y.distance));

                    vertexData.Reverse();

                    //One vertice is above the water, the rest is below
                    //if (vertexData[0].distance > 0f && vertexData[1].distance < 0f && vertexData[2].distance < 0f)
                    if (vertexData[1].distance < 0f)
                        AddTrianglesOneAboveWater(vertexData);
                    //Two vertices are above the water, the other is below
                    //else if (vertexData[0].distance > 0f && vertexData[1].distance > 0f && vertexData[2].distance < 0f)
                    else
                        AddTrianglesTwoAboveWater(vertexData);
                }
            }
        }

        //Build the new triangles where one of the old vertices is above the water
        private void AddTrianglesOneAboveWater(List<VertexData> vertexData)
        {
            //H is always at position 0
            var H = vertexData[0].globalVertexPos;

            //Left of H is M
            //Right of H is L

            //Find the index of M
            var M_index = vertexData[0].index - 1;
            if (M_index < 0) M_index = 2;

            //We also need the heights to water
            var h_H = vertexData[0].distance;
            var h_M = 0f;
            var h_L = 0f;

            var M = Vector3.zero;
            var L = Vector3.zero;

            //This means M is at position 1 in the List
            if (vertexData[1].index == M_index)
            {
                M = vertexData[1].globalVertexPos;
                L = vertexData[2].globalVertexPos;

                h_M = vertexData[1].distance;
                h_L = vertexData[2].distance;
            }
            else
            {
                M = vertexData[2].globalVertexPos;
                L = vertexData[1].globalVertexPos;

                h_M = vertexData[2].distance;
                h_L = vertexData[1].distance;
            }


            //Now we can calculate where we should cut the triangle to form 2 new triangles
            //because the resulting area will always form a square

            //Point I_M
            var MH = H - M;

            var t_M = -h_M / (h_H - h_M);

            var MI_M = t_M * MH;

            var I_M = MI_M + M;


            //Point I_L
            var LH = H - L;

            var t_L = -h_L / (h_H - h_L);

            var LI_L = t_L * LH;

            var I_L = LI_L + L;


            //Save the data, such as normal, area, etc      
            //2 triangles below the water  
            underWaterTriangleData.Add(new TriangleData(M, I_M, I_L));
            underWaterTriangleData.Add(new TriangleData(M, I_L, L));
        }

        //Build the new triangles where two of the old vertices are above the water
        private void AddTrianglesTwoAboveWater(List<VertexData> vertexData)
        {
            //H and M are above the water
            //H is after the vertice that's below water, which is L
            //So we know which one is L because it is last in the sorted list
            var L = vertexData[2].globalVertexPos;

            //Find the index of H
            var H_index = vertexData[2].index + 1;
            if (H_index > 2) H_index = 0;


            //We also need the heights to water
            var h_L = vertexData[2].distance;
            var h_H = 0f;
            var h_M = 0f;

            var H = Vector3.zero;
            var M = Vector3.zero;

            //This means that H is at position 1 in the list
            if (vertexData[1].index == H_index)
            {
                H = vertexData[1].globalVertexPos;
                M = vertexData[0].globalVertexPos;

                h_H = vertexData[1].distance;
                h_M = vertexData[0].distance;
            }
            else
            {
                H = vertexData[0].globalVertexPos;
                M = vertexData[1].globalVertexPos;

                h_H = vertexData[0].distance;
                h_M = vertexData[1].distance;
            }


            //Now we can find where to cut the triangle

            //Point J_M
            var LM = M - L;

            var t_M = -h_L / (h_M - h_L);

            var LJ_M = t_M * LM;

            var J_M = LJ_M + L;


            //Point J_H
            var LH = H - L;

            var t_H = -h_L / (h_H - h_L);

            var LJ_H = t_H * LH;

            var J_H = LJ_H + L;


            //Save the data, such as normal, area, etc
            //1 triangle below the water
            underWaterTriangleData.Add(new TriangleData(L, J_H, J_M));
        }

        //Display the underwater mesh
        public void DisplayMesh(Mesh mesh, string name, List<TriangleData> triangesData)
        {
            var vertices = new List<Vector3>();
            var triangles = new List<int>();

            //Build the mesh
            for (var i = 0; i < triangesData.Count; i++)
            {
                //From global coordinates to local coordinates
                var p1 = boatTrans.InverseTransformPoint(triangesData[i].p1);
                var p2 = boatTrans.InverseTransformPoint(triangesData[i].p2);
                var p3 = boatTrans.InverseTransformPoint(triangesData[i].p3);

                vertices.Add(p1);
                triangles.Add(vertices.Count - 1);

                vertices.Add(p2);
                triangles.Add(vertices.Count - 1);

                vertices.Add(p3);
                triangles.Add(vertices.Count - 1);
            }

            //Remove the old mesh
            mesh.Clear();

            //Give it a name
            mesh.name = name;

            //Add the new vertices and triangles
            mesh.vertices = vertices.ToArray();

            mesh.triangles = triangles.ToArray();

            mesh.RecalculateBounds();
        }

        //Help class to store triangle data so we can sort the distances
        private class VertexData
        {
            //The distance to water from this vertex
            public float distance;

            //The global Vector3 position of the vertex
            public Vector3 globalVertexPos;

            //An index so we can form clockwise triangles
            public int index;
        }
    }
}