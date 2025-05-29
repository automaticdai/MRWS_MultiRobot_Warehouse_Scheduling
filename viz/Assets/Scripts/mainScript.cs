using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using JetBrains.Annotations;
using Unity.VisualScripting;
using UnityEngine;


public class mainScript : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created

    public GameObject robotOriginal;
    public GameObject robotContainer;

    public GameObject shelfOriginal;
    public GameObject shelfContainer;

    public GameObject goalOriginal;
    public GameObject goalContainer;

    public GameObject canvas;


    public Dictionary<string, GameObject> robotDict = new Dictionary<string, GameObject>();
    public Dictionary<string, GameObject> shelfDict = new Dictionary<string, GameObject>();
    public Dictionary<string, GameObject> goalDict = new Dictionary<string, GameObject>();


    public Dictionary<string, GameObject> itemObjects = new Dictionary<string, GameObject>();

    public List<string> items = new List<string>();

    public bool waitingForStart = true;

    private System.Random rand = new System.Random();

    private TMPro.TextMeshProUGUI bottomtext;

    private int itemDisplayCtr = 0;

    void Start()
    {
        bottomtext = GameObject.FindGameObjectWithTag("BottomText").GetComponent<TMPro.TextMeshProUGUI>();
        //CreateRobot("testrobo1", 1, 2);
        //CreateRobot("testrobo2", 2, 3);

        //CreateShelf("shelf1", 2, 2);
        //CreateShelf("shelf2", 2, 3);
        //CreateShelf("shelf3", 2, 4);

        //CreateGoal("goal1", 0, 0);

    }

    public void CreateRobot(string name, int x, int y)
    {
        GameObject robotClone = Instantiate(robotOriginal, new Vector3(0.5f + x, 0.5f, 0.5f + y), robotOriginal.transform.rotation);
        robotClone.transform.parent = robotContainer.transform;
        robotClone.name = name;
        robotClone.GetComponent<robot>().setReference(this);
        Renderer rend = robotClone.GetComponent<Renderer>();
        Material mat = new Material(Shader.Find("Standard"));

        mat.color = GenerateRandomColor();
        rend.material = mat;


        robotDict[robotClone.name] = robotClone;
    }

    public void CreateItem(string name)
    {
        this.items.Add(name);
        int sidenum = Int32.Parse(name.Substring(4)) + 3;
        Debug.Log($"Creating item with {sidenum} sides");
        createPolygonObj(sidenum, itemDisplayCtr, -2, name);
        itemDisplayCtr++;
    }

    public Color GenerateRandomColor()
    {
        int red = rand.Next(0, 255);
        int blue = rand.Next(0, 255);
        int green = rand.Next(0, 255);
        return new Color((float)red / 255, (float)blue / 255, (float)green / 255);
    }

    public void CreateShelf(string name, int x, int y, string itemName)
    {
        GameObject shelfClone = Instantiate(shelfOriginal, new Vector3(0.5f + x, 1.5f, 0.5f + y), shelfOriginal.transform.rotation);
        shelfClone.transform.parent = shelfContainer.transform;
        shelfClone.name = name;

        shelfDict[shelfClone.name] = shelfClone;

        GameObject itemClone = Instantiate(this.itemObjects[itemName], new Vector3(0.25f + x, 2.0f, 0.25f +y), shelfOriginal.transform.rotation);
        itemClone.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);

    }

    public void CreateGoal(string name, int x, int y)
    {
        GameObject goalClone = Instantiate(goalOriginal, new Vector3(0.5f + x, 0.5f, 0.5f + y), goalOriginal.transform.rotation);
        goalClone.transform.parent = goalContainer.transform;
        goalClone.name = name;
        goalClone.GetComponent<goal>().setReference(this);
        goalDict[goalClone.name] = goalClone;
    }


    public void startVisualisation()
    {
        this.waitingForStart = false;

    }

    // Update is called once per frame
    void Update()
    {

        if (!this.waitingForStart)
        {
            string textstring = "Items: ";
            bottomtext.text = textstring;

        }
    }

    public void correctScreenPosition()
    {
        int counter = 250;
        foreach (GameObject g in this.itemObjects.Values)
        {
            Camera cam = Camera.main;

            Debug.Log($"camera z {cam.transform.position.y}");
            Vector3 p = cam.ScreenToWorldPoint(new Vector3(counter, 70, cam.transform.position.y));

            g.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
            g.transform.position = new Vector3(p.x, p.y, p.z);

            counter += 100;
        }
    }


    void createPolygonObj(int sides, int x, int y, string name)
    {
        GameObject newobj = new GameObject();
        newobj.name = "Item";
        
        var meshFilter = newobj.AddComponent<MeshFilter>();
        var meshRenderer = newobj.AddComponent<MeshRenderer>();

        meshRenderer.material = new Material(Shader.Find("Standard"));
        meshRenderer.material.color = GenerateRandomColor();
        meshFilter.mesh = createMesh(sides);

        this.itemObjects.Add(name, newobj);
    }

    Mesh createMesh(int num_verts) 
    {
        Mesh mesh = new Mesh();
        var vertices = new Vector3[2*(num_verts + 1)];
        var uvs = new Vector2[2*(num_verts + 1)];


        var tris = new int[(2*num_verts*3) + (num_verts*3*2)];



        var normals = new Vector3[num_verts + 1];
        vertices[0] = new Vector3(0, 0, 0);

        for (int i = 0; i < num_verts; i++) 
        {
            vertices[i + 1] = new Vector3((float)Math.Cos(((float)i / num_verts)*2*Math.PI), 0, (float)Math.Sin(((float)i / num_verts) * 2 * Math.PI));

            
        }

        vertices[num_verts + 1] = new Vector3(0, 0.2f, 0);

        for (int i = 0; i < num_verts; i++)
        {
            vertices[num_verts + i + 2] = new Vector3((float)Math.Cos(((float)i / num_verts) * 2 * Math.PI), 0.2f, (float)Math.Sin(((float)i / num_verts) * 2 * Math.PI));

        
        }


        for (int i = 0; i < vertices.Count(); i++)
        {
            uvs[i] = new Vector2(vertices[i].x, vertices[i].z);
        }

        for (int i = 0; i < num_verts; i++)
        {
            tris[i*3] = 0;
            tris[i*3 + 1] = i + 1;
            if (i + 2 <= num_verts)
            {
                tris[i*3 + 2] = i + 2;
            }
            else 
            {
                tris[i*3 + 2] = 1;
            }
        }

        for (int i = 0; i < num_verts; i++)
        {
            tris[(i * 3) + (num_verts * 3) + 2] = num_verts + 1;

            tris[(i * 3) + 1 + (num_verts * 3)] = i + 1 + num_verts + 1;
            if (i + 2 <= num_verts)
            {
                tris[(i * 3) + (num_verts * 3)] = i + 2 + num_verts + 1;
            }
            else
            {
                tris[(i * 3) + (num_verts * 3)] = 1 + num_verts + 1;
            }
        }

        int offset = (num_verts * 3 * 2);
        for (int i = 0; i < num_verts; i++) 
        {
            int bottom = i + 2;
            if (bottom > num_verts) 
            {
                bottom = 1; 
            }

            int top = i + 2 + num_verts + 1;
            if (top == vertices.Count()) 
            {
                top = num_verts + 2;
            }

            tris[offset + (i * 6)] = i + 1;
            tris[offset + (i * 6) + 1] = i + 1 + num_verts + 1;
            tris[offset + (i * 6) + 2] = bottom;

            tris[offset + (i * 6) + 5] = i + 1 + num_verts + 1;
            tris[offset + (i * 6) + 4] = bottom;
            tris[offset + (i * 6) + 3] = top;
        }


        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = tris;

        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        mesh.Optimize();
        mesh.name = $"mesh{num_verts}";

        return mesh;
    }
}
