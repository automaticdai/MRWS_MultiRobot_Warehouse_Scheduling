using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

/// <summary>
/// Owns the scene's simulation entities and the on-screen status line.
/// </summary>
public class mainScript : MonoBehaviour
{
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

    /// <summary>An order the simulator has announced but not yet completed.</summary>
    class OrderView
    {
        public string Name;
        public int Priority;
        public string[] Items;
    }

    private readonly List<OrderView> activeOrders = new();
    private int completedOrderCount;
    private bool orderDisplayDirty = true;

    // Item clones parented to nothing in particular (the one sitting on each
    // shelf). Tracked so a reset can remove them.
    private readonly List<GameObject> loosePropItems = new();

    private System.Random rand = new System.Random();

    private TMPro.TextMeshProUGUI bottomtext;

    private int itemDisplayCtr = 0;

    void Start()
    {
        GameObject textObj = GameObject.FindGameObjectWithTag("BottomText");
        if (textObj != null)
        {
            bottomtext = textObj.GetComponent<TMPro.TextMeshProUGUI>();
        }
    }

    // ---- lookups ----------------------------------------------------------
    // A message can name something that does not exist: the simulator may have
    // been restarted, or a RESET may have cleared the scene mid-flight.

    public robot FindRobot(string name)
    {
        if (robotDict.TryGetValue(name, out GameObject obj) && obj != null)
        {
            return obj.GetComponent<robot>();
        }
        Debug.LogWarning("No such robot in scene: " + name);
        return null;
    }

    public goal FindGoal(string name)
    {
        if (goalDict.TryGetValue(name, out GameObject obj) && obj != null)
        {
            return obj.GetComponent<goal>();
        }
        Debug.LogWarning("No such goal in scene: " + name);
        return null;
    }

    public GameObject ItemPrototype(string itemName)
    {
        if (itemObjects.TryGetValue(itemName, out GameObject prototype) && prototype != null)
        {
            return prototype;
        }
        Debug.LogWarning("No prototype for item: " + itemName);
        return null;
    }

    // ---- entity creation --------------------------------------------------

    public void CreateRobot(string name, int x, int y)
    {
        GameObject robotClone = Instantiate(robotOriginal, new Vector3(0.5f + x, 0.5f, 0.5f + y), robotOriginal.transform.rotation);
        robotClone.transform.parent = robotContainer.transform;
        robotClone.name = name;

        robot component = robotClone.GetComponent<robot>();
        component.setReference(this);
        component.snapToPosition(x, y);

        Renderer rend = robotClone.GetComponent<Renderer>();
        if (rend != null)
        {
            rend.material = new Material(Shader.Find("Standard"));
            component.setBaseColour(GenerateRandomColor());
        }

        robotDict[robotClone.name] = robotClone;
    }

    public void CreateItem(string name)
    {
        if (itemObjects.ContainsKey(name))
        {
            return;
        }
        this.items.Add(name);

        // Item N is drawn as an (N+3)-sided prism, so items stay visually
        // distinct without needing art for each one.
        int sidenum = Int32.Parse(name.Substring(4)) + 3;
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

        GameObject prototype = ItemPrototype(itemName);
        if (prototype != null)
        {
            GameObject itemClone = Instantiate(prototype, new Vector3(0.25f + x, 2.0f, 0.25f + y), shelfOriginal.transform.rotation);
            itemClone.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
            loosePropItems.Add(itemClone);
        }
    }

    public void CreateGoal(string name, int x, int y)
    {
        GameObject goalClone = Instantiate(goalOriginal, new Vector3(0.5f + x, 0.5f, 0.5f + y), goalOriginal.transform.rotation);
        goalClone.transform.parent = goalContainer.transform;
        goalClone.name = name;
        goalClone.GetComponent<goal>().setReference(this);
        goalDict[goalClone.name] = goalClone;
    }

    // ---- orders -----------------------------------------------------------

    public void CreateOrder(string name, int priority, string[] orderItems)
    {
        activeOrders.Add(new OrderView { Name = name, Priority = priority, Items = orderItems });
        orderDisplayDirty = true;
    }

    public void CompleteOrder(string name)
    {
        int index = activeOrders.FindIndex(o => o.Name == name);
        if (index < 0)
        {
            Debug.LogWarning("Completed an order that was never announced: " + name);
            return;
        }
        activeOrders.RemoveAt(index);
        completedOrderCount++;
        orderDisplayDirty = true;
    }

    // ---- session ----------------------------------------------------------

    public void startVisualisation()
    {
        this.waitingForStart = false;
    }

    /// <summary>
    /// Tear the scene back down to nothing. The simulator sends RESET before
    /// each run, so a batch of runs does not pile every run's robots, shelves
    /// and items into the same scene.
    /// </summary>
    public void ResetVisualisation()
    {
        DestroyAll(robotDict.Values);
        DestroyAll(goalDict.Values);
        DestroyAll(shelfDict.Values);
        DestroyAll(itemObjects.Values);
        DestroyAll(loosePropItems);

        robotDict.Clear();
        goalDict.Clear();
        shelfDict.Clear();
        itemObjects.Clear();
        loosePropItems.Clear();
        items.Clear();

        activeOrders.Clear();
        completedOrderCount = 0;
        itemDisplayCtr = 0;
        orderDisplayDirty = true;

        waitingForStart = true;
    }

    static void DestroyAll(IEnumerable<GameObject> objects)
    {
        foreach (GameObject obj in objects.ToList())
        {
            if (obj != null)
            {
                Destroy(obj);
            }
        }
    }

    // ---- status line ------------------------------------------------------

    void Update()
    {
        // Only rebuilt when something changed. The previous version assembled a
        // string every frame and threw it away.
        if (bottomtext == null || !orderDisplayDirty)
        {
            return;
        }

        bottomtext.text = BuildStatusLine();
        orderDisplayDirty = false;
    }

    string BuildStatusLine()
    {
        if (waitingForStart)
        {
            return "Waiting for simulator...";
        }

        StringBuilder line = new StringBuilder();
        line.Append("Active: ").Append(activeOrders.Count)
            .Append("   Done: ").Append(completedOrderCount);

        foreach (OrderView order in activeOrders)
        {
            line.Append("   | ").Append(order.Name)
                .Append(" p").Append(order.Priority)
                .Append(" [").Append(string.Join(",", order.Items)).Append(']');
        }
        return line.ToString();
    }

    // ---- item palette -----------------------------------------------------

    /// <summary>
    /// Lay the item prototypes out in a row alongside the warehouse, so they
    /// read as a legend of what each shape means.
    ///
    /// This used to be positioned with ScreenToWorldPoint and hard-coded pixel
    /// offsets, which put the row somewhere different at every resolution and
    /// ran off the screen once there were more than a handful of items.
    /// </summary>
    public void LayOutItemPalette(int warehouseWidth, int warehouseHeight)
    {
        if (itemObjects.Count == 0)
        {
            return;
        }

        float spacing = Mathf.Max(1.0f, (float)warehouseWidth / itemObjects.Count);
        float z = -2.0f;
        int index = 0;

        foreach (GameObject prototype in itemObjects.Values)
        {
            if (prototype == null)
            {
                continue;
            }
            prototype.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
            prototype.transform.position = new Vector3(0.5f + (index * spacing), 0.5f, z);
            index++;
        }
    }

    // ---- item geometry ----------------------------------------------------

    void createPolygonObj(int sides, int x, int y, string name)
    {
        GameObject newobj = new GameObject();
        newobj.name = "Item";

        var meshFilter = newobj.AddComponent<MeshFilter>();
        var meshRenderer = newobj.AddComponent<MeshRenderer>();

        meshRenderer.material = new Material(Shader.Find("Standard"));
        meshRenderer.material.color = GenerateRandomColor();
        meshFilter.mesh = createMesh(sides);

        newobj.transform.position = new Vector3(x, 0.5f, y);

        this.itemObjects.Add(name, newobj);
    }

    Mesh createMesh(int num_verts)
    {
        Mesh mesh = new Mesh();
        var vertices = new Vector3[2 * (num_verts + 1)];
        var uvs = new Vector2[2 * (num_verts + 1)];

        var tris = new int[(2 * num_verts * 3) + (num_verts * 3 * 2)];

        vertices[0] = new Vector3(0, 0, 0);

        for (int i = 0; i < num_verts; i++)
        {
            vertices[i + 1] = new Vector3((float)Math.Cos(((float)i / num_verts) * 2 * Math.PI), 0, (float)Math.Sin(((float)i / num_verts) * 2 * Math.PI));
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
            tris[i * 3] = 0;
            tris[i * 3 + 1] = i + 1;
            if (i + 2 <= num_verts)
            {
                tris[i * 3 + 2] = i + 2;
            }
            else
            {
                tris[i * 3 + 2] = 1;
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
