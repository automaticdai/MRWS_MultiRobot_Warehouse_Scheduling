using System.Collections.Generic;
using UnityEngine;

public class robot : MonoBehaviour
{
    public List<GameObject> items = new();

    public mainScript MainScript;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void setReference(mainScript mainscript) 
    { 
        MainScript = mainscript;
    }

    public void setRobotPosition(int x, int y) 
    {
        this.transform.position = new Vector3(x + 0.5f, 0.5f, y + 0.5f);
        foreach (var item in items) 
        {
            item.transform.position = new Vector3(x + 0.75f, item.transform.position.y, y + 0.75f);
        }
    }

    public void addToInventory(string itemName) 
    {
        GameObject itemClone = Instantiate(MainScript.itemObjects[itemName], new Vector3(0.25f + this.transform.position.x , 3.0f + (items.Count * 0.1f), 0.25f + this.transform.position.z), this.transform.rotation);
        itemClone.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
        items.Add(itemClone);
    }

    public void removeFromInventory(string itemName) 
    {
        items.RemoveAt(items.Count - 1);
    }


    public void clearInventory() 
    {
        foreach (var item in items) 
        {
            GameObject.Destroy(item);
        }
        items.Clear();
    }

}
