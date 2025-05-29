using System.Collections.Generic;
using UnityEngine;

public class goal : MonoBehaviour 
{

    public List<GameObject> items = new();

    public mainScript MainScript;

    public void addToInventory(string itemName)
    {
        GameObject itemClone = Instantiate(MainScript.itemObjects[itemName], new Vector3(this.transform.position.x - 0.25f, 1.0f + (items.Count * 0.1f), this.transform.position.z - 0.25f), this.transform.rotation);
        itemClone.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
        items.Add(itemClone);
    }

    public void removeFromInventory(string itemName)
    {
        items.RemoveAt(items.Count - 1);
    }

    public void setReference(mainScript mainscript)
    {
        MainScript = mainscript;
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