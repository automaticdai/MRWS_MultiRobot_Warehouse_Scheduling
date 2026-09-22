using System.Collections.Generic;
using UnityEngine;

public class goal : MonoBehaviour
{
    const float StackBaseHeight = 1.0f;
    const float StackItemSpacing = 0.25f;
    const float StackOffset = 0.25f;

    public List<GameObject> items = new();

    public mainScript MainScript;

    public void setReference(mainScript mainscript)
    {
        MainScript = mainscript;
    }

    public void addToInventory(string itemName)
    {
        GameObject prototype = MainScript.ItemPrototype(itemName);
        if (prototype == null)
        {
            return;
        }

        GameObject itemClone = Instantiate(
            prototype,
            new Vector3(transform.position.x - StackOffset,
                        StackBaseHeight + (items.Count * StackItemSpacing),
                        transform.position.z - StackOffset),
            transform.rotation);
        itemClone.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
        items.Add(itemClone);
    }

    public void removeFromInventory(string itemName)
    {
        // LIFO, like the simulator's own inventory: the item leaving is the
        // one on top, so its name is not needed to find it.
        if (items.Count == 0)
        {
            return;
        }
        GameObject top = items[items.Count - 1];
        items.RemoveAt(items.Count - 1);
        if (top != null)
        {
            Destroy(top);
        }
    }

    public void clearInventory()
    {
        foreach (var item in items)
        {
            if (item != null)
            {
                Destroy(item);
            }
        }
        items.Clear();
    }
}
