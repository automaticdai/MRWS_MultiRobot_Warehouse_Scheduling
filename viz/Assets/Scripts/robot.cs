using System.Collections.Generic;
using UnityEngine;

public class robot : MonoBehaviour
{
    // How fast the model slides towards the cell the simulator has put it in.
    // The simulator moves in discrete steps; interpolating between them is what
    // makes the visualisation read as movement rather than teleporting.
    public float moveSpeed = 6.0f;

    // Height the carried stack sits at, and the gap between items in it.
    const float CarryBaseHeight = 1.2f;
    const float CarryItemSpacing = 0.25f;
    const float CarryOffset = 0.25f;

    public List<GameObject> items = new();

    public mainScript MainScript;

    Vector3 targetPosition;
    Color baseColour = Color.white;
    string currentFault = "";

    void Awake()
    {
        targetPosition = transform.position;
    }

    void Update()
    {
        if (transform.position != targetPosition)
        {
            transform.position = Vector3.MoveTowards(
                transform.position, targetPosition, moveSpeed * Time.deltaTime);
        }
        PositionCarriedItems();
    }

    public void setReference(mainScript mainscript)
    {
        MainScript = mainscript;
    }

    /// <summary>Remembered so a fault tint can be undone on recovery.</summary>
    public void setBaseColour(Color colour)
    {
        baseColour = colour;
        ApplyColour();
    }

    public void setRobotPosition(int x, int y)
    {
        targetPosition = new Vector3(x + 0.5f, 0.5f, y + 0.5f);
    }

    /// <summary>Put the model where the simulator says, with no animation.</summary>
    public void snapToPosition(int x, int y)
    {
        targetPosition = new Vector3(x + 0.5f, 0.5f, y + 0.5f);
        transform.position = targetPosition;
        PositionCarriedItems();
    }

    public void setFault(string faultType)
    {
        currentFault = faultType;
        ApplyColour();
    }

    public void clearFault()
    {
        currentFault = "";
        ApplyColour();
    }

    public bool IsFaulted()
    {
        return currentFault != "";
    }

    void ApplyColour()
    {
        Renderer rend = GetComponent<Renderer>();
        if (rend == null)
        {
            return;
        }
        rend.material.color = FaultColour(currentFault, baseColour);
    }

    /// <summary>
    /// Hue encodes fault state, matching the debug GUI: red for a permanent
    /// battery failure, amber for one that recharges, orange for actuators,
    /// purple for sensors.
    /// </summary>
    public static Color FaultColour(string faultType, Color healthy)
    {
        switch (faultType)
        {
            case "battery_critical": return new Color(0.94f, 0.27f, 0.27f);
            case "battery_low": return new Color(0.96f, 0.62f, 0.04f);
            case "actuator": return new Color(0.98f, 0.57f, 0.24f);
            case "sensor": return new Color(0.66f, 0.33f, 0.97f);
            default: return healthy;
        }
    }

    void PositionCarriedItems()
    {
        for (int i = 0; i < items.Count; i++)
        {
            if (items[i] == null)
            {
                continue;
            }
            items[i].transform.position = new Vector3(
                transform.position.x + CarryOffset,
                CarryBaseHeight + (i * CarryItemSpacing),
                transform.position.z + CarryOffset);
        }
    }

    public void addToInventory(string itemName)
    {
        GameObject prototype = MainScript.ItemPrototype(itemName);
        if (prototype == null)
        {
            return;
        }

        GameObject itemClone = Instantiate(prototype, transform.position, transform.rotation);
        itemClone.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
        items.Add(itemClone);
        PositionCarriedItems();
    }

    public void removeFromInventory(string itemName)
    {
        // The simulator's inventory is a LIFO stack, so the item leaving is
        // always the one on top; the name is not needed to identify it.
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
