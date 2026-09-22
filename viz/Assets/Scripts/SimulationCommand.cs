using System;
using System.Collections.Generic;
using UnityEngine;

public enum CommandName
{
    INVALIDCOMMAND,
    START,
    RESET,
    WAREHOUSESIZE,

    CREATEROBOT,
    MOVEROBOT,

    CREATESHELF,

    CREATEGOAL,
    ITEM,
    ITEMGAINED,
    ITEMLOST,
    CLEARINV,

    ORDERCREATE,
    ORDERCOMPLETE,

    ROBOTFAULT,
    ROBOTRECOVERED
}

[System.Serializable]
public class SimulationCommand
{
    public CommandName Command;

    public string command;
    public string objName;

    public int posX;
    public int posY;

    public string itemName;

    // Order priority. Deliberately an int: this used to arrive as a quoted
    // string in posX, which JsonUtility cannot map onto an int field, so the
    // value never parsed.
    public int prio;

    // One of battery_critical, battery_low, actuator, sensor.
    public string faultType;

    // Static so JsonUtility never sees them and every command does not
    // allocate four lists just to be validated.
    static readonly HashSet<CommandName> NeedsPosition = new()
    {
        CommandName.WAREHOUSESIZE, CommandName.CREATEROBOT,
        CommandName.CREATESHELF, CommandName.CREATEGOAL
    };

    static readonly HashSet<CommandName> NeedsObjName = new()
    {
        CommandName.CREATEROBOT, CommandName.MOVEROBOT,
        CommandName.CREATESHELF, CommandName.CREATEGOAL,
        CommandName.ITEMGAINED, CommandName.ITEMLOST, CommandName.CLEARINV,
        CommandName.ORDERCREATE, CommandName.ORDERCOMPLETE,
        CommandName.ROBOTFAULT, CommandName.ROBOTRECOVERED
    };

    static readonly HashSet<CommandName> NeedsItemName = new()
    {
        CommandName.CREATESHELF, CommandName.ITEM,
        CommandName.ITEMGAINED, CommandName.ITEMLOST, CommandName.ORDERCREATE
    };

    static readonly HashSet<CommandName> NeedsFaultType = new()
    {
        CommandName.ROBOTFAULT
    };

    public SimulationCommand()
    {
        posX = -1;
        posY = -1;
        prio = -1;
        objName = "";
        itemName = "";
        faultType = "";
    }

    public void VerifyCommand()
    {
        if (!Enum.TryParse(command, out Command))
        {
            Debug.LogWarning("Unknown command from simulator: " + command);
            Command = CommandName.INVALIDCOMMAND;
            return;
        }

        if (NeedsPosition.Contains(Command) && (posX == -1 || posY == -1))
        {
            Reject("requires position inputs");
            return;
        }

        if (NeedsObjName.Contains(Command) && objName == "")
        {
            Reject("requires object name");
            return;
        }

        if (NeedsItemName.Contains(Command) && itemName == "")
        {
            Reject("requires item name");
            return;
        }

        if (NeedsFaultType.Contains(Command) && faultType == "")
        {
            Reject("requires fault type");
            return;
        }
    }

    void Reject(string because)
    {
        Debug.LogWarning("Invalid command - " + command + " " + because);
        Command = CommandName.INVALIDCOMMAND;
    }

    /// <summary>Items of an ORDERCREATE, which arrive pipe-delimited.</summary>
    public string[] ItemNames()
    {
        return itemName.Split('|');
    }

    override public string ToString()
    {
        return "Command Object: Name: " + command + " Item Name " + itemName
             + " X: " + posX + " Y: " + posY + " objName " + objName;
    }
}
