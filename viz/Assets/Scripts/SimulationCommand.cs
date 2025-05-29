using System;
using System.Collections.Generic;
using System.Xml.Xsl;
using UnityEngine;

public enum CommandName 
{ 
    INVALIDCOMMAND,
    START, 
    RESET,
    WAREHOUSESIZE,
    
    CREATEROBOT,
    MOVEROBOT,
    ROBOTPICKUP,
    ROBOTPUTDOWN,

    CREATESHELF,

    CREATEGOAL,
    ITEM,
    ITEMGAINED,
    ITEMLOST,
    CLEARINV
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


    public List<CommandName> needsPosition = new(){ CommandName.WAREHOUSESIZE, CommandName.CREATEROBOT, CommandName.CREATESHELF, CommandName.CREATEGOAL };
    public List<CommandName> needsObjName = new() { CommandName.CREATEROBOT, CommandName.MOVEROBOT, CommandName.ROBOTPICKUP, CommandName.ROBOTPUTDOWN, CommandName.CREATESHELF, CommandName.CREATEGOAL, CommandName.ITEMGAINED, CommandName.ITEMLOST, CommandName.CLEARINV};
    public List<CommandName> needsItemName = new() { CommandName.CREATESHELF, CommandName.ITEM, CommandName.ITEMGAINED, CommandName.ITEMLOST};


    public SimulationCommand() 
    {
        posX = -1;
        posY = -1;
        objName = "";
        itemName = "";
    }


    public void VerifyCommand()
    {
        if (!Enum.TryParse(command, out Command))
        {
            Debug.Log("Invalid Command");
            return;
        }

        if (needsPosition.Contains(Command) && (posX == -1 || posY == -1)) 
        {
            Debug.Log("Invalid Command - Command " + command + " requires position inputs");
            this.Command = CommandName.INVALIDCOMMAND;
            return;
        }

        if (needsObjName.Contains(Command) && objName == "") 
        {
            Debug.Log("Invalid Command - Command " + command + " requires object name");
            this.Command = CommandName.INVALIDCOMMAND;
            return;
        }

        if (needsItemName.Contains(Command) && itemName == "")
        {
            Debug.Log("Invalid Command - Command " + command + " requires item name");
            this.Command = CommandName.INVALIDCOMMAND;
            return;
        }
    }

    override public string ToString() 
    {
        return "Command Object: Name: " + command + " Item Name " + itemName + " X: " + posX + " Y: " + posY + " objName " + objName;
    }


}