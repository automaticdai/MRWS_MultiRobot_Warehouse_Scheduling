using System;
using System.Collections.Generic;
using UnityEngine;
public class CommandExecutor : MonoBehaviour
{
    public GameObject MainScript;
    public GameObject Floor;

    private List<string> commandsToParse = new List<string>();

    public void Update()
    {
        foreach (string command in commandsToParse) 
        {
            SimulationCommand cmd = parseJsonCommand(command);
            if (cmd.Command != CommandName.INVALIDCOMMAND)
            {
                executeCommand(cmd);
                //Debug.Log("COMMANDS LENGTH WAS " + commandsToParse.Count);
            }
            else 
            {
                Debug.Log("Skipping invalid command");
            }
        }
        commandsToParse.Clear();
    }

    public SimulationCommand parseJsonCommand(string jsonCommand)
    {
        SimulationCommand cmd = new SimulationCommand();
        try
        {
            JsonUtility.FromJsonOverwrite(jsonCommand, cmd);
        }
        catch (ArgumentException) 
        {
            Debug.Log("NON JSON COMMAND DETECTED");
            return cmd;
        }

        Debug.Log(cmd);
        
        cmd.VerifyCommand();

        if (cmd.Command == CommandName.INVALIDCOMMAND) 
        {
            Debug.Log("INVALID COMMAND DETECTED");
        }
        return cmd;
    }

    public void executeCommand(SimulationCommand cmd)
    {
        if (cmd.Command == CommandName.START)
        {
            MainScript.GetComponent<mainScript>().startVisualisation();
        }
        if (MainScript.GetComponent<mainScript>().waitingForStart)
        {
            Debug.Log("COMMAND IGNORED, SEND START");
            return;
        }
        if (cmd.Command == CommandName.CREATEROBOT)
        {
            MainScript.GetComponent<mainScript>().CreateRobot(cmd.objName, cmd.posX, cmd.posY);
        }
        else if (cmd.Command == CommandName.WAREHOUSESIZE)
        {
            Floor.GetComponent<plane>().setSize(cmd.posX, cmd.posY);
            MainScript.GetComponent<mainScript>().correctScreenPosition();
        }
        else if (cmd.Command == CommandName.MOVEROBOT)
        {
            MainScript.GetComponent<mainScript>().robotDict[cmd.objName].GetComponent<robot>().setRobotPosition(cmd.posX, cmd.posY);
        }
        else if (cmd.Command == CommandName.CREATESHELF)
        {
            MainScript.GetComponent<mainScript>().CreateShelf(cmd.objName, cmd.posX, cmd.posY, cmd.itemName);
        }
        else if (cmd.Command == CommandName.CREATEGOAL)
        {
            MainScript.GetComponent<mainScript>().CreateGoal(cmd.objName, cmd.posX, cmd.posY);
        }
        else if (cmd.Command == CommandName.ITEM)
        {
            MainScript.GetComponent<mainScript>().CreateItem(cmd.itemName);
        }
        else if (cmd.Command == CommandName.ITEMGAINED)
        {
            if (cmd.objName.Contains("robot"))
            {
                MainScript.GetComponent<mainScript>().robotDict[cmd.objName].GetComponent<robot>().addToInventory(cmd.itemName);
            }
            else if (cmd.objName.Contains("goal"))
            {
                MainScript.GetComponent<mainScript>().goalDict[cmd.objName].GetComponent<goal>().addToInventory(cmd.itemName);
            }
        }
        else if (cmd.Command == CommandName.ITEMLOST)
        {
            if (cmd.objName.Contains("robot"))
            {
                MainScript.GetComponent<mainScript>().robotDict[cmd.objName].GetComponent<robot>().removeFromInventory(cmd.itemName);
            }
            else if (cmd.objName.Contains("goal"))
            {
                MainScript.GetComponent<mainScript>().goalDict[cmd.objName].GetComponent<goal>().removeFromInventory(cmd.itemName);
            }
        }



        else if (cmd.Command == CommandName.CLEARINV) 
        {
            if (cmd.objName.Contains("robot"))
            {
                MainScript.GetComponent<mainScript>().robotDict[cmd.objName].GetComponent<robot>().clearInventory();
            }
            else if (cmd.objName.Contains("goal"))
            {
                MainScript.GetComponent<mainScript>().goalDict[cmd.objName].GetComponent<goal>().clearInventory();
            }

        }



}

public void addCommandToParse(string cmd) 
    {
        //Debug.Log("adding command");
        commandsToParse.Add(cmd);
    }




}
