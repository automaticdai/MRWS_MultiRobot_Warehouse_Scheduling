using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Turns simulator messages into scene changes. Runs on the main thread.
/// </summary>
public class CommandExecutor : MonoBehaviour
{
    public GameObject MainScript;
    public GameObject Floor;

    [Tooltip("Log every message received. A run sends thousands, so this is " +
             "expensive enough to dominate frame time; leave it off unless debugging.")]
    public bool verboseLogging = false;

    private List<string> commandsToParse = new List<string>();
    private mainScript main;

    void Awake()
    {
        main = MainScript.GetComponent<mainScript>();
    }

    public void Update()
    {
        if (commandsToParse.Count == 0)
        {
            return;
        }

        foreach (string command in commandsToParse)
        {
            SimulationCommand cmd = parseJsonCommand(command);
            if (cmd.Command != CommandName.INVALIDCOMMAND)
            {
                executeCommand(cmd);
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
            Debug.LogWarning("Non-JSON message discarded: " + jsonCommand);
            return cmd;
        }

        if (verboseLogging)
        {
            Debug.Log(cmd);
        }

        cmd.VerifyCommand();
        return cmd;
    }

    public void executeCommand(SimulationCommand cmd)
    {
        // RESET has to pass the gate as well as START: it is what clears the
        // previous run, and the simulator sends it before starting the next.
        if (cmd.Command == CommandName.RESET)
        {
            main.ResetVisualisation();
            return;
        }

        if (cmd.Command == CommandName.START)
        {
            main.startVisualisation();
            return;
        }

        if (main.waitingForStart)
        {
            if (verboseLogging)
            {
                Debug.Log("Ignoring " + cmd.command + " before START");
            }
            return;
        }

        switch (cmd.Command)
        {
            case CommandName.WAREHOUSESIZE:
                Floor.GetComponent<plane>().setSize(cmd.posX, cmd.posY);
                main.LayOutItemPalette(cmd.posX, cmd.posY);
                break;

            case CommandName.CREATEROBOT:
                main.CreateRobot(cmd.objName, cmd.posX, cmd.posY);
                break;

            case CommandName.MOVEROBOT:
                {
                    robot target = main.FindRobot(cmd.objName);
                    if (target != null)
                    {
                        target.setRobotPosition(cmd.posX, cmd.posY);
                    }
                    break;
                }

            case CommandName.CREATESHELF:
                main.CreateShelf(cmd.objName, cmd.posX, cmd.posY, cmd.itemName);
                break;

            case CommandName.CREATEGOAL:
                main.CreateGoal(cmd.objName, cmd.posX, cmd.posY);
                break;

            case CommandName.ITEM:
                main.CreateItem(cmd.itemName);
                break;

            case CommandName.ITEMGAINED:
                ForInventory(cmd.objName,
                             r => r.addToInventory(cmd.itemName),
                             g => g.addToInventory(cmd.itemName));
                break;

            case CommandName.ITEMLOST:
                ForInventory(cmd.objName,
                             r => r.removeFromInventory(cmd.itemName),
                             g => g.removeFromInventory(cmd.itemName));
                break;

            case CommandName.CLEARINV:
                ForInventory(cmd.objName, r => r.clearInventory(), g => g.clearInventory());
                break;

            case CommandName.ORDERCREATE:
                main.CreateOrder(cmd.objName, cmd.prio, cmd.ItemNames());
                break;

            case CommandName.ORDERCOMPLETE:
                main.CompleteOrder(cmd.objName);
                break;

            case CommandName.ROBOTFAULT:
                {
                    robot target = main.FindRobot(cmd.objName);
                    if (target != null)
                    {
                        target.setFault(cmd.faultType);
                    }
                    break;
                }

            case CommandName.ROBOTRECOVERED:
                {
                    robot target = main.FindRobot(cmd.objName);
                    if (target != null)
                    {
                        target.clearFault();
                    }
                    break;
                }
        }
    }

    /// <summary>
    /// Inventory messages address robots and goals alike, and a message can
    /// arrive for something that no longer exists after a reset.
    /// </summary>
    void ForInventory(string objName, Action<robot> onRobot, Action<goal> onGoal)
    {
        if (objName.Contains("robot"))
        {
            robot target = main.FindRobot(objName);
            if (target != null)
            {
                onRobot(target);
            }
        }
        else if (objName.Contains("goal"))
        {
            goal target = main.FindGoal(objName);
            if (target != null)
            {
                onGoal(target);
            }
        }
    }

    public void addCommandToParse(string cmd)
    {
        commandsToParse.Add(cmd);
    }
}
