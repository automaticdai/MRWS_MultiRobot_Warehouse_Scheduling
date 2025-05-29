using System;
using UnityEngine;

public class plane : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        setSize(20, 10);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    // Starting size is 10x10
    public void setSize(int x, int y) 
    {
        float xScale = (float)x / 10;
        float yScale = (float)y / 10;
        float xShift = (float)x / 2 ;
        float yShift = (float)y / 2;

        this.transform.position = new Vector3(xShift, 0, yShift);
        this.transform.localScale = new Vector3(xScale, 1, yScale);

        int camdist;

        if (16*x > 9*y)
        {
            camdist = x/2 + x/10;
        }
        else
        {
            camdist = y/2 + y/10; 
        }

        GameObject.FindGameObjectWithTag("MainCamera").transform.position = new Vector3(x / 2, camdist, y / 2);




    }


}
