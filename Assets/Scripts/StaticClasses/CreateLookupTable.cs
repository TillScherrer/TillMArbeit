using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CreateLookupTable : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        int lookupSize = 100;
        float[] lookupTable = new float[lookupSize+1];
        lookupTable[0] = 1;

        string str = "float[] lookupTable = new float[]{1, ";
        for (int lIndex = 1; lIndex <= lookupSize; lIndex++)
        {

            int sampels = 1000;
            float gearImpact = (float)lIndex/lookupSize; //this is the value setten by user between 0 and 1
            float maxAccel = 1 + gearImpact;
            float maxMinAccel = 1;//1 / (2 - (1 / maxAccel));
            float minMinAccel = 0;
            float lastIterationTotalTimeSpend = -1;

            for (int j = 0; j < 100; j++)
            {
                float checkedMinAccel = (maxMinAccel + minMinAccel) * 0.5f;

                //float[] speeds = new float[sampels];
                float[] timeSpendThere = new float[sampels];
                for (float i = 0; i < sampels; i++)
                {
                    float timeRatio = i / ((float)sampels - 1);
                    float sampleAcceleration = Mathf.Lerp(checkedMinAccel, maxAccel, timeRatio);
                    timeSpendThere[(int)i] = 1f / sampleAcceleration / sampels;
                }
                float totalTimeSpend = 0;
                for (int i = 0; i < sampels; i++)
                {
                    totalTimeSpend += timeSpendThere[i];
                    //totalAverageAcceleration += speeds[i] * timeSpendThere[i]/sampels;
                }

                lastIterationTotalTimeSpend = totalTimeSpend;
                if (totalTimeSpend > 1)
                {
                    minMinAccel = checkedMinAccel;
                }
                else if (totalTimeSpend < 1)
                {
                    maxMinAccel = checkedMinAccel;
                }
                else
                {
                    break;
                }
            }
            //Debug.Log("minAccel is " + maxMinAccel + ", leading to a total time scale of " + lastIterationTotalTimeSpend + " ");
            lookupTable[lIndex] = maxMinAccel;
            str += (maxMinAccel.ToString().Replace(",", "."))+"f";
            if (lIndex < lookupSize) str += ", ";
        }
        str += "}";
        Debug.Log(str);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
