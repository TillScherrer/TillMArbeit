using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[Serializable]

public class FadingKey
{
    float value = 0f ;
    bool isPressed = false;
    [Tooltip("Pressing this key does not have its full effect immediately. Its effect ramps up continously to 100% within the increasement time if the key is pressed and falls off from 100% to 0% within the decreasement time. This leads to smoother controll and mimics how real car drivers act")]
    [SerializeField] KeyCode keyboardInput;
    [Tooltip("Pressing this key does not have its full effect immediately. Its effect ramps up continously to 100% within the increasement time if the key is pressed and falls off from 100% to 0% within the decreasement time. This leads to smoother controll and mimics how real car drivers act")]
    [Range(0,0.6f)]
    [SerializeField] float increasementTime = 0.2f;
    [Tooltip("Pressing this key does not have its full effect immediately. Its effect ramps up continously to 100% within the increasement time if the key is pressed and falls off from 100% to 0% within the decreasement time. This leads to smoother controll and mimics how real car drivers act")]
    [Range(0, 0.6f)]
    [SerializeField] float decreasementTime = 0.15f;

    public float Value { get => value;}
    public bool IsPressed { get => isPressed; set => isPressed = value; }
    public KeyCode KeyboardInput { get => keyboardInput;}


    // Update is called once per frame
    public void Update()
    {
        if (IsPressed)
        {
            value += Time.deltaTime/ increasementTime;
            if(value > 1) value = 1;
        }
        else
        {
            value -= Time.deltaTime/decreasementTime;
            if(value < 0) value = 0;
        }
    }
}
