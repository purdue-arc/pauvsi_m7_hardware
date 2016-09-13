void updateIndicatorLEDs()
{
  //determine whether to turn LED 13 on or off
  if(LED_13_State == LEDMODE_INITIALIZING)
  {
    if(millis() - LEDTimer[0] < INITIALIZING_BLINK_RATE)
    {
      digitalWrite(LED13PIN, HIGH);
    }
    else if(millis() - LEDTimer[0] < INITIALIZING_BLINK_RATE * 2)
    {
      digitalWrite(LED13PIN, LOW);
    }
    else
    {
      LEDTimer[0] = millis();
      digitalWrite(LED13PIN, HIGH);
    }
  }
  else if(LED_13_State == LEDMODE_ERROR)
  {
    if(millis() - LEDTimer[0] < ERROR_BLINK_RATE)
    {
      digitalWrite(LED13PIN, HIGH);
    }
    else if(millis() - LEDTimer[0] < ERROR_BLINK_RATE * 2)
    {
      digitalWrite(LED13PIN, LOW);
    }
    else
    {
      LEDTimer[0] = millis();
      digitalWrite(LED13PIN, HIGH);
    }
  }
}

void setLED13state(int state)
{
  LED_13_State = state;
}

void setLEDAstate(int state)
{
  LED_A_State = state;
}

void setLEDBState(int state)
{
  LED_B_State = state;
}

void setupIndicatorLEDs()
{
  pinMode(LED13PIN, OUTPUT);
}

