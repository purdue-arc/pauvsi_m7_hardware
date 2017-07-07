#include <ax12.h>

#define SetID(id, newid) (ax12SetRegister(id, AX_ID, newid))

void setup()
{
}

void loop()
{
  for(int i=0; i<255; ++i)
  {
    SetID(i, 1);
  }
}
