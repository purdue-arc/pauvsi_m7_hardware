#include <ax12.h>

#define SetID(id, newid) (ax12SetRegister(id, AX_ID, newid))
#define SetMaxTorque(id, pos)(ax12SetRegister(id, AX_MAX_TORQUE_L, pos))

int new_id = 1;

void setup()
{
  new_id = 1;
}

void loop()
{
  for(int i=0; i<255; ++i)
  {
    SetID(i, new_id);
  }
  
  SetMaxTorque(new_id, 512); 
}
