# A message intended to send the motor status
float32   position
float32   velocity
float32   effort
uint32    state

bool      is_operational
string    error_msg

string[]  reg_names
float32[] reg_values

string[]  flag_names
bool[]    flag_values

string[] digital_in_names
bool[] digital_in_states
string[] safety_names
bool[] safety_states