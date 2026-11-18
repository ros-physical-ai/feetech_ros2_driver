std_msgs/Header header

# Arrays align in tandem; elements correspond to the respective servo index in id[]
int32 comm_state
int32[] id
bool[] torque_state
int32[] hw_state
int16[] present_temperature   # Unit: Celsius
int16[] present_input_voltage # Unit: 0.1 V (121 means 12.1 V)
int16[] present_current       # Unit: mA
int16[] present_load          # Unit: 0.1%

int32 COMM_STATE_OK = 0
int32 COMM_STATE_ITEM_READ_FAIL = -5
int32 COMM_STATE_HARDWARE_ERROR = -16
