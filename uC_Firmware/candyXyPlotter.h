void init_steppers();
/*
void enable_stepper(short stepperAttrArrayEl);
void disable_stepper(short stepperAttrArrayEl);
void disable_all_steppers();
*/
void stop_all_movements();
void goto_machine_zero();
void move_to_min(short stepperArrayEl);
void do_step();
short reached_target_or_end(short stepperAttrArrayEl);
bool read_switch(byte pin);
void calcCurrentPositionInMm(short stepperAttrArrayEl);
short determineMovementForNewTargetPos(short stepperAttrArrayEl);
void init_process_string();
bool has_command(char key, char instruction[], int string_size);
double search_string(char key, char instruction[], int string_size);
void process_string(char instruction[], int size);