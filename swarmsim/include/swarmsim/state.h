#ifndef state_h
#define state_h
enum States { Idle = 0, Ready, Armed, Takingoff, Autonomous, Reached };
enum Phases {Planning = 0, Optimization, Execution };
#endif

/**
 * need for an interim optimization phase: not to block the main thread for 
 * optimization to complete. Optimization will be invoked at optimization phase and 
 * will gather the "future" in the execution phase 
*/
