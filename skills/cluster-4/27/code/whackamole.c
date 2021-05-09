typdef enum {			// Set of states enumerated
       STATE_0,         // Starting state
       STATE_1,         // State where user tries to hit the mole
       STATE_2,         // state where the score is incremented
       MAX_STATES
       } state_e;

typedef enum {			// Set of events enumerated
	POP_MOLE_EVENT,                 // A mole is popped up for user to hit
    USER_MISS_TIME_LEFT_EVENT,      // User misses the mole but still has time left to try
    USER_MISS_NO_TIME_EVENT,        // User misses the mole and has no time left to try
    USER_HITS_MOLE_EVENT,           // User successfully hits the mole
    START_AGAIN_EVENT,              // Score has been updated
	MAX_EVENTS
	} event_e;

state_e state = STATE_0;	// Starting state
state_e next_state;
event_e event;

while(1)			// When event occurs, event handler does some stuff
{
	event = read_event();
	if (state == STATE_0)
	{
		if (event == POP_MOLE_EVENT)
		{
			next_state = try_to_hit_mole_event_handler();   //STATE_1
		}
    }
	else if (state == STATE_1)
	{
		if (event == USER_MISS_TIME_LEFT_EVENT)
		{
			next_state = try_to_hit_mole_event_handler();   //STATE_1
		}
        else if (event == USER_MISS_NO_TIME_EVENT)
		{
			next_state = start_event_handler();             //STATE_0
		}
        else if (event == USER_HITS_MOLE_EVENT)
        {
            next_state = update_score_event_handler();      //STATE_2
        }
    }
	else if (state == STATE_2)
	{
		if (event == START_AGAIN_EVENT)
		{
			next_state == start_event_handler();            //STATE_0
		}
	}
	state = next_state;
}