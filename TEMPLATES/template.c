//Template structure of sensor implementation in user-sensor.c file

... 

/*==============================================================*/
/* Local Function Prototypes					*/
/*==============================================================*/

void template_state_poll( unsigned char *arg );


/*==============================================================*/
/* USER SEMAPHORE INITIALIZATION				*/
/*==============================================================*/

void
semaphore_initialize( void )
{
	if (create_semaphore(*) < 0) 
	{
		logger("ERROR", "Semaphore initialization failed for sensor bus * ");
	}
}

/*==============================================================
 * USER SENSOR STATE POLL INITIALIZATION
 *==============================================================*/

void
user_sensor_state_poll( void )
{
	/*==============================================================*/
	/* State Poll Functions call					*/
	/*==============================================================*/

	template_state_poll( 0 );
}


/*==============================================================
 * USER MODULE SENSORS INITIALIZATION
 *==============================================================*/

void
user_module_sensor_init( void )
{
	/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	+	TEMPLATE Sensor					          +
	+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	sd[*].scan_function = template_callback_function;
}



/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+	TEMPLATE Sensor				 	    	  +
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void 
template_callback_function( void )
{

...

	sd[*].last_sensor_reading = *;		//result of sensor reading
	sd[*].sensor_scanning_enabled = 1;	//0b - disabled
	sd[*].event_messages_enabled = 1;	//0b - disabled
	sd[*].unavailable = 0;

...

}

void
template_state_poll( unsigned char *arg )
{
	unsigned char template_timer_handle;

	template_callback_function();	
	
	// Re-start the timer
	timer_add_callout_queue( (void *)&template_timer_handle,
		       	0*SEC, template_state_poll, 0 ); /* 0 sec timeout */
}


