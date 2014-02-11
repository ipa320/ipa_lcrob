/*******************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Quirl  (Quelle: Raser) 
********************************************************************************
********************************************************************************
***  Autor: Winfried Baum, Alejandro Merello, Maik Siee
*******************************************************************************/
/** \file scheduler.c
 *
 * Scheduler's setup and implementation
 *
 *  Version: 1.0
 */

#include "config.h"
#include "params.h"
#include "scheduler.h"



static float distRefl[256] = {
  0.3000, 0.3000, 0.3000, 0.3000, 0.3000, 0.3000, 0.3000, 0.3000,
  0.3000, 0.3000, 0.3000, 0.3000, 0.3000, 0.3000, 0.3000, 0.3000,
  0.2892, 0.2791, 0.2705, 0.2617, 0.2551, 0.2469, 0.2420, 0.2321,
  0.2265, 0.2169, 0.2050, 0.2065, 0.1999, 0.1953, 0.1897, 0.1863,
  0.1819, 0.1777, 0.1741, 0.1706, 0.1662, 0.1622, 0.1586, 0.1552,
  0.1519, 0.1487, 0.1457, 0.1424, 0.1385, 0.1340, 0.1310, 0.1281,
  0.1253, 0.1229, 0.1206, 0.1184, 0.1162, 0.1141, 0.1121, 0.1100,
  0.1076, 0.1052, 0.1033, 0.1014, 0.0996, 0.0980, 0.0964, 0.0948,
  0.0933, 0.0918, 0.0903, 0.0890, 0.0876, 0.0863, 0.0850, 0.0836, 
  0.0823, 0.0810, 0.0797, 0.0787, 0.0776, 0.0766, 0.0755, 0.0745,
  0.0734, 0.0724, 0.0713, 0.0703, 0.0694, 0.0686, 0.0678, 0.0670,
  0.0662, 0.0654, 0.0645, 0.0637, 0.0629, 0.0621, 0.0613, 0.0604,
  0.0597, 0.0590, 0.0584, 0.0577, 0.0571, 0.0564, 0.0558, 0.0551,
  0.0546, 0.0540, 0.0534, 0.0529, 0.0523, 0.0518, 0.0512, 0.0507,
  0.0501, 0.0495, 0.0490, 0.0484, 0.0479, 0.0473, 0.0468, 0.0462,
  0.0457, 0.0451, 0.0447, 0.0442, 0.0438, 0.0433, 0.0429, 0.0424,
  0.0420, 0.0415, 0.0411, 0.0406, 0.0402, 0.0397, 0.0393, 0.0388,
  0.0384, 0.0379, 0.0375, 0.0370, 0.0366, 0.0361, 0.0357, 0.0352,
  0.0345, 0.0335, 0.0326, 0.0316, 0.0307, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300,
  0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300, 0.0300
};



/**Scheduler layer 0: 
 */
void sched_layer0(void)
{
}

/**Scheduler layer 1: Not used
 */
void sched_layer1(void)
{

}

/**Scheduler layer 2: Runs the ADC Converter, IR Sensors, Motor Control and Buttons
 */
void sched_layer2(void)
{	
	adc_cycle();					//Reads ADC input

	//if(!hullStop()) fallStop();			//Fall avoidance if the hull is not being manually directed

	vrad_ctrl(soll_v, v_phi);			//Wheel motion control
	
	//odo_cycle();					//Performs odometry update
	
	if(!START_BUTTON)				//Start Button Actions
	{
		power = STATE_ON;
		x0 = ix; y0 = iy; phi0 = iphi;			//Sets the reference point for line
		//usart1_sendbyte(TB_AUTO);			//Updates the state in the cell phone
		
	}
	if(!STOP_BUTTON)				//stop Button Actions
	{
		power = STATE_OFF;
		if(poweroff_count++ >= STOP_BUTTON_OFF_TIME) POWER=OFF;		//If pressed for STOP_BUTTON_OFF_TIME [cs] Power's Off
		//usart1_sendbyte(TB_AUTO);					//Updates the state in the cell phone
	}
	else poweroff_count = 0;						//Resets Power Off Timer

}

/**Scheduler layer 3: Runs the State Machine
 */
void sched_layer3(void)
{	

/*
	USrange(ALL_SENSORS);
		sprintf(s,"Sensor0 %d\n",r0[1]);
		usart0_puts(s);
		usart0_sendbyte(0x0D);
*/

	IRrange();
		sprintf(s,"Sensor1 %d\n",r1[1]);
		usart0_puts(s);
		usart0_sendbyte(0x0D);



		r2[1] = r1[1] >> 2;
		sprintf(s,"shift %d\n",r2[1]);
		usart0_puts(s);
		usart0_sendbyte(0x0D);

		r2[1] = distRefl[r1[1] >> 2] * 100;

		sprintf(s,"cm %d\n\n",r2[1]);
		usart0_puts(s);
		usart0_sendbyte(0x0D);




	if(!CTR_FORW)  direction = FORW;	
	if(!CTR_BACK)  direction = BACK;	
	if(!CTR_LEFT)  direction = LEFT;	
	if(!CTR_RIGHT) direction = RIGHT;	
	if(!CTR_STOP)  direction = STOP;

	switch(direction)
	{
		case FORW:	soll_v =  SPEED_FW;	v_phi = 0;		break;
		case BACK:	soll_v = -SPEED_FW;	v_phi = 0;		break;
		case LEFT:	soll_v =  SPEED_FW;	v_phi =  D_90_GRAD;	break;
		case RIGHT:	soll_v =  SPEED_FW;	v_phi = -D_90_GRAD;	break;
		case STOP:	soll_v = 0;		v_phi = 0;		break;
	}



	/*
	if(!CTR_STOP)				
	{
		vel = 0;
		direction = STOP;
		mot_ctrl(direction, vel);
	}


	if(ButtActL == 0 && !CTR_LEFT)		//button is pressed (rising edge)
	{
		delay(20);			//bouncing obstacle
 		ButtActL = 1;		
			sprintf(s,"\n button is pressed (rising edge) %d\n",ButtActL);
			usart0_puts(s);
			usart0_sendbyte(0x0D);
	}
	else if (ButtActL == 1 &&  CTR_LEFT)  ButtActL = 0;	//debouncing pass 
	else if (ButtActL == 1 && !CTR_LEFT)  ButtActL = 2;	//button is held
	else if (ButtActL == 2 &&  CTR_LEFT)  ButtActL = 3;	//button is released (falling edge)
 	else if (ButtActL == 3 &&  CTR_LEFT)  ButtActL = 0;	//button released

		
		if(ButtActL == 2 && !CTR_FORW)				
		{
			direction = CIRCLE;
			mot_ctrl(direction, 511);
			delay(100);
		}

		else if(ButtActL == 2 && !CTR_BACK)				
		{
			direction = LEFT;
			mot_ctrl(direction, 511);
			delay(100);
		}

		else if (ButtActL == 3 && CTR_FORW && CTR_BACK )
		{
			direction = LEFT;
			mot_ctrl(direction, 511);
		}





	if(ButtActR == 0 && !CTR_RIGHT)	//button is pressed (rising edge)
	{
		delay(20);			//bouncing obstacle
 		ButtActR = 1;		
			sprintf(s,"\n button is pressed (rising edge) %d\n",ButtActR);
			usart0_puts(s);
			usart0_sendbyte(0x0D);
	}
	else if (ButtActR == 1 &&  CTR_RIGHT)  ButtActR = 0;	//debouncing pass 
	else if (ButtActR == 1 && !CTR_RIGHT)  ButtActR = 2;	//button is held
	else if (ButtActR == 2 &&  CTR_RIGHT)  ButtActR = 3;	//button is released (falling edge)
 	else if (ButtActR == 3 &&  CTR_RIGHT)  ButtActR = 0;	//button released

		
		if(ButtActR == 2 && !CTR_FORW)				
		{
			direction = CIRCLE;
			mot_ctrl(direction, -511);
			delay(100);

			sprintf(s,"\n direction = CIRCLE %d\n",ButtActR);
			usart0_puts(s);
			usart0_sendbyte(0x0D);
		}

		else if(ButtActR == 2 && !CTR_BACK)				
		{
			sprintf(s,"\n direction = neg RIGHT %d\n",ButtActR);
			usart0_puts(s);
			usart0_sendbyte(0x0D);

			direction = RIGHT;
			mot_ctrl(direction, -511);
			delay(100);
		}

		else if (ButtActR == 3 && CTR_FORW && CTR_BACK )
		{
			direction = RIGHT;
			mot_ctrl(direction, 511);
			
			sprintf(s,"\n direction = pos RIGHT %d\n",ButtActR);
			usart0_puts(s);
			usart0_sendbyte(0x0D);
		}
*/
	 
	//check battery state 
	battState();




/*
	//STATE MACHINE
	//(The computation of the current state and the actions to be taken in the given state are programmed separately
	//purely for simpler programming and debugging purposes. In the first section no modifications in the behavior of
	//the robot are made. In the second section no modifications in the robot's state are made.)		
	
	//Compute current state
	switch(power)
	{
		case STATE_ON:
			//delta[0] = delta[1];					//Save old delta.
			//delta[1] = -(sin(phi0)*(ix-x0)-cos(phi0)*(iy-y0));	//The distance of the robot to the line.
			switch(obstacle)
			{
				case !STATE_OBSTACLE_DETECTED:			 			//If no obstacle is detected
					range(FRONT_SENSORS);						//Ranges the frontal sensors sequentially
					if((r0[1]< US_SENSOR_TRIGGER)||(r1[1]< US_SENSOR_TRIGGER))	//If either of the frontal US Sensors finds an obstacle
					{
						obstacle=STATE_OBSTACLE_DETECTED;		
						us_unit2=!STATE_SEEN_BY_UNIT2;			//Obstacle in front hasn't been detected by SFR10_UNIT_2
						phi_obstacle=iphi;
						x1=ix;y1=iy;					//Stores the current position and orientation of the robot on the line
					}
					break;
				case STATE_OBSTACLE_DETECTED:					//If obstacle is detected.
					range(ALL_SENSORS);					//Ranges all the sensors sequentially.
					switch(us_unit2)
					{
						case !STATE_SEEN_BY_UNIT2: 
								if((r2[1]< US_SENSOR_TRIGGER)||(iphi-phi_obstacle>D_90_GRAD)||(iphi-phi_obstacle<-D_270_GRAD)) us_unit2=STATE_SEEN_BY_UNIT2;	
								//If the lateral sensor detects the obstacle or already rotated 90° continue, lateral sensor missed obstacle
							break;
						case STATE_SEEN_BY_UNIT2:
							switch(line2)
							{
								case !STATE_FURTHER_POINT_OF_LINE:
										if((sqrt(pow(ix-x1,2)+pow(iy-y1,2))>=100)&&(delta[0]*delta[1]<0.0)) 
										//Determines if the robot is on a further point of the line			
										{		//If it's not too close to the point where the obstacle was detected
											line2=STATE_FURTHER_POINT_OF_LINE;	//and crossed the line					
										}
										else if((r0[1]< US_SENSOR_TRIGGER)||(r1[1]< US_SENSOR_TRIGGER)) { us_unit2=!STATE_SEEN_BY_UNIT2; phi_obstacle=iphi; perpendicular=0; }			//Another obstacle detected while avoiding one
											else if( (delta[1]>70) && ( ( (phi0<=-D_85_GRAD)&&(isAngle(D_265_GRAD+phi0)) ) || ( (phi0>-D_85_GRAD)&&(isAngle(phi0-D_95_GRAD)) ) ) ) perpendicular = 1;	//If the robot is pointing in the direction of the line and it's not to close it goes straight for it,		
									break;					//instead of sorrounding the obstacles.
								case STATE_FURTHER_POINT_OF_LINE:		//If the robot is in a further point of the line 
									if(!isAngle(phi0))			//it has successfully avoided the obstacles. Then the 
										{angle=!STATE_ALIGNED;}		//state variable 'angle' contains the relation of the 
									else					//current angle of the robot to the angle of the line
									{
										stateReset();			//In this state the robot is in a further point of the line and currently in the
									}					//orientation of the line it should follow, meaning that the obstacle has
									break;					//been successfully avoided and can carry on the straight line
							}
							break;
					}
					break;
					
			}
			break;
		case STATE_OFF:
			stateReset();	//Reset all states and variables
			break;
	}

	//Takes action for the current state
	switch(power)
	{
		case STATE_ON:
			if(hullStop()) {soll_v = 0; v_phi = 0; vrad_ctrl(soll_v, v_phi); power = STATE_OFF; break;} //Stops if it percieves a disturbance in the hall effect force 
			switch(obstacle)
			{
				case !STATE_OBSTACLE_DETECTED:	//Follows the line.
					if(soll_v < SPEED_FW*.4) soll_v = SPEED_FW*.4; else if(soll_v < SPEED_FW) soll_v+= SPEED_FW/15; //Accelerate to SPEED_FW (Don't start lower than 40%)
					if(soll_v > SPEED_FW) soll_v = SPEED_FW;
					if(((r0[1]< US_SENSOR_DISTANT)||(r1[1]< US_SENSOR_DISTANT))&&(soll_v>SPEED_FW*.5)) soll_v = SPEED_FW*.5;
					v_phi = -(iphi-phi0);		// Near pi angle dealt with in mot.c with trigonometric functions.
					break;
				case STATE_OBSTACLE_DETECTED:
					switch(us_unit2)
					{
						case !STATE_SEEN_BY_UNIT2:
							soll_v = SPEED_OB; v_phi = D_90_GRAD;		// Rotates anti-clockwise
							break;
						case STATE_SEEN_BY_UNIT2:
							switch(line2)
							{	
								case !STATE_FURTHER_POINT_OF_LINE:
									if(!perpendicular)
									{	
										//###########################################		
										//P distance controller for object avoidance
										Px = AVOIDANCE_P;						//P Controller constant
										soll_x = AVOIDANCE_REF_DIST;					//Reference distance
										e_x = soll_x - (double)(r2[0]+r2[1])/2;				//Difference between Reference and Measurement
										if(fabs(delta[1])<50.0) soll_v = (int) ((double)SPEED_OB*fabs(delta[1])/125+.6*(double)SPEED_OB); else soll_v=SPEED_OB;
									//As it aproaches the line reduces it's speed from 100% at 5[cm] to 60% at 0 [cm] and vice versa when it moves away from the line.
										v_phi = Px * e_x;						//Output calculation
										if(v_phi>1.5) v_phi=1.5; else if(v_phi<-0.4) v_phi=-0.4;	//Output limits -0.4363323
										//###########################################
									}
									else 
									{ 
										if(fabs(delta[1])<50.0) soll_v = (int) ((double)SPEED_OB*fabs(delta[1])/125+.6*(double)SPEED_OB); else soll_v=SPEED_OB; 
									//As it aproaches the line reduces it's speed from 100% at 5[cm] to 60% at 0 [cm] and vice versa when it moves away from the line.
										v_phi = 0; 
									}
									break;
								case STATE_FURTHER_POINT_OF_LINE:
									switch(angle)
									{
										case !STATE_ALIGNED:
											if(fabs(iphi-phi0) < D_45_GRAD) soll_v= (int)((double)SPEED_OB*.51*fabs(iphi-phi0)+.6*(double)SPEED_OB); 
											else if(fabs(iphi-phi0) > D_315_GRAD) soll_v= (int)((double)SPEED_OB*.51*(2*M_PI-fabs(iphi-phi0))+.6*(double)SPEED_OB);
											else soll_v = SPEED_OB; //As it aproaches the angle reduces it's angular speed from 100% at 45° to 60% at 0°
											v_phi = D_90_GRAD; 		// Rotates clockwise
											break;
										case STATE_ALIGNED:
											break;
									}
									break;
							}
							break;
					}
					break;
			}
			break;
		case STATE_OFF:
			switch(btControl)
			{
				case BT_LEFT:
				case '4':	soll_v = SPEED_FW;	v_phi = D_90_GRAD;	break;
				case BT_RIGHT:
				case '6':	soll_v = SPEED_FW;	v_phi = -D_90_GRAD;	break;
				default:	hullMovement();					break;
			}
			break;
		case STATE_BT:
			switch(btControl)
			{
				case '1':	soll_v = -SPEED_FW;	v_phi = D_45_GRAD;	break;
				case BT_UP:
				case '2':	soll_v = SPEED_FW;	v_phi = 0;		break;
				case '3':	soll_v = -SPEED_FW;	v_phi = -D_45_GRAD;	break;
				case BT_LEFT:
				case '4':	soll_v = SPEED_FW;	v_phi = D_90_GRAD;	break;
				case '5':	soll_v = 0;		v_phi = 0;		break;
				case BT_RIGHT:
				case '6':	soll_v = SPEED_FW;	v_phi = -D_90_GRAD;	break;
				case '7':	soll_v = SPEED_FW;	v_phi = D_45_GRAD;	break;
				case BT_DOWN:
				case '8':	soll_v = -SPEED_FW;	v_phi = 0;		break;
				case '9':	soll_v = SPEED_FW;	v_phi = -D_45_GRAD;	break;
			}
			break;
	}




	
	iphi0=iphi;
			//iphi0 keeps the last angle orientation of the robot required for isAngle(ref)
*/
}//End sched_layer3







/*
#if HALL_CALIBRATION
void hullMovement(void)
{
	sprintf(s,"ADC_HALL1: %d\tADC_HALL2: %d\tADC_HALL3: %d\t",adc_read(ADC_HALL1),adc_read(ADC_HALL2),adc_read(ADC_HALL3));
	usart0_puts(s);
	usart0_sendbyte(0x0D);
}
#else
*/
/*
void fallStop(void)
{
	if(adc_read(ADC_IR1) < IR_EDGE_TRIGGER || adc_read(ADC_IR2) < IR_EDGE_TRIGGER || adc_read(ADC_IR3) < IR_EDGE_TRIGGER)
	{
		if((btControl!='7')&&(btControl!='8')&&(btControl!='9')&&(btControl!=BT_DOWN))
		{
			soll_v = v_phi = 0;
			vrad_ctrl(soll_v, v_phi);
		}
		if(power==STATE_ON) power = STATE_OFF;
	}
}

*/

void IRrange(void)
{
	r1[1] = adc_read(ADC_IR1);

}



/*
void range(int8_t mode)			//mode... operating mode e.g. FRONT_SENSORS 0x01 , ALL_SENSORS 0x00
{	
	static int8_t us_toggle;	//us_toggle... counter var to switch US-sensors 

	if((us_toggle>2)&&(!mode)) 	us_toggle=0;	//us_toggle reset: ALL_SENSORS
	if((us_toggle>1)&&( mode)) 	us_toggle=0;	//us_toggle reset: FRONT_SENSORS
	
	if(mode) off[2]=1;		//if operating mode == FRONT_SENSORS 

	switch(us_toggle)	
	{
		case 0:	r0[0]=r0[1];	// save last value
				r0[1] = (off[0]) ? 100 : srf10_range_read(SRF10_UNIT_0);
				//If the sensor has been off overwrite the last measurement (This avoids false obstacle detection. 100 is just arbitrary large value) 
				if(!r0[1]) r0[1]=r0[0];				// If reads 0 (Infinity) keep old value

				// Sensor SRF10_UNIT_1
				srf10_range_req(SRF10_UNIT_1); 			// Ranging request to "in centimiters"
				off[1]=0;					// Change OFF state 
				break;		

		case 1:	r1[0]=r1[1];				
				r1[1] = (off[1]) ? 100 : srf10_range_read(SRF10_UNIT_1);
				if(!r1[1]) r1[1]=r1[0];	
		
				// Sensor SRF10_UNIT_0
				if(mode) { srf10_range_req(SRF10_UNIT_0); off[0]=0; }
				
				// Sensor SRF10>> 2_UNIT_2
				else { srf10_range_req(SRF10_UNIT_2); off[2]=0; }
				break;

		case 2:	r2[0]=r2[1];				
				r2[1] = (off[2]) ? 100 : srf10_range_read(SRF10_UNIT_2);
				if(!r2[1]) r2[1]=r2[0];	

				// Sensor SRF10_UNIT_0
				srf10_range_req(SRF10_UNIT_0); off[0]=0;
				break;		
	}
	us_toggle++;
}
*/
/*
void ranger(int8_t mode)			//mode... operating mode e.g. FRONT_SENSORS 0x01 , ALL_SENSORS 0x00
{	
	static int8_t us_toggle;	//us_toggle... counter var to switch US-sensors 

	if((us_toggle>2)&&(!mode)) 	us_toggle=0;	//us_toggle reset: ALL_SENSORS
	if((us_toggle>1)&&( mode)) 	us_toggle=0;	//us_toggle reset: FRONT_SENSORS
	
	if(mode) off[2]=1;		//if operating mode == FRONT_SENSORS 

	switch(us_toggle)	
	{
		case 0:	r0[0]=r0[1];	// save last value
				r0[1] = (off[0]) ? 100 : srf10_range_read(SRF10_UNIT_0);
				//If the sensor has been off overwrite the last measurement (This avoids false obstacle detection. 100 is just arbitrary large value) 
				if(!r0[1]) r0[1]=r0[0];				// If reads 0 (Infinity) keep old value

				// Sensor SRF10_UNIT_1
				srf10_range_req(SRF10_UNIT_1); 			// Ranging request to "in centimiters"
				off[1]=0;					// Change OFF state 
				break;		

		case 1:	r1[0]=r1[1];				
				r1[1] = (off[1]) ? 100 : srf10_range_read(SRF10_UNIT_1);
				if(!r1[1]) r1[1]=r1[0];	
		
				// Sensor SRF10_UNIT_0
				if(mode) { srf10_range_req(SRF10_UNIT_0); off[0]=0; }
				
				// Sensor SRF10_UNIT_2
				else { srf10_range_req(SRF10_UNIT_2); off[2]=0; }
				break;

	
	}
	us_toggle++;
}
*/


void USsetup(void)
{
	//srf10_change_address(unsigned char sensor,SRF10_UNIT_0);

	// Sensor SRF10_UNIT_0
	srf10_range_req(SRF10_UNIT_0); 			// Ranging request to "in centimiters"
	srf10_set_range(SRF10_UNIT_0,RANGE_1M);		// set max range
	srf10_set_gain(SRF10_UNIT_0, GAIN_200);		// set lower gain for faster cyles and adjustment
	delay(10); off[0]=0;				// set stateVar to "Sensor initialization done"


}

void USrange(int8_t mode)		
{	

	if(off[0]) USsetup();				// Check Sensor initialization
	
	srf10_range_req(SRF10_UNIT_0); 			// Ranging request to "in centimiters"
	delay(10);
	r0[1] = srf10_range_read(SRF10_UNIT_0);
	delay(100);

}







void battState(void)
{	
	//Battery state calculation with hysteresis
	if(adc_read(ADC_VBAT) > LOW_100) batt_state=BATT_100;
	else
		if((adc_read(ADC_VBAT) < TOP_75)&&(adc_read(ADC_VBAT) > LOW_75)) batt_state=BATT_75;
		else
			if((adc_read(ADC_VBAT) < TOP_50)&&(adc_read(ADC_VBAT) > LOW_50)) batt_state=BATT_50;
			else
				if((adc_read(ADC_VBAT) < TOP_25)&&(adc_read(ADC_VBAT) > LOW_25)) batt_state=BATT_25;
				else
					if(adc_read(ADC_VBAT) < TOP_0) batt_state=BATT_0;


	if(!batt_state)	//Handles first state hysteresis problem
	{
		if((adc_read(ADC_VBAT) <= LOW_100)&&(adc_read(ADC_VBAT) >= TOP_75)) batt_state=BATT_75;
		else
			if((adc_read(ADC_VBAT) <= LOW_75)&&(adc_read(ADC_VBAT) >= TOP_50)) batt_state=BATT_50;
			else
				if((adc_read(ADC_VBAT) <= LOW_50)&&(adc_read(ADC_VBAT) >= TOP_25)) batt_state=BATT_25;
				else
					if((adc_read(ADC_VBAT) <= LOW_25)&&(adc_read(ADC_VBAT) >= TOP_0)) batt_state=BATT_0;
		
	}
	
	//
	switch(batt_state)	
	{
		case BATT_0:
			BATT_LED1 = ON;		//min
			BATT_LED2 = OFF;	//25%
			BATT_LED3 = OFF;	//50%
			BATT_LED4 = OFF;	//75%
			BATT_LED5 = OFF;	//100%
			break;
		case BATT_25:
			BATT_LED1 = ON;		//min
			BATT_LED2 = ON;		//25%
			BATT_LED3 = OFF;	//50%
			BATT_LED4 = OFF;	//75%
			BATT_LED5 = OFF;	//100%
			break;
		case BATT_50:
			BATT_LED1 = ON;		//min
			BATT_LED2 = ON;		//25%
			BATT_LED3 = ON;		//50%
			BATT_LED4 = OFF;	//75%
			BATT_LED5 = OFF;	//100%
			break;
		case BATT_75:
			BATT_LED1 = ON;		//min
			BATT_LED2 = ON;		//25%
			BATT_LED3 = ON;		//50%
			BATT_LED4 = ON;		//75%
			BATT_LED5 = OFF;	//100%
			break;
		case BATT_100:
			BATT_LED1 = ON;		//min
			BATT_LED2 = ON;		//25%
			BATT_LED3 = ON;		//50%
			BATT_LED4 = ON;		//75%
			BATT_LED5 = ON;		//100%
			break;
	}


	if((adc_read(ADC_VBAT)< LOW_CHARGE)&&(emptyBatTimer++ >= BATT_OFF_TIME)) POWER=OFF;			
	//If the battery charge is below the lower acceptable level for more that a couple seconds, automatically turns off the robot.
	else emptyBatTimer=0;

	if(batt_state != BATT_0) {LED1_RED = OFF; LED1_GREEN = ON;}
	else {LED1_GREEN = OFF; LED1_RED = ON;}
}


int8_t isAngle(double ref)
{
	if(fabs(ref)>NEAR_PI) return (fabs(iphi)>NEAR_PI);		
	//Takes care of angle near PI where a bug was detected, there was no zero crossing when |ref|> angular resolution in sched_layer3 (.144).
	else if(fabs(ref)<NEAR_0) return (fabs(iphi)<NEAR_0);
	else return(((iphi0-ref)*(iphi-ref)<0.0)&&((iphi0-ref)*(iphi-ref)>-1.0));		//Zero crossing to detect angle
}


void stateReset(void)
{
	direction = STOP; 	

	angle=STATE_ALIGNED;
	line2=!STATE_FURTHER_POINT_OF_LINE;
	us_unit2=!STATE_SEEN_BY_UNIT2;
	obstacle=!STATE_OBSTACLE_DETECTED;
	perpendicular=0;
	r0[1]=r1[1]=r2[1]=100;		//Sensor measurements  
	off[0]=off[1]=off[2]=1; 	//state var checks if USsensor initialization is done
}


void bluetooth(void)
{
	uint8_t com=0;		// Serves for comment recognition during Bluetooth connection and disconnection i.e. (CONNECT '0012-37-FF1AD5')
	unsigned char c;	// Byte received from Bluetooth.
	soll_v = 0; v_phi = 0;	
	while(1)
	{		
		c = usart1_recvbyte();						//Bluetooth received byte, hangs here if nothing is received.
		if((c==0x27)&&(!((com++)%2))) {LED3 ^= ON; if(power==STATE_BT)power=STATE_OFF;}	//If comment symbol (') and pair means connection established or disconnected, toggles the LED
		else if(!(com%2))							//If inside a comment doesn't consider input
		{
			switch (c) 	//Actions for Bluetooth received bytes
			{
				case BT_MANUAL:		power = STATE_BT; btControl=0;			break;	//Bluetooth Manual Mode
				case BT_AUTO:		power = STATE_OFF;				break;	//Tablobot Automatic Mode
				case BT_START:		if(power!=STATE_BT)					//Automatic Mode Start
							{
								power = STATE_ON; 
								x0 = ix; y0 = iy; phi0 = iphi;
							}	
							break;
				case BT_STOP:		if(power!=STATE_BT) power = STATE_OFF;		break;	//Automatic Mode Stop
				case BATT_REQ:		usart1_sendbyte(batt_state); 			break;	//Respond Battery State Request with the Battery State
				default:			btControl=c; 				break;	//Default: Manual Control, handled in sched_layer3()
			}
		
		}
	}
}

/**Interrupt function to generate the scheduler cycles
 */
ISR(TIMER3_COMPA_vect)
{
	cli();
	TIMSK3 &= ~(1 << OCIE3A);
	static uint8_t count1, count2, count3;

	sched_layer0();
	count1++;
	if(count1 == SCHED_PER1)  
	{
		count1 = 0;
		sched_layer1();
		count2++;

		if(count2 == SCHED_PER2) 
		{
			count2 = 0;
			sched_layer2();
			count3++;
			
			//life sign on LED
			FREE_C2	= OFF;		//Pin2
			FREE_C1	= OFF;		//Pin1

			if(count3 == SCHED_PER3)
			{
				count3 = 0;
				sched_layer3();

				//life sign on LED
				FREE_C2	= ON;		//Pin2
				FREE_C1	= ON;		//Pin1
				
			}
		}
	}
	TIMSK3 |= (1 << OCIE3A);
	sei();
}

void sched_init(void) 
{
	//Normal Port operation
	TCCR3B |= ((1 << WGM32) | (1 << CS31));		//CTC Mode, clk/8
	OCR3A = SCHED_PER0;				//Compare Register mit 18 laden (10us)

	TIMSK3 |= (1 << OCIE3A);			//Output Compare A Match Interrupt Enable
	
	digio_init();	//Initializes the microcontroller's I/O Ports.
	usart_init();	//Initializes the used USART modules.
	stdio_init();	//Redirects the standard I/O to USART0.
	i2c_init ();	//Initializes the I2C module.
	mot_init();	//Initializes the registers that control the motors.
	adc_init();	//Initialization of the ADC.
	stateReset();	//Initializes the state machine.
}
