extern float PID, pwmLeft, pwmRight, error, previous_error;
extern float pid_d;
extern float pid_i;
extern float pid_d;
/////////////////PID CONSTANTS/////////////////
extern double kp;//3.55
extern double ki;//0.003
extern double kd;//2.05
///////////////////////////////////////////////
double throttle=1300; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady

void PID_controlX(float angle){
		/*First calculate the error between the desired angle and 
		*the real measured angle*/
		error = angle - desired_angle;
				
		/*Next the proportional value of the PID is just a proportional constant
		*multiplied by the error*/

		pid_p = kp*error;

		/*The integral part should only act if we are close to the
		desired position but we want to fine tune the error. That's
		why I've made a if operation for an error between -2 and 2 degree.
		To integrate we just sum the previous integral value with the
		error multiplied by  the integral constant. This will integrate (increase)
		the value each loop till we reach the 0 point*/
		if(-3 < error && error < 3)  pid_i = pid_i+(ki*error);  


		/*The last part is the derivate. The derivate acts upon the speed of the error.
		As we know the speed is the amount of error that produced in a certain amount of
		time divided by that time. For taht we will use a variable called previous_error.
		We substract that value from the actual error and divide all by the elapsed time. 
		Finnaly we multiply the result by the derivate constant*/

		pid_d = kd*((error - previous_error)/interval);

		/*The final PID values is the sum of each of this 3 parts*/
		PID = pid_p + pid_i + pid_d;

		/*We know that the min value of PWM signal is 1000us and the max is 2000. So that
		tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
		have a value of 2000us the maximum value that we could sybstract is 1000 and when
		we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
		to reach the maximum 2000us*/
		if(PID < -1000) PID = -1000;

		if(PID > 1000)	PID = 1000;


		/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
		pwmLeft = throttle + PID;
		pwmRight = throttle - PID;


		/*Once again we map the PWM values to be sure that we won't pass the min
		and max values. Yes, we've already maped the PID values. But for example, for 
		throttle value of 1300, if we sum the max PID value we would have 2300us and
		that will mess up the ESC.*/
		//Right
		if(pwmRight < 1000) pwmRight= 1000;
		
		if(pwmRight > 2000) pwmRight=2000;

		//Left
		if(pwmLeft < 1000)	pwmLeft= 1000;

		if(pwmLeft > 2000) pwmLeft=2000;


		/*Finnaly using the servo function we create the PWM pulses with the calculated
		width for each pulse*/
		pwmLeft = map(pwmLeft, 1000, 2000, 50, 100);
		pwmRight = map(pwmRight, 1000, 2000, 50, 100);
		TIM4->CCR1 = pwmLeft;
		TIM4->CCR2 = pwmRight;

		previous_error = error; //Remember to store the previous error.

}