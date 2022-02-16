#include "PID.h"
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#define TICK_SECOND 1000

pid_t pid_create(pid_t pid, _iq* in, _iq* out, _iq* set, float kp, float ki, float kd,enum pid_control_method pid_method)
{
	pid->input = in;
	pid->output = out;
	pid->setpoint = set;
	pid->automode = false;
    pid->pid_method = pid_method;
	pid_limits(pid, 0, 255);

	// Set default sample time to 100 ms
	pid->sampletime = 100 * (TICK_SECOND / 1000);

	pid_direction(pid, E_PID_DIRECT);
    pid_tune(pid, kp, ki, kd);
    
	pid->lasttime = HAL_GetTick() - pid->sampletime;
    rt_kprintf("PID INIT:\r\n");
    rt_kprintf("Kp = %08x ,fKp = %f\r\n",pid->Kp,_IQtoF(pid->Kp));
    rt_kprintf("Ki = %08x ,fKi = %f\r\n",pid->Ki,_IQtoF(pid->Ki));
    rt_kprintf("Kd = %08x ,fKd = %f\r\n",pid->Kd,_IQtoF(pid->Kd));
   
	return pid;
}

bool pid_need_compute(pid_t pid)
{
	// Check if the PID period has elapsed
	return((HAL_GetTick() - pid->lasttime) >= pid->sampletime) ? true : false;
}

void position_pid_compute(pid_t pid)
{
    _iq Error;
    _iq Up,Ui,Ud;    
    // Check if control is enabled
	if (!pid->automode)
		return;
	
	_iq in = *(pid->input);
	// Compute error
	_iq error = (*(pid->setpoint)) - in;
    Up = _IQmpy(pid->Kp,error);
	// Compute integral
    Ui = _IQmpy(pid->Ki,error);
	pid->iterm += Ui;
	if (pid->iterm > pid->omax)
		pid->iterm = pid->omax;
	else if (pid->iterm < pid->omin)
		pid->iterm = pid->omin;
	// Compute differential on input
	_iq dinput = in - pid->lastin;
    //kd
    Ud = _IQmpy(pid->Kd,dinput);
	// Compute PID output
	_iq out = Up + pid->iterm - Ud;
	// Apply limit to output value    
	(*pid->output) = _IQsat(out,pid->omax,pid->omin);
	// Keep track of some variables for next execution
	pid->lastin = in;
	pid->lasttime = HAL_GetTick();
}

void inc_pid_compute(pid_t pid)
{
    #if 0
	// Check if control is enabled
	if (!pid->automode)
		return;
	
	_iq in = *(pid->input);
	// Compute error
	_iq cur_error = (*(pid->setpoint)) - in;

    _iq err1 = cur_error - pid->last_err;
	_iq err2 = cur_error - 2 * pid->last_err + pid->previous_err;
	// Compute PID output
	_iq out = pid->Kp * err1 
        + pid->Ki * cur_error 
        + pid->Kd * err2;
    pid->previous_err = pid->last_err;
    pid->last_err = cur_error;
	// Apply limit to output value
	if (out > pid->omax)
		out = pid->omax;
	else if (out < pid->omin)
		out = pid->omin;
	// Output to pointed variable
	(*pid->output) = out;
	pid->lasttime = HAL_GetTick();
    #endif
}

void pid_compute(pid_t pid)
{
    if(pid->pid_method == E_POSITION_METHOD)
    {
        position_pid_compute(pid);
    }
    else
    {
        inc_pid_compute(pid);
    }
}



void pid_tune(pid_t pid, float kp, float ki, float kd)
{
    float fKp,fKi,fKd;
	// Check for validity
	if (kp < 0 || ki < 0 || kd < 0)
		return;
	if(pid->pid_method == E_POSITION_METHOD)
    {
        //Compute sample time in seconds
        float ssec = ((float) pid->sampletime) / ((float) TICK_SECOND);

        fKp = kp;
        fKi = ki * ssec;
        fKd = kd / ssec;
        rt_kprintf("sec = %f\r\n",ssec);
    }
    else
    {
        fKp = kp;
        fKi = ki;
        fKd = kd;        
    }
    if (pid->direction == E_PID_REVERSE) {
        fKp = 0 - fKp;
        fKi = 0 - fKi;
        fKd = 0 - fKd;
    }  
    rt_kprintf("fKp = %f ,fKi = %f,fKd = %f\r\n",fKp,fKi,fKd);
    pid->Kp = _IQ(fKp);
    pid->Ki = _IQ(fKi);
    pid->Kd = _IQ(fKd);    
}

void pid_sample(pid_t pid, uint32_t time)
{
    float fKi,fKd;
	if (time > 0) {
		float ratio = (float) (time * (TICK_SECOND / 1000)) / (float) pid->sampletime;
		fKi *= ratio;
		fKd /= ratio;
		pid->sampletime = time * (TICK_SECOND / 1000);
 		pid->Ki = _IQ(fKi);;
		pid->Kd = _IQ(fKd);       
	}
}

void pid_limits(pid_t pid, _iq min, _iq max)
{
	if (min >= max) return;
	pid->omin = min;
	pid->omax = max;
	//Adjust output to new limits
	if (pid->automode) {
		if (*(pid->output) > pid->omax)
			*(pid->output) = pid->omax;
		else if (*(pid->output) < pid->omin)
			*(pid->output) = pid->omin;

		if (pid->iterm > pid->omax)
			pid->iterm = pid->omax;
		else if (pid->iterm < pid->omin)
			pid->iterm = pid->omin;
	}
    rt_kprintf("LimitMax = %08x ,fLimitMax = %f\r\n",pid->omax,_IQtoF(pid->omax));  
    rt_kprintf("LimitMin = %08x ,fLimitMin = %f\r\n",pid->omin,_IQtoF(pid->omin));      
}

void pid_auto(pid_t pid)
{
	// If going from manual to auto
	if (!pid->automode) {
		pid->iterm = *(pid->output);
		pid->lastin = *(pid->input);
		if (pid->iterm > pid->omax)
			pid->iterm = pid->omax;
		else if (pid->iterm < pid->omin)
			pid->iterm = pid->omin;
		pid->automode = true;
	}
}

void pid_manual(pid_t pid)
{
	pid->automode = false;
}

void pid_direction(pid_t pid, enum pid_control_directions dir)
{
	if (pid->automode && pid->direction != dir) {
		pid->Kp = (0 - pid->Kp);
		pid->Ki = (0 - pid->Ki);
		pid->Kd = (0 - pid->Kd);
	}
	pid->direction = dir;
}
