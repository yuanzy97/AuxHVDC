/**********************************************************************************************************************************/
/**   The user defined model for the HVDC auxiliary control -- first-order lag transfer function                                 **/
/**   Author: Zhiyong Yuan                                                                                                       **/
/**   There are three control modes:                                                                                             **/
/**      1-Step change of the power order                                                                                        **/
/**      2-Auxiliary signal control based on the frequency difference                                                            **/
/**      3-Auxiliary signal control based on the angle difference                                                                **/
/**   Ver1.5: 1) Build a framework of the HVDC auxiliary control with multiple options                                           **/
/**           2) Add the AC voltage recovery detection on the terminal AC bus of the inverter                                    **/
/**           3) Set a droop control of the sigval[0] after the dAng reaches the peak                                            **/
/**              instead of after the system reaches new stable conditon                                                         **/
/**           4) Add the part of angle unwrapping function (New method)                                                          **/
/**   Dyd data record formant:                                                                                                   **/
/**   epcmod    41311 "CELILO1     " 500.00  "1 "   26097 "SYLMAR1     " 230.00  "1 "  1 : #27 "HVDCAU_UDM2_5.p" 8.0000          **/
/**   "dcbusr" 3 "dcbusi" 5 "dcmoder" 2 "auxmode" 3 "Kp" 0.007 "Tr" 0.05 "Xmax" 3720 "Xmin" 2000 "dbLow" -10 "dbHigh" 5          **/
/**   "tUp" 5 "tDown" 3 "VacLim" 0.92 "tDur" 1.0  "aDroop" 0.35  "Xfinal" 3150                                                   **/
/**********************************************************************************************************************************/

define INIT  2
define SORC  3
define ALGE  4
define RATE  5
define OUTP  7
define NETW  8
define Pi  3.14159265

/* All these internal variables will be commonly used during the dynamic simulation*/
@mx = dypar[0].cmi
@k  = model[@mx].k
@fr = model[@mx].bus
@to = model[@mx].to
$id = model[@mx].id
$ck = model[@mx].ck
@ns = model[@mx].sec
@extbsf = busd[@fr].extnum  /* external number of bus From */
@extbst = busd[@to].extnum  /* external number of bus To */

@mode = dypar[0].mode

if ( @mode > 2 )
	@k_epcdc = epcmod[@mx].v10            /* index number of the EPCDC dynamic model */
	@j_dccr = epcmod[@mx].v11             /* index number of the DCC converter of the rectifier */
	@j_dcci = epcmod[@mx].v12             /* index number of the DCC converter of the inverter */
	@time_flag_start = epcmod[@mx].v14    /* Flag for the fault inception 0-no fault, 1-fault starting */
	@time_point_start = epcmod[@mx].v15   /* Time point when fault occurs and FreqDiff > 0.1Hz */
	@time_flag_end = epcmod[@mx].v16      /* Flag for the dynamic ending 0-fault duration, 1-dynamic ending */
	@time_point_end = epcmod[@mx].v17     /* Time point when sytem reaches to a new stable condition */
	@angle_offset_begin = epcmod[@mx].v18 /* Offset of the angle diffrence at the beginning */
	@droop_afterflt = epcmod[@mx].v19     /* droop factor used for ramping down the DC power (Not used) */
	/* Set a list to save the AngleDiff for the detection of the peak value of AngleDiff or the new system stable condition */
	@list_afterflt0 = epcmod[@mx].v20     /* The elements in the list used for saving the AngleDiff for each 0.5 seconds*/
	@list_afterflt1 = epcmod[@mx].v21     /* The elements in the list used for saving the AngleDiff for each 0.5 seconds*/
	@list_afterflt2 = epcmod[@mx].v22     /* The elements in the list used for saving the AngleDiff for each 0.5 seconds*/
	@list_afterflt3 = epcmod[@mx].v23     /* The elements in the list used for saving the AngleDiff for each 0.5 seconds*/
	@list_afterflt4 = epcmod[@mx].v24     /* The elements in the list used for saving the AngleDiff for each 0.5 seconds*/
	@list_afterflt5 = epcmod[@mx].v25     /* The elements in the list used for saving the AngleDiff for each 0.5 seconds*/ 
    /* Save the initial values of the DC link */    
	@IDC_begin = epcmod[@mx].v26          /* The initial DC current */
	@VDC_begin = epcmod[@mx].v27          /* The initial DC voltage */
	@PAC_begin = epcmod[@mx].v28          /* The initial active power on the terminal bus */
	@QAC_begin = epcmod[@mx].v29          /* The initial reactive power on the terminal bus */
	/* Set a list to save the AC voltage for the detection of voltage recovery */
	@list_bfract0 = epcmod[@mx].v30       /* The elements in the list used for saving the AC voltage for each time step */
	@list_bfract1 = epcmod[@mx].v31       /* The elements in the list used for saving the AC voltage for each time step */
	@list_bfract2 = epcmod[@mx].v32       /* The elements in the list used for saving the AC voltage for each time step */
	@list_bfract3 = epcmod[@mx].v33       /* The elements in the list used for saving the AC voltage for each time step */
	@list_bfract4 = epcmod[@mx].v34       /* The elements in the list used for saving the AC voltage for each time step */
	@s0_angpeak = epcmod[@mx].v35         /* The peak value of the angle difference */
	@ang_unwrap_from = epcmod[@mx].v36    /* The unwrapped angle for the From bus */
	@ang_unwrap_to = epcmod[@mx].v37      /* The unwrapped angle for the To bus */
endif

switch (@mode)

    case INIT: 

		/* Get the index of epcdc dynamic model */
		epcmod[@mx].v10 = model_index(0, "epcdc", @fr, @to, $id, @ns, -1)     
		@k_epcdc = epcmod[@mx].v10
		
		if ( @k_epcdc < 0 )  
			logterm("** ERROR in HVDCAU MODEL FROM ", busident(@fr)," TO ", busident(@to), ", ID = ", $id, "  CAN'T FIND THE EPCDC TO CONTROL. ** <")
			logdy  ("** ERROR in HVDCAU MODEL FROM ", busident(@fr)," TO ", busident(@to), ", ID = ", $id, "  CAN'T FIND THE EPCDC TO CONTROL. ** <")
			model[@mx].st = 0
			goto  finished
		else
			logterm("** EPCDC MODEL FROM ", busident(@fr)," TO ", busident(@to), ", ID = ", $id, "  connected. <")
			logdy  ("** EPCDC MODEL FROM ", busident(@fr)," TO ", busident(@to), ", ID = ", $id, "  connected. <")
		endif
		
		/* Get the index of dcc Table for rectifier terminal */
		epcmod[@mx].v11 = rec_index(1, 11, @extbsf, epcmod[@mx].dcbusr, $id, @ns, -1)
		@j_dccr = epcmod[@mx].v11
		
		/* Get the index of dcc Table for inverter terminal */
		epcmod[@mx].v12 = rec_index(1, 11, @extbst, epcmod[@mx].dcbusi, $id, @ns, -1) 
		@j_dcci = epcmod[@mx].v12 
		
		/* Get the HVDC control mode according to the power flow setting */
		@dcmode_pf = dcc[@j_dccr].mode
		/* Get the mode parameter from dyd data */
		@dcmode = epcmod[@mx].dcmoder
		@temp = abs(@dcmode - @dcmode_pf)	
		if ( @temp > 0)
			logterm("** ERROR in HVDCAU MODEL FROM ", busident(@fr)," TO ", busident(@to), ", ID = ", $id, "  THE PARAMETER OF THE HVDC OPERATION MODE IS DIFFERENT FROM THE SETTING IN POWR FLOW ** <")
			logdy("** ERROR in HVDCAU MODEL FROM ", busident(@fr)," TO ", busident(@to), ", ID = ", $id, "  THE PARAMETER OF THE HVDC OPERATION MODE IS DIFFERENT FROM THE SETTING IN POWR FLOW ** <")
			model[@mx].st = 0
			goto  finished
		else
			logterm("** HVDCAU MODEL FROM ", busident(@fr)," TO ", busident(@to), ", ID = ", $id, "  connected. <")
			logdy  ("** HVDCAU MODEL FROM ", busident(@fr)," TO ", busident(@to), ", ID = ", $id, "  connected. <")
		endif	
		
		/* Set the initial values for the variables */
		epcmod[@mx].v14 = 0
		epcmod[@mx].v15 = 0
		epcmod[@mx].v16 = 0
		epcmod[@mx].v17 = 0	

		/* Get the initial value of the angle difference */
		epcmod[@mx].v18 = (volt[@fr].va - volt[@to].va)*180/Pi
		@angle_offset_begin = epcmod[@mx].v18

		/* Set the initial value of the droop after the fault */
		epcmod[@mx].v19 = 1
		@droop_afterflt = epcmod[@mx].v19
		
		/* Set the initial value of the list "list_afterflt" */
		epcmod[@mx].v20 = 0.5
		epcmod[@mx].v21 = 0.5
		epcmod[@mx].v22 = 0.5
		epcmod[@mx].v23 = 0.5
		epcmod[@mx].v24	= 0.5
		epcmod[@mx].v25 = 0.5
		
		/*Get the initial value of the DCC rectifier table*/
		epcmod[@mx].v26 = dcc[@j_dccr].idc
		@IDC_begin = epcmod[@mx].v26
		epcmod[@mx].v27 = dcc[@j_dccr].vdc
		@VDC_begin = epcmod[@mx].v27
		epcmod[@mx].v28 = dcc[@j_dccr].pac
		@PAC_begin = epcmod[@mx].v28
		epcmod[@mx].v29 = dcc[@j_dccr].qac
		@QAC_begin = epcmod[@mx].v29
		epcmod[@mx].v30 = 0.90
		epcmod[@mx].v31 = 0.90
		epcmod[@mx].v32 = 0.90
		epcmod[@mx].v33 = 0.90
		epcmod[@mx].v34 = 0.90
		epcmod[@mx].v35 = 0.0
		epcmod[@mx].v36 = volt[@fr].va
		epcmod[@mx].v37 = volt[@to].va
		
		logterm("Debug: Before fault IDC_begin: ", @IDC_begin, "A, VDC_begin: ", @VDC_begin, " kV, PAC_begin: ", @PAC_begin, " MW, QAC_begin: " , @QAC_begin, " MVar.<")	
		logterm("** Debug: HVDC ",  $id, " Rectifier Mode=", @dcmode, ". Initial power is: ", @PAC_begin, " MW. Initial curent is: ", @IDC_begin, " A.<")
		logterm("** Debug: HVDC ",  $id, " Alpha: ", dcc[@j_dccr].alpha, " Degree, Cmax: ", epcdc[@k_epcdc].cmax, " A.<")	
		
		/* initialize state variables for debug */ 
		epcmod[@mx].s0 = 0       
		epcmod[@mx].s1 = volt[@fr].vm
		epcmod[@mx].s2 = dcc[@j_dccr].vdc
		epcmod[@mx].s3 = (volt[@fr].va - volt[@to].va)*180/Pi
		epcmod[@mx].s4 = (netw[@fr].f - netw[@to].f)*60
		
		/* 1st output variable initialization */
		epcmod[@mx].v0 = epcmod[@mx].s0     
		epcmod[@mx].v1 = epcmod[@mx].s1
		epcmod[@mx].v2 = epcmod[@mx].s2
		epcmod[@mx].v3 = epcmod[@mx].s3
		epcmod[@mx].v4 = epcmod[@mx].s4	
		
		/* output channel head definition */
		channel_head[0].type = "Uaux"       
		channel_head[0].cmin = epcmod[@mx].Xmin
		channel_head[0].cmax = epcmod[@mx].Xmax
		channel_head[1].type = "Vac1"
		channel_head[1].cmin = 0.0
		channel_head[1].cmax = 2.0
		channel_head[2].type = "Vdc1"
		channel_head[2].cmin = 50.
		channel_head[2].cmax = 1000.
		channel_head[3].type = "dAng"
		channel_head[3].cmin = -180.
		channel_head[3].cmax = 180.
		channel_head[4].type = "dFrq"
		channel_head[4].cmin = -1.
		channel_head[4].cmax = 1.
		
	break
    case RATE:
	
		@time_now = dypar[0].time
		@time_delta = dypar[0].delt
		
		@freq1 = (1 + netw[@fr].f)*60
		@freq2 = (1 + netw[@to].f)*60
		@volt1 = volt[@fr].vm
		@volt2 = volt[@to].vm
		@volt_dc = dcc[@j_dccr].vdc
		@ang1_rt = volt[@fr].va
		@ang2_rt = volt[@to].va
		
		/* Unwrap the angle at the From bus*/
		@ang1 = @ang1_rt
		@temp = (@ang1_rt - @ang_unwrap_from)*180/Pi
		if ( @temp < -300)
			@round1 = round(@temp/360, 0)
			@ang1 = @ang1_rt + 2*Pi*abs(@round1)
		endif
		if (@temp > 300)
			@round1 = round(@temp/360, 0)
			@ang1 = @ang1_rt - 2*Pi*abs(@round1)
		endif
		
		/* Unwrap the angle at the To bus*/
		@ang2 = @ang2_rt
		@temp20 = (@ang2_rt - @ang_unwrap_to)*180/Pi
		if ( @temp20 < -300)
			@round2 = round(@temp20/360, 0)
			@ang2 = @ang2_rt + 2*Pi*abs(@round2 )
		endif
		if ( @temp20 > 300)
			@round2 = round(@temp20/360, 0)
			@ang2 = @ang2_rt - 2*Pi*abs(@round2)
		endif
		
		/*Save the unwrapped angle*/
		epcmod[@mx].v36 = @ang1
		epcmod[@mx].v37 = @ang2
		
		/* Get the difference of the frequency and angle on the two terminal buses of HVDC system */
		@f_diff_rt = @freq1 -@freq2 
		@a_diff_rt = (@ang1 - @ang2)*180/Pi 

		/* Get the scheduled power or current of HVDC */
		@dcmode = epcmod[@mx].dcmoder
		
		if ((@dcmode = 1) or (@dcmode = 4))
			@X_sched = dcc[@j_dccr].i_sched
			@I_P_init = @IDC_begin
		elseif ((@dcmode = 2) or (@dcmode = 5))
			@X_sched = dcc[@j_dccr].p_sched
			@I_P_init = @PAC_begin
		else
			@X_sched =  @PAC_begin
			@I_P_init = @PAC_begin 
		endif
		
		/* save the voltage in the list during the simulation*/
		epcmod[@mx].v34 = epcmod[@mx].v33
		epcmod[@mx].v33 = epcmod[@mx].v32
		epcmod[@mx].v32 = epcmod[@mx].v31
		epcmod[@mx].v31 = epcmod[@mx].v30
		epcmod[@mx].v30 = @volt2  /*store the voltage on the inverter bus into this list*/ 

		/* Get the mode of the auxiliay control signal ***/
		/* auxmode=1: Step change of the DC power order */
		/* auxmode=2: Auxiliary signal control in terms of the frequency difference */
		/* auxmode=3: Auxiliary signal control in terms of the angle difference */
		@auxmode = epcmod[@mx].auxmode
		
		switch (@auxmode)

			case 1:
			
				/*Get the time point when the Freq > dbHigh and the AC voltage on the bus of inverter recoveres to normal value.*/
				if ((@f_diff_rt > epcmod[@mx].dbHigh) and (@time_flag_start = 0))
					if ((@list_bfract0 >= epcmod[@mx].VacLim) and (@list_bfract0 >= @list_bfract1) and (@list_bfract1 > @list_bfract2) and (@list_bfract2 > @list_bfract3) and (@list_bfract3 > @list_bfract4))
						epcmod[@mx].v14 = 1
						epcmod[@mx].v15 = @time_now
						logterm("Auxiliary control (Mode 1) starts on bus  ", @extbsf, " at t=", @time_now, " s. <<")
						logdy("Auxiliary control (Mode 1) starts on bus  ", @extbsf, " at t=", @time_now, " s. <<")
					endif
				endif
				/*Get the time point when the Freq < dbLow and the AC voltage on the bus of inverter recoveres to normal value.*/
				if ((@f_diff_rt < epcmod[@mx].dbLow) and (@time_flag_start = 0))
					if ((@list_bfract0 >= epcmod[@mx].VacLim) and (@list_bfract0 >= @list_bfract1) and (@list_bfract1 > @list_bfract2) and (@list_bfract2 > @list_bfract3) and (@list_bfract3 > @list_bfract4))
						epcmod[@mx].v14 = 1
						epcmod[@mx].v15 = @time_now
						logterm("Auxiliary control (Mode 1) starts on bus  ", @extbsf, " at t=", @time_now, " s. <<")
						logdy("Auxiliary control (Mode 1) starts on bus  ", @extbsf, " at t=", @time_now, " s. <<")
					endif
				endif	
				
				/*Make a step change of the DC power order with a slope of Kp*/
				if (@time_flag_start = 1)
					@temp_Iorder = @X_sched + @I_P_init*epcmod[@mx].Kp
					epcmod[@mx].s0 = @temp_Iorder
				else
					epcmod[@mx].s0 = @X_sched
				endif
				
				/* Check the DC power to pass the limitation or not*/
				if ((@dcmode = 1) or (@dcmode = 4))
					/* For the I_order limitation */
					@I_max = epcmod[@mx].Xmax
					@I_min = epcmod[@mx].Xmin
					if ( epcmod[@mx].s0 > @I_max)
						epcmod[@mx].s0 = @I_max
						epcmod[@mx].zs0 = epcmod[@mx].s0
					endif	
					if ( epcmod[@mx].s0 < @I_min)
						epcmod[@mx].s0 = @I_min
						epcmod[@mx].zs0 = epcmod[@mx].s0
					endif
				elseif ((@dcmode = 2) or (@dcmode = 5))
					/* For the P_order limitation */
					@P_max = epcmod[@mx].Xmax/@IDC_begin*@PAC_begin
					@P_min = epcmod[@mx].Xmin/@IDC_begin*@PAC_begin
					if ( epcmod[@mx].s0 > @P_max)
						epcmod[@mx].s0 = @P_max
						epcmod[@mx].zs0 = epcmod[@mx].s0
					endif	
					if ( epcmod[@mx].s0 < @P_min)
						epcmod[@mx].s0 = @P_min
						epcmod[@mx].zs0 = epcmod[@mx].s0
					endif
				endif
				
				/* check if the time is larger than the duraiton*/
				@time_end = @time_point_start + epcmod[@mx].tUp
				
				/* IMPORTANT: Update the schedule power or current */
				if ((@dcmode = 1) or (@dcmode = 4))
					if(@time_now < @time_end)
						dcc[@j_dccr].i_sched = epcmod[@mx].s0
					else
						dcc[@j_dccr].i_sched = @IDC_begin
					endif
				elseif ((@dcmode = 2) or (@dcmode = 5))
					if(@time_now < @time_end)
						dcc[@j_dccr].p_sched = epcmod[@mx].s0
					else
						dcc[@j_dccr].p_sched = @PAC_begin
					endif
				else
					logterm("** Noting is changed! **<")
				endif
		
			break
			case 2:
				
			    /****************************************************************************************/
				/* We can design different control functions to build the auxiliary control signal*/
				
				/* Update the derivative of the state s0 */
				epcmod[@mx].ds0 = (@f_diff_rt*epcmod[@mx].Kp - epcmod[@mx].s0)/epcmod[@mx].Tr
				
				/****************************************************************************************/

				/* IMPORTANT: Set the ds0=0 before starting the control. */
				if (@time_flag_start = 0)
					epcmod[@mx].ds0 = 0
				endif

				/* Set the time flag as 1 when the Freq > dbHigh or Freq < dbLow */
				/* Get the time point when the Freq > dbHigh and the AC voltage on the bus of inverter recoveres to normal value.*/			
				if ((@f_diff_rt > epcmod[@mx].dbHigh) and (@time_flag_start = 0))
					if ((@list_bfract0 >= epcmod[@mx].VacLim) and (@list_bfract0 >= @list_bfract1) and (@list_bfract1 > @list_bfract2) and (@list_bfract2 > @list_bfract3) and (@list_bfract3 > @list_bfract4))
						epcmod[@mx].v14 = 1
						epcmod[@mx].v15 = @time_now
						logterm("Auxiliary control (Mode 2) starts on bus  ", @extbsf, " at t=", @time_now, " s. <<")
						logdy("Auxiliary control (Mode 2) starts on bus  ", @extbsf, " at t=", @time_now, " s. <<")
					endif
				endif
				/* Or get the time point when the Freq < dbLow and the AC voltage on the bus of inverter recoveres to normal value. */
				if ((@f_diff_rt < epcmod[@mx].dbLow) and (@time_flag_start = 0))
					if ((@list_bfract0 >= epcmod[@mx].VacLim) and (@list_bfract0 >= @list_bfract1) and (@list_bfract1 > @list_bfract2) and (@list_bfract2 > @list_bfract3) and (@list_bfract3 > @list_bfract4))
						epcmod[@mx].v14 = 1
						epcmod[@mx].v15 = @time_now
						logterm("Auxiliary control (Mode 2) starts on bus  ", @extbsf, " at t=", @time_now, " s. <<")
						logdy("Auxiliary control (Mode 2) starts on bus  ", @extbsf, " at t=", @time_now, " s. <<")
					endif
				endif

				/* Get the power or current of HVDC to check the overloading auxiliary signal */
				@X_v = dcc[@j_dccr].idc
				if ( @X_v > epcmod[@mx].Xmax)
					logterm ("Auxiliary control (Mode 2) on bus ", @extbsf, ": IDC reaches its maximum setting at t=", @time_now, " s. <")
					logdy ("Auxiliary control (Mode 2) on bus ", @extbsf, ": IDC reaches its maximum setting at t=", @time_now, " s. <")
				endif
				
				/* IMPORTANT: Update the auxiliary signal s0 */
				model[@k_epcdc].sigval[0] = epcmod[@mx].s0
				
			break
			case 3:
			
			    /****************************************************************************************/
				/* We can design different control functions to build the auxiliary control signal*/
				
				/*Get the angle difference without the offset*/
				@a_diff = @a_diff_rt - @angle_offset_begin
				
				/* Update the derivative of the state s0 */
				epcmod[@mx].ds0 = (@a_diff*epcmod[@mx].Kp - epcmod[@mx].s0)/epcmod[@mx].Tr
			
				/****************************************************************************************/
				
				/* IMPORTANT: Set the ds0=0 before starting the control. */
				if (@time_flag_start = 0)
					epcmod[@mx].ds0 = 0
				endif
				
				/* Set the time flag as 1 when the Freq > dbHigh or Freq < dbLow */
				/* Get the time point when the angle > dbHigh and the AC voltage on the bus of inverter recoveres to normal value. */
				if ((@a_diff > epcmod[@mx].dbHigh) and (@time_flag_start = 0))
					if ((@list_bfract0 >= epcmod[@mx].VacLim) and (@list_bfract0 >= @list_bfract1) and (@list_bfract1 > @list_bfract2) and (@list_bfract2 > @list_bfract3) and (@list_bfract3 > @list_bfract4))
						epcmod[@mx].v14 = 1
						epcmod[@mx].v15 = @time_now
						logterm("Auxiliary control (Mode 3)  starts on bus ", @extbsf, " at t=", @time_now, " s. <<")
						logdy ("Auxiliary control (Mode 3)  starts on bus ", @extbsf, " at t=", @time_now, " s. <<")
					endif
				endif
				/* Get the time point when the angle < dbLow and the AC voltage on the bus of inverter recoveres to normal value. */
				if ((@a_diff < epcmod[@mx].dbLow) and (@time_flag_start = 0))
					if ((@list_bfract0 >= epcmod[@mx].VacLim) and (@list_bfract0 >= @list_bfract1) and (@list_bfract1 > @list_bfract2) and (@list_bfract2 > @list_bfract3) and (@list_bfract3 > @list_bfract4))
						epcmod[@mx].v14 = 1
						epcmod[@mx].v15 = @time_now
						logterm("Auxiliary control (Mode 3) starts on bus ", @extbsf, " at t=", @time_now, " s. <<")
						logdy ("Auxiliary control (Mode 3) starts on bus ", @extbsf, " at t=", @time_now, " s. <<")
					endif
				endif

				if (@time_flag_start = 1)
					
					/* Set a delay for the transient */
					@t_transient = 0.05
					/* Check the time point for every N timesteps */
					if(@time_now > @time_point_start + @t_transient)
						@N = 4
						@temp2 = (@time_now - @time_point_start)/@time_delta
						@temp4 = mod(@temp2, @N)
						/* Save the freqDiff into the list for each N timesteps */
						if((@temp4 < 1.0) and (@temp4 > 0.0001))
							epcmod[@mx].v25 = epcmod[@mx].v24
							epcmod[@mx].v24 = epcmod[@mx].v23
							epcmod[@mx].v23 = epcmod[@mx].v22
							epcmod[@mx].v22 = epcmod[@mx].v21
							epcmod[@mx].v21 = epcmod[@mx].v20
							epcmod[@mx].v20 = @a_diff  /*store the a_diff into this list*/ 
						endif
						/* detect the extreme value of the a_diff through the six elements in the list */
						if ((@list_afterflt0 < @list_afterflt1) and (@list_afterflt1 < @list_afterflt2) and (@list_afterflt2 > @list_afterflt3) and (@list_afterflt3 > @list_afterflt4) and (@list_afterflt4 > @list_afterflt5) )
							@temp11 = 1
						else
							@temp11 = 0
						endif
							
						/* Make a flag to show the peak of the angle difference is found. */
						if ((@temp11 = 1 ) and (@time_flag_end = 0))
							epcmod[@mx].v16 = 1
							epcmod[@mx].v17 = @time_now
							epcmod[@mx].v35 = epcmod[@mx].s0  /* Save the peak value of s0 when reaching the peak of angle difference curve. */
							logterm("Auxiliary control (Mode 3) on bus ", @extbsf, ". Keep the constant DC power at t=", @time_now, " s. <<")
							logdy ("Auxiliary control (Mode 3) on bus ", @extbsf, ". Keep the constant DC power at t=", @time_now, " s. <<")
						endif
					endif
				endif
				
				/* Decrease the DC power to the normal setting after dDur seconds. */
				if (@time_flag_end = 1) 
					if (@time_now < @time_point_end + epcmod[@mx].tDur)
						epcmod[@mx].s0 = @s0_angpeak
						epcmod[@mx].v0 = epcmod[@mx].s0
					else
						@temp13 = (epcmod[@mx].Xfinal - @IDC_begin)/1000
						@temp_x = (@time_now - @time_point_end - epcmod[@mx].tDur)/@time_delta
						@temp_Iorder = @s0_angpeak - @temp_x*epcmod[@mx].aDroop/1000
						if (@temp13 > 0)
							if(@temp_Iorder < @temp13)
								@temp_Iorder = @temp13
							endif
						else
							if (@temp_Iorder < 0)
								@temp_Iorder = 0.0
							endif
						endif
						epcmod[@mx].s0 = @temp_Iorder
						epcmod[@mx].v0 = epcmod[@mx].s0
					endif
				endif
				
				/* Get the current of HVDC to check the overloading auxiliary signal */
				@X_v = dcc[@j_dccr].idc
				if ( @X_v > epcmod[@mx].Xmax)
					logterm ("Auxiliary control (Mode 3) on bus ", @extbsf, ": IDC reaches its maximum setting at t=", @time_now, " s. <")
					logdy ("Auxiliary control (Mode 3) on bus ", @extbsf, ": IDC reaches its maximum setting at t=", @time_now, " s. <")
				endif
				
				/* IMPORTANT: Update the auxiliary signal s0 */
				model[@k_epcdc].sigval[0] = epcmod[@mx].s0
				
			break
		endcase

	break
	case ALGE:

		epcmod[@mx].s1 = @volt1
		epcmod[@mx].s2 = @volt_dc
		epcmod[@mx].s3 = @a_diff_rt
		epcmod[@mx].s4 = @f_diff_rt
		
		epcmod[@mx].v0 = epcmod[@mx].s0
		epcmod[@mx].v1 = epcmod[@mx].s1
		epcmod[@mx].v2 = epcmod[@mx].s2
		epcmod[@mx].v3 = epcmod[@mx].s3
		epcmod[@mx].v4 = epcmod[@mx].s4
		
	break
endcase

end