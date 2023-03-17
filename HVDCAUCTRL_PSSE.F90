C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
C        This is an user defined model of the HVDC auxiliary control 
C        Designed and writen by Zhiyong(Alan) Yuan
C        There are 15 CON and 1 STATE. 1 VAR is used to save SIG
C        Data record is: BUSID 'USRAUX' 1  1 'HVDCAUCTRL' 17 0 NI NC NS NV data list /
C        BUSID should be the device name which the AUX signal is associated
C        Data record ver1 is: '1' 'USRAUX' 1  1 'HVDCAUCTRL' 17 0 0 15 1 1 /
C                             1.0  -0.020  0.020  -5.000  5.000 0.920  3720  2500 3100 /
C                             41311  26097  0.550  20  0.050  2.0 / 
C
C        The useful functions and variables are:
C        BUSDAT(IBUS, STRING, RVAL, IERR) - for bus voltage and bus angle, IBUS is the bus number
C        BUSEXS(IBUS, IERR) - check for the existence of a specific bus, IBUS is the bus number
C        DC2DAT_2(NAME, IBUS, STRING, RVAL, IERR) - for DC state with real value, v_sched, I_sched, Pac, Vdc, Idc
C        DC2INT_2(NAME, STRING, IVAL, IERR) - for - for DC state with integer value, control mode 
C        DC2CNX(IDVX) - for DC line table index
C        DSRVAL(STRING, INDX, RVAL, IERR) - for dynamic simulation setting with real value, time, dt
C        BSSEQN (IBUS, IS, *alt) - for getting the bus sequence number IS in terms of the bus number IBUS
C        VOLT(IS), BSFREQ(IS) - the voltage and frequency of the bus in terms of bus sequence number IS
C        TIME, DELT - for dynamic simulation setting, time, dt
C        DC2SIG(ISGX, IDVX) - IMPORTANT: for the auxiliary signal input of the DC line
C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC

      SUBROUTINE HVDCAUCTRL(IDVX, IDVT, ISGX, ISLOT)
      INCLUDE 'COMON4.INS'
      INTEGER IDVX, IDVT, ISGX, ISLOT
C
C     IDVX = DEVICE INDEX FOR THE TWO-TERMINAL DC LINE
C     IDVT = DEVICE TYPE FOR THE AUXILIARY SIGNAL, IDVT=1 FOR TWO-TERMINAL DC
C     ISGX = AUXILIARY-SIGNAL INDEX FOR THE TWO-TERMINAL DC LINE, FROM 1 TO 4
C     ISLOT = ARRAY ALLOCATION TABLE INDEX
C     J    = STRTAU(1,ISLOT)  [USES CON(J)   THROUGH   CON(J+14) ]
C     K    = STRTAU(2,ISLOT)  [USES STATE(K) THROUGH STATE(K+0)  ]
C     L    = STRTAU(3,ISLOT)  [USES DC2SIG(ISGX,IDVX) AND VAR(L) ]
C
      INTEGER    J, K, L
      REAL       S, FDBHIGH, FDBLOW, ADBHIGH, ADBLOW, VACLIM, DCAUMAX, DCAUMIN, PFINAL, DROOP, KP, TR
      INTEGER    IDC2CNX, IBUS, IERR, FLAG_FLT_START, FLAG_FLT_END, ISFROM, ISTO, AUXMODE
      REAL       TIME_FLT_START, TIME_FLT_END, PAC_BEGIN, IDC_BEGIN, VDC_BEGIN
      REAL       FREQDIFF, ANGLEDIFF 
C     
      CHARACTER  IM*2,BUSNAME*18, STRING*20
C     The used strings for bus and simulation are: 'KV', 'ANGLE', 'TIME', 'DELT' 
C     The used strings for DC are: 'MDC', 'DCCUR', 'SETVAL', 'VSCHD', 'PAC', 'KVDC', 'ANGLE' 
C
      INTRINSIC MAX, MIN, FLOOR, CEILING, MOD
      EXTERNAL BADMID

C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
C     MODE = 8 The description of the model data record
C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
      IF (MODE .EQ. 8)
C       J = 1
        CON_DSCRPT(J)    = 'AuxMode, the auxiliary control mode, 1-Step change, 2-Frequency, 3-Angle'
        CON_DSCRPT(J+1)  = 'FreqDBHigh, the high setting of the deadband for frequency difference'
        CON_DSCRPT(J+2)  = 'FreqDBLow, the low setting of the deadband for frequency difference'
        CON_DSCRPT(J+3)  = 'AngDBHigh, the high setting of the deadband for angle difference'
        CON_DSCRPT(J+4)  = 'AngDBLow, the low setting of the deadband for angle difference'
        CON_DSCRPT(J+5)  = 'VacLim, the lowest AC voltage of terminal buses of DC line'
        CON_DSCRPT(J+6)  = 'IdcMax, the max DC current during dynamic'
        CON_DSCRPT(J+7)  = 'IdcMin, the min DC current during dynamic'
        CON_DSCRPT(J+8)  = 'IdcFinal, the final setting of the DC current'
        CON_DSCRPT(J+9)  = 'BusNumFrom, the bus number 1 of the mornitoring signal'
        CON_DSCRPT(J+10) = 'BusNumTo, the bus number 2 of the mornitoring signal'
		CON_DSCRPT(J+11) = 'Droop, the droop for dereasing the aux signal after modulation period'
        CON_DSCRPT(J+12) = 'Kp, the proportional factor'
        CON_DSCRPT(J+13) = 'Tr, the time constant for the first-order block'
		CON_DSCRPT(J+14) = 'Tdur, the holding time of the peak value of aux signal'
        RETURN
        FIN

C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
C     MODE > 4 For the processing way of DOCU, DYDA, etc.
C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
      IF (MODE.GT.4)   GOTO 8401

C     CCCCCCCCCCCCCCCCCC Model Parameters Checking CCCCCC
C     DC line index negative if DC line is off-line
      WRITE(LPDEV,"(' The DC line Index connected auxiliary control: IDVX=',I10)") IDVX
      IF(IDVX .EQ. 0) 
	     WRITE(LPDEV,"('The DC line is wrong when IDVX=',I10, 'BUSID should be the name of the HVDC line !')") IDVX
	     RETURN
       
C     Get the DC line connection table number in terms of DC line index
      IDC2CNX = DC2CNX(IDVX)
	  WRITE(LPDEV,"(' The DC line connection table Index: IDC2CNX=',I10)") IDC2CNX
      IF(IDC2CNX .LT. 0)  
	     WRITE(LPDEV,"('The DC line connection table is not found, when IDC2CNX=',I10, '')") IDC2CNX
	     RETURN

C     This model doesn't support the longterm simulation
      IF(MIDTRM)  GOTO 9401

C     The location of the model data and variables
      J = STRTAU(1, ISLOT)
      K = STRTAU(2, ISLOT)
      L = STRTAU(3, ISLOT)

C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC	  
C     MODE = 1 Initialization 
C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
      IF(MODE .EQ. 1)
         S = DC2SIG(ISGX, IDVX)
         DCAUMAX = CON(J+6)
         DCAUMIN = CON(J+7)
         STATE(K) = 0.0
         VAR(L)  = S
        
         IF( S.GT.DCAUMAX .OR. S.LT.DCAUMIN)
            WRITE(LPDEV,"(' HVDCAUCTRL at HVDc line',I7,'initialized out of Limits! ')") NUMBUS(IDC2CNX)
            RETURN
            FIN
        RETURN
        FIN

C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
C     MODE = 2 Derivative of the state variables
C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
      IF( MODE .EQ. 2 )
C	     Get the frequency difference on the two terminal  buses.
		 ISFROM = 0
		 ISTO   = 0
		 CALL BSSEQN(CON(J+9), ISFROM, *10)
		 CALL BSSEQN(CON(J+10), ISTO, *20)
10		 INTICN(J+10) = ISFROM
20		 INTICN(J+11) = ISTO
		 FREQDIFF = BSFREQ(ISFROM) - BSFREQ(ISTO)

		 AUXMODE = CON(J)
		
		 IF (AUXMODE .EQ. 1)
		
		 ELSEIF (AUXMODE .EQ. 2)
		
		 ELSEIF (AUXMODE .EQ. 3)

		 ELSE
		 
			RETURN 
			FIN

		RETURN
        FIN

C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
C     MODE = 3 Update the algebraic variables, Output of the model
C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
      IF(MODE .EQ. 3)
         DC2SIG(ISGX, IDVX) = VAR(L)
		 AUXMODE = CON(J)
		
		 IF (AUXMODE .EQ. 1)
		
		 ELSEIF (AUXMODE .EQ. 2)
		
		 ELSEIF (AUXMODE .EQ. 3)
		 
		 ELSE	
		
     		RETURN 
			FIN         
		 
		 RETURN
         FIN

C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
C     MODE=4 Update the NINTG
C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
      IF(MODE.EQ.4)
         NINTEG = MAX(NINTEG,K+1)
         RETURN
         FIN

C     For the processing way when MODE=5 DOCU report mode=6 DYDA data output =7 DOCU data checking mode
8401  IDC2CNX = DC2CNX(IDVX)

C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
C     MODE=5, DOCU report
C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
      IF(MODE.EQ.5)   
	     RETURN
		 FIN

C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
C     MODE=6 DYDA data output for export the parameters to dyr file
C     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
      IF(MODE.EQ.6)
         J=STRTIN(1,ISLOT)
         WRITE(IPRT,6401) IBUS,IM,(CON(K),K = J,J+8)
6401     FORMAT(I7,' ''USRMDL'' ',A2,' ''HVDCAUCTRL'' ','     4     0     0     9     4     1',/7X,5G13.5,/7X,4G13.5,'/')
         RETURN
         FIN

C     For the non-longterm simulation
9401  CALL BADMID(MC,IDC2CNX,'HVDCAUCTRL')
      RETURN

      RETURN
      END