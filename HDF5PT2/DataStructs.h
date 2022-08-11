#pragma once
#include<string>
#include<minwindef.h>
#include <atltypes.h>
#define		LR_MODE			2
#define		MAX_CAMS		16 
#define		Last_ID			440
#define		ID_LAST_PROFILE	32
#define		INDEX_TO_2		2
#define		INDEX_TO_4		4
enum _Selected_Device_ {
	IS_BEAMSCOPE = 1,/**<1: */
	IS_BEAMR = 2,/**<2: */
	IS_BEAMMAP = 3,/**<3: */
	IS_BEAMC = 4,/**<4: */
	IS_WINCAM = 5,/**<5: */
	IS_WINCAM_DIV = 6,/**<6: */
	IS_WINCAM_LOG = 7,/**<7: */
	IS_TWOD_SCAN = 8,/**<8: */

	IS_WINCAM_COMP = 9,/**<9: */
	IS_WINCAM_COMP3 = 10,/**<10: */
	IS_WINCAM_COMP4 = 11,/**<11: */
	IS_WINCAM_COMP5 = 12,/**<12: */
	LAST_DEVICE = 13,/**<13: */
};
typedef	struct {
	int		xBegin;
	int		xSize;
	int		yBegin;
	int		ySize;
}	CAPTURE_RECT;

typedef	struct {
	COLORREF	Colors[5];
	double		UpperLimit;
	double		LowerLimit;
	BOOL		EnableTest;
}	BUTTON_DATA;

typedef	struct	{  

	DWORD		Signature1  ;		// = (int('D')<<24)+(int('R')<<16)+(int('I')<<8)+'.' ;
	double		MaxTravel_MM ;
	double		StepSize_UM ;
	double		SlitToLenseDistance ;	
	double		ClipLevels[LR_MODE] ;	
	double		FocalLength ;	
	double		Wavelength ;	

	double		xySpans[2] ;	
	double		dSpares[20] ;	

	int			ImageWidth ;
	int			ImageHeight ;

	int			M2Start ;
	int			M2Stop  ;
	int			M2AverageCount  ;
	int			M2Samples  ;
	int			Direction ;
	int			Target ;
	int			Position ;
	int			Speed	 ; 
	int			ScanSlits ;
	int			ClipModes[2] ;
	int			Spares[98] ;
	DWORD		Signature2  ;		// = (int('D')<<24)+(int('R')<<16)+(int('I')<<8)+'.' ;
}	AUX_MOTOR			 ;

typedef	struct {
	double	left;
	double	right;
	double	top;
	double	bottom;
	CRect	Actual;//This is a class
	short	Zoom;
	short	Valid;
	char	Mode;
}	INCLUSION_REGION;

typedef	struct {
	int		Signature;		// = (int('D')<<24)+(int('R')<<16)+(int('I')<<8)+'.' ;
	int		Type;		// TYPE_WC_IMAGE_DATA
	int		Index;
	int		Beams;
	int		Size;
	int		Width;// Number of horizontal pixels
	int		Height;// Number of vertical   pixels
	int		CameraUpdateNumber;
	double	XpixelSize;	//Pixel horizontal
	double	YpixelSize;	//Pixel vertical
	int		Bits;//Normal = 16
	int		ReadyForProcessImage;
	int		ImageryPeak; //Peak pixel value in imagery that will be used in image processing (after HyperCal, filtering, averaging).. This should be used in all calculations. (4/21/2020)			
	int		Xoffset;// x start offset (unused pixels)
	int		Yoffset;// y start offset (unused pixels)
	int		Xlimit;// imagers total number of x pixels
	int		Ylimit;// imagers total number of y pixels
	int		spareInt;
	CPoint	pPeakCenter; //This is a class
	double	DefinedFluencePower;
	double	pUserCentroid[2];
	double	Centroid[2];
	double	GeoCentroid[2];
	double	Baseline;
	double	UserCentroid[2];
	double	GeoCenter[2];
	double	PeakCentroid[2];
	double	Orientation;
	double	Ellipticity;
	double	MajorWidth;
	double	MinorWidth;
	double	MeanWidth;
	double	PeakFluencePower;
	int		BufferSize;
	int		iShutterSetting;
	double  Homogeneity;
	double	readTECCelsius;
	double	IsoXInclusionRegionRadius_um;
	double	IsoYInclusionRegionRadius_um;
	double	Sigma4Ellip;
	double	Sigma4EllipAngle;
	double	IsoXWidth_um;
	double	IsoYWidth_um;
	double	ShutterSetting;
	double	BaselineStd;
	double	Gamma;
	double	MajorWidth_dXX_WinCamD;
	double	MinorWidth_dXX_WinCamD;
	double	dXX_WinCamD;
	double	A_dXX_WinCamD;
	double	P_dXX_WinCamD;
	double	IXX_WinCamD;
	double	Theta_XX_WinCamD;
	double	GaussianFit;
	double	ImageTemp_C;
	double	basic_Centroid[8];
	int		Busy;
	int		Minimum;
	int		NumberAveraged;
	int		UsedInAverage;
	int		WasFullResolution;
	double	PowerFactor;
	char	PowerLabel[20];
	double	CorrectPower;
	double	InitialResult;
	double	PowerInDB;
	int		UseOldPowerData;
	int		LogSaved;
	int		MinLevel;
	int		AdcPeak; //Peak pixel value coming off of the hardware (before HyperCal, filtering, averaging).. Also represents stitched image global peak in .l_wcf. (4/21/2020)
	int		WasLogged;
	int		Camera;
	time_t	CaptureTime;
	int		GammaDone;
	int		Was_TwoD_Ssan;
	double	PeakToAverage;
	double	Ewidth_WinCamD;
	int		Was_WinCamDiv;
	int		SatPixels;
	double	FPS;
	double	meanPixelValueAboveClipA;
	double	PowerInCentroidTarget;
	double  PlateauUniformity;
	int		PixelIntensity;
	double	CameraGain;
	int		MatrixIndex;
	double	PowerShutterSetting;
	int		IsM2Data;
	double	UcmM2Zlocation;
	double	UcmM2SlitToLense;
	double	UcmM2LenseToCameraFace;
	double	UcmM2LenseFocalLength;
	double  UcmM2Wavelength;
	int		M2Data;
	int		ConnectionType;
	int		AdcMinimum;
	double	LD;
	double	ZoDelta;
	double	MFactor;
	int		CameraType;
	int		AdcAverage;
	int		PeakFound;
	int		iBaseline;
	int		uFIR_Gain;
	int		CTE_State;
	int		MeasurePeak;
	int		FullResolution;
	double	PowerInInclusionRegion;
	int		HyperCalGood;
	int		IlluminatedPixels;
	int		AdcOffset;
	int		Temp1;
	int		ShutterState;//0 is default, no shutter. 1 is shutter close. 2 is shutter open
	int		XSampleRate;//The rate at which to sample pixels when creating the image
	int		LineLaserCaptureWidth;
	unsigned	int		AssembleTime_ms_lowByte;
	unsigned	int		AssembleTime_ms_highByte;
	int		IncludedGlobalWarning;
	BYTE	ByteSpare[2];
	BYTE	Image_Flags;				// Really a bunch of bool variables, but we are saving memory here.
	BYTE	CameraIndex;
	int		ParentCamera;
	double	TotalPower;
	int		CentroidType;
	double	pCentroid[2];
	double	pGeoCentroid[2];
	double	pPeakCentroid[2];
	int		NewData;
	int		ExtraLine;
	BYTE	wcData[1];
}	WC_IMAGE_DATA;
typedef	struct {
	DWORD		Signature1;		// = (int('D')<<24)+(int('R')<<16)+(int('I')<<8)+'.' ;

	char		PowerLabel[LAST_DEVICE][LR_MODE][20];

	int			Test1;

	int			Palette;
	int			InkSaver;
	int			CurrentCamera;
	int			CurrentDevice;
	int			DisplayGammaEnabled;
	int			FluenceIsRound;
	int			FastUpdate;
	int			ISO_MajorMinor_XY_Select;
	int			AutoOrientXhairs[8][LR_MODE];
	int			ForceToZero[8][LR_MODE];
	int			ForceTo45[8][LR_MODE];
	int			AutoSnap;
	int			ShowWinCamSlits;
	int			LockInclusion;
	int			FlipVert;
	int			BackDibValid;
	int			FluenceOn;
	int			Test3;
	int			FluenceIsMM;
	int			FluenceAverageEn;
	int			iShutterCount[MAX_CAMS];
	int			FluencePeakEn;
	int			FluenceDefinedEn;
	int			FluencePeakAverageEn;
	int			FluencePeakDefinedEn;
	int			Test4;
	int			IsPWLocked;
	int			DisplayMicrons;
	int			Tracking;
	int			BeamScopeActiveSlits;
	int			oldLastFileType;

	int			old_PowerShutterSetting[Last_ID][2];

	BYTE		AutoGain[Last_ID];
	BYTE		ShowClip[Last_ID];
	BYTE		oldShowGFit[Last_ID];
	BYTE		ShowTopHat[Last_ID];
	BYTE		ShowGrid[Last_ID];
	BYTE		DisplayMode[Last_ID];
	BYTE		DrawingMode[Last_ID];
	BYTE		ProfileZoom[Last_ID];
	BYTE		ShowMaxDev[Last_ID];
	BYTE		AutoRange[Last_ID];

	int			Test5;

	int			iFilter;
	int			Wires3D;
	int			LButton3D;
	int			AverageNumber;
	int			NumberToAverage;
	int			ResetAverageAt;
	int			ResetOnDrift;
	CRect		M2rect;//Class
	CRect		LogRect;//Class
	CRect		WanderRect;//Class
	CRect		MC2rect;//Class
	CRect		BeamMap_M2rect;//Class
	CRect		BeamC_M2rect;//Class
	CRect		BeamMap_DivRect;//Class
	CRect		BeamC_DivRect;//Class

	int			Test6;
	CAPTURE_RECT		BackgroundRect;//nested struct

	int			M2Lock;
	int			ViewM2As3D;
	int			ViewXdata;

	int			HeadRotationIsFixed;

	int			ZeroPhase;
	int			BCpair;

	int			WanderRadiusAuto;
	int			ReverseXsign;
	int			ReverseYsign;
	int			WanderFullColor;
	int			oldWanderRadius;
	int			IsAbsolute[LAST_DEVICE];
	int			CameraLock;
	int			TimedOut;
	int			FirmwareRev;
	int			XternalClock;
	int			ShowM2Debug;
	int			BCSlits;
	int			BeamC_axis;
	int			BCSlitsPairs;
	int			xBeamScopeModeM2;
	int			HeadType;
	int			xBeamScopeMode;
	int			BeamMapMode;
	int			xBeamRMode;
	int			xLogging;
	int			Test7;
	int			EmergencyRename;
	int			BS_PulseMode;
	int			xxBS_PulseModeFreq;
	int			BackgroundValid;
	int			IRcorrectionValid;
	int			Searches;
	int			SlitsUsed[LAST_DEVICE];

	int			SetGains[ID_LAST_PROFILE];
	int			SetRange[ID_LAST_PROFILE];
	int			ShowDivergence;

	int			Test8;

	double		old_AutoShutterLimitLow;
	double		old_AutoShutterLimitHigh;
	int			Profile_Flag;
	int			old_AutoShutterLimit;
	int			TriggerEnabled;
	int			TriggerIsInput;
	int			TriggerOnPositive;
	int			TriggerImpedance;
	int			spareInt2;
	int			m_TriggerIsInput;
	int			AutoTriggerWithShutter;
	int			AutoCameraSelect;
	int			CameraThere[MAX_CAMS];
	int			WinCamD_AutoTrigger;
	int			Spare_int;
	int			ApproximateFrequency;
	int			TriggerTiming;
	int			TriggerMilliseconds;
	int			TriggerIsSynced;
	int			CameraGain_old[MAX_CAMS];
	int			Test9;
	int			BaselineLocked;
	int			LockedBaseline;
	int			CameraType;
	int			OreintationOff;
	int			StartOffset[MAX_CAMS];
	int			CameraBits[MAX_CAMS];
	int			LastBaseLine[MAX_CAMS];
	int			ResetAverage[8];
	int			SetupPeak;
	int			SetupCapture;
	int			PeakLock;
	int			CameraOffset[MAX_CAMS];
	int			WaitingForTrigger;
	int			DenBit;
	int			Test10;
	int			PhaseBit;
	int			WC_CameraType[MAX_CAMS];
	int			LastMatchCount;
	int			CamRegisters[32];
	int			CameraAdcBits[MAX_CAMS];
	int			PciBits[MAX_CAMS];
	int			CamReg2;
	int			MatchPoint;
	int			CaptureBackground;
	int			IrCameraEnabled;
	int			CompOrder;
	int			CaptureCount;
	int			AutoBaselineSub;
	int			TestMode;
	int			ShutterOpen;
	int			ImageHoldOff;
	int			TriggerShots;
	int			SelectedCamera;
	int			TriggerClock;
	int			TriggerReady;
	int			Test11;
	int			CustomX;
	int			CustomY;
	int			WinCamFilter[LR_MODE];
	int			UseInclusionDib[LR_MODE];
	int			SizeToggle;
	int			TestCount;
	int			Test_uSeconds;
	int			UseAltSlits[LAST_DEVICE];
	int			BS_PulseModeFreq[2];
	int			BackGroundSubtraction[MAX_CAMS];
	int			WinCamNormalized;		// This should really be declared a boolean variable.
	int			StageSpeed;
	int			TwoD_AutoScale;
	int			TwoD_Width;
	int			TwoD_Height;
	int			TwoD_Reference;
	int			ForceEepromToOld;


	int			AngularDeviationEnabled[INDEX_TO_2][LAST_DEVICE][LR_MODE];
	int			AngularDeviationMode[INDEX_TO_2][LAST_DEVICE][LR_MODE];

	int			UseRecallPowerData[LAST_DEVICE][LR_MODE];
	int			DontAsk;
	int			MaxSpeed;
	int			RampTime;
	int			CustomSetup;
	int			BeamFit;

	int			ImageLogCount;
	int			ImageLogSeconds;
	int			ImageLogInvalid;
	int			ExportDataToExcel;
	int			UseEffSlits;
	int			LinesToScan;
	int			CalSign;
	int			MajorMinorMethod;
	int			DontUseSemicolon;
	int			ShowUcVc;
	int			SigmaWarning;
	int			StopMotorOnExit;
	int			SamplLimit;
	int			WanderSequence;
	int			AutoRotate3D;
	int			ShowCrosshairs[8];
	int			BeamMapDefaultXc;
	int			ShowClipIntercept;
	int			ShowUniformity;

	BOOL		RealTimeLogging;//antiquated
	int			NonunifomrityOnOff;//no longer used
	BOOL		ThumbAutoZoom;
	BOOL		YequalX;
	BOOL		ShowRoughness;
	int			WinCam_Divergence_Cameras;
	int			WanderRadius[2];

	int			ForceSpan[4];
	int			ForceCenter[4];
	int			AnimationOn;
	int			AnimationPeriod_ms;
	int			ThreeDenabled;
	int			TwoDenabled;
	int			IbisRaw[8];
	int			IbisFine[8];
	int			IbisGain[8];
	int			AutoTriggerOn;
	int			DisabledCalibration;
	int			LapTop;
	int			PITB_index;
	int			LimitExposure[2];
	int			xLimitAdcPeak[2];

	int			LimitExposureOn;
	int			LimitAdcPeakOn;

	int			ShowRMSUniformity;
	int			ShowEdgeSteepness;
	int			spareInt5;
	int			spareInt6;
	int			spareInt1;
	int			spareInt7;
	int			spareInt8;
	double		spareDbl1;

	double		BucketEllipseWidth;
	double		BucketEllipseHeight;
	int			BucketOrientation;
	int			BucketShape;
	int			DontUsePLS;

	int			BucketEdge;
	int			Pole50;
	int			AutoBucket;
	int			LimitGainOn;
	int			Frames;
	int			M2sensitivity;

	int			RawOfffset[10];
	int			DeviceVersion;
	int			RawMode;
	int			M2show3D;
	int			MultipleCameras;
	int			FullResolution;

	int			UcmM2Samples;
	int			UcmM2Average;
	double		UcmM2WaveLength;

	double		UcmM2LenseToCameraFace;
	double		UcmM2LenseFocalLength;
	double		UcmStageMaxTravel;

	BYTE		ClipModes[INDEX_TO_2][LR_MODE];//a 0 means a clip level, a 1 means 4sigma

	int			SlowClock;
	int			CameraUpdateVersion;
	int			PwmPercent;
	int			uMapRate;

	double		UcmStageStart;
	double		UcmStageEnd;

	int			uMapTestGain;
	int			LastDacRaw;
	int			FlipHorz;
	int			uMap_Speed;  // 0 = slow, 1 = fast
	double		d_AdcOffset;
	int			Zo_Average;
	int			M2_ViewSource;
	int			uScope_Speed;  // 0 = slow, 1 = fast

	int			P8MotorRate;
	int			P8Position;
	int			P8AdcRate;
	int			DivergenceOption;
	int			DisableLED;
	int			AlternateDetector;

	double		AutoShutterLimitLow[8];
	double		AutoShutterLimitHigh[8];
	int			AutoShutter[8];
	int			AutoShutterLimit[8];

	int			PersistentReference[LAST_DEVICE];
	int			CalOffset;

	int			ShowBlueLine;
	int			CompFileSize;
	int			PolarCamEnabled;

	char		PolarCam_FileName[80];


	BYTE		UseMultiBeam;
	BYTE		MultiBeamMode;
	BYTE		Spares_MB[2];
	int			WhichMultiBeam;
	int			PhantomHead;
	int			IsAbsolute_alt[LAST_DEVICE];
	int			PersistentReference_alt[LAST_DEVICE];
	int			unused_UseD63;
	int			AngularPointing;
	int			CameraAutoSelect;


	int			EnableGrid;
	int			EnableCenterBullseye;
	int			EnableCentroidBullseye;
	int			InverseGridColor;
	int			InverseCenterColor;
	int			InverseCentroidColor;

	int			GridLineSizeInPixels;
	int			CenterLineSizeInPixels;
	int			CentroidLineSizeInPixels;
	int			EnableTargetMatch;

	COLORREF	GridColorRef;
	COLORREF	CenterColorRef;
	COLORREF	CentroidColorRef;
	int			GridTest;


	int			DontCalcRunning_NU;
	int			DontCalcStopped_NU;
	int			DontCalcRunning_Ipeak;
	int			DontCalcStopped_Ipeak;

	int			DontGraphRunning_Ipeak;
	int			DontGraphStopped_Ipeak;
	int			DontCalcRunning_Clocking;
	int			DontCalcStopped_Clocking;

	int			DontGraphRunning_Clocking;
	int			DontGraphStopped_Clocking;
	int			DontCalcRunning_GFit;
	int			DontCalcStopped_GFit;

	int			DontGraphRunning_GFit;
	int			DontGraphStopped_GFit;

	int			RoAutoConvergence;
	int			AutoSpeedAdjust;
	int			WC_AutoGain[8];
	int			WC_Tracking;
	int			NoShowInclusion[8];
	int			Enable_ISO_correction;
	int			Enable_Live_GFit;
	int			SoftwareApertureMode;
	int			NoHelpText;
	int			ShowAdcHistogram;
	BOOL		FIR_AutoBaseline;
	BOOL		UseCTE;
	int			uFIR2_Gain;
	int			DeviceUsed;
	int			Apply_All_Cameras;
	int			TheView;
	int			WinCamExcellDataFile;
	int			Trigger_Timeout_ms;
	int			ForceTo90[8][LR_MODE];
	int			HyperCal;
	int			DisableCalc;
	int			TestState;
	int			Cmos_new;
	int			DisablePopups;

	int			Trigger_in_is_optical;
	int			LCM_Trigger_Mode;
	int			Initial_Load;
	int			CenterPosition_uM[2];///X Y position of center
	int			MultiBeamRegions; //MultiBeam continued
	int			MultiBeamPercentageHundredthsExp;
	int			PlateauUniformity;
	int			ShowHistogram;
	int			spare100[8];
	int			CompensationFileEnabled;
	char		CompensationFileFolder[120];
	int			DisableAutoReMap;

	BOOL		CalculatingGaussianFitU;
	BOOL		CalculatingGaussianFitV;

	BOOL		DisableSoftwareApertureForProfiles;

	int			DXXForceToCircle;			//used as a BOOL

	int			Rotate90AntiClockwise;

	int			OutlierFilter;

	int			AncillaryLensDivergenceEnabled[INDEX_TO_2][LAST_DEVICE][LR_MODE];
	BOOL		Hold_Max_Flag;				// When TRUE indicates that averaging is turned off and high water is applied per pixel.

	int 		HomogeneityNumberOfMaxima;
	int			InvertLLPSImageStitchingOrder;
	int			lldata_border_radio_state;	// Contains enum for auto/manual/custom.
	int			hg_radio_state;				// Contains enum for auto/manual/custom.


	int			SoftwareApertureModes[8];
	double		SoftwareApertureFixedDiameters[8];
	double 		SoftwareApertureRatios[8];
	double 		SoftwareApertureFixedXSizes[8];
	double		SoftwareApertureFixedYSizes[8];
	int			CenterApertureOnCoordinates[8]; //used as a BOOL
	int			UserSetApertureCoordinateXs[8]; //in terms of pixels
	int			UserSetApertureCoordinateYs[8]; //in terms of pixels

	int			svSpares[758];

	BOOL		pal_constructed;
	int			pal_lo;
	int			pal_hi;

	int			NormalizeFactor;				// This is propagated into the OpenGl 2D/3D programs.

	int			CenterApertureOnCoordinate; //used as a BOOL
	int			UserSetApertureCoordinateX; //in terms of pixels
	int			UserSetApertureCoordinateY; //in terms of pixels

	int			CentroidType;
	int			DeviceRunning[LAST_DEVICE];
	BOOL		ShowM2[LAST_DEVICE];

	BOOL		DisableCOMComs;
	BOOL		JitterControl;
	BOOL		ShowRotation;
	BOOL		BeamScopeAutoSearch;
	BOOL		LiveMode;
	BOOL		AttachNotes;
	BOOL		AutoNameFiles;

	BOOL		UseISOInclusionRegion;
	BOOL		Live3D;
	BOOL		Show3D;
	BOOL		AutoUpdate2D;
	BOOL		AutoUpdate3D;
	BYTE		oClipModes[INDEX_TO_2][Last_ID][LR_MODE];
	BYTE		BS_ShowMode;
	int			Test12;
	double		oClipLevels[INDEX_TO_2][Last_ID][LR_MODE];

	double		NewCorrectPower[LAST_DEVICE][LR_MODE];
	double		ViewAngle;
	double		TiltAngle;
	double		Mfactor[LAST_DEVICE][LR_MODE];
	double		SigmaPercent[LAST_DEVICE][LR_MODE];
	double		ScreenGamma;
	double		FluenceR;
	double		CrosshairAngle[8];
	double		SetCrosshairAngle[8];

	double		OverScan[LAST_DEVICE];
	double		Xc[LAST_DEVICE][8];
	double		Yc[LAST_DEVICE][8];
	double		Xg[LAST_DEVICE][8];
	double		Yg[LAST_DEVICE][8];
	double		Xp[LAST_DEVICE][8];
	double		Yp[LAST_DEVICE][8];
	double		Xu[LAST_DEVICE][8];
	double		Yu[LAST_DEVICE][8];

	double		Power[LAST_DEVICE];
	double		Exposure[MAX_CAMS];
	double		WinCamGains[8];

	int			Test13;

	double		InitialResult[LAST_DEVICE][LR_MODE];
	double		PowerFactor[LAST_DEVICE][LR_MODE];
	double		PowerInDB[LAST_DEVICE][LR_MODE];
	double		LastPower[LAST_DEVICE][LR_MODE];
	double		CorrectPower[LAST_DEVICE][LR_MODE];

	double		FluenceAvergeResult;
	double		FluenceAverageArea;
	double		FluenceAv;
	double		HomoGenEnd[2];

	double		FluencePeakResult;
	double		FluencePeakArea_um;
	double		FluenceDiameter_um;
	double		HomoGenActualStart[2];

	double		FluencePk;
	double		HomoGenStart[2];

	double		FluenceDefinedResult;
	double		FluenceDefinedArea_um;
	double		FluenceDef;
	double		HomoGenActualEnd[2];

	double		HomogeneityAutoPercentage;
	double		HomogeneityLowerThresholdPercentage;
	double		HomogeneityCustomPercentage;

	double		CurrentPixels[INDEX_TO_2];// neeeds link

	int			Test14;

	double		BaseLineMultiply;
	double		AutoTrigMax;
	double		AutoTrigMin;
	double		PixelFactorX[MAX_CAMS][LR_MODE];
	double		PixelFactorY[MAX_CAMS][LR_MODE];
	double		xPixelSize;
	double		yPixelSize;
	double		IrGamma;
	double		CompHi;
	double		CompLow;

	CPoint		CrosshairCenter[8];//class
	CPoint		PanCenter[8];//class

	// misc types
	CAPTURE_RECT		CaptureRect[8];//nested struct

	AUX_MOTOR			AuxMotor;//nested struct


	CRect				LastFluenceLocation;//class
	char				CompensationFileName[80];
	char				FluenceArray[120];
	char				SoftwareVersion[40];

	int					Test15;

	WC_IMAGE_DATA* ActiveImage;//nested struct
	BUTTON_DATA			ButtonData[Last_ID];//nested struct

	double				ScanSpan[3];
	double				ScanCenter[3];
	double				CentroidClip;
	double				SoftwareApertureFixedXSize;

	double				FixedAngle;

	double				ActiveAngle;
	double				SoftwareApertureFixedYSize;
	double				vAngle;

	double				ZeroXc[4][LAST_DEVICE][8];
	double				ZeroYc[4][LAST_DEVICE][8];

	double				Wavelength[LAST_DEVICE][LR_MODE];
	double				Zc;
	double				Filter[LR_MODE];
	double				AngularDisplacement[LAST_DEVICE][LR_MODE];

	double				PersistentData[60];

	double				TwoDMarks[4];
	double				GeoCentroidClip;
	double				CustomSlitSize;
	double				SlitRadiusMM;
	BYTE				CalSequnce[5][16];
	BYTE				CalSlitSize[5][16];
	double				YoffsetInuM;

	double				ram_Harm[4];
	double				ram_Phase[4];
	double				ram_Exp[4];
	double				ram_Amp[4];
	double				CorrectionAmplitude;
	double				WanderTimeInterval;

	double				GFitCoefficient[8];
	double				GFitRoughness[8];
	double				TwoD_OverScan;
	double				EWidthClipLevel[2];

	double				FitRatio;  // for auto zoom of thumbnail

	double				homogeneity_low_limit[2];
	double				homogeneity_high_limit[2];
	double				double_spares[2];
	double				SourceFrequncy;
	double				LimitGain[2];
	double				LimitAdcPeak[2];
	double				dLockedBaseline;
	double				old_SetGain[8];
	double				FluenceTotalArea;
	double				ReferencePower;
	double				DelayMilliseconds;
	double				ShutterMilliseconds;
	double				PowerShutterSetting[LAST_DEVICE][8];

	double				ClipLevels[INDEX_TO_2][LR_MODE];

	double				UcmZoDelta;

	double				LockedXc[4][LAST_DEVICE][8];
	double				LockedYc[4][LAST_DEVICE][8];

	double				VEB_offset;


	double				GridSizeInMilliMeters;
	double				CenterSizeInMilliMeters;
	double				CentroidSizeInMilliMeters;

	double				GridToleranceInMicrons;

	double				GridOffset[2];

	double				DefaultPowerInput[2];
	double				Radius_In_Microns[2];

	double				RoTarget;

	double				SoftwareApertureFixedDiameter;

	double				SoftwareApertureRatio;

	double				VEB_fixed_offset;

	double				BeamScopeOffset;
	double				BeamScopeGain;
	double				BeamMapGain;
	double				BeamMapOffset;

	double				Not_Auto_Gain;

	double				PowerToInclude;
	double				Camera_VEB_offset[4];

	double				LineLaserStart_um;
	double				LineLaserEnd_um;
	double				frameRateThrottle;

	double				lldata_begin_um;			// Defines the start of the line laser data collection aperture.
	double				lldata_end_um;				// Defines the end of the line laser data collection aperture.
	double				lldata_cutoff;				// Sets the automatic size of the line laser data collection aperture, as magnitude percentage.

	double				Spare0[2];
	double				Spare1[2];

	double				lldata_percent_of_line;

	double				doubleSpares[1];

	double				ISO_Cliplevel;

	double				PlsFactor;
	int					Test16;

	int					LogEnabledData[INDEX_TO_4][LAST_DEVICE];
	double				LogAtSeconds[INDEX_TO_4][LAST_DEVICE];
	double				LogForHours[INDEX_TO_4][LAST_DEVICE];
	char				LogFileName[INDEX_TO_4][LAST_DEVICE][512];

	INCLUSION_REGION	InclusionRegion[INDEX_TO_4][LR_MODE];//nested struct
	char				IRerror[200];
	char				PrintNotes[LAST_DEVICE][2000];
	int					Test17;
	char				Notes[LAST_DEVICE][LR_MODE][512];

	int					Test18;
	char				LastSoftwareVersion[40];
	int					Test19;
	int					Test20;

	char				PassWord[40];
	DWORD				Signature2;		// = (int('D')<<24)+(int('R')<<16)+(int('I')<<8)+'.' ;
}	SAVED_DATA2;
typedef struct {
	unsigned char m_char;
	unsigned long m_long;
	std::string m_string;
	unsigned char m_fixed_array[100][100];
	unsigned char m_flex_array[1];
} SaveSample;