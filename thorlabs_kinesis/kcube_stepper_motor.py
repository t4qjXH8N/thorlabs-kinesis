"Bindings for Thorlabs Integrated Stepper Motor DLL"
from ctypes import (
    Structure,
    cdll,
    c_bool,
    c_short,
    c_int,
    c_uint,
    c_int16,
    c_int32,
    c_char,
    c_byte,
    c_long,
    c_float,
    c_double,
    POINTER,
    CFUNCTYPE,
)

from thorlabs_kinesis._utils import (
    c_word,
    c_dword,
    bind
)

lib = cdll.LoadLibrary("Thorlabs.MotionControl.KCube.StepperMotor.dll")

# enum FT_Status
FT_OK = c_short(0x00)
FT_InvalidHandle = c_short(0x01)
FT_DeviceNotFound = c_short(0x02)
FT_DeviceNotOpened = c_short(0x03)
FT_IOError = c_short(0x04)
FT_InsufficientResources = c_short(0x05)
FT_InvalidParameter = c_short(0x06)
FT_DeviceNotPresent = c_short(0x07)
FT_IncorrectDevice = c_short(0x08)
FT_Status = c_short

# enum MOT_MotorTypes
MOT_NotMotor = c_int(0)
MOT_DCMotor = c_int(1)
MOT_StepperMotor = c_int(2)
MOT_BrushlessMotor = c_int(3)
MOT_CustomMotor = c_int(100)
MOT_MotorTypes = c_int

# enum MOT_TravelModes
MOT_TravelModeUndefined = c_int(0x00)
MOT_Linear = c_int(0x01)
MOT_Rotational = c_int(0x02)
MOT_TravelModes = c_int

# enum MOT_TravelDirection
MOT_TravelDirectionUndefined = c_short(0x00)
MOT_Forwards = c_short(0x01)
MOT_Reverse = c_short(0x02)
MOT_TravelDirection = c_short

# enum MOT_HomeLimitSwitchDirection
MOT_LimitSwitchDirectionUndefined = c_short(0x00)
MOT_ReverseLimitSwitch = c_short(0x01)
MOT_ForwardLimitSwitch = c_short(0x04)
MOT_HomeLimitSwitchDirection = c_short

# enum MOT_DirectionSense
MOT_Normal = c_short(0x00)
MOT_Backwards = c_short(0x01)
MOT_DirectionSense = c_short
KMOT_WheelDirectionSense = c_short

# enum MOT_JogModes
MOT_JogModeUndefined = c_short(0x00)
MOT_Continuous = c_short(0x01)
MOT_SingleStep = c_short(0x02)
MOT_JogModes = c_short

# enum MOT_StopModes
MOT_StopModeUndefined = c_short(0x00)
MOT_Immediate = c_short(0x01)
MOT_Profiled = c_short(0x02)
MOT_StopModes = c_short

# enum MOT_ButtonModes
MOT_ButtonModeUndefined = c_word(0x00)
MOT_JogMode = c_word(0x01)
MOT_Preset = c_word(0x02)
MOT_ButtonModes = c_word

# enum MOT_LimitSwitchModes
MOT_LimitSwitchModeUndefined = c_word(0x00)
MOT_LimitSwitchIgnoreSwitch = c_word(0x01)
MOT_LimitSwitchMakeOnContact = c_word(0x02)
MOT_LimitSwitchBreakOnContact = c_word(0x03)
MOT_LimitSwitchMakeOnHome = c_word(0x04)
MOT_LimitSwitchBreakOnHome = c_word(0x05)
MOT_PMD_Reserved = c_word(0x06)
MOT_LimitSwitchIgnoreSwitchSwapped = c_word(0x81)
MOT_LimitSwitchMakeOnContactSwapped = c_word(0x82)
MOT_LimitSwitchBreakOnContactSwapped = c_word(0x83)
MOT_LimitSwitchMakeOnHomeSwapped = c_word(0x84)
MOT_LimitSwitchBreakOnHomeSwapped = c_word(0x85)
MOT_LimitSwitchModes = c_word

# enum MOT_LimitSwitchSWModes
MOT_LimitSwitchSWModeUndefined = c_word(0x00)
MOT_LimitSwitchIgnored = c_word(0x01)
MOT_LimitSwitchStopImmediate = c_word(0x02)
MOT_LimitSwitchStopProfiled = c_word(0x03)
MOT_LimitSwitchIgnored_Rotational = c_word(0x81)
MOT_LimitSwitchStopImmediate_Rotational = c_word(0x82)
MOT_LimitSwitchStopProfiled_Rotational = c_word(0x83)
MOT_LimitSwitchSWModes = c_word

# enum MOT_LimitsSoftwareApproachPolicy
DisallowIllegalMoves = c_int16(0)
AllowPartialMoves = c_int16(1)
AllowAllMoves = c_int16(2)
MOT_LimitsSoftwareApproachPolicy = c_int16

# enum MOT_PID_LoopMode
MOT_PIDLoopModeDisabled = c_word(0x00)
MOT_PIDOpenLoopMode = c_word(0x01)
MOT_PIDClosedLoopMode = c_word(0x02)
MOT_PID_LoopMode = c_word

# enum MOT_MovementModes
LinearRange = c_int(0x00)
RotationalUnlimited = c_int(0x01)
RotationalWrapping = c_int(0x02)
MOT_MovementModes = c_int

# enum MOT_MovementDirections
Quickest = c_int(0x00)
Forwards = c_int(0x01)
Reverse = c_int(0x02)
MOT_MovementDirections = c_int

# KMOT wheel mode
KMOT_WheelModeConstantVelocity = c_short(0x01)
KMOT_WheelModeJog = c_short(0x02)
KMOT_WheelModeMoveAbsolute = c_short(0x03)
KMOT_WheelMode = c_short

# KMOT Trigger Port Mode
KMOT_TriggerPortModeDisabled = c_short(0x00)
KMOT_TriggerPortModeInputGPL = c_short(0x01)
KMOT_TriggerPortModeInputMoveRelative = c_short(0x02)
KMOT_TriggerPortModeInputMoveAbsolute = c_short(0x03)
KMOT_TriggerPortModeInputMoveHome = c_short(0x04)
KMOT_TriggerPortMode = c_short

# TODO: further trigger states
#10 Trigger Output - General purpose output (SetDigitalOutputs)
#11 Trigger Output - Set when device moving
#12 Trigger Output - Set when at max velocity
#13 Trigger Output - Set when at predefine position steps
#14 Trigger Output - TBD mode

# KMOT Trigger Port Polarity
KMOT_TriggerPortPolarityHigh = c_short(0x01)
KMOT_TriggerPortPolarityLow = c_short(0x02)
KMOT_TriggerPortPolarity = c_short

# Stages Type
KST_StageTypeZST6 = c_word(0x20)
KST_StageTypeZST13 = c_word(0x21)
KST_StageTypeZST25 = c_word(0x22)
KST_StageTypeZST206 = c_word(0x30)
KST_StageTypeZST213 = c_word(0x31)
KST_StageTypeZST225 = c_word(0x32)
KST_StageTypeZFS206 = c_word(0x40)
KST_StageTypeZFS213 = c_word(0x41)
KST_StageTypeZFS225 = c_word(0x42)
KST_StageTypeNR360 = c_word(0x70)
KST_StageTypePLSX = c_word(0x72)
KST_StageTypePLSXHIRES = c_word(0x73)
KST_StageTypePLSFW103 = c_word(0x75)
KST_StageType = c_word


class KMOT_MMIParams(Structure):
    _fields_ = [
        ("DisplayDimIntensity", c_int16),
        ("DisplayIntensity", c_int16),
        ("DisplayTimeout", c_int16),
        ("PresetPos1", c_int32),
        ("PresetPos2", c_int32),
        ("Reserved", (4 * c_int16)),
        ("WheelAccelaration", c_int32),
        ("WheelDirectionSense", MOT_DirectionSense),
        ("WheelMaxVelocity", c_int32),
        ("WheelMode", KMOT_WheelMode)
    ]


class KMOT_TriggerConfig(Structure):
    _fields_ = [
        ("Trigger1Mode", KMOT_TriggerPortMode),
        ("Trigger1Polarity", KMOT_TriggerPortPolarity),
        ("Trigger2Mode", KMOT_TriggerPortMode),
        ("Trigger2Polarity", KMOT_TriggerPortPolarity)
    ]


class KMOT_TriggerParams(Structure):
    _fields_ = [
        ("CycleCount", c_int32),
        ("reserved", (6 * c_int32)),
        ("TriggerIntervalFwd", c_int32),
        ("TriggerIntervalRev", c_int32),
        ("TriggerPulseCountFwd", c_int32),
        ("TriggerPulseCountRev", c_int32),
        ("TriggerPulseWidth", c_int32),
        ("TriggerStartPositionFwd", c_int32),
        ("TriggerStartPositionRev", c_int32)
    ]


class MOT_HomingParameters(Structure):
    _fields_ = [("direction", MOT_TravelDirection),
                ("limitSwitch", MOT_HomeLimitSwitchDirection),
                ("offsetDistance", c_uint),
                ("velocity", c_uint)]


class MOT_VelocityParameters(Structure):
    _fields_ = [("acceleration", c_int),
                ("maxVelocity", c_int),
                ("minVelocity", c_int)
                ]


class MOT_JogParameters(Structure):
    _fields_ = [("mode", MOT_JogModes),
                ("stepSize", c_uint),
                ("stopMode", MOT_StopModes),
                ("velParams", MOT_VelocityParameters)
                ]


class MOT_LimitSwitchParameters(Structure):
    _fields_ = [("anticlockwiseHardwareLimit", MOT_LimitSwitchModes),
                ("anticlockwisePosition", c_dword),
                ("clockwiseHardwareLimit", MOT_LimitSwitchModes),
                ("clockwisePosition", c_dword),
                ("softLimitMode", MOT_LimitSwitchSWModes)]


class MOT_PIDLoopEncoderParams(Structure):
    _fields_ = [("differentialGain", c_int),
                ("integralGain", c_int),
                ("loopMode", MOT_PID_LoopMode),
                ("PIDOutputLimit", c_int),
                ("PIDTolerance", c_int),
                ("proportionalGain", c_int)
                ]


class MOT_PowerParameters(Structure):
    _fields_ = [("movePercentage", c_word),
                ("restPercentage", c_word)
                ]


class TLI_DeviceInfo(Structure):
    _fields_ = [("description", (65 * c_char)),
                ("isCustomType", c_bool),
                ("isKnownType", c_bool),
                ("isLaser", c_bool),
                ("isPiezoDevice", c_bool),
                ("isRack", c_bool),
                ("maxChannels", c_short),
                ("maxPaddles", c_short),
                ("motorType", MOT_MotorTypes),
                ("PID", c_dword),
                ("serialNo", (9 * c_char)),
                ("typeID", c_dword)]


class TLI_HardwareInformation(Structure):
    _fields_ = [("deviceDependantData", (12 * c_byte)),
                ("firmwareVersion", c_dword),
                ("hardwareVersion", c_word),
                ("modelNumber", (8 * c_char)),
                ("modificationState", c_word),
                ("notes", (48 * c_char)),
                ("numChannels", c_short),
                ("serialNumber", c_dword),
                ("type", c_word)
                ]


TLI_BuildDeviceList = bind(lib, "TLI_BuildDeviceList", None, c_short)
TLI_GetDeviceInfo = bind(lib, "TLI_GetDeviceInfo", [POINTER(c_char), POINTER(TLI_DeviceInfo)], c_short)
TLI_GetDeviceListExt = bind(lib, "TLI_GetDeviceListExt", [POINTER(c_char), c_dword], c_short)
TLI_GetDeviceListSize = bind(lib, "TLI_GetDeviceListSize", None, c_short)
TLI_GetDeviceListByTypeExt = bind(lib, "TLI_GetDeviceListByTypeExt", [POINTER(c_char), c_dword, c_int], c_short)
TLI_GetDeviceListByTypesExt = bind(lib, "TLI_GetDeviceListByTypesExt", [POINTER(c_char), c_dword, POINTER(c_int), c_int], c_short)

# TLI_GetDeviceList  <- TODO: Implement SAFEARRAY first. BENCHTOPSTEPPERMOTOR_API short __cdecl TLI_GetDeviceList(SAFEARRAY** stringsReceiver);
# TLI_GetDeviceListByType  <- TODO: Implement SAFEARRAY first. BENCHTOPSTEPPERMOTOR_API short __cdecl TLI_GetDeviceListByType(SAFEARRAY** stringsReceiver, int typeID);
# TLI_GetDeviceListByTypes  <- TODO: Implement SAFEARRAY first. BENCHTOPSTEPPERMOTOR_API short __cdecl TLI_GetDeviceListByTypes(SAFEARRAY** stringsReceiver, int * typeIDs, int length);


SCC_CanDeviceLockFrontPanel = bind(lib, "SCC_CanDeviceLockFrontPanel", [POINTER(c_bool)], c_bool)
SCC_CanHome = bind(lib, "SCC_CanHome", [POINTER(c_char)], c_bool)
SCC_CanMoveWithoutHomingFirst = bind(lib, "SCC_CanMoveWithoutHomingFirst", [POINTER(c_char)], c_bool)
SCC_CheckConnection = bind(lib, "SCC_CheckConnection", [POINTER(c_char)], c_bool)
SCC_ClearMessageQueue = bind(lib, "SCC_ClearMessageQueue", [POINTER(c_char)], c_short)
SCC_Close = bind(lib, "SCC_Close", [POINTER(c_char)], c_short)
SCC_DisableChannel = bind(lib, "SCC_DisableChannel", [POINTER(c_char)], c_short)
SCC_EnableChannel = bind(lib, "SCC_EnableChannel", [POINTER(c_char)], c_short)
SCC_EnableLastMsgTimer = bind(lib, "SCC_EnableLastMsgTimer", [POINTER(c_char), c_bool, c_int32])
SCC_GetBacklash = bind(lib, "SCC_GetBacklash", [POINTER(c_char)], c_long)
SCC_GetBowIndex = bind(lib, "SCC_GetBowIndex", [POINTER(c_char)], c_short)
SCC_GetCalibrationFile = bind(lib, "SCC_GetCalibrationFile", [POINTER(c_char), POINTER(c_char), c_short], c_bool)
SCC_GetDeviceUnitFromRealValue = bind(lib, "SCC_GetDeviceUnitFromRealValue", [POINTER(c_char), c_double, POINTER(c_int), c_int], c_short)
SCC_GetDigitalOutputs = bind(lib, "SCC_GetDigitalOutputs", [POINTER(c_char)], c_byte)
SCC_GetEncoderCounter = bind(lib, "SCC_GetEncoderCounter", [POINTER(c_char)], c_long)
SCC_GetFrontPanelLocked = bind(lib, "SCC_GetFrontPanelLocked", [POINTER(c_char)], c_bool)
SCC_GetHardwareInfo = bind(lib, "SCC_GetHardwareInfo", [POINTER(c_char), POINTER(c_char), c_dword, POINTER(c_word), POINTER(c_word), POINTER(c_char), c_dword, POINTER(c_dword), POINTER(c_word), POINTER(c_word)], c_short)
SCC_GetHardwareInfoBlock = bind(lib, "SCC_GetHardwareInfoBlock", [POINTER(c_char), POINTER(TLI_HardwareInformation)], c_short)
SCC_GetHomingParamsBlock = bind(lib, "SCC_GetHomingParamsBlock", [POINTER(c_char), POINTER(MOT_HomingParameters)], c_short)
SCC_GetHomingVelocity = bind(lib, "SCC_GetHomingVelocity", [POINTER(c_char)], c_uint)
SCC_GetHubBay = bind(lib, "SCC_GetHubBay", [POINTER(c_char)], c_char)
SCC_GetJogMode = bind(lib, "SCC_GetJogMode", [POINTER(c_char), POINTER(MOT_JogModes), POINTER(MOT_StopModes)], c_short)
SCC_GetJogParamsBlock = bind(lib, "SCC_GetJogParamsBlock", [POINTER(c_char), POINTER(MOT_JogParameters)], c_short)
SCC_GetJogStepSize = bind(lib, "SCC_GetJogStepSize", [POINTER(c_char)], c_uint)
SCC_GetJogVelParams = bind(lib, "SCC_GetJogVelParams", [POINTER(c_char), POINTER(c_int), POINTER(c_int)], c_short)
SCC_GetLimitSwitchParams = bind(lib, "SCC_GetLimitSwitchParams", [POINTER(c_char), POINTER(MOT_LimitSwitchModes), POINTER(MOT_LimitSwitchModes), POINTER(c_uint), POINTER(c_uint), POINTER(MOT_LimitSwitchSWModes)], c_short)
SCC_GetLimitSwitchParamsBlock = bind(lib, "SCC_GetLimitSwitchParamsBlock", [POINTER(c_char), POINTER(MOT_LimitSwitchParameters)], c_short)
SCC_GetMMIParams = bind(lib, "SCC_GetMMIParams", [POINTER(c_char), POINTER(KMOT_WheelMode), POINTER(c_int32), POINTER(c_int32), POINTER(KMOT_WheelDirectionSense), POINTER(c_int32), POINTER(c_int32), POINTER(c_int16)])
SCC_GetMMIParamsBlock = bind(lib, "SCC_GetMMIParamsBlock", [POINTER(KMOT_MMIParams)], c_short)
SCC_GetMMIParamsExt = bind(lib, "SCC_GetMMIParamsExt", [POINTER(c_char), POINTER(KMOT_WheelMode), POINTER(c_int32), POINTER(c_int32), POINTER(KMOT_WheelDirectionSense), POINTER(c_int32), POINTER(c_int32), POINTER(c_int16), POINTER(c_int16), POINTER(c_int16)], c_short)
SCC_GetMotorParams = bind(lib, "SCC_GetMotorParams", [POINTER(c_char), POINTER(c_long), POINTER(c_long), POINTER(c_float)], c_short)
SCC_GetMotorParamsExt = bind(lib, "SCC_GetMotorParamsExt", [POINTER(c_char), POINTER(c_double), POINTER(c_double), POINTER(c_double)], c_short)
SCC_GetMotorTravelLimits = bind(lib, "SCC_GetMotorTravelLimits", [POINTER(c_char), POINTER(c_double), POINTER(c_double)], c_short)
SCC_GetMotorTravelMode = bind(lib, "SCC_GetMotorTravelMode", [POINTER(c_char)], MOT_TravelModes)
SCC_GetMotorVelocityLimits = bind(lib, "SCC_GetMotorVelocityLimits", [POINTER(c_char), POINTER(c_double), POINTER(c_double)], c_short)
SCC_GetMoveAbsolutePosition = bind(lib, "SCC_GetMoveAbsolutePosition", [POINTER(c_char)], c_int)
SCC_GetMoveRelativeDistance = bind(lib, "SCC_GetMoveRelativeDistance", [POINTER(c_char)], c_int)
SCC_GetNextMessage = bind(lib, "SCC_GetNextMessage", [POINTER(c_char), POINTER(c_word), POINTER(c_word), POINTER(c_dword)], c_bool)
SCC_GetNumberPositions = bind(lib, "SCC_GetNumberPositions", [POINTER(c_char)], c_int)
SCC_GetPIDLoopEncoderCoeff = bind(lib, "SCC_GetPIDLoopEncoderCoeff", [POINTER(c_char)], c_double)
SCC_GetPIDLoopEncoderParams = bind(lib, "SCC_GetPIDLoopEncoderParams", [POINTER(c_char), POINTER(MOT_PIDLoopEncoderParams)], c_short)
SCC_GetPosition = bind(lib, "SCC_GetPosition", [POINTER(c_char)], c_int)
SCC_GetPositionCounter = bind(lib, "SCC_GetPositionCounter", [POINTER(c_char)], c_long)
SCC_GetPowerParams = bind(lib, "SCC_GetPowerParams", [POINTER(c_char), POINTER(MOT_PowerParameters)], c_short)
SCC_GetRealValueFromDeviceUnit = bind(lib, "SCC_GetRealValueFromDeviceUnit", [POINTER(c_char), c_int, POINTER(c_double), c_int], c_short)
SCC_GetSoftLimitMode = bind(lib, "SCC_GetSoftLimitMode", [POINTER(c_char)], MOT_LimitsSoftwareApproachPolicy)
SCC_GetSoftwareVersion = bind(lib, "SCC_GetSoftwareVersion", [POINTER(c_char)])
SCC_GetStageAxisMaxPos = bind(lib, "SCC_GetStageAxisMaxPos", [POINTER(c_char)], c_int)
SCC_GetStageAxisMinPos = bind(lib, "SCC_GetStageAxisMinPos", [POINTER(c_char)], c_int)
SCC_GetStatusBits = bind(lib, "SCC_GetStatusBits", [POINTER(c_char)], c_dword)
#SCC_GetTriggerConfigParams = bind(lib, "SCC_GetTriggerConfigParams", [POINTER(c_char), POINTER(KMO)])
# TODO: Trigger parameters
SCC_GetVelParams = bind(lib, "SCC_GetVelParams", [POINTER(c_char), POINTER(c_int), POINTER(c_int)], c_short)
SCC_GetVelParamsBlock = bind(lib, "SCC_GetVelParamsBlock", [POINTER(c_char), POINTER(MOT_VelocityParameters)], c_short)
SCC_HasLastMsgTimerOverrun = bind(lib, "SCC_HasLastMsgTimerOverrun", [POINTER(c_char)], c_bool)
SCC_Home = bind(lib, "SCC_Home", [POINTER(c_char)], c_short)
SCC_Identify = bind(lib, "SCC_Identify", [POINTER(c_char)])
SCC_IsCalibrationActive = bind(lib, "SCC_IsCalibrationActive", [POINTER(c_char)], c_bool)
SCC_LoadNamedSettings = bind(lib, "SCC_LoadNamedSettings", [POINTER(c_char), POINTER(c_char)])
SCC_LoadSettings = bind(lib, "SCC_LoadSettings", [POINTER(c_char)], c_bool)
SCC_MessageQueueSize = bind(lib, "SCC_MessageQueueSize", [POINTER(c_char)], c_int)
SCC_MoveAbsolute = bind(lib, "SCC_MoveAbsolute", [POINTER(c_char)], c_short)
SCC_MoveAtVelocity = bind(lib, "SCC_MoveAtVelocity", [POINTER(c_char), MOT_TravelDirection], c_short)
SCC_MoveJog = bind(lib, "SCC_MoveJog", [POINTER(c_char), MOT_TravelDirection], c_short)
SCC_MoveRelative = bind(lib, "SCC_MoveRelative", [POINTER(c_char), c_int], c_short)
SCC_MoveRelativeDistance = bind(lib, "SCC_MoveRelativeDistance", [POINTER(c_char)], c_short)
SCC_MoveToPosition = bind(lib, "SCC_MoveToPosition", [POINTER(c_char), c_int], c_short)
SCC_NeedsHoming = bind(lib, "SCC_NeedsHoming", [POINTER(c_char)], c_bool)
SCC_Open = bind(lib, "SCC_Open", [POINTER(c_char)], c_short)
SCC_PersistSettings = bind(lib, "SCC_PersistSettings", [POINTER(c_char)], c_bool)
SCC_PollingDuration = bind(lib, "SCC_PollingDuration", [POINTER(c_char)], c_long)
SCC_RegisterMessageCallback = bind(lib, "SCC_RegisterMessageCallback", [POINTER(c_char), CFUNCTYPE(None)])
SCC_RequestBacklash = bind(lib, "SCC_RequestBacklash", [POINTER(c_char)], c_short)
SCC_RequestBowIndex = bind(lib, "SCC_RequestBowIndex", [POINTER(c_char)], c_short)
SCC_RequestDigitalOutputs = bind(lib, "SCC_RequestTriggerSwitches", [POINTER(c_char)], c_short)
SCC_RequestEncoderCounter = bind(lib, "SCC_RequestEncoderCounter", [POINTER(c_char)], c_short)
SCC_RequestFrontPanelLocked = bind(lib, "SCC_FrontPanelLocked", [POINTER(c_char)], c_short)
SCC_RequestHomingParams = bind(lib, "SCC_RequestHomingParams", [POINTER(c_char)], c_short)
SCC_RequestJogParams = bind(lib, "SCC_RequestJogParams", [POINTER(c_char)], c_short)
SCC_RequestLimitSwitchParams = bind(lib, "SCC_RequestLimitSwitchParams", [POINTER(c_char)], c_short)
SCC_RequestMMIParams = bind(lib, "SCC_RequestMMIParams", [POINTER(c_char)], c_short)
SCC_RequestMoveAbsolutePosition = bind(lib, "SCC_RequestMoveAbsolutePosition", [POINTER(c_char)], c_short)
SCC_RequestMoveRelativeDistance = bind(lib, "SCC_RequestMoveRelativeDistance", [POINTER(c_char)], c_short)
SCC_RequestPIDLoopEncoderParams = bind(lib, "SCC_RequestPIDLoopEncoderParams", [POINTER(c_char)], c_short)
SCC_RequestPosition = bind(lib, "SCC_RequestPosition", [POINTER(c_char)], c_short)
SCC_RequestPosTriggerParams = bind(lib, "SCC_RequestPosTriggerParams", [POINTER(c_char)], c_short)
SCC_RequestPowerParams = bind(lib, "SCC_RequestPowerParams", [POINTER(c_char)], c_short)
SCC_RequestSettings = bind(lib, "SCC_RequestSettings", [POINTER(c_char)], c_short)
SCC_RequestStatusBits = bind(lib, "SCC_RequestStatusBits", [POINTER(c_char)], c_short)
SCC_RequestVelParams = bind(lib, "SCC_RequestVelParams", [POINTER(c_char)], c_short)
SCC_ResetRotationModes = bind(lib, "SCC_ResetRotationModes", [POINTER(c_char)], c_short)
SCC_ResumeMoveMessages = bind(lib, "SCC_ResumeMoveMessages", [POINTER(c_char)], c_short)
SCC_SetBacklash = bind(lib, "SCC_SetBacklash", [POINTER(c_char), c_long], c_short)
SCC_SetBowIndex = bind(lib, "SCC_SetBowIndex", [POINTER(c_char), c_short], c_short)
SCC_SetCalibrationFile = bind(lib, "SCC_SetCalibrationFile", [POINTER(c_char), POINTER(c_char), c_bool])
SCC_SetDigitalOutputs = bind(lib, "SCC_SetDigitalOutputs", [POINTER(c_char), c_byte], c_short)
SCC_SetDirection = bind(lib, "SCC_SetDirection", [POINTER(c_char), c_bool], c_short)
SCC_SetEncoderCounter = bind(lib, "SCC_SetEncoderCounter", [POINTER(c_char), c_long], c_short)
SCC_SetFrontPanelLock = bind(lib, "SetFrontPanelLock", [POINTER(c_char), c_bool], c_short)
SCC_SetHomingParamsBlock = bind(lib, "SCC_SetHomingParamsBlock", [POINTER(c_char), POINTER(MOT_HomingParameters)], c_short)
SCC_SetHomingVelocity = bind(lib, "SCC_SetHomingVelocity", [POINTER(c_char), c_uint], c_short)
SCC_SetJogMode = bind(lib, "SCC_SetJogMode", [POINTER(c_char), MOT_JogModes, MOT_StopModes], c_short)
SCC_SetJogParamsBlock = bind(lib, "SCC_SetJogParamsBlock", [POINTER(c_char), POINTER(MOT_JogParameters)], c_short)
SCC_SetJogStepSize = bind(lib, "SCC_SetJogStepSize", [POINTER(c_char), c_uint], c_short)
SCC_SetJogVelParams = bind(lib, "SCC_SetJogVelParams", [POINTER(c_char), c_int, c_int], c_short)
SCC_SetLimitsSoftwareApproachPolicy = bind(lib, "SCC_SetLimitsSoftwareApproachPolicy", [POINTER(c_char), MOT_LimitsSoftwareApproachPolicy])
SCC_SetLimitSwitchParams = bind(lib, "SCC_SetLimitSwitchParams", [POINTER(c_char), MOT_LimitSwitchModes, MOT_LimitSwitchModes, c_uint, c_uint, MOT_LimitSwitchSWModes], c_short)
SCC_SetLimitSwitchParamsBlock = bind(lib, "SCC_SetLimitSwitchParamsBlock", [POINTER(c_char), POINTER(MOT_LimitSwitchParameters)], c_short)
SCC_SetMMIParams = bind(lib, "SCC_SetMMIParams", [POINTER(c_char), KMOT_WheelMode, c_int32, c_int32, KMOT_WheelDirectionSense, c_int32, c_int32, c_int16], c_short)
SCC_SetMMIParamsBlock = bind(lib, "SCC_SetMMIParamsBlock", [POINTER(c_char), KMOT_MMIParams], c_short)
SCC_SetMMIParamsExt = bind(lib, "SCC_SetMMIParamsExt", [POINTER(c_char), KMOT_WheelMode, c_int32, c_int32, KMOT_WheelDirectionSense, c_int32, c_int32, c_int16, c_int16, c_int16])
SCC_SetMotorParams = bind(lib, "SCC_SetMotorParams", [POINTER(c_char), c_long, c_long, c_float], c_short)
SCC_SetMotorParamsExt = bind(lib, "SCC_SetMotorParamsExt", [POINTER(c_char), c_double, c_double, c_double], c_short)
SCC_SetMotorTravelLimits = bind(lib, "SCC_SetMotorTravelLimits", [POINTER(c_char), c_double, c_double], c_short)
SCC_SetMotorTravelMode = bind(lib, "SCC_SetMotorTravelMode", [POINTER(c_char), MOT_TravelModes], c_short)
SCC_SetMotorVelocityLimits = bind(lib, "SCC_SetMotorVelocityLimits", [POINTER(c_char), c_double, c_double], c_short)
SCC_SetMoveAbsolutePosition = bind(lib, "SCC_SetMoveAbsolutePosition", [POINTER(c_char), c_int], c_short)
SCC_SetMoveRelativeDistance = bind(lib, "SCC_SetMoveRelativeDistance", [POINTER(c_char), c_int], c_short)
SCC_SetPIDLoopEncoderCoeff = bind(lib, "SCC_SetPIDLoopEncoderCoeff", [POINTER(c_char), c_double], c_short)
SCC_SetPIDLoopEncoderParams = bind(lib, "SCC_SetPIDLoopEncoderParams", [POINTER(c_char), POINTER(MOT_PIDLoopEncoderParams)], c_short)
SCC_SetPositionCounter = bind(lib, "SCC_SetPositionCounter", [POINTER(c_char), c_long], c_short)
SCC_SetPowerParams = bind(lib, "SCC_SetPowerParams", [POINTER(c_char), POINTER(MOT_PowerParameters)], c_short)
SCC_SetRotationModes = bind(lib, "SCC_SetRotationModes", [POINTER(c_char), MOT_MovementModes, MOT_MovementDirections], c_short)
SCC_SetStageAxisLimits = bind(lib, "SCC_SetStageAxisLimits", [POINTER(c_char), c_int, c_int], c_short)
SCC_SetStageType = bind(lib, "SCC_SetStageType", [POINTER(c_char), KST_StageType])
SCC_SetVelParams = bind(lib, "SCC_SetVelParams", [POINTER(c_char), c_int, c_int], c_short)
SCC_SetVelParamsBlock = bind(lib, "SCC_SetVelParamsBlock", [POINTER(c_char), POINTER(MOT_VelocityParameters)], c_short)
SCC_StartPolling = bind(lib, "SCC_StartPolling", [POINTER(c_char), c_int], c_bool)
SCC_StopImmediate = bind(lib, "SCC_StopImmediate", [POINTER(c_char)], c_short)
SCC_StopPolling = bind(lib, "SCC_StopPolling", [POINTER(c_char)])
SCC_StopProfiled = bind(lib, "SCC_StopProfiled", [POINTER(c_char)], c_short)
SCC_SuspendMoveMessages = bind(lib, "SCC_SuspendMoveMessages", [POINTER(c_char)], c_short)
SCC_UsesPIDLoopEncoding = bind(lib, "SCC_UsesPIDLoopEncoding", [POINTER(c_char)], c_bool)
SCC_WaitForMessage = bind(lib, "SCC_WaitForMessage", [POINTER(c_char), POINTER(c_word), POINTER(c_word), POINTER(c_dword)], c_bool)