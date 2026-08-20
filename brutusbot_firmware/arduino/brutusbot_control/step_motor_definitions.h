constexpr int kMotor1_Pin_Ena = 4;
constexpr int kMotor1_Pin_Dir = 3;
constexpr int kMotor1_Pin_Clk = 2;

constexpr int kMotor2_Pin_Ena = 7;
constexpr int kMotor2_Pin_Dir = 6;
constexpr int kMotor2_Pin_Clk = 5;

constexpr int kMotor1_FullStepsPerRev = 200;
constexpr int kMotor1_DriveMicroSteps = 8;
constexpr int kMotor1_EffectiveStepsPerRev = kMotor1_FullStepsPerRev * kMotor1_DriveMicroSteps;
constexpr float kMotor1_FactorDegreesToEffSteps = (float)kMotor1_EffectiveStepsPerRev / 360.0;
constexpr float kMotor1_FactorEffStepsToDegrees = 360.0 / kMotor1_EffectiveStepsPerRev;

constexpr float kMotor1_MaxSpeedEffSteps = 5000.0; // tunned with no load and drive setup of 8 micro steps
constexpr float kMotor1_MaxAccelEffSteps = 5000.0; // to be tunned yet

constexpr float kMotor1_MaxSpeed = kMotor1_MaxSpeedEffSteps * kMotor1_FactorEffStepsToDegrees;
constexpr float kMotor1_MaxAccel = kMotor1_MaxAccelEffSteps * kMotor1_FactorEffStepsToDegrees;