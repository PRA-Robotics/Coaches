package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRelicRecovery;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus;

@TeleOp(name = "RoverVuforia_V02_01", group = "")
public class RoverVuforia_V02_01 extends LinearOpMode {

  private VuforiaRoverRuckus vuforiaRoverRuckus;
  private AndroidTextToSpeech androidTextToSpeech;
  private DcMotor rightMotor;
  private DcMotor leftMotor;
  private TfodRoverRuckus tfodRoverRuckus;
  private VuforiaRelicRecovery vuforiaRelicRecovery;

  VuforiaBase.TrackingResults vuMarkResult;
  String DepotString;
  double avgRotZ;
  String XYZstring;
  double avgX;
  double avgY;
  double sumX;
  double sumY;
  double sumRotZ;
  String TrackableName;
  double BlueDepotX;
  double BlueDepotY;
  double RedDepotX;
  double RedDepotY;
  double BluDepotRot;
  double RedDepotRot;
  double BluXDelta;
  double BluYDelta;
  double RedXDelta;
  double RedYDelta;
  double BluDepotLen;
  double RedDepotLen;
  String PwrString;
  double TurnPwr;
  double DrivePwr;
  double WheelCircumference;
  double SecPerRev;
  double zloopcnt;
  double zRotationTime;
  double zTimeToDriveToDepot;
  double zTargRot;
  List recognitions;
  String ZObjString;
  String zDriveString;
  double zRotPosition;
  double MotorCntsPerShaftRotation;
  double zTargPos;
  double RobotRotationDistance;
  double zDrvPos;
  double zTargLen;
  double FrontWallX;
  double FrontWallY;
  double BackWallX;
  double BackWallY;
  double BlueWallX;
  double BlueWallY;
  double RedWallX;
  double RedWallY;
  double zTourDeltaX;
  double zTourDeltaY;
  double zTourRotation;
  double zTourLen;
  String zTourString;
  double zTourLenAdjFactor;
  double zAvgingLoopCount;

  /**
   * Describe this function...
   */
  private boolean BlueWall() {
    vuMarkResult = vuforiaRoverRuckus.track("BluePerimeter");
    // Is a VuMark visible?
    return vuMarkResult.isVisible;
  }

  /**
   * Describe this function...
   */
  private boolean BackWall() {
    vuMarkResult = vuforiaRoverRuckus.track("BackPerimeter");
    // Is a VuMark visible?
    return vuMarkResult.isVisible;
  }

  /**
   * Describe this function...
   */
  private boolean RedWall() {
    vuMarkResult = vuforiaRoverRuckus.track("RedPerimeter");
    // Is a VuMark visible?
    return vuMarkResult.isVisible;
  }

  /**
   * Describe this function...
   */
  private boolean FrontWall() {
    vuMarkResult = vuforiaRoverRuckus.track("FrontPerimeter");
    // Is a VuMark visible?
    return vuMarkResult.isVisible;
  }

  /**
   * Describe this function...
   */
  private void FormatDrvrString() {
    zDriveString += " Tpos: " + JavaUtil.formatNumber(zRotPosition, 0);
    zDriveString += " Trot: " + JavaUtil.formatNumber(zTargRot, 0);
    zDriveString += "  Stime:  " + JavaUtil.formatNumber(zTimeToDriveToDepot, 2);
    zDriveString += " Spos:  " + JavaUtil.formatNumber(zDrvPos, 0);
    zDriveString += "  Slen:" + JavaUtil.formatNumber(zTargLen, 0);
    zDriveString += "  WCir:" + JavaUtil.formatNumber(WheelCircumference, 0);
    zDriveString += "  RobotRotationDistance:" + JavaUtil.formatNumber(RobotRotationDistance, 0);
  }

  /**
   * Determine x, y & zRotation for the phone that's facing a target
   */
  private void Calc_Display() {
    FindXY_Zrot();
    FormatXYZ_string();
    telemetry.addData("Loc", XYZstring);
    // zRotation is incorrect at RED WALL, it shows -7..+7 deg when should be -183..+183 deg
    if (RedWall()) {
      // Fix zRotation bug when facing red Wall
      if (avgRotZ >= -10 && avgRotZ <= 10) {
        if (avgRotZ < 0) {
          // zRot is -10..-1 deg change to -170..-179 deg
          avgRotZ = -(180 + avgRotZ);
        } else {
          // zRot is 0.. +10 deg change to +180..+170 deg
          avgRotZ = 180 - avgRotZ;
        }
      }
      FormatXYZ_string();
      telemetry.addData("Fixed RedWall Rotation", XYZstring);
    }
    Calc_vector_to_Blue_Depot();
    Calc_vector_to_Red_Depot();
    telemetry.addData("to Depot", DepotString);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void DriveToBackWall() {
    Calc_vector_to_Wall_Target(BackWallX, BackWallY);
    TargetTourWall();
    if (BackWall()) {
      TrackableName = vuMarkResult.name;
      Calc_Display();
    } else {
      // Try 1 turn to wall to see if target visible
      TurnTest(-45);
      if (BackWall()) {
        TrackableName = vuMarkResult.name;
        Calc_Display();
      } else {
        // Target not there, set x,y, rot to what they should be, if we were in front of target
        avgX = BackWallX;
        avgY = BackWallY;
        avgRotZ = -90;
        TrackableName = "Back";
      }
    }
  }

  /**
   * Find average x & y position of phone on field and z rotation
   */
  private void FindXY_Zrot() {
    for (int count = 0; count < zAvgingLoopCount; count++) {
      sumX = sumX + vuMarkResult.x;
      sumY = sumY + vuMarkResult.y;
      sumRotZ = sumRotZ + vuMarkResult.zAngle;
      telemetry.update();
    }
    avgX = sumX / zAvgingLoopCount;
    avgY = sumY / zAvgingLoopCount;
    avgRotZ = sumRotZ / zAvgingLoopCount;
  }

  /**
   * Describe this function...
   */
  private void Format_Blue_Depot_string() {
    DepotString += " Blue x: " + JavaUtil.formatNumber(BluXDelta, 0);
    DepotString += " y: " + JavaUtil.formatNumber(BluYDelta, 0);
    DepotString += " len: " + JavaUtil.formatNumber(BluDepotLen, 0);
    DepotString += " rot:  " + JavaUtil.formatNumber(BluDepotRot, 0);
  }

  /**
   * Calculate length and rotation to a tour target
   */
  private void Calc_vector_to_Wall_Target(double TourTargX, double TourTargY) {
    zTourDeltaX = TourTargX - avgX;
    zTourDeltaY = TourTargY - avgY;
    if (RedWall()) {
      // Fix zRotation bug when facing red Wall
      if (avgRotZ >= -10 && avgRotZ <= 10) {
        if (avgRotZ < 0) {
          // zRot is -10..-1 deg change to -170..-179 deg
          avgRotZ = -(180 + avgRotZ);
        } else {
          // zRot is 0.. +10 deg change to +180..+170 deg
          avgRotZ = 180 - avgRotZ;
        }
      }
      telemetry.addData("Fixed RedWall Rotation", XYZstring);
    }
    zTourRotation = -(Math.atan2(zTourDeltaX, zTourDeltaY) / Math.PI * 180) - avgRotZ;
    zTourLen = Math.sqrt(Math.pow(zTourDeltaX, 2) + Math.pow(zTourDeltaY, 2)) * zTourLenAdjFactor;
    Format_Tour_string();
  }

  /**
   * From current phone position, calculate, turn and drive to front
   * wall, Once there, attempt to acquire front target if successful
   * use this as new phone location, turn 1 time 45deg towards
   * wall and try to acquire front target again. if successful
   * use phone location if not jam front X, Y and use what
   * the rotation would be for facing the front wall.
   */
  private void DriveToFrontWall() {
    Calc_vector_to_Wall_Target(FrontWallX, FrontWallY);
    TargetTourWall();
    if (FrontWall()) {
      TrackableName = vuMarkResult.name;
      Calc_Display();
    } else {
      // Try 1 turn to wall to see if target visible
      TurnTest(-45);
      if (BlueWall()) {
        TrackableName = vuMarkResult.name;
        Calc_Display();
      } else {
        // Target not there, set x,y, rot to what they should be, if we were in front of target
        avgX = FrontWallX;
        avgY = FrontWallY;
        avgRotZ = 90;
        TrackableName = "Front";
      }
    }
  }

  /**
   * Given a zTargRot(nation) and zRotPosition, turn robot to
   * that rotation which should be the same as the position
   */
  private void TurnToTarget() {
    FormatDrvrString();
    telemetry.addData("$", zDriveString);
    androidTextToSpeech.speak("Turn to depot");
    telemetry.update();
    if (zTargRot >= 0) {
      // If positive rotation angle
      rightMotor.setTargetPosition(rightMotor.getCurrentPosition() - (int) zRotPosition);
      leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + (int) zRotPosition);
      zTargPos = leftMotor.getCurrentPosition() + (int) zRotPosition;
    } else {
      // Negative rotation angle
      rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + (int) zRotPosition);
      leftMotor.setTargetPosition(leftMotor.getCurrentPosition() - (int) zRotPosition);
      zTargPos = leftMotor.getCurrentPosition() - (int) zRotPosition;
    }
    while (rightMotor.isBusy()) {
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      leftMotor.setPower(TurnPwr);
      rightMotor.setPower(TurnPwr);
    }
  }

  /**
   * Calculate relative X & Y, Relative Rotation, & vector length to Red Depot,
   */
  private void Calc_vector_to_Red_Depot() {
    RedXDelta = RedDepotX - avgX;
    RedYDelta = RedDepotY - avgY;
    RedDepotRot = -(Math.atan2(RedXDelta, RedYDelta) / Math.PI * 180) - avgRotZ;
    RedDepotLen = Math.sqrt(Math.pow(RedXDelta, 2) + Math.pow(RedYDelta, 2));
    Format_Red_Depot_string();
  }

  /**
   * Describe this function...
   */
  private void DriveToRedWall() {
    Calc_vector_to_Wall_Target(RedWallX, RedWallY);
    TargetTourWall();
    if (RedWall()) {
      TrackableName = vuMarkResult.name;
      Calc_Display();
    } else {
      // Try 1 turn to wall to see if target visible
      TurnTest(-45);
      if (RedWall()) {
        TrackableName = vuMarkResult.name;
        Calc_Display();
      } else {
        // Target not there, set x,y, rot to what they should be, if we were in front of target
        avgX = RedWallX;
        avgY = RedWallY;
        avgRotZ = 180;
        TrackableName = "Red";
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Format_Red_Depot_string() {
    DepotString += " Red  x: " + JavaUtil.formatNumber(RedXDelta, 0);
    DepotString += " y: " + JavaUtil.formatNumber(RedYDelta, 0);
    DepotString += " len: " + JavaUtil.formatNumber(RedDepotLen, 0);
    DepotString += " rot: " + JavaUtil.formatNumber(RedDepotRot, 0);
  }

  /**
   * Calculate delta X, delta Y, relative rotation and vector
   * length to blue depot from wherever phone is located
   */
  private void Calc_vector_to_Blue_Depot() {
    BluXDelta = BlueDepotX - avgX;
    BluYDelta = BlueDepotY - avgY;
    BluDepotRot = -(Math.atan2(BluXDelta, BluYDelta) / Math.PI * 180) - avgRotZ;
    BluDepotLen = Math.sqrt(Math.pow(BluXDelta, 2) + Math.pow(BluYDelta, 2));
    Format_Blue_Depot_string();
  }

  /**
   * Describe this function...
   */
  private void FormatPwrString() {
    PwrString += "Loop" + JavaUtil.formatNumber(zloopcnt, 0);
    PwrString += "LeftPwr: " + JavaUtil.formatNumber(leftMotor.getPower(), 2);
    PwrString += ", RightPwr: " + JavaUtil.formatNumber(rightMotor.getPower(), 2);
  }

  /**
   * Describe this function...
   */
  private void DriveToBlueWall() {
    Calc_vector_to_Wall_Target(BlueWallX, BlueWallY);
    TargetTourWall();
    if (BlueWall()) {
      TrackableName = vuMarkResult.name;
      Calc_Display();
    } else {
      // Try 1 turn to wall to see if target visible
      TurnTest(-45);
      if (BlueWall()) {
        TrackableName = vuMarkResult.name;
        Calc_Display();
      } else {
        // Target not there, set x,y, rot to what they should be, if we were in front of target
        avgX = BlueWallX;
        avgY = BlueWallY;
        avgRotZ = 0;
        TrackableName = "Blue";
      }
    }
  }

  /**
   * Given zTourRotation & zTourLen calculate positions for
   * turn and driving then turn and drive to tour location
   */
  private void TargetTourWall() {
    // If rotation more than 180 deg use comlplement
    if (zTourRotation > 180) {
      zTourRotation = zTourRotation - 360;
    } else if (zTourRotation < -180) {
      zTourRotation = zTourRotation + 360;
    }
    zRotationTime = (SecPerRev * (Math.abs(zTourRotation) / 360)) / TurnPwr;
    zRotPosition = MotorCntsPerShaftRotation * ((RobotRotationDistance * (Math.abs(zTourRotation) / 360)) / WheelCircumference);
    zTimeToDriveToDepot = (SecPerRev * (zTourLen / WheelCircumference)) / DrivePwr;
    zDrvPos = MotorCntsPerShaftRotation * (zTourLen / WheelCircumference);
    zTargRot = zTourRotation;
    TurnToTarget();
    sleep(3000);
    telemetry.addData("@", zTourString);
    androidTextToSpeech.speak("Rotate");
    sleep(3000);
    telemetry.update();
    zTargLen = zTourLen;
    androidTextToSpeech.speak("Roll");
    DriveToTarget();
    sleep(3000);
    telemetry.update();
  }

  /**
   * Given a Vuforia derived position, calculate relative rotation angle
   * and relative rotation position as well as relative position to
   * go turn and drive to blue depot. It also calls TurnToTarget
   * and DriveToTarget to move to robot to inside the blue depot
   */
  private void TargetBlueDepot() {
    zRotationTime = (SecPerRev * (Math.abs(BluDepotRot) / 360)) / TurnPwr;
    zRotPosition = MotorCntsPerShaftRotation * ((RobotRotationDistance * (Math.abs(BluDepotRot) / 360)) / WheelCircumference);
    zTimeToDriveToDepot = (SecPerRev * (BluDepotLen / WheelCircumference)) / DrivePwr;
    zDrvPos = MotorCntsPerShaftRotation * (BluDepotLen / WheelCircumference);
    zTargRot = BluDepotRot;
    TurnToTarget();
    sleep(3000);
    telemetry.update();
    zTargLen = BluDepotLen;
    DriveToTarget();
  }

  /**
   * Describe this function...
   */
  private void Format_Tour_string() {
    zTourString += "Tour Dx: " + JavaUtil.formatNumber(zTourDeltaX, 0);
    zTourString += " Dy: " + JavaUtil.formatNumber(zTourDeltaY, 0);
    zTourString += " Dlen: " + JavaUtil.formatNumber(zTourLen, 0);
    zTourString += " Drot:  " + JavaUtil.formatNumber(zTourRotation, 0);
  }

  /**
   * Describe this function...
   */
  private void TargetRedDepot() {
    zRotationTime = (SecPerRev * (Math.abs(RedDepotRot) / 360)) / TurnPwr;
    zRotPosition = Math.round(MotorCntsPerShaftRotation * ((RobotRotationDistance * (Math.abs(RedDepotRot) / 360)) / WheelCircumference));
    zTimeToDriveToDepot = (SecPerRev * (Math.abs(RedDepotLen) / 360)) / DrivePwr;
    zDrvPos = MotorCntsPerShaftRotation * (RedDepotLen / WheelCircumference);
    zTargRot = RedDepotRot;
    TurnToTarget();
    sleep(3000);
    telemetry.update();
    zTargLen = RedDepotLen;
    DriveToTarget();
  }

  /**
   * Given a length, drive straight for that length
   */
  private void DrvTest(double zLen) {
    zDrvPos = MotorCntsPerShaftRotation * (zLen / WheelCircumference);
    zTargLen = zLen;
    DriveToTarget();
  }

  /**
   * Describe this function...
   */
  private void FormatObjString() {
    double ZObjCount;
    double ZObjIdx;

    ZObjString += "Loop" + JavaUtil.formatNumber(zloopcnt, 0);
    ZObjCount = recognitions.size();
    ZObjString += " Objects: " + JavaUtil.formatNumber(ZObjCount, 0);
    ZObjIdx = 0;
    if (ZObjCount > 0) {
      // TODO: Enter the type for variable named recognition
      /* for (UNKNOWN_TYPE recognition : recognitions) {
        ZObjIdx = ZObjIdx + 1;
        ZObjString += " Obj #: " + JavaUtil.formatNumber(ZObjIdx, 0);
        ZObjString += " " + recognition.getLabel();
      }*/
    }
    androidTextToSpeech.speak(ZObjString);
  }

  /**
   * initialiives variables used during opcode loop.
   */
  private void Initialize_vars() {
    double sumZ;
    double DistWalltoOrigin;
    boolean target;
    double recognition;
    double WheelDiameter;
    double WheelSpacing;
    double RotAdjFactor;
    double DrvLenAdjFactor;

    // Distance from wall to origin is [6 squares X 2ft/sq x 12 in/ft X 25.4 mm/in]/2 because the origin is 1/2 way between walls
    DistWalltoOrigin = (25.4 * 12 * 6 * 2) / 2;
    // Blue depot is in Blue-Front quadrant (x,y) (-,+)
    // Depot is 23.5in square X 25.4mm/in, The middle is 1/2 this distance
    BlueDepotX = -(DistWalltoOrigin - 25.4 * (23.5 / 2));
    BlueDepotY = Math.abs(BlueDepotX);
    // Red depot is in Red-Back quadrant (x,y) (+,-)
    // Depot is 23.5in square X 25.4mm/in, The middle is 1/2 this distance
    RedDepotX = DistWalltoOrigin - 25.4 * (23.5 / 2);
    RedDepotY = -Math.abs(RedDepotX);
    // Tour Len Adjustment Factor is multiplied times length of travel for tour only
    zTourLenAdjFactor = 1.0;
    // Establish target tour postions
    FrontWallX = -(DistWalltoOrigin - 25.4 * 23.5);
    FrontWallY = 0;
    BackWallX = DistWalltoOrigin - 25.4 * 23.5;
    BackWallY = 0;
    BlueWallX = 0;
    BlueWallY = DistWalltoOrigin - 25.4 * 23.5;
    RedWallX = 0;
    RedWallY = -(DistWalltoOrigin - 25.4 * 23.5);
    // power level to drive to depot
    DrivePwr = 0.15;
    // power level to Turn to depot
    TurnPwr = 0.1;
    // Hank Robot: Wheel Dia=4,25'; Spacing=15.5'
    // Nav Robot: Wheel Dia=5.875;' Spacing=12.25'
    // At 25.4mm to in
    WheelDiameter = 5.875 * 25.4;
    // Wheel circumference=2PiXr where R is diameter/2
    // Drive Length Adjustment Factor = Distance drove / distance requested
    DrvLenAdjFactor = 1;
    WheelCircumference = WheelDiameter * Math.PI * DrvLenAdjFactor;
    // Robot Rotation Distance based on Wheel Spacing (diameter of robot circle)
    WheelSpacing = 12.25 * 25.4;
    // Rotation Adjustment Factor = % of rotation desicei/% of rotation attained, it is the shortfall on a rotation
    RotAdjFactor = 1.047188; //in blocks code? it was (90 / 85) * (90 / 91);
    // Robot Rotation Distance= Pi*WheelSpacing(diameter of robot circle)
    RobotRotationDistance = WheelSpacing * Math.PI * RotAdjFactor;
    // According to NeveRest 20 spec 537.6 counts per shaft rotation
    MotorCntsPerShaftRotation = 537.6;
    // According to NeveRest 20 spec 340RPM unloaded
    // #sec per one DC motor rev, AndyMark 20 New
    SecPerRev = 1 / (340 / 60);
    PwrString = "";
    XYZstring = "";
    zDriveString = "";
    DepotString = "";
    ZObjString = "";
    zTourString = "";
    zAvgingLoopCount = 34;
    sumX = 0;
    sumY = 0;
    sumZ = 0;
    sumRotZ = 0;
    target = false;
    recognition = 0;
  }

  /**
   * Given a rotation angle, turn robot that many degrees
   */
  private void TurnTest(double zDegree) {
    zRotPosition = MotorCntsPerShaftRotation * ((RobotRotationDistance * (Math.abs(zDegree) / 360)) / WheelCircumference);
    zTargRot = zDegree;
    TurnToTarget();
  }

  /**
   * Given a zDrvPos(inion), drive straight to that position
   */
  private void DriveToTarget() {
    FormatDrvrString();
    telemetry.addData("#: ", zDriveString);
    rightMotor.setTargetPosition(rightMotor.getCurrentPosition() - (int) zDrvPos);
    leftMotor.setTargetPosition(leftMotor.getCurrentPosition() - (int) zDrvPos);
    zTargPos = leftMotor.getCurrentPosition() + (int) zDrvPos;
    androidTextToSpeech.speak("Driivng");
    while (rightMotor.isBusy()) {
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      leftMotor.setPower(DrivePwr);
      rightMotor.setPower(DrivePwr);
    }
  }

  /**
   * Describe this function...
   */
  private void FormatXYZ_string() {
    XYZstring += "loop:" + zloopcnt;
    XYZstring += "Target:" + TrackableName;
    XYZstring += "X= " + JavaUtil.formatNumber(avgX, 0);
    XYZstring += ", Y= " + JavaUtil.formatNumber(avgY, 0);
    XYZstring += ",  RotZ: " + JavaUtil.formatNumber(avgRotZ, 0);
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    String Depot;
    double goldMineralX;
    double silverMineral1X;
    double silverMineral2X;

    vuforiaRoverRuckus = new VuforiaRoverRuckus();
    androidTextToSpeech = new AndroidTextToSpeech();
    rightMotor = hardwareMap.dcMotor.get("rightMotor");
    leftMotor = hardwareMap.dcMotor.get("leftMotor");
    tfodRoverRuckus = new TfodRoverRuckus();
    vuforiaRelicRecovery = new VuforiaRelicRecovery();

    // Initialize Vuforia (use default settings).
    vuforiaRoverRuckus.initialize("", VuforiaLocalizer.CameraDirection.BACK,
        false, false, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
        0, 0, 0, 0, 0, 0, true);
    tfodRoverRuckus.initialize(vuforiaRoverRuckus, 4, true, true);
    // Prompt user to push start button.
    zloopcnt = 0;
    androidTextToSpeech.initialize();
    androidTextToSpeech.setLanguageAndCountry("en", "US");
    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    telemetry.addData("Driving Example", "Press start to continue...");
    telemetry.update();
    // Wait until user pushes start button.
    waitForStart();
    Depot = "Blue";
    if (opModeIsActive()) {
      // Activate Vuforia software.
      tfodRoverRuckus.activate();
      vuforiaRoverRuckus.activate();
      Initialize_vars();
      while (opModeIsActive()) {
        zloopcnt = zloopcnt + 1;
        telemetry.addData("Loop:", zloopcnt);
        // Turns robot (deg) & drives length (mm), 1 time
        /*if (zloopcnt==1)  {

            TurnTest(180); // turn negative 45 degrees
            sleep(1000);
          // DrvTest(457); // straight drive 457mm=~1.5feet

        } */


        // Get the tracking results.
        if (RedWall() || BlueWall() || FrontWall() || BackWall()) {
          // Tarrget visible, calculate position, turn and drive to Depot
          TrackableName = vuMarkResult.name;
          Calc_Display();
          sleep(1000);
          zTourString = "";
          zDriveString = "";
          // If at a target wall, do wall tour from there Counter Clockwise
          if (RedWall()) {
            DriveToBackWall();
            DriveToBlueWall();
            DriveToFrontWall();
            DriveToRedWall();
          } else if (BackWall()) {
            DriveToBlueWall();
            DriveToFrontWall();
            DriveToRedWall();
            DriveToBackWall();
          } else if (BlueWall()) {
            DriveToFrontWall();
            DriveToRedWall();
            DriveToBackWall();
            DriveToBlueWall();
          } else {
            // @Front Wall
            DriveToRedWall();
            DriveToBackWall();
            DriveToBlueWall();
            DriveToFrontWall();
          }
          telemetry.update();
        } else {
          // No target visible, use joystick
          telemetry.addData("TFobj: ", ZObjString);
          FormatPwrString();
          telemetry.addData("no VuMarks are visible", PwrString);
          sleep(1000);
          telemetry.update();
        }
        telemetry.update();
        Initialize_vars();
      }
    }
    vuforiaRelicRecovery.deactivate();
    // Deactivate before exiting.

    vuforiaRoverRuckus.close();
    androidTextToSpeech.close();
    tfodRoverRuckus.close();
    vuforiaRelicRecovery.close();
  }
}
