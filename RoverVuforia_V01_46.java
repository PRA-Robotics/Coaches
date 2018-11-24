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

@TeleOp(name = "RoverVuforia_V01_46 (Blocks to Java)", group = "")
public class RoverVuforia_V01_46 extends LinearOpMode {

  private VuforiaRoverRuckus vuforiaRoverRuckus;
  private DcMotor rightMotor;
  private DcMotor leftMotor;
  private AndroidTextToSpeech androidTextToSpeech;
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
  String Depot;
  double zloopcnt;
  double zRotationTime;
  double zTimeToDriveToDepot;
  double ZdepotRot;
  List recognitions;
  String ZObjString;
  String zDriveString;
  double zRotPosition;
  double zCntsPerShaftRotation;
  double RobotRotation;

  /**
   * Describe this function...
   */
  private boolean BlueWal() {
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
  private void Initialize_vars() {
    double sumZ;
    double DistWalltoOrigin;
    boolean target;
    double recognition;
    double WheelDiameter;
    double WheelSpacing;
    double RotAdjFactor;

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
    // power level to drive to depot
    DrivePwr = 0.3;
    // power level to Turn to depot
    TurnPwr = 0.1;
    // #sec per one DC motor rev, AndyMark 20 New
    // which does 340RPM @100% power no load
    // adjusted for turn power level
    // Wheel circumference=2PiXr where R is diameter/2
    // At 25.4mm to in
    // As measured it's 4.25" but we are not turning enough,
    // So we are going to adjust the wheel diameter to compensate
    WheelDiameter = 4.25 * 25.4;
    WheelCircumference = WheelDiameter * Math.PI;
    // Robot Rotation Distance= Pi*WheelSpacing(diameter of robot circle)
    WheelSpacing = 15.5 * 25.4;
    // Rotation Adjustment Factor = % of rotation desicei/% of rotation attained, it is the shortfall on a rotation
    RotAdjFactor = 1.106;
    // Robot Rotation Distance= Pi*WheelSpacing(diameter of robot circle)
    RobotRotation = WheelSpacing * Math.PI * RotAdjFactor;
    // According to NeveRest 20 spec 537.6 counts per shaft rotation
    zCntsPerShaftRotation = 537.6;
    // According to NeveRest 20 spec 340RPM unloaded
    SecPerRev = 1 / (340 / 60);
    PwrString = "";
    XYZstring = "";
    zDriveString = "";
    DepotString = "";
    ZObjString = "";
    sumX = 0;
    sumY = 0;
    sumZ = 0;
    sumRotZ = 0;
    target = false;
    recognition = 0;
  }

  /**
   * Describe this function...
   */
  private void Calc_Display() {
    FindXY_Zrot();
    FormatXYZ_string();
    telemetry.addData("Loc", XYZstring);
    Calc_vector_to_Blue_Depot();
    Calc_vector_to_Red_Depot();
    telemetry.addData("to Depot", DepotString);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void FindXY_Zrot() {
    for (int count = 0; count < 33; count++) {
      sumX = sumX + vuMarkResult.x;
      sumY = sumY + vuMarkResult.y;
      sumRotZ = vuMarkResult.zAngle;
      telemetry.update();
    }
    avgX = sumX / 33;
    avgY = sumY / 33;
    avgRotZ = sumRotZ;
  }

  /**
   * Describe this function...
   */
  private void DriveToRedDepot() {
    zRotationTime = (SecPerRev * (Math.abs(RedDepotRot) / 360)) / TurnPwr;
    zRotPosition = Math.round(zCntsPerShaftRotation * ((RobotRotation * (Math.abs(RedDepotRot) / 360)) / WheelCircumference));
    zTimeToDriveToDepot = (SecPerRev * (Math.abs(RedDepotLen) / 360)) / DrivePwr;
    ZdepotRot = RedDepotRot;
    DriveToDepot();
  }

  /**
   * Describe this function...
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
  private void Calc_vector_to_Red_Depot() {
    RedXDelta = RedDepotX - avgX;
    RedYDelta = RedDepotY - avgY;
    RedDepotRot = Math.atan2(RedYDelta, RedXDelta) / Math.PI * 180 - avgRotZ;
    RedDepotLen = Math.sqrt(Math.pow(RedXDelta, 2) + Math.pow(RedYDelta, 2));
    Format_Red_Depot_string();
  }

  /**
   * Describe this function...
   */
  private void DriveToBlueDepot() {
    zRotationTime = (SecPerRev * (Math.abs(BluDepotRot) / 360)) / TurnPwr;
    zRotPosition = Math.round(zCntsPerShaftRotation * ((RobotRotation * (Math.abs(BluDepotRot) / 360)) / WheelCircumference));
    zTimeToDriveToDepot = (SecPerRev * (BluDepotLen / WheelCircumference)) / DrivePwr;
    ZdepotRot = BluDepotRot;
    DriveToDepot();
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
   * Describe this function...
   */
  private void Format_Blue_Depot_string() {
    DepotString += " Blue x: ";
    DepotString += "" + JavaUtil.formatNumber(BluXDelta, 0);
    DepotString += " y: ";
    DepotString += "" + JavaUtil.formatNumber(BluYDelta, 0);
    DepotString += " len: ";
    DepotString += "" + JavaUtil.formatNumber(BluDepotLen, 0);
    DepotString += " rot:  ";
    DepotString += "" + JavaUtil.formatNumber(BluDepotRot, 0);
  }

  /**
   * Describe this function...
   */
  private void Format_Red_Depot_string() {
    DepotString += " Red  x: ";
    DepotString += "" + JavaUtil.formatNumber(RedXDelta, 0);
    DepotString += " y: ";
    DepotString += "" + JavaUtil.formatNumber(RedYDelta, 0);
    DepotString += " len: ";
    DepotString += "" + JavaUtil.formatNumber(RedDepotLen, 0);
    DepotString += " rot: ";
    DepotString += "" + JavaUtil.formatNumber(RedDepotRot, 0);
  }

  /**
   * Describe this function...
   */
  private void DriveToDepot() {
    double zTargPos;

    FormatDrvrString();
    telemetry.addData("$", zDriveString);
    if (ZdepotRot >= 0) {
      // If positive rotation angle
      rightMotor.setTargetPosition(rightMotor.getCurrentPosition() - Math.round(zRotPosition));
      leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + zRotPosition);
      zTargPos = leftMotor.getCurrentPosition() + zRotPosition;
    } else {
      // Negative rotation angle
      rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + zRotPosition);
      leftMotor.setTargetPosition(leftMotor.getCurrentPosition() - zRotPosition);
      zTargPos = leftMotor.getCurrentPosition() - zRotPosition;
    }
    while (zTargPos != leftMotor.getCurrentPosition()) {
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      leftMotor.setPower(TurnPwr);
      rightMotor.setPower(TurnPwr);
    }
    // The Y axis of a joystick ranges from -1 in its topmost position
    // to +1 in its bottommost position. We negate this value so that
    // the topmost position corresponds to maximum forward power.
    leftMotor.setPower(0);
    rightMotor.setPower(0);
    androidTextToSpeech.speak("Turn to depot");
    // Sleep for turn time less than one sec, with power to motors
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
  private void FormatDrvrString() {
    zDriveString += "TurnTime:" + JavaUtil.formatNumber(zRotationTime, 3);
    zDriveString += "TurnPos" + JavaUtil.formatNumber(zRotPosition, 3);
    zDriveString += " BluRot: " + JavaUtil.formatNumber(BluDepotRot, 3);
    zDriveString += " RedRot: " + JavaUtil.formatNumber(RedDepotRot, 3);
    zDriveString += " DrvTime: " + JavaUtil.formatNumber(zTimeToDriveToDepot, 3);
    zDriveString += " BluLen:" + JavaUtil.formatNumber(BluDepotLen, 3);
    zDriveString += " RedLen:" + JavaUtil.formatNumber(RedDepotLen, 3);
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
      for (UNKNOWN_TYPE recognition : recognitions) {
        ZObjIdx = ZObjIdx + 1;
        ZObjString += " Obj #: " + JavaUtil.formatNumber(ZObjIdx, 0);
        ZObjString += " " + recognition.getLabel();
      }
    }
    androidTextToSpeech.speak(ZObjString);
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double goldMineralX;
    double silverMineral1X;
    double silverMineral2X;

    vuforiaRoverRuckus = new VuforiaRoverRuckus();
    rightMotor = hardwareMap.dcMotor.get("rightMotor");
    leftMotor = hardwareMap.dcMotor.get("leftMotor");
    androidTextToSpeech = new AndroidTextToSpeech();
    tfodRoverRuckus = new TfodRoverRuckus();
    vuforiaRelicRecovery = new VuforiaRelicRecovery();

    // Initialize Vuforia (use default settings).
    vuforiaRoverRuckus.initialize("", VuforiaLocalizer.CameraDirection.BACK,
        false, false, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
        0, 0, 0, 0, 0, 0, true);
    tfodRoverRuckus.initialize(vuforiaRoverRuckus, 0.4, true, true);
    // Prompt user to push start button.
    zloopcnt = 0;
    androidTextToSpeech.initialize();
    androidTextToSpeech.setLanguageAndCountry("en", "US");
    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        telemetry.addData("Count", zloopcnt);
        // Get the tracking results.
        if (RedWall() || BlueWal() || FrontWall() || BackWall()) {
          // Tarrget visible, calculate position, turn and drive to Depot
          TrackableName = vuMarkResult.name;
          Calc_Display();
          if (Depot.equals("Blue")) {
            // 1st pass drive to Blue depot, then Red Depot
            DriveToBlueDepot();
            Depot = "";
          } else {
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
