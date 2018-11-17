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

@TeleOp(name = "RoverVuforia_v01_44 (Blocks to Java)", group = "")
public class RoverVuforia_v01_44 extends LinearOpMode {

  private VuforiaRoverRuckus vuforiaRoverRuckus;
  private TfodRoverRuckus tfodRoverRuckus;
  private AndroidTextToSpeech androidTextToSpeech;
  private DcMotor rightMotor;
  private DcMotor leftMotor;
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
  double SecPerRev;
  String Depot;
  double zloopcnt;
  double zRotationTime;
  double zTimeToDriveToDepot;
  double ZdepotRot;
  List recognitions;

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
    double sumRotX;
    double sumRotY;
    double DistWalltoOrigin;
    double WheelCircumference;
    boolean target;

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
    PwrString = "";
    XYZstring = "";
    // Wheel circumference=2PiXr where R is diameter/2
    // At 25.4mm to in
    WheelCircumference = 2 * Math.PI * (5 / 2) * 25.4;
    // power level to drive to depot
    DrivePwr = 0.5;
    // power level to Turn to depot
    TurnPwr = 0.1;
    // #sec per one DC motor rev, AndyMark 20 New
    // which does 340RPM @100% power no load
    // adjusted for turn power level
    SecPerRev = 1 / (340 / 60);
    DepotString = "";
    sumX = 0;
    sumY = 0;
    sumZ = 0;
    sumRotX = 0;
    sumRotY = 0;
    sumRotZ = 0;
    target = false;
    target = false;
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
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double goldMineralX;
    double silverMineral1X;
    double silverMineral2X;

    vuforiaRoverRuckus = new VuforiaRoverRuckus();
    tfodRoverRuckus = new TfodRoverRuckus();
    androidTextToSpeech = new AndroidTextToSpeech();
    rightMotor = hardwareMap.dcMotor.get("rightMotor");
    leftMotor = hardwareMap.dcMotor.get("leftMotor");
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
    telemetry.addData("VuMark Example", "Press start to continue...");
    telemetry.update();
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    // Wait until user pushes start button.
    waitForStart();
    Depot = "Blue";
    if (opModeIsActive()) {
      // Activate Vuforia software.
      Initialize_vars();
      tfodRoverRuckus.activate();
      vuforiaRoverRuckus.activate();
      while (opModeIsActive()) {
        zloopcnt = zloopcnt + 1;
        // Get the tracking results.
        if (RedWall() || BlueWal() || FrontWall() || BackWall()) {
          // Tarrget visible, calculate position, turn and drive to Depot
          TrackableName = vuMarkResult.name;
          Calc_Display();
          if (Depot.equals("Blue")) {
            // 1st pass drive to Blue depot, then Red Depot
            DriveToBlueDepot();
          } else {
            DriveToRedDepot();
          }
        } else {
          // No target visible, use joystick
          recognitions = tfodRoverRuckus.getRecognitions();
          FormatObjString();
          telemetry.addData("# Objects Recognized", recognitions.size());
          if (recognitions.size() == 3) {
            goldMineralX = -1;
            silverMineral1X = -1;
            silverMineral2X = -1;
            // TODO: Enter the type for variable named recognition
            for (UNKNOWN_TYPE recognition : recognitions) {
              if (recognition.getLabel().equals("Gold Mineral")) {
                goldMineralX = recognition.getLeft();
              } else if (silverMineral1X == -1) {
                silverMineral1X = recognition.getLeft();
              } else {
                silverMineral2X = recognition.getLeft();
              }
            }
            // Make sure we found one gold mineral and two silver minerals.
            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
              if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                telemetry.addData("Gold Mineral Position", "Left");
              } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                telemetry.addData("Gold Mineral Position", "Right");
              } else {
                telemetry.addData("Gold Mineral Position", "Center");
              }
            }
          }
          // The Y axis of a joystick ranges from -1 in its topmost position
          // to +1 in its bottommost position. We negate this value so that
          // the topmost position corresponds to maximum forward power.
          leftMotor.setPower(-(gamepad1.left_stick_y / 3));
          rightMotor.setPower(-(gamepad1.right_stick_y / 3));
          FormatPwrString();
          telemetry.addData("no VuMarks are visible", PwrString);
          sleep(5000);
        }
        Initialize_vars();
        telemetry.update();
      }
      vuforiaRelicRecovery.deactivate();
    }
    // Deactivate before exiting.

    vuforiaRoverRuckus.close();
    tfodRoverRuckus.close();
    androidTextToSpeech.close();
    vuforiaRelicRecovery.close();
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
  private void DriveToBlueDepot() {
    zRotationTime = (SecPerRev * (Math.abs(BluDepotRot) / 360)) / TurnPwr;
    zTimeToDriveToDepot = (SecPerRev * (Math.abs(BluDepotLen) / 360)) / DrivePwr;
    ZdepotRot = BluDepotRot;
    DriveToDepot();
  }

  /**
   * Describe this function...
   */
  private void DriveToDepot() {
    if (zRotationTime <= 1) {
      telemetry.addData("<1 sec turn time", DepotString);
      // Less than one sec of turn time
      if (ZdepotRot >= 0) {
        // If positive rotation angle
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        leftMotor.setPower(TurnPwr);
        rightMotor.setPower(-TurnPwr);
      } else {
        // Negative rotation angle
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        leftMotor.setPower(-TurnPwr);
        rightMotor.setPower(TurnPwr);
      }
      androidTextToSpeech.speak("Turning to depot for less than 1 second");
      // Sleep for turn time less than one sec, with power to motors
      sleep(Math.round(1000 * zRotationTime));
      if (zTimeToDriveToDepot <= 1 - zRotationTime) {
        leftMotor.setPower(DrivePwr);
        rightMotor.setPower(DrivePwr);
        // Sleep for time it takes to drive to depot
        androidTextToSpeech.speak("Driving to Depot > 1 second minus turn time");
        sleep(Math.round(1000 * zRotationTime));
        // At Blue Depot, stop and next time see target try Red
        Depot = "Red";
        androidTextToSpeech.speak("At Depot stop motors");
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        leftMotor.setPower(0);
        rightMotor.setPower(0);
      } else {
        // More than (1sec-turn time) drive time to get to depot
        telemetry.addData("> 1 sec drive time", DepotString);
        androidTextToSpeech.speak("Driving to depot, > 1sec ");
        leftMotor.setPower(DrivePwr);
        rightMotor.setPower(DrivePwr);
        //  sleep for one sec with motors powered, turning to depot
        sleep(1 - zRotationTime);
      }
    } else {
      // more than one second of turn time
      androidTextToSpeech.speak("More than 1 second turn time");
      if (ZdepotRot >= 0) {
        // If positive rotation angle
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        leftMotor.setPower(TurnPwr);
        rightMotor.setPower(-TurnPwr);
      } else {
        // Negative rotation angle
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        leftMotor.setPower(-TurnPwr);
        rightMotor.setPower(TurnPwr);
      }
      androidTextToSpeech.speak("Turning to Depot for more than 1 second");
      sleep(1000);
    }
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
  private void DriveToRedDepot() {
    zRotationTime = (SecPerRev * (Math.abs(RedDepotRot) / 360)) / TurnPwr;
    zTimeToDriveToDepot = (SecPerRev * (Math.abs(RedDepotLen) / 360)) / DrivePwr;
    ZdepotRot = RedDepotRot;
    DriveToDepot();
  }

  /**
   * Describe this function...
   */
  private void FormatPwrString() {
    PwrString += "LeftPwr: " + JavaUtil.formatNumber(leftMotor.getPower(), 2);
    PwrString += ", RightPwr: " + JavaUtil.formatNumber(rightMotor.getPower(), 2);
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
  private void FormatObjString() {
    String ZObjString;
    double ZObjCount;
    double ZObjIdx;

    ZObjCount = recognitions.size();
    ZObjIdx = 0;
    if (ZObjCount > 0) {
      ZObjString += "Objects Recognized" + JavaUtil.formatNumber(ZObjCount, 0);
      // TODO: Enter the type for variable named recognition
      for (UNKNOWN_TYPE recognition : recognitions) {
        ZObjIdx = ZObjIdx + 1;
        ZObjString += "Object Number" + JavaUtil.formatNumber(ZObjIdx, 0);
        ZObjString += " " + recognition.getLabel();
      }
      androidTextToSpeech.speak(ZObjString);
    }
  }
}
