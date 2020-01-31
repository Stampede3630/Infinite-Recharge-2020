package frc.robot;

public class PathChooser 
{
    public PathChooser() 
    {

    }

    public void stringToPath(String mode) //0 - closest(intake), 1-leftmost 2-rightmost 3-closest(deploy) (Largest from champs)
    {
        switch (mode) {
            case "LeftLS":
                Constants.robotAngle = Robot.choosers.reverseAngle(-180);
                if(Constants.ballManipulator)
                {
                    Constants.pipeline = "intake"; //we shouldn't be using this anywhere
                    Constants.pipelineNumber = Constants.closestDeploy;
                }
                else 
                {
                    Constants.pipeline = "intake";
                    Constants.pipelineNumber = Constants.closestIntake;
                }
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                Constants.ballBottom = false;
                Constants.ballTop = true;
                break;
            case "LeftFarRS":
                Constants.robotAngle = Robot.choosers.reverseAngle(-150);
                Constants.pipelineNumber = Constants.closestDeploy;
                Constants.pipeline = "deploy";
                Constants.ballFollowerOn = false;
                Constants.toRocket = true;
                break;
            case "LeftMidRS":
                Constants.robotAngle = Robot.choosers.reverseAngle(-90);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.closestDeploy;
                Constants.ballFollowerOn = false;
                Constants.toRocket = true;
                break;
            case "LeftNearRS":
                Constants.robotAngle = Robot.choosers.reverseAngle(-30);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.closestDeploy;
                Constants.ballFollowerOn = false;
                Constants.toRocket = true;
                break;
            case "LeftFarCS":
                Constants.robotAngle = Robot.choosers.reverseAngle(90);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.leftmost;
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                break;
            case "LeftMidCS":
                Constants.robotAngle = Robot.choosers.reverseAngle(90);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.closestDeploy;
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                break;
            case "LeftNearCS":
                Constants.robotAngle = Robot.choosers.reverseAngle(90);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.rightmost;
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                break;
            case "RightFarCS":
                Constants.robotAngle = Robot.choosers.reverseAngle(-90);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.rightmost;
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                break;
            case "RightMidCS":
                Constants.robotAngle = Robot.choosers.reverseAngle(-90);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.closestDeploy;
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                break;
            case "RightNearCS":
                Constants.robotAngle = Robot.choosers.reverseAngle(-90);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.leftmost;
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                break;
            case "LeftFaceCS":
                Constants.robotAngle = Robot.choosers.reverseAngle(0);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.leftmost;
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                break;
            case "RightFaceCS":
                Constants.robotAngle = Robot.choosers.reverseAngle(0);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.rightmost;
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                break;
            case "RightFarRS":
                Constants.robotAngle = Robot.choosers.reverseAngle(150);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.closestDeploy;
                Constants.ballFollowerOn = false;
                Constants.toRocket = true;
                break;
            case "RightMidRS":
                Constants.robotAngle = Robot.choosers.reverseAngle(90);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.closestDeploy;
                Constants.ballFollowerOn = false;
                Constants.toRocket = true;
                break;
            case "RightNearRS":
                Constants.robotAngle = Robot.choosers.reverseAngle(30);
                Constants.pipeline = "deploy";
                Constants.pipelineNumber = Constants.closestDeploy;
                Constants.ballFollowerOn = false;
                Constants.toRocket = true;
                break;
            case "RightLS":
                Constants.robotAngle = Robot.choosers.reverseAngle(-180);
                if(Constants.ballManipulator)
                {
                    Constants.pipeline = "intake"; //we shouldn't be using this anywhere
                    Constants.pipelineNumber = Constants.closestDeploy;
                }
                else 
                {
                    Constants.pipeline = "intake";
                    Constants.pipelineNumber = Constants.closestIntake;
                }
                Constants.ballFollowerOn = false;
                Constants.toRocket = false;
                Constants.ballBottom = false;
                Constants.ballTop = true;
                break;
            case "GrabBall":
                Constants.pipeline = "intake";
                Constants.pipelineNumber = Constants.closestIntake;
                Constants.ballFollowerOn = true;
                Constants.toRocket = false;
                Constants.ballBottom = true;
                Constants.ballTop = false;
                break;
            default:
                Constants.robotAngle = Robot.choosers.reverseAngle(0);
                break;      
        }
        // set limelight pipeline
    }
}