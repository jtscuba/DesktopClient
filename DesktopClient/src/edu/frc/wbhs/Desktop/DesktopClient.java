/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.frc.wbhs.Desktop;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import java.util.Scanner;



/**
 *
 * @author Josh
 */



public class DesktopClient {
	
	NetworkTable Input;
	NetworkTable Output;
	
	ColorImage image;
	BinaryImage thresholdImage;
	BinaryImage filteredImage;
	
	
	
	
	double CurrentVelocity,CurrentAngle,TargetAngle,TargetDistance,Mode;
	
		//Camera constants used for distance calculation
	final int Y_IMAGE_RES = 480;		//X Image resolution in pixels, should be 120, 240 or 480
	final double VIEW_ANGLE = 49;		//Axis M1013
	//final double VIEW_ANGLE = 41.7;		//Axis 206 camera
	//final double VIEW_ANGLE = 37.4;  //Axis M1011 camera
	final double PI = 3.141592653;

	//Score limits used for target identification
	final int RECTANGULARITY_LIMIT = 40;
	final int ASPECT_RATIO_LIMIT = 55;

	//Score limits used for hot target determination
	final int TAPE_WIDTH_LIMIT = 50;
	final int VERTICAL_SCORE_LIMIT = 50;
	final int LR_SCORE_LIMIT = 50;

	//Minimum area of particles to be considered
	final int AREA_MINIMUM = 150;

	//Maximum number of particles to process
	final int MAX_PARTICLES = 8;
	
	TargetReport target = new TargetReport();
	int verticalTargets[] = new int[MAX_PARTICLES];
	int horizontalTargets[] = new int[MAX_PARTICLES];
	int verticalTargetCount, horizontalTargetCount;
	
	
	AxisCamera camera;          // the axis camera object (connected to the switch)
	CriteriaCollection cc;      // the criteria for doing the particle filter operation
	
	
	public class Scores
	{
		double rectangularity;
		double aspectRatioVertical;
		double aspectRatioHorizontal;
	}
	
	public static void main(String[] args)
	{
		while(true)
		{
			NetworkTable Output = NetworkTable.getTable("Output");
			Scanner user_input = new Scanner( System.in );
			System.out.println("New P value");
			Output.putString("P", user_input.next());
			System.out.println("New I value");
			Output.putString("I", user_input.next());
			System.out.println("New D value");
			Output.putString("D", user_input.next());
		}
	}
	
	public void Start()
	{
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress("10.47.37.2");
		Input = NetworkTable.getTable("Input");
		Output = NetworkTable.getTable("Output");
//		cc = new CriteriaCollection();      // create the criteria for the particle filter
//		cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
		
		
	}
	public void UpdateReadings()
	{
		CurrentVelocity = Input.getNumber("Velocity", CurrentVelocity);
		CurrentAngle = Input.getNumber("Angle", CurrentAngle);
		Mode = Input.getNumber("Target", 0.0);
	}
	public void PushReadings()
	{
		Output.putNumber("TargetDistance",TargetDistance);
		Output.putNumber("TargetAngle", TargetAngle);
	}
	
	public void TakePicture()
	{
		try
		{
			image = camera.getImage();     // comment if using stored images
			//ColorImage image;                           // next 2 lines read image from flash on cRIO
			//image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash
			thresholdImage = image.thresholdHSV(105, 137, 230, 255, 133, 183);// keep only green objects
			//thresholdImage.write("/threshold.bmp");
			BinaryImage filteredImage = thresholdImage.particleFilter(cc);           // filter out small particles
			//filteredImage.write("/filteredImage.bmp");
		}
		
		catch (AxisCameraException ex) 
		{        // this is needed if the camera.getImage() is called
			ex.printStackTrace();
		}
		catch (NIVisionException ex) 
		{
			ex.printStackTrace();
		}
		
	}
	
	double scoreRectangularity(ParticleAnalysisReport report) 
	{
		if (report.boundingRectWidth * report.boundingRectHeight != 0) 
		{
			return 100 * report.particleArea / (report.boundingRectWidth * report.boundingRectHeight);
		} else 
		{
			return 0;
		}
	}
	
	double ratioToScore(double ratio) {
		return (Math.max(0, Math.min(100 * (1 - Math.abs(1 - ratio)), 100)));
	}

	
	public double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean vertical) throws NIVisionException {
		double rectLong, rectShort, aspectRatio, idealAspectRatio;

		rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
		rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
		idealAspectRatio = vertical ? (4.0 / 32) : (23.5 / 4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall

		//Divide width by height to measure aspect ratio
		if (report.boundingRectWidth > report.boundingRectHeight) 
		{
			//particle is wider than it is tall, divide long by short
			aspectRatio = ratioToScore((rectLong / rectShort) / idealAspectRatio);
		} else 
		{
			//particle is taller than it is wide, divide short by long
			aspectRatio = ratioToScore((rectShort / rectLong) / idealAspectRatio);
		}
		return aspectRatio;
	}

	
	boolean scoreCompare(Scores scores, boolean vertical) {
		boolean isTarget = true;

		isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
		if (vertical) {
			isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
		} else {
			isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
		}

		return isTarget;
	}
	
		boolean hotOrNot(TargetReport target) 
		{
		boolean isHot = true;

		isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
		isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
		isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);

		return isHot;
		}
	
	

	public void FindHotGoal()
	{
		try 
		{
			Scores scores[] = new Scores[filteredImage.getNumberParticles()];
			horizontalTargetCount = verticalTargetCount = 0;
			
			if (filteredImage.getNumberParticles() > 0) 
			{
					for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles(); i++) {
						ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
						scores[i] = new Scores();
						
						//Score each particle on rectangularity and aspect ratio
						scores[i].rectangularity = scoreRectangularity(report);
						scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, i, true);
						scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, i, false);

						//Check if the particle is a horizontal target, if not, check if it's a vertical target
						if (scoreCompare(scores[i], false)) {
							System.out.println("particle: " + i + "is a Horizontal Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
							horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
						} else if (scoreCompare(scores[i], true)) {
							System.out.println("particle: " + i + "is a Vertical Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
							verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
						} else {
							System.out.println("particle: " + i + "is not a Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
						}
						System.out.println("rect: " + scores[i].rectangularity + "ARHoriz: " + scores[i].aspectRatioHorizontal);
						System.out.println("ARVert: " + scores[i].aspectRatioVertical);
				}
				
				//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
					target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
					target.verticalIndex = verticalTargets[0];
					for (int i = 0; i < verticalTargetCount; i++) {
						ParticleAnalysisReport verticalReport = filteredImage.getParticleAnalysisReport(verticalTargets[i]);
						for (int j = 0; j < horizontalTargetCount; j++) {
							ParticleAnalysisReport horizontalReport = filteredImage.getParticleAnalysisReport(horizontalTargets[j]);
							double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;

							//Measure equivalent rectangle sides for use in score calculation
							horizWidth = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
							vertWidth = NIVision.MeasureParticle(filteredImage.image, verticalTargets[i], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
							horizHeight = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);

							//Determine if the horizontal target is in the expected location to the left of the vertical target
							leftScore = ratioToScore(1.2 * (verticalReport.boundingRectLeft - horizontalReport.center_mass_x) / horizWidth);
							//Determine if the horizontal target is in the expected location to the right of the  vertical target
							rightScore = ratioToScore(1.2 * (horizontalReport.center_mass_x - verticalReport.boundingRectLeft - verticalReport.boundingRectWidth) / horizWidth);
							//Determine if the width of the tape on the two targets appears to be the same
							tapeWidthScore = ratioToScore(vertWidth / horizHeight);
							//Determine if the vertical location of the horizontal target appears to be correct
							verticalScore = ratioToScore(1 - (verticalReport.boundingRectTop - horizontalReport.center_mass_y) / (4 * horizHeight));
							total = leftScore > rightScore ? leftScore : rightScore;
							total += tapeWidthScore + verticalScore;

							//If the target is the best detected so far store the information about it
							if (total > target.totalScore) {
								target.horizontalIndex = horizontalTargets[j];
								target.verticalIndex = verticalTargets[i];
								target.totalScore = total;
								target.leftScore = leftScore;
								target.rightScore = rightScore;
								target.tapeWidthScore = tapeWidthScore;
								target.verticalScore = verticalScore;
							}
						}
						//Determine if the best target is a Hot target
						target.Hot = hotOrNot(target);
					}	
					
					
					
					
					
					
			}

		}	
		catch(NIVisionException ex )
		{
			ex.printStackTrace();		
		}

		
	}
	
	

	

	
	public class TargetReport 
	{

		int verticalIndex;
		int horizontalIndex;
		boolean Hot;
		double totalScore;
		double leftScore;
		double rightScore;
		double tapeWidthScore;
		double verticalScore;
	}

	
	
	
	
	
}
