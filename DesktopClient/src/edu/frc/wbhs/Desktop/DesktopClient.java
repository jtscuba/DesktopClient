/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.frc.wbhs.Desktop;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 *
 * @author Josh
 */
public class DesktopClient {
	
	NetworkTable Input;
	NetworkTable Output;
	
	double CurrentVelocity,CurrentAngle,TargetAngle,TargetDistance,Mode;
	
	public static void main(String[] args)
	{
		
	}
	
	public void Start()
	{
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress("10.47.37.2");
		Input = NetworkTable.getTable("Input");
		Output = NetworkTable.getTable("Output");
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
	
	
	
	
	
}
