import com.neuronrobotics.bowlerstudio.creature.MobileBaseLoader
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins
import com.neuronrobotics.sdk.addons.kinematics.AbstractLink
import com.neuronrobotics.sdk.addons.kinematics.DHLink
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager

import eu.mihosoft.vrl.v3d.CSG
import eu.mihosoft.vrl.v3d.Cylinder
import eu.mihosoft.vrl.v3d.RoundedCube
import eu.mihosoft.vrl.v3d.RoundedCylinder
import eu.mihosoft.vrl.v3d.Transform
import javafx.scene.transform.Affine

CSG moveDHValues(CSG incoming,DHLink dh ){
	TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
	Transform move = com.neuronrobotics.bowlerstudio.physics.TransformFactory.nrToCSG(step)
	return incoming.transformed(move)
}

if(args==null) {
	HashMap<String,Object> measurmentsMotor = Vitamins.getConfiguration(  "LewanSoulMotor","lx_224")
	double motorz =  measurmentsMotor.body_z
	double centerTheMotorsValue=motorz/2;
	MobileBase base=DeviceManager.getSpecificDevice( "Standard6dof",{
		//If the device does not exist, prompt for the connection
		
		MobileBase m = MobileBaseLoader.fromGit(
			"https://github.com/Halloween2020TheChild/GroguMechanicsCad.git",
			"hephaestus.xml"
			)
		return m
	})
	def motorLocation = new TransformNR(0,0,0,new RotationNR())
	motorLocation=motorLocation.times(new TransformNR(0,0,0,new RotationNR(0,-90,0)))
	args = [base.getAllDHChains().get(0),2,centerTheMotorsValue,motorLocation]
}


int linkIndex = args[1]
DHParameterKinematics d= args[0];
ArrayList<DHLink> dhLinks = d.getChain().getLinks()
DHLink dh = dhLinks.get(linkIndex)
// Hardware to engineering units configuration
LinkConfiguration  conf = d.getLinkConfiguration(linkIndex);
// Engineering units to kinematics link (limits and hardware type abstraction)
AbstractLink abstractLink = d.getAbstractLink(linkIndex);
// Transform used by the UI to render the location of the object

//Horn section
Affine manipulator = dh.getListener();
def type=	d.getLinkConfiguration(linkIndex-1).getShaftType()
def size = d.getLinkConfiguration(linkIndex-1).getShaftSize()
CSG vitaminCad=   Vitamins.get(	type,size)
.movez(args[2])
def HornModel=moveDHValues(vitaminCad,dh)
HornModel.setManipulator(manipulator)

//END horn

//Bearing 
HashMap<String, Object> hornCOnfig = Vitamins.getConfiguration(type,size)
def mountPlateToHornTop = hornCOnfig.get("mountPlateToHornTop")
def bearingHeight =mountPlateToHornTop-2
CSG thrust = Vitamins.get("ballBearing","Thrust_1andAHalfinch")
						.movez(bearingHeight)
thrust.setManipulator(manipulator)
//END bearing


TransformNR motorLocation=args[3]

//Motor for next link
CSG motor=   Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
def motorModel = motor.transformed(TransformFactory.nrToCSG(motorLocation))
motorModel.setManipulator(manipulator)

// Keepaway for motor of this link
HashMap<String, Object> motormeasurments = Vitamins.getConfiguration(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
def b_y = motormeasurments.body_x/2
def hyp =Math.sqrt(b_y*b_y+b_y*b_y)
def centerTobottom = args[2]+motormeasurments.shoulderHeight
def centerToTop=args[2]+mountPlateToHornTop
def kwCanheight =args[2]+centerTobottom
def linkageThicknessSMallShaftLen = motormeasurments.bottomShaftLength

CSG keepawayCan = new Cylinder(hyp+1, kwCanheight+1).toCSG()
					.toZMax()
					.movez(args[2]+1)
CSG shaftKW = new Cylinder(motormeasurments.bottomShaftDiameter/2+0.5, kwCanheight+linkageThicknessSMallShaftLen*4).toCSG()
				.toZMax()
				.movez(args[2])			
keepawayCan=moveDHValues(keepawayCan.union(shaftKW),dh)
keepawayCan.setManipulator(manipulator)
// END motor keepaway

//Horn keepaway
CSG hornkw = new Cylinder(hornCOnfig.hornDiameter/2+1, mountPlateToHornTop+1).toCSG()

hornkw.setManipulator(manipulator)
// end horn keepaway

// Hull Bulding BLocks
def cornerRad=2
double baseCorRad = Vitamins.getConfiguration("ballBearing","Thrust_1andAHalfinch").outerDiameter/2+5
double linkThickness = baseCorRad-centerToTop
double linkYDimention = motormeasurments.body_x;
CSG linkBuildingBlockRoundCyl = new Cylinder(linkYDimention/2,linkYDimention/2,linkThickness,30)
.toCSG()
CSG linkBuildingBlockRoundSqu = new RoundedCube(linkYDimention,linkYDimention,linkThickness)
.cornerRadius(cornerRad)
.toCSG()
.toZMin()
CSG linkBuildingBlockRound = new RoundedCylinder(linkYDimention/2,linkThickness)
.cornerRadius(cornerRad)
.toCSG()
//END building blocks

//Sprint path section
def backsetBoltOne = -linkYDimention/2-5
def backsetBoltTwo=-25.0/2
double grooveDepth=1
double springRadius=35
double springboltRotation=22
double springSupportLength = linkYDimention+linkThickness*2.0+30
double linkOneSupportWidth=40+linkThickness*2
CSG grooveInner = new Cylinder(springRadius,springRadius-grooveDepth/2,linkThickness/2,60).toCSG()
CSG grooveOuter = new Cylinder(springRadius-grooveDepth/2,springRadius,linkThickness/2,60).toCSG().movez(linkThickness/2)
CSG springPathCore=grooveInner.union(grooveOuter)

CSG springPathCoreKW=springPathCore.getBoundingBox()
springPathCore=springPathCore

double offsetSprings = centerToTop+linkThickness
def springPathDriveSideCutout=springPathCore.hull().toZMax()
					.movez(offsetSprings)
					.union(springPathCore.hull().toZMax()
					.movez(-linkOneSupportWidth/2))
def springPath=springPathCore.difference(springPathCoreKW.toXMin().toYMin())
							.difference(springPathCoreKW.toXMin().toYMin().rotz(35))
				.rotz(-35)
def radiusOfSpringBoltKW = springRadius/4
def springMountKW =  new Cylinder(radiusOfSpringBoltKW,linkThickness).toCSG()
						.union(new Cylinder(radiusOfSpringBoltKW,linkThickness).toCSG().movey(-dh.getR()-springRadius))
						.union(new Cylinder(radiusOfSpringBoltKW,linkThickness).toCSG().movex(-springRadius))
						.hull()
						.movey(-springRadius-4)
						.rotz(springboltRotation)
					
def springPathDriveMountKW = moveDHValues(springMountKW.toZMax().movez(offsetSprings),dh)
def springPathPassiveMountKW = moveDHValues(springMountKW.toZMin().movez(-springSupportLength/2-0.5),dh)

def springPathDrive = moveDHValues(springPath.toZMax().movez(offsetSprings),dh)
def springPathPassive = moveDHValues(springPath.toZMin().movez(-baseCorRad),dh)
def nutMeasurments= Vitamins.getConfiguration("heatedThreadedInsert", "M5")
double nutheight=nutMeasurments.installLength
double nutsertRadiout = nutMeasurments.diameter/2.0
def SpringLug = new RoundedCylinder(nutsertRadiout*2, nutheight).cornerRadius(cornerRad).toCSG()
def SpringLugLowerSupport =CSG.unionAll([SpringLug.movey(nutsertRadiout*3),SpringLug.movex(-nutsertRadiout*3).movey(nutsertRadiout*3)])
def springMount = SpringLug.union([SpringLugLowerSupport.movez(nutheight)]).hull()
					.difference(new Cylinder(nutsertRadiout,nutheight).toCSG())
					.difference(new Cylinder(5.1/2.0,nutheight*2).toCSG().movez(nutheight))
					.movex(springRadius+nutsertRadiout)
					.rotz(45)
def springMountDrive=moveDHValues(springMount.mirrorz().toZMax().movez(offsetSprings-nutheight),dh)
def springMountPassive=moveDHValues(springMount.toZMin().movez(-baseCorRad+nutheight),dh)

//end Spring path section

//Drive Side

double bodyEdgeToShaft = motormeasurments.body_x/2.0
def boltheight  =d.getDH_R(linkIndex) +bodyEdgeToShaft+3
CSG shaftMount = linkBuildingBlockRound
					.movez(centerToTop)
CSG nutsert = moveDHValues(
				Vitamins.get("heatedThreadedInsert", "M5")
				.toZMax().movez(centerToTop),dh)
				.movez(backsetBoltOne)
				.movex(boltheight)
CSG nutsert2=nutsert.movez(backsetBoltTwo)

CSG bolt = moveDHValues(
	Vitamins.get("capScrew", "M5")
	.movez(centerToTop+linkThickness),dh)
	.movez(backsetBoltOne)
	.movex(boltheight).setManipulator(manipulator)
CSG bolt2=bolt.movez(backsetBoltTwo)
.setManipulator(manipulator)
CSG driveSide = moveDHValues(shaftMount,dh)
CSG driveConnector = driveSide.movex(boltheight)
						.movez(backsetBoltOne)
CSG driveBolt2=driveConnector.movez(backsetBoltTwo)

CSG driveUnit=driveConnector
					.union(driveSide)
					.union(driveBolt2)
					.hull()
driveSide=driveSide.union(driveUnit)
				.union(springPathDrive)
				.difference(HornModel)
				.difference([bolt,bolt2])
driveSide.setManipulator(manipulator)
//END Drive side



//Servo mount
def supportBeam= new RoundedCube(baseCorRad*2.0,motormeasurments.body_y+linkThickness*2,15)
					.cornerRadius(cornerRad).toCSG()
					.toZMax()
					.movey(-motormeasurments.body_x/2+linkThickness/4)
					.transformed(TransformFactory.nrToCSG(motorLocation))
//END Servo Mount
					



//PassiveSIde
def passiveTHickness = baseCorRad-centerTobottom
println "Link thickness = "+linkThickness+" passive side = "+passiveTHickness
CSG passiveMount = new RoundedCylinder(linkYDimention/2,passiveTHickness)
					.cornerRadius(cornerRad)
					.toCSG()
					.movez(-baseCorRad)

CSG passiveSide = moveDHValues(passiveMount,dh)
CSG passivConnector = passiveSide.movex(boltheight)
						.movez(backsetBoltOne)
CSG passivBolt2=passivConnector.movez(backsetBoltTwo)

CSG passiveUnit=	passivConnector
						.union(passiveSide)
						.union(passivBolt2)
						.hull()

							
passiveSide=passiveSide.union([passiveUnit])
				
					
passiveSide.setManipulator(manipulator)
//End Passive Side

// Bearing Mount
def baseCoreheight = vitaminCad.getTotalZ()-mountPlateToHornTop

CSG baseCore = new Cylinder(baseCorRad,baseCorRad,baseCoreheight-0.25,36).toCSG()
				.toZMax()
				.movez(mountPlateToHornTop-0.25)
				.union(supportBeam)
				.union([passivConnector,passivBolt2,driveConnector,driveBolt2])
				.hull()
				.union(passiveSide)
				.union(springPathPassive)
				
				.difference(thrust)
				.difference(driveSide)
				.difference(motorModel)
				.difference(hornkw)
				.difference([nutsert,nutsert2])
				.difference([bolt,bolt2])
				.difference(keepawayCan)
				.union([springMountDrive,springMountPassive])
				.setManipulator(manipulator)
//END Bearing Mount
def vitamins =	[HornModel,thrust,hornkw,bolt,bolt2]
driveSide.setManufacturing({
	it.rotx(90).toZMin()
})
baseCore.setManufacturing({
	it.rotx(90).toZMin()
})
driveSide.setName("elbowDriveLink")
baseCore.setName("elbowThrustBearingSideLink")

return [driveSide,baseCore].collect{it.setColor(javafx.scene.paint.Color.LIGHTPINK)}


