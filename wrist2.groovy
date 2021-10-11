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
import javafx.scene.paint.Color
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
	def motorLocation = new TransformNR(0,0,centerTheMotorsValue,new RotationNR())
	def d =base.getAllDHChains().get(0)
	motorLocation=new TransformNR(0,0,d.getDH_D(5)-centerTheMotorsValue,new RotationNR(0,0,0))
					.times(motorLocation
						.times(new TransformNR(0,0,d.getDH_D(4),new RotationNR(0,-90,0))))
	
	args = [d,4,centerTheMotorsValue,motorLocation]
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
def bearingHeight =mountPlateToHornTop-2 +d.getDH_D(linkIndex+1)
CSG thrust = Vitamins.get("ballBearing","Thrust_1andAHalfinch")
						.movez(bearingHeight)
thrust.setManipulator(manipulator)
//END bearing


TransformNR motorLocation=args[3]

//Motor for next link
CSG motor=   Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
def part = motor.transformed(TransformFactory.nrToCSG(motorLocation))
part.setManipulator(manipulator)

// Keepaway for motor of this link
HashMap<String, Object> motormeasurments = Vitamins.getConfiguration(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
def b_y = motormeasurments.body_x/2
def hyp =Math.sqrt(b_y*b_y+b_y*b_y)
def centerTobottom = args[2]+motormeasurments.shoulderHeight
def centerToTop=args[2]+mountPlateToHornTop
def kwCanheight =args[2]+centerTobottom
def linkageThicknessSMallShaftLen = motormeasurments.bottomShaftLength
println "xValue = "+b_y+" hyp = "+hyp
CSG keepawayCan = new Cylinder(hyp, kwCanheight).toCSG()
					.toZMax()
					.movez(args[2])
CSG shaftKW = new Cylinder(motormeasurments.bottomShaftDiameter/2+0.5, kwCanheight+linkageThicknessSMallShaftLen*4).toCSG()
				.toZMax()
				.movez(args[2])			
keepawayCan=moveDHValues(keepawayCan.union(shaftKW),dh)
keepawayCan.setManipulator(manipulator)
// END motor keepaway

//Horn keepaway
CSG hornkw = new Cylinder(hornCOnfig.hornDiameter/2+1, mountPlateToHornTop+1).toCSG()
			.movez(d.getDH_D(linkIndex+1))
hornkw.setManipulator(manipulator)
// end horn keepaway

// Hull Bulding BLocks
def cornerRad=2
double linkThickness = mountPlateToHornTop
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

//Drive Side
CSG shaftMount = linkBuildingBlockRound
					.movez(centerToTop)

CSG driveSide = moveDHValues(shaftMount,dh)
				.difference(HornModel)
driveSide.setManipulator(manipulator)
//END Drive side

//PassiveSIde
CSG passiveMount = linkBuildingBlockRound	
					.movez(-centerTobottom-linkThickness)

CSG passiveSide = moveDHValues(passiveMount,dh)
					.difference(keepawayCan)
passiveSide.setManipulator(manipulator)
//End Passive Side

passiveSide.setColor(Color.RED)

return [HornModel,part,hornkw,driveSide,passiveSide,thrust].collect{it.setColor(javafx.scene.paint.Color.RED)}
