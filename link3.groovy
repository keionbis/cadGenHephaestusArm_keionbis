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
	def motorLocation = new TransformNR(0,0,centerTheMotorsValue,new RotationNR())
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
Affine manipulator = dh.getListener();
def type=	d.getLinkConfiguration(linkIndex-1).getShaftType()
def size = d.getLinkConfiguration(linkIndex-1).getShaftSize()
CSG vitaminCad=   Vitamins.get(	type,size)
.movez(args[2])
vitaminCad=moveDHValues(vitaminCad,dh)

def mountPlateToHornTop = Vitamins.getConfiguration(type,size).get("mountPlateToHornTop")
def bearingHeight =args[2]+mountPlateToHornTop-2
CSG thrust = Vitamins.get("ballBearing","Thrust_1andAHalfinch")
						.movez(bearingHeight)
						
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
// END keepaway



thrust.setManipulator(manipulator)
vitaminCad.setManipulator(manipulator)
keepawayCan.setManipulator(manipulator)
return [vitaminCad,thrust,part,keepawayCan]