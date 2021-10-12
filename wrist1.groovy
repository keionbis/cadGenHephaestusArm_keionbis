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
	args = [base.getAllDHChains().get(0),3,centerTheMotorsValue,motorLocation]
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
CSG shaft=   Vitamins.get(	type,size)
				
shaft=moveDHValues(shaft,dh)
def mountPlateToHornTop = Vitamins.getConfiguration(type,size).get("mountPlateToHornTop")
def bearingHeight =mountPlateToHornTop-2
CSG thrust = moveDHValues(Vitamins.get("ballBearing","Thrust_1andAHalfinch")
						.movez(bearingHeight),dh)
thrust.setManipulator(manipulator)

TransformNR motorLocation=args[3]
CSG motor=   Vitamins.get(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
def motorModel = motor.transformed(TransformFactory.nrToCSG(motorLocation))
				.setManipulator(manipulator)

shaft.setManipulator(manipulator)

//Servo mount
HashMap<String, Object> motormeasurments = Vitamins.getConfiguration(conf.getElectroMechanicalType(),conf.getElectroMechanicalSize())
double linkThickness = mountPlateToHornTop
def cornerRad=2
double baseCorRad = Vitamins.getConfiguration("ballBearing","Thrust_1andAHalfinch").outerDiameter/2+5

def supportBeam= moveDHValues(new RoundedCube(motormeasurments.body_y+linkThickness*2.0,motormeasurments.body_x+linkThickness*2,25)
						.cornerRadius(cornerRad).toCSG()
						.toZMax()
						.movez(d.getDH_D(linkIndex)-motormeasurments.body_x/2)
					,dh)
//END Servo Mount

// Bearing Mount
HashMap<String, Object> hornCOnfig = Vitamins.getConfiguration(type,size)
def baseCoreheight = shaft.getTotalZ()-mountPlateToHornTop
CSG hornKW = moveDHValues(new Cylinder(hornCOnfig.hornDiameter/2+1, d.getDH_D(linkIndex)).toCSG()
							.movez(mountPlateToHornTop+7)
	,dh)
				.setManipulator(manipulator)
CSG baseCore = moveDHValues(new Cylinder(baseCorRad,baseCorRad,baseCoreheight,36).toCSG()
								.toZMin()
								.movez(mountPlateToHornTop)
				,dh)
				.union(supportBeam.toYMax().movey(supportBeam.getMinY()+cornerRad*2))
				.hull()
				.union(supportBeam)
				.difference(shaft)
				.difference(thrust)
				.difference(motorModel)
				.difference(hornKW)
				.setManipulator(manipulator)
//END Bearing Mount


return [shaft,thrust,motorModel,baseCore].collect{it.setColor(javafx.scene.paint.Color.GREY)}



