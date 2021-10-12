import com.neuronrobotics.bowlerstudio.creature.MobileBaseLoader
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
import eu.mihosoft.vrl.v3d.Parabola
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
	args = [base.getAllDHChains().get(0),5,centerTheMotorsValue,motorLocation]
}
TransformNR motorLocation=args[3]
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
//Vitamins 
def type=	d.getLinkConfiguration(linkIndex-1).getShaftType()
def size = d.getLinkConfiguration(linkIndex-1).getShaftSize()
CSG vitaminCad=   Vitamins.get(	type,size)
vitaminCad.setManipulator(manipulator)


def mountPlateToHornTop = Vitamins.getConfiguration(type,size).get("mountPlateToHornTop")
def baseCoreheight = vitaminCad.getTotalZ()-mountPlateToHornTop
def bearingHeight =mountPlateToHornTop-2
CSG thrust = Vitamins.get("ballBearing","Thrust_1andAHalfinch")
						.movez(bearingHeight)
thrust.setManipulator(manipulator)
def thrustMeasurments= Vitamins.getConfiguration("ballBearing","Thrust_1andAHalfinch")
double baseCorRad = thrustMeasurments.outerDiameter/2+5
//END vitamins

//Mount holes
CSG mount = Vitamins.get("heatedThreadedInsert", "M5")
			.toZMax()
			.movez(baseCoreheight+mountPlateToHornTop)
			.movex(25.0)
def mounts =[]
for(def i=0;i<360;i+=90) {
	mounts.add(mount.rotz(i).setManipulator(manipulator))
}
//end Mount holes

// Link

CSG allignment = Parabola.coneByHeight(4, 8)
						.rotx(-90)
						.toZMin()
						.movez(baseCoreheight)
						.toYMin()
						.movey(-baseCorRad)
						.rotz(45)
CSG baseCore = new Cylinder(baseCorRad,baseCorRad,baseCoreheight,36).toCSG()
				.movez(mountPlateToHornTop)
				.difference(thrust)
				.difference(vitaminCad)
				.difference(allignment)
				.difference(mounts)
baseCore.setManipulator(manipulator)
//END link


mounts.addAll([vitaminCad,thrust,baseCore])

return mounts.collect{it.setColor(javafx.scene.paint.Color.PINK)}

