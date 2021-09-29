import com.neuronrobotics.bowlerstudio.creature.MobileBaseLoader
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.common.DeviceManager

if(args==null) {
	MobileBase base=DeviceManager.getSpecificDevice( "CarlTheWalkingRobot",{
		//If the device does not exist, prompt for the connection
		
		MobileBase m = MobileBaseLoader.fromGit(
			"https://github.com/Halloween2020TheChild/GroguMechanicsCad.git",
			"hephaestus.xml"
			)
		return m
	})
	args = [base.getAllDHChains().get(0),5]
}
