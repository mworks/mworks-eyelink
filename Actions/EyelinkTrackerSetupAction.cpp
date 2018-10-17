//
//  EyelinkTrackerSetupAction.cpp
//  Eyelink
//
//  Created by Christopher Stawarz on 10/17/18.
//

#include "EyelinkTrackerSetupAction.hpp"


BEGIN_NAMESPACE_MW


const std::string EyelinkTrackerSetupAction::DEVICE("device");
const std::string EyelinkTrackerSetupAction::CALIBRATION_TYPE("calibration_type");


void EyelinkTrackerSetupAction::describeComponent(ComponentInfo &info) {
    Action::describeComponent(info);
    
    info.setSignature("action/eyelink_tracker_setup");
    
    info.addParameter(DEVICE);
    info.addParameter(CALIBRATION_TYPE, "HV9");
}


EyelinkTrackerSetupAction::EyelinkTrackerSetupAction(const ParameterValueMap &parameters) :
    Action(parameters),
    weakDevice(parameters[DEVICE].getRegistry()->getObject<Eyelink>(parameters[DEVICE].str())),
    calibrationType(variableOrText(parameters[CALIBRATION_TYPE]))
{
    if (weakDevice.expired()) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN,
                              "Device is not an EyeLink interface",
                              parameters[DEVICE].str());
    }
}


bool EyelinkTrackerSetupAction::execute() {
    if (auto sharedDevice = weakDevice.lock()) {
        sharedDevice->doTrackerSetup(calibrationType->getValue().getString());
    }
    return true;
}


END_NAMESPACE_MW
