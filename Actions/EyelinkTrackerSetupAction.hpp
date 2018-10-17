//
//  EyelinkTrackerSetupAction.hpp
//  Eyelink
//
//  Created by Christopher Stawarz on 10/17/18.
//

#ifndef EyelinkTrackerSetupAction_hpp
#define EyelinkTrackerSetupAction_hpp

#include "Eyelink.h"


BEGIN_NAMESPACE_MW


class EyelinkTrackerSetupAction : public Action {
    
public:
    static const std::string DEVICE;
    static const std::string CALIBRATION_TYPE;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit EyelinkTrackerSetupAction(const ParameterValueMap &parameters);
    
    bool execute() override;
    
private:
    const boost::weak_ptr<Eyelink> weakDevice;
    const VariablePtr calibrationType;
    
};


END_NAMESPACE_MW


#endif /* EyelinkTrackerSetupAction_hpp */
