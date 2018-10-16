/*
 *  Eyelink.h
 *  Eyelink
 *
 *  Created by Philipp Schwedhelm on 13.12.10.
 *  Copyright 2010 German Primate Center. All rights reserved.
 *
 */

#ifndef Eyelink_H_
#define Eyelink_H_


BEGIN_NAMESPACE_MW


class Eyelink : public IODevice, boost::noncopyable {
    
public:
    static const std::string IP;
    static const std::string RX;
    static const std::string RY;
    static const std::string LX;
    static const std::string LY;
    static const std::string EX;
    static const std::string EY;
    static const std::string EZ;
    static const std::string H_RX;
    static const std::string H_RY;
    static const std::string H_LX;
    static const std::string H_LY;
    static const std::string P_RX;
    static const std::string P_RY;
    static const std::string P_LX;
    static const std::string P_LY;
    static const std::string P_R;
    static const std::string P_L;
    static const std::string E_DIST;
    static const std::string EYE_TIME;
    static const std::string UPDATE_PERIOD;
    
    static void describeComponent(ComponentInfo &info);
    
    explicit Eyelink(const ParameterValueMap &parameters);
    ~Eyelink();
    
    bool initialize() override;
    bool startDeviceIO() override;
    bool stopDeviceIO() override;
    
private:
    bool update();
    void handleSample(const FSAMPLE &sample, MWTime sampleTime);
    
    using unique_lock = std::unique_lock<std::mutex>;
    static unique_lock::mutex_type& eyelinkDriverLock;
    static bool eyelinkInitialized;
    
    const std::string tracker_ip;
    const boost::shared_ptr<Variable> e_rx;
    const boost::shared_ptr<Variable> e_ry;
    const boost::shared_ptr<Variable> e_lx;
    const boost::shared_ptr<Variable> e_ly;
    const boost::shared_ptr<Variable> e_x;
    const boost::shared_ptr<Variable> e_y;
    const boost::shared_ptr<Variable> e_z;
    const boost::shared_ptr<Variable> h_rx;
    const boost::shared_ptr<Variable> h_ry;
    const boost::shared_ptr<Variable> h_lx;
    const boost::shared_ptr<Variable> h_ly;
    const boost::shared_ptr<Variable> p_rx;
    const boost::shared_ptr<Variable> p_ry;
    const boost::shared_ptr<Variable> p_lx;
    const boost::shared_ptr<Variable> p_ly;
    const boost::shared_ptr<Variable> p_r;
    const boost::shared_ptr<Variable> p_l;
    const double e_dist;
    const boost::shared_ptr<Variable> e_time;
    const MWorksTime update_period;
    
    const boost::shared_ptr<Clock> clock;
    boost::shared_ptr<ScheduleTask> schedule_node;
    
    char version_info[256];
    int tracker_version;
    char data_file_name[11];
    int errors;
    bool running;
    
};


END_NAMESPACE_MW


#endif /* Eyelink_H_ */
