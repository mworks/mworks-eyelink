/*
 *  Eyelink.cpp
 *  Eyelink
 *
 *  Created by Philipp Schwedhelm on 13.12.10.
 *  Copyright 2010 German Primate Center. All rights reserved.
 *
 */

#include "Eyelink.h"


BEGIN_NAMESPACE_MW


// Allocate the lock on the heap so it's never destructed
Eyelink::unique_lock::mutex_type& Eyelink::eyelinkDriverLock = *(new unique_lock::mutex_type);

bool Eyelink::eyelinkInitialized = false;


const std::string Eyelink::IP("tracker_ip");
const std::string Eyelink::RX("eye_rx");
const std::string Eyelink::RY("eye_ry");
const std::string Eyelink::LX("eye_lx");
const std::string Eyelink::LY("eye_ly");
const std::string Eyelink::EX("eye_x");
const std::string Eyelink::EY("eye_y");
const std::string Eyelink::EZ("eye_z");
const std::string Eyelink::H_RX("href_rx");
const std::string Eyelink::H_RY("href_ry");
const std::string Eyelink::H_LX("href_lx");
const std::string Eyelink::H_LY("href_ly");
const std::string Eyelink::P_RX("pupil_rx");
const std::string Eyelink::P_RY("pupil_ry");
const std::string Eyelink::P_LX("pupil_lx");
const std::string Eyelink::P_LY("pupil_ly");
const std::string Eyelink::P_R("pupil_size_r");
const std::string Eyelink::P_L("pupil_size_l");
const std::string Eyelink::E_DIST("tracking_dist");
const std::string Eyelink::EYE_TIME("eye_time");
const std::string Eyelink::UPDATE_PERIOD("data_interval");


void Eyelink::describeComponent(ComponentInfo &info) {
    IODevice::describeComponent(info);
    
    info.setSignature("iodevice/eyelink");
    
    info.addParameter(IP);
    info.addParameter(RX, false);
    info.addParameter(RY, false);
    info.addParameter(LX, false);
    info.addParameter(LY, false);
    info.addParameter(EX, false);
    info.addParameter(EY, false);
    info.addParameter(EZ, false);
    info.addParameter(H_RX, false);
    info.addParameter(H_RY, false);
    info.addParameter(H_LX, false);
    info.addParameter(H_LY, false);
    info.addParameter(P_RX, false);
    info.addParameter(P_RY, false);
    info.addParameter(P_LX, false);
    info.addParameter(P_LY, false);
    info.addParameter(P_R, false);
    info.addParameter(P_L, false);
    info.addParameter(E_DIST, false);
    info.addParameter(EYE_TIME, false);
    info.addParameter(UPDATE_PERIOD, true, "1ms");
}


static inline VariablePtr optionalVariable(const ParameterValue &param) {
    if (param.empty()) {
        return VariablePtr();
    }
    return VariablePtr(param);
}


Eyelink::Eyelink(const ParameterValueMap &parameters) :
    IODevice(parameters),
    tracker_ip(parameters[IP].str()),
    e_rx(optionalVariable(parameters[RX])),
    e_ry(optionalVariable(parameters[RY])),
    e_lx(optionalVariable(parameters[LX])),
    e_ly(optionalVariable(parameters[LY])),
    e_x(optionalVariable(parameters[EX])),
    e_y(optionalVariable(parameters[EY])),
    e_z(optionalVariable(parameters[EZ])),
    h_rx(optionalVariable(parameters[H_RX])),
    h_ry(optionalVariable(parameters[H_RY])),
    h_lx(optionalVariable(parameters[H_LX])),
    h_ly(optionalVariable(parameters[H_LY])),
    p_rx(optionalVariable(parameters[P_RX])),
    p_ry(optionalVariable(parameters[P_RY])),
    p_lx(optionalVariable(parameters[P_LX])),
    p_ly(optionalVariable(parameters[P_LY])),
    p_r(optionalVariable(parameters[P_R])),
    p_l(optionalVariable(parameters[P_L])),
    e_dist(parameters[E_DIST].empty() ? 0.0 : double(parameters[E_DIST])),
    e_time(optionalVariable(parameters[EYE_TIME])),
    update_period(parameters[UPDATE_PERIOD]),
    clock(Clock::instance()),
    errors(0)
{
    if ((e_dist == 0.0) && (e_x || e_y || e_z)) {
        throw SimpleException(M_IODEVICE_MESSAGE_DOMAIN, (boost::format("%s is required to compute %s, %s, and %s")
                                                          % E_DIST % EX % EY % EZ));
    }
}


bool Eyelink::initialize() {
    unique_lock lock(eyelinkDriverLock);
    
    if (eyelinkInitialized) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,"Eyelink was previously initialized! Trying again, but this is dangerous!!");
    }
    
    eyelinkInitialized = false;
    
    // Initializes the link
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Trying to find Eyelink System at %s",tracker_ip);
    if (set_eyelink_address(const_cast<char *>(tracker_ip.c_str()))) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,"Failed to set Tracker to address %s", tracker_ip.c_str());
    }
    
    if (open_eyelink_connection(0)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN,"Failed to connect to Tracker at %s", tracker_ip.c_str());
    } else {
        ELINKNODE node;
        
        // generate data file name
        time_t now = time(nullptr);
        struct tm* t = gmtime(&now);
        
        sprintf(data_file_name, "%02d%06d.edf",(t->tm_year-100),t->tm_yday*1440 + t->tm_hour*60 + t->tm_min);
        //YYMMMMMM : YY=Years since 2k, MMMMMM=Minutes in current year
        
        if (open_data_file(data_file_name)) {
            mwarning(M_IODEVICE_MESSAGE_DOMAIN,"Eyelink datafile setting failed (%s)",data_file_name);
        } else {
            mprintf(M_IODEVICE_MESSAGE_DOMAIN,"Eyelink logs to local file %s",data_file_name);
        }
        
        // Tell the tracker what data to include in samples
        if (OK_RESULT != eyecmd_printf("link_sample_data = LEFT,RIGHT,GAZE,HREF,PUPIL,AREA")) {
            mwarning(M_IODEVICE_MESSAGE_DOMAIN,
                     "Eyelink did not respond to link_sample_data command; sample data may be incomplete");
        }
        
        // tell the tracker who we are
        eyelink_set_name((char*)("MWorks_over_Socket"));
        
        // verify the name
        if (eyelink_get_node(0,&node) != OK_RESULT) {
            merror(M_IODEVICE_MESSAGE_DOMAIN,"Error, EyeLink doesn't respond");
        } else {
            eyemsg_printf((char*)("%s connected"), node.name);
        }
        
        // Enable link data reception
        eyelink_reset_data(1);
    }
    
    // Eyelink should now be in "TCP-Link Open" mode
    
    if (eyelink_is_connected()) {
        tracker_version = eyelink_get_tracker_version(version_info);
        mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Eyelink %d System Version %s connected via Socket",tracker_version,version_info);
        
        eyelinkInitialized = true;
        stopped = true;
    } else {
        merror(M_IODEVICE_MESSAGE_DOMAIN,"Error, Eyelink Connection could not be established");
    }
    
    return eyelinkInitialized;
}


Eyelink::~Eyelink() {
    unique_lock lock(eyelinkDriverLock);
    
    if (eyelinkInitialized) {
        if (!stopped) {
            mwarning(M_IODEVICE_MESSAGE_DOMAIN,"Eyelink is still running !");
            //eyelink stop recording
            if (eyelink_is_connected()) { stop_recording(); }
        }
        
        if (schedule_node) {
            schedule_node->cancel();
            schedule_node.reset();
        }
        
        if (eyelink_is_connected()) {
            // Places EyeLink tracker in off-line (idle) mode
            set_offline_mode();
            // close any open data files
            
            if ( close_data_file() == 0 ) {
                mprintf(M_IODEVICE_MESSAGE_DOMAIN,"Eyelink closed data file %s.",data_file_name);
            }
            
            // disconnect from tracker
            if (eyelink_close(1)) {
                merror(M_IODEVICE_MESSAGE_DOMAIN, "Could not close Eyelink connection");
            }
            
            //close_eyelink_system();
            
            mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Eyelink %d System Version %s disconnected.",tracker_version,version_info);
            
        } else {
            mwarning(M_IODEVICE_MESSAGE_DOMAIN,"Error, Eyelink Shutdown failed");
        }
        
        eyelinkInitialized = false;
    }
}

bool Eyelink::update() {
    unique_lock lock(eyelinkDriverLock);
    
    FSAMPLE evt;
    MWTime inputtime;
    
    if (eyelink_is_connected())	{
        while (eyelink_get_next_data(nullptr)) {
            if (eyelink_in_data_block(1,0)) { //only if data contains samples
                eyelink_get_float_data(&evt);
                
                inputtime = this->clock->getCurrentTimeUS();
                
                /*
                 // occasionally, send the current time together with the sample time back to the tracker (and log it there)
                 if ( ack_msg_counter++ % 512 == 0 )
                 eyemsg_printf((char*)"SAMPLE %ld received %lld",(long)evt.time,inputtime);
                 */
                
                // now update all the variables
                if (e_time) e_time->setValue( (long)evt.time ,inputtime);
                
                if (evt.gx[RIGHT_EYE] != MISSING_DATA &&
                    evt.gy[RIGHT_EYE] != MISSING_DATA &&
                    evt.gx[LEFT_EYE] != MISSING_DATA &&
                    evt.gy[LEFT_EYE] != MISSING_DATA &&
                    (e_x || e_y || e_z))
                {
                    double p43x = evt.gx[LEFT_EYE]/e_dist + 1;
                    double p43y = evt.gy[LEFT_EYE]/e_dist;
                    
                    double p21x = evt.gx[RIGHT_EYE]/e_dist - 2;
                    double p21y = evt.gy[RIGHT_EYE]/e_dist;
                    
                    double d4321 = p43x * p21x + p43y * p21y + 1;
                    double d4343 = p43x * p43x + p43y * p43y + 1;
                    double d2121 = p21x * p21x + p21y * p21y + 1;
                    
                    double denom = d2121 * d4343 - d4321 * d4321;
                    
                    if (std::abs(denom) > 1e-6) { // should always be true when e_dist is really tracking range
                        double numer = p43x * d4321 - p21x * d4343;
                        
                        double mua = numer / denom;
                        double mub = (p43x + d4321 * mua) / d4343;
                        
                        double pax = 1 + mua * p21x;
                        double pay = mua * p21y;
                        double paz = -1 + mua; //-p4321z + mua * p4321z;
                        double pbx = mub * p43x;
                        double pby = mub * p43y;
                        double pbz = -1 + mub; //-p4321z + mub * p4321z;
                        
                        if (e_x) e_x->setValue(pax + 0.5*(pbx-pax),inputtime);
                        if (e_y) e_y->setValue(pay + 0.5*(pby-pay),inputtime);
                        if (e_z) e_z->setValue(paz + 0.5*(pbz-paz),inputtime);
                    }
                } else {
                    if (e_x && e_x->getValue().getFloat() != MISSING_DATA)
                        e_x->setValue((float)MISSING_DATA,inputtime);
                    if (e_y && e_y->getValue().getFloat() != MISSING_DATA)
                        e_y->setValue((float)MISSING_DATA,inputtime);
                    if (e_z && e_z->getValue().getFloat() != MISSING_DATA)
                        e_z->setValue((float)MISSING_DATA,inputtime);
                }
                
                if (evt.gx[RIGHT_EYE] != MISSING_DATA &&
                    evt.gy[RIGHT_EYE] != MISSING_DATA)
                {
                    if (e_rx) e_rx->setValue( evt.gx[RIGHT_EYE] ,inputtime);
                    if (e_ry) e_ry->setValue( evt.gy[RIGHT_EYE] ,inputtime);
                } else {
                    if (e_rx && e_rx->getValue().getFloat() != MISSING_DATA)
                        e_rx->setValue((float)MISSING_DATA,inputtime);
                    if (e_ry && e_ry->getValue().getFloat() != MISSING_DATA)
                        e_ry->setValue((float)MISSING_DATA,inputtime);
                }
                
                if (evt.gx[LEFT_EYE] != MISSING_DATA &&
                    evt.gy[LEFT_EYE] != MISSING_DATA)
                {
                    if (e_lx) e_lx->setValue( evt.gx[LEFT_EYE] ,inputtime);
                    if (e_ly) e_ly->setValue( evt.gy[LEFT_EYE] ,inputtime);
                } else {
                    if (e_lx && e_lx->getValue().getFloat() != MISSING_DATA)
                        e_lx->setValue((float)MISSING_DATA,inputtime);
                    if (e_ly && e_ly->getValue().getFloat() != MISSING_DATA)
                        e_ly->setValue((float)MISSING_DATA,inputtime);
                }
                
                if (evt.hx[RIGHT_EYE] != -7936.0f &&
                    evt.hy[RIGHT_EYE] != -7936.0f)
                {
                    if (h_rx) h_rx->setValue( evt.hx[RIGHT_EYE] ,inputtime);
                    if (h_ry) h_ry->setValue( evt.hy[RIGHT_EYE] ,inputtime);
                } else {
                    if (h_rx && h_rx->getValue().getFloat() != MISSING_DATA)
                        h_rx->setValue((float)MISSING_DATA,inputtime);
                    if (h_ry && h_ry->getValue().getFloat() != MISSING_DATA)
                        h_ry->setValue((float)MISSING_DATA,inputtime);
                }
                
                if (evt.hx[LEFT_EYE] != -7936.0f &&
                    evt.hy[LEFT_EYE] != -7936.0f)
                {
                    if (h_lx) h_lx->setValue( evt.hx[LEFT_EYE] ,inputtime);
                    if (h_ly) h_ly->setValue( evt.hy[LEFT_EYE] ,inputtime);
                } else {
                    if (h_lx && h_lx->getValue().getFloat() != MISSING_DATA)
                        h_lx->setValue((float)MISSING_DATA,inputtime);
                    if (h_ly && h_ly->getValue().getFloat() != MISSING_DATA)
                        h_ly->setValue((float)MISSING_DATA,inputtime);
                }
                
                if (evt.px[RIGHT_EYE] != MISSING_DATA &&
                    evt.py[RIGHT_EYE] != MISSING_DATA)
                {
                    if (p_rx) p_rx->setValue( evt.px[RIGHT_EYE] ,inputtime);
                    if (p_ry) p_ry->setValue( evt.py[RIGHT_EYE] ,inputtime);
                } else {
                    if (p_rx && p_rx->getValue().getFloat() != MISSING_DATA)
                        p_rx->setValue((float)MISSING_DATA,inputtime);
                    if (p_ry && p_ry->getValue().getFloat() != MISSING_DATA)
                        p_ry->setValue((float)MISSING_DATA,inputtime);
                }
                
                if (evt.px[LEFT_EYE] != MISSING_DATA &&
                    evt.py[LEFT_EYE] != MISSING_DATA )
                {
                    if (p_lx) p_lx->setValue( evt.px[LEFT_EYE] ,inputtime);
                    if (p_ly) p_ly->setValue( evt.py[LEFT_EYE] ,inputtime);
                } else {
                    if (p_lx && p_lx->getValue().getFloat() != MISSING_DATA)
                        p_lx->setValue((float)MISSING_DATA,inputtime);
                    if (p_ly && p_ly->getValue().getFloat() != MISSING_DATA)
                        p_ly->setValue((float)MISSING_DATA,inputtime);
                }
                
                if (evt.pa[RIGHT_EYE] != 0)
                {
                    if (p_r) p_r->setValue( evt.pa[RIGHT_EYE] ,inputtime);
                } else {
                    if (p_r && p_r->getValue().getFloat() != 0)
                        p_r->setValue((float)MISSING_DATA,inputtime);
                }
                
                if (evt.pa[LEFT_EYE] != 0) {
                    if (p_l) p_l->setValue( evt.pa[LEFT_EYE] ,inputtime);
                } else {
                    if (p_l && p_l->getValue().getFloat() != 0)
                        p_l->setValue((float)MISSING_DATA,inputtime);
                }
            }
        }
    } else {
        if (++errors * update_period > (MWorksTime)1000000) { //just a quick hack but impossible to ignore by the user
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Fatal Error! EyeLink Connection Lost!!");
            errors = 0;
        }
    }
    
    return true;
}


bool Eyelink::startDeviceIO() {
    unique_lock lock(eyelinkDriverLock);
    
    if (eyelink_is_connected() && stopped) {
        //Eyelink to offline mode
        set_offline_mode();
        // Eyelink to record mode
        if (start_recording(0,1,1,0)) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Eyelink does not start!");
        }
        
        boost::weak_ptr<Eyelink> weakThis(component_shared_from_this<Eyelink>());
        schedule_node = Scheduler::instance()->scheduleUS(std::string(FILELINE ": ") + getTag(),
                                                          update_period, //defer first start one period
                                                          update_period, //repeat interval
                                                          M_REPEAT_INDEFINITELY,
                                                          [weakThis]() {
                                                              if (auto sharedThis = weakThis.lock()) {
                                                                  sharedThis->update();
                                                              }
                                                              return nullptr;
                                                          },
                                                          M_DEFAULT_IODEVICE_PRIORITY,
                                                          M_DEFAULT_IODEVICE_WARN_SLOP_US,
                                                          M_DEFAULT_IODEVICE_FAIL_SLOP_US,
                                                          M_MISSED_EXECUTION_DROP);
        
        stopped = false;
        mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Eyelink successfully started.");
    } else {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "Warning! Could not start EyeLink! (StartIO)");
    }
    
    return !stopped;
}


bool Eyelink::stopDeviceIO() {
    unique_lock lock(eyelinkDriverLock);
    
    if (!stopped) {
        if (schedule_node) {
            schedule_node->cancel();
            schedule_node.reset();
        }
        
        if (eyelink_is_connected()) {
            //eyelink stop recording
            stop_recording();
            //go to eyelink offline mode
            set_offline_mode();
        } else {
            mwarning(M_IODEVICE_MESSAGE_DOMAIN, "Warning! Could not stop EyeLink! Connection Lost!! (StopIO)");
        }
        
        if (e_time) e_time->setValue( (float)MISSING_DATA );
        if (e_rx) e_rx->setValue( (float)MISSING_DATA );
        if (e_ry) e_ry->setValue( (float)MISSING_DATA );
        if (e_lx) e_lx->setValue( (float)MISSING_DATA );
        if (e_ly) e_ly->setValue( (float)MISSING_DATA );
        if (e_x) e_x->setValue( (float)MISSING_DATA );
        if (e_y) e_y->setValue( (float)MISSING_DATA );
        if (e_z) e_z->setValue( (float)MISSING_DATA );
        if (h_rx) h_rx->setValue( (float)MISSING_DATA );
        if (h_ry) h_ry->setValue( (float)MISSING_DATA );
        if (h_lx) h_lx->setValue( (float)MISSING_DATA );
        if (h_ly) h_ly->setValue( (float)MISSING_DATA );
        if (p_rx) p_rx->setValue( (float)MISSING_DATA );
        if (p_ry) p_ry->setValue( (float)MISSING_DATA );
        if (p_lx) p_lx->setValue( (float)MISSING_DATA );
        if (p_ly) p_ly->setValue( (float)MISSING_DATA );
        if (p_r) p_r->setValue( (float)MISSING_DATA );
        if (p_l) p_l->setValue( (float)MISSING_DATA );
        
        stopped = true;
        mprintf(M_IODEVICE_MESSAGE_DOMAIN, "Eyelink successfully stopped.");
    }
    
    return stopped;
}


END_NAMESPACE_MW
