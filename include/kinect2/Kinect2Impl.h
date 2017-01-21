#ifndef KINECT2_KINECT2IMPL_H
#define KINECT2_KINECT2IMPL_H
//# INCLUDES ##################################################################
#include <kinect2/Kinect2.h>
#include <string>
#include <memory>
#include <map>

//# TYPEDEFS / CONSTANTS ######################################################

//# DECLARATION ###############################################################

namespace kinect2
{
namespace private_ns
{

class Kinect2ImplHolder;

//#############################################################################

class Kinect2Impl
{

public:
    Kinect2Impl();
    virtual ~Kinect2Impl();

    Kinect2::tReturnCode open();
    Kinect2::tReturnCode close();
    bool isOpen() const;

    Kinect2::tReturnCode start();
    Kinect2::tReturnCode stop();
    Kinect2::tReturnCode restart();
    bool isRunning() const;

    Kinect2::tReturnCode update();

    Kinect2::tReturnCode setPipelineType(
            const Kinect2::tPipelineType i_ePipelineType);
    Kinect2::tReturnCode setDeviceId(const int i_nDeviceId);

    void setMinDepth(const float i_fMinDepth);
    void setMaxDepth(const float i_fMaxDepth);
    void setBilateralFilter(const bool i_bBilateralFilter);
    void setEdgeAwareFilter(const bool i_bEdgeAwareFilter);
    float getMinDepth() const;
    float getMaxDepth() const;
    bool isBilateralFilter() const;
    bool isEdgeAwareFilter() const;

    void setRgb(const bool i_bRgb);
    void setDepth(const bool i_bDepth);
    void setIr(const bool i_bIr);

    void setSync(const bool i_bSync);
    void setUndistort(const bool i_bUndistort);
    void setRegister(const bool i_bRegister);
    void setIrUndistort(const bool i_bIrUndistort);
    void setBigDepth(const bool i_bBigDepth);

    int getDeviceId() const;
    const std::string& getDeviceSerial() const;
    Kinect2::tPipelineType getPipelineType() const;

    Kinect2::tColorCameraParams getColorCameraParameters() const;
    Kinect2::tIrCameraParams getIrCameraParameters() const;

    bool isSync() const;
    bool isRgb() const;
    bool isDepth() const;
    bool isIr() const;
    bool isUndistort() const;
    bool isRegister() const;
    bool isIrUndistort() const;
    bool isBigDepth() const;

    int hasNewFrame() const;
    void getFrames(const int i_nTypes, tFrameMap* i_pFrameMap) const;

    bool hasNewFrameRgb() const;
    const std::shared_ptr<Frame>& getFrameRgb() const;
    bool hasNewFrameDepth() const;
    const std::shared_ptr<Frame>& getFrameDepth() const;
    bool hasNewFrameIr() const;
    const std::shared_ptr<Frame>& getFrameIr() const;
    bool hasNewFrameUndistorted() const;
    const std::shared_ptr<Frame>& getFrameUndistorted() const;
    bool hasNewFrameRegistered() const;
    const std::shared_ptr<Frame>& getFrameRegistered() const;
    bool hasNewFrameIrUndistorted() const;
    const std::shared_ptr<Frame>& getFrameIrUndistorted() const;
    bool hasNewFrameBigDepth() const;
    const std::shared_ptr<Frame>& getFrameBigDepth() const;

private:
    void fetchFrames();

    Kinect2::tReturnCode setup();
    Kinect2::tReturnCode reopen();
    void installListeners();
    void cleanListeners();
    void waitForListeners();
    void copyFramesSync();
    void copyFramesRgb();
    void copyFramesDepthIr();
    void applyRegistration();
    void applyUndistortion();
    void applyUndistortionOrRegistration(const bool i_bForce = false);
    void applyIrUndistortion();

    Kinect2::tPipelineType ePipelineType;
    int nDeviceId;
    std::string strSerial;
    bool bRgb;
    bool bDepth;
    bool bIr;
    bool bUndistort;
    bool bRegister;
    bool bIrUndistort;
    bool bBigDepth;
    bool bSync;
    bool bOpen;
    bool bRunning;

    bool bNewRgb;
    bool bNewDepth;
    bool bNewIr;
    bool bNewUndistorted;
    bool bNewRegistered;
    bool bNewIrUndistorted;
    bool bNewBigDepth;

    Kinect2ImplHolder* p;
};

//#############################################################################

}   // ns private_ns
}   // ns kinect2

//#############################################################################

#endif // KINECT2_KINECT2IMPL_H
