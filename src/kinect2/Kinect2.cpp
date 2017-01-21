//# INCLUDES ##################################################################
#include <kinect2/Kinect2.h>
#include <kinect2/Kinect2Impl.h>

//# TYPEDEFS / CONSTANTS ######################################################

//# DECLARATION ###############################################################

namespace kinect2
{

//#############################################################################
Kinect2::Kinect2() : pImpl(new private_ns::Kinect2Impl)
{}

//#############################################################################
Kinect2::~Kinect2()
{
    delete pImpl;
}

//#############################################################################
Kinect2::tReturnCode Kinect2::open()
{
    return pImpl->open();
}

//#############################################################################
Kinect2::tReturnCode Kinect2::close()
{
    return pImpl->close();
}

//#############################################################################
bool Kinect2::isOpen() const
{
    return pImpl->isOpen();
}

//#############################################################################
Kinect2::tReturnCode Kinect2::start()
{
    return pImpl->start();
}

//#############################################################################
Kinect2::tReturnCode Kinect2::stop()
{
    return pImpl->stop();
}

//#############################################################################
Kinect2::tReturnCode Kinect2::restart()
{
    return pImpl->restart();
}

//#############################################################################
bool Kinect2::isRunning() const
{
    return pImpl->isRunning();
}


//#############################################################################
Kinect2::tReturnCode Kinect2::update()
{
    return pImpl->update();
}

//#############################################################################
Kinect2::tReturnCode Kinect2::setPipelineType(
        const Kinect2::tPipelineType i_ePipelineType)
{
    return pImpl->setPipelineType(i_ePipelineType);
}

//#############################################################################
Kinect2::tReturnCode Kinect2::setDeviceId(const int i_nDeviceId)
{
    return pImpl->setDeviceId(i_nDeviceId);
}

//#############################################################################
void Kinect2::setMinDepth(const float i_fMinDepth)
{
    pImpl->setMinDepth(i_fMinDepth);
}

//#############################################################################
void Kinect2::setMaxDepth(const float i_fMaxDepth)
{
    pImpl->setMaxDepth(i_fMaxDepth);
}

//#############################################################################
void Kinect2::setBilateralFilter(const bool i_bBilateralFilter)
{
    pImpl->setBilateralFilter(i_bBilateralFilter);
}

//#############################################################################
void Kinect2::setEdgeAwareFilter(const bool i_bEdgeAwareFilter)
{
    pImpl->setEdgeAwareFilter(i_bEdgeAwareFilter);
}

//#############################################################################
float Kinect2::getMinDepth() const
{
    return pImpl->getMinDepth();
}

//#############################################################################
float Kinect2::getMaxDepth() const
{
    return pImpl->getMaxDepth();
}

//#############################################################################
bool Kinect2::isBilateralFilter() const
{
    return pImpl->isBilateralFilter();
}

//#############################################################################
bool Kinect2::isEdgeAwareFilter() const
{
    return pImpl->isEdgeAwareFilter();
}

//#############################################################################
void Kinect2::setRgb(const bool i_bRgb)
{
    pImpl->setRgb(i_bRgb);
}

//#############################################################################
void Kinect2::setDepth(const bool i_bDepth)
{
    pImpl->setDepth(i_bDepth);
}

//#############################################################################
void Kinect2::setIr(const bool i_bIr)
{
    pImpl->setIr(i_bIr);
}

//#############################################################################
void Kinect2::setSync(const bool i_bSync)
{
    pImpl->setSync(i_bSync);
}

//#############################################################################
void Kinect2::setUndistort(const bool i_bUndistort)
{
    pImpl->setUndistort(i_bUndistort);
}

//#############################################################################
void Kinect2::setRegister(const bool i_bRegister)
{
    pImpl->setRegister(i_bRegister);
}

//#############################################################################
void Kinect2::setIrUndistort(const bool i_bIrUndistort)
{
    pImpl->setIrUndistort(i_bIrUndistort);
}

//#############################################################################
void Kinect2::setBigDepth(const bool i_bBigDepth)
{
    pImpl->setBigDepth(i_bBigDepth);
}

//#############################################################################
int Kinect2::getDeviceId() const
{
    return pImpl->getDeviceId();
}

//#############################################################################
const std::string& Kinect2::getDeviceSerial() const
{
    return pImpl->getDeviceSerial();
}

//#############################################################################
Kinect2::tPipelineType Kinect2::getPipelineType() const
{
    return pImpl->getPipelineType();
}

//#############################################################################
Kinect2::tColorCameraParams Kinect2::getColorCameraParameters() const
{
    return pImpl->getColorCameraParameters();
}

//#############################################################################
Kinect2::tIrCameraParams Kinect2::getIrCameraParameters() const
{
    return pImpl->getIrCameraParameters();
}

//#############################################################################
bool Kinect2::isSync() const
{
    return pImpl->isSync();
}

//#############################################################################
bool Kinect2::isRgb() const
{
    return pImpl->isRgb();
}

//#############################################################################
bool Kinect2::isDepth() const
{
    return pImpl->isDepth();
}

//#############################################################################
bool Kinect2::isIr() const
{
    return pImpl->isIr();
}

//#############################################################################
bool Kinect2::isUndistort() const
{
    return pImpl->isUndistort();
}

//#############################################################################
bool Kinect2::isRegister() const
{
    return pImpl->isRegister();
}

//#############################################################################
bool Kinect2::isIrUndistort() const
{
    return pImpl->isIrUndistort();
}

//#############################################################################
bool Kinect2::isBigDepth() const
{
    return pImpl->isBigDepth();
}

//#############################################################################
int Kinect2::hasNewFrame() const
{
    return pImpl->hasNewFrame();
}

//#############################################################################
void Kinect2::getFrames(const int i_nTypes, tFrameMap* i_pFrameMap) const
{
    pImpl->getFrames(i_nTypes, i_pFrameMap);
}

//#############################################################################
bool Kinect2::hasNewFrameRgb() const
{
    return pImpl->hasNewFrameRgb();
}

//#############################################################################
const tFramePtr& Kinect2::getFrameRgb() const
{
    return pImpl->getFrameRgb();
}

//#############################################################################
bool Kinect2::hasNewFrameDepth() const
{
    return pImpl->hasNewFrameDepth();
}

//#############################################################################
const tFramePtr& Kinect2::getFrameDepth() const
{
    return pImpl->getFrameDepth();
}

//#############################################################################
bool Kinect2::hasNewFrameIr() const
{
    return pImpl->hasNewFrameIr();
}

//#############################################################################
const tFramePtr& Kinect2::getFrameIr() const
{
    return pImpl->getFrameIr();
}

//#############################################################################
bool Kinect2::hasNewFrameUndistorted() const
{
    return pImpl->hasNewFrameUndistorted();
}

//#############################################################################
const tFramePtr& Kinect2::getFrameUndistorted() const
{
    return pImpl->getFrameUndistorted();
}

//#############################################################################
bool Kinect2::hasNewFrameRegistered() const
{
    return pImpl->hasNewFrameRegistered();
}

//#############################################################################
const tFramePtr& Kinect2::getFrameRegistered() const
{
    return pImpl->getFrameRegistered();
}

//#############################################################################
bool Kinect2::hasNewFrameIrUndistorted() const
{
    return pImpl->hasNewFrameIrUndistorted();
}

//#############################################################################
const tFramePtr& Kinect2::getFrameIrUndistorted()
{
    return pImpl->getFrameIrUndistorted();
}

//#############################################################################
bool Kinect2::hasNewFrameBigDepth() const
{
    return pImpl->hasNewFrameBigDepth();
}

//#############################################################################
const tFramePtr& Kinect2::getFrameBigDepth() const
{
    return pImpl->getFrameBigDepth();
}

//#############################################################################

}   // ns kinect2

//#############################################################################
