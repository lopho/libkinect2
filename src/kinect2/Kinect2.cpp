//# INCLUDES ##################################################################
#include <cstring>

#include <kinect2/Kinect2.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>

#include <iostream>

//# TYPEDEFS / CONSTANTS ######################################################


//# DECLARATION ###############################################################

namespace kinect2
{

//#############################################################################
//#############################################################################

namespace private_ns
{

class Kinect2ImplHolder
{
public:
    Kinect2ImplHolder() :
        pFreenect2(new libfreenect2::Freenect2),
        pDeviceConfig(new libfreenect2::Freenect2Device::Config),
        pDevice(NULL),
        pPipeline(NULL),
        pRegistration(NULL),
        pListenerSync(NULL),
        pListenerRgb(NULL),
        pListenerDepthIr(NULL),
        pFrameMapSync(new libfreenect2::FrameMap),
        pFrameMapRgb(new libfreenect2::FrameMap),
        pFrameMapDepthIr(new libfreenect2::FrameMap),
        pFrameRgbBuf(new libfreenect2::Frame(1920, 1080, 4)),
        pFrameRgb(new Frame(1920, 1080, 4, pFrameRgbBuf->data)),
        pFrameDepthBuf(new libfreenect2::Frame(512, 424, 4)),
        pFrameDepth(new Frame(512, 424, 4, pFrameDepthBuf->data)),
        pFrameIrBuf(new libfreenect2::Frame(512, 424, 4)),
        pFrameIr(new Frame(512, 424, 4, pFrameIrBuf->data)),
        pFrameUndistortedBuf(new libfreenect2::Frame(512, 424, 4)),
        pFrameUndistorted(new Frame(512, 424, 4, pFrameUndistortedBuf->data)),
        pFrameRegisteredBuf(new libfreenect2::Frame(512, 424, 4)),
        pFrameRegistered(new Frame(512, 424, 4, pFrameRegisteredBuf->data)),
        pFrameIrUndistortedBuf(new libfreenect2::Frame(512, 424, 4)),
        pFrameIrUndistorted(new Frame(512, 424, 4, pFrameIrUndistortedBuf->data)),
        pFrameBigDepthBuf(new libfreenect2::Frame(1920, 1082, 4)),
        pFrameBigDepth(new Frame(1920, 1080, 4,
                                 &pFrameBigDepthBuf->data[1920*4])),
        pDummyListener(new libfreenect2::SyncMultiFrameListener(0))
    {}
    virtual ~Kinect2ImplHolder()
    {}

public:
    const std::shared_ptr<libfreenect2::Freenect2> pFreenect2;
    const std::shared_ptr<libfreenect2::Freenect2Device::Config> pDeviceConfig;
    std::shared_ptr<libfreenect2::Freenect2Device> pDevice;
    libfreenect2::PacketPipeline* pPipeline;
    std::shared_ptr<libfreenect2::Registration> pRegistration;
    std::shared_ptr<libfreenect2::SyncMultiFrameListener> pListenerSync;
    std::shared_ptr<libfreenect2::SyncMultiFrameListener> pListenerRgb;
    std::shared_ptr<libfreenect2::SyncMultiFrameListener> pListenerDepthIr;
    const std::shared_ptr<libfreenect2::FrameMap> pFrameMapSync;
    const std::shared_ptr<libfreenect2::FrameMap> pFrameMapRgb;
    const std::shared_ptr<libfreenect2::FrameMap> pFrameMapDepthIr;
    const std::shared_ptr<libfreenect2::Frame> pFrameRgbBuf;
    const tFramePtr pFrameRgb;
    const std::shared_ptr<libfreenect2::Frame> pFrameDepthBuf;
    const tFramePtr pFrameDepth;
    const std::shared_ptr<libfreenect2::Frame> pFrameIrBuf;
    const tFramePtr pFrameIr;
    const std::shared_ptr<libfreenect2::Frame> pFrameUndistortedBuf;
    const tFramePtr pFrameUndistorted;
    const std::shared_ptr<libfreenect2::Frame> pFrameRegisteredBuf;
    const tFramePtr pFrameRegistered;
    const std::shared_ptr<libfreenect2::Frame> pFrameIrUndistortedBuf;
    const tFramePtr pFrameIrUndistorted;
    const std::shared_ptr<libfreenect2::Frame> pFrameBigDepthBuf;
    const tFramePtr pFrameBigDepth;

    const std::shared_ptr<libfreenect2::SyncMultiFrameListener> pDummyListener;
};

}   // private_ns

//#############################################################################
//#############################################################################
Frame::tPixelFormat convertPixelFormat(
        const libfreenect2::Frame::Format i_ePixelFormat)
{
    switch (i_ePixelFormat)
    {
        case libfreenect2::Frame::Invalid:
            return Frame::Invalid;
            break;
        case libfreenect2::Frame::Raw:
            return Frame::Raw;
            break;
        case libfreenect2::Frame::Float:
            return Frame::Float;
            break;
        case libfreenect2::Frame::BGRX:
            return Frame::BGRX;
            break;
        case libfreenect2::Frame::RGBX:
            return Frame::RGBX;
            break;
        case libfreenect2::Frame::Gray:
            return Frame::Gray;
            break;
    }
    return Frame::Invalid;
}

//#############################################################################
bool copyFrame(const libfreenect2::Frame* i_pFrame,
               Frame* o_pFrame, const Frame::tFrameType i_eType,
               const bool i_bCopyData = true)
{
    if (NULL == i_pFrame || NULL == o_pFrame)
    {
        return false;
    }
    // void* memcpy( void* dest, const void* src, std::size_t count );
    if (   (i_pFrame->width != o_pFrame->nWidth)
        || (i_pFrame->height != o_pFrame->nHeight)
        || (i_pFrame->bytes_per_pixel != o_pFrame->nBytesPerPixel))
    {
        if (Frame::tFrameType::BigDepth != i_eType)
        {
            return false;
        }
    }

    o_pFrame->nTimestamp = i_pFrame->timestamp;
    o_pFrame->nSequence = i_pFrame->sequence;
    o_pFrame->fExposure = i_pFrame->exposure;
    o_pFrame->fGain = i_pFrame->gain;
    o_pFrame->fGamma = i_pFrame->gamma;
    o_pFrame->nStatus = i_pFrame->status;
    o_pFrame->eFormat = convertPixelFormat(i_pFrame->format);
    o_pFrame->eType = i_eType;

    if (libfreenect2::Frame::Invalid == i_pFrame->format)
    {
        if (   Frame::tFrameType::Undistorted == i_eType
            || Frame::tFrameType::IrUndistorted == i_eType
            || Frame::tFrameType::BigDepth == i_eType)
        {
            o_pFrame->eFormat = Frame::Float;
        }
        else if (Frame::tFrameType::Registered == i_eType)
        {
            o_pFrame->eFormat = Frame::BGRX;
        }
    }

    if (i_bCopyData)
    {
        size_t nSize = 0;
        size_t nOffset = 0;
        if (libfreenect2::Frame::Raw == i_pFrame->format)
        {
            nSize = i_pFrame->bytes_per_pixel;
        }
        else if (Frame::tFrameType::BigDepth == i_eType)
        {
            nSize =   i_pFrame->bytes_per_pixel
                    * i_pFrame->width
                    * (i_pFrame->height-2);
            nOffset = i_pFrame->bytes_per_pixel * i_pFrame->width;
        }
        else
        {
            nSize =   i_pFrame->bytes_per_pixel
                    * i_pFrame->width
                    * i_pFrame->height;
        }
        std::memcpy(o_pFrame->pData, &i_pFrame->data[nOffset], nSize);
    }

    return true;
}

//#############################################################################

bool copyFrame(const libfreenect2::Frame* i_pFrame,
               libfreenect2::Frame* o_pFrame,
               const bool i_bCopyData = true)
{
    if (NULL == i_pFrame || NULL == o_pFrame)
    {
        return false;
    }
    // void* memcpy( void* dest, const void* src, std::size_t count );
    if (   (i_pFrame->width != o_pFrame->width)
        || (i_pFrame->height != o_pFrame->height)
        || (i_pFrame->bytes_per_pixel != o_pFrame->bytes_per_pixel))
    {
        return false;
    }

    o_pFrame->timestamp = i_pFrame->timestamp;
    o_pFrame->sequence = i_pFrame->sequence;
    o_pFrame->exposure = i_pFrame->exposure;
    o_pFrame->gain = i_pFrame->gain;
    o_pFrame->gamma = i_pFrame->gamma;
    o_pFrame->status = i_pFrame->status;
    o_pFrame->format = i_pFrame->format;

    if (i_bCopyData)
    {
        const size_t nSize = (i_pFrame->format == libfreenect2::Frame::Raw)
                        ? i_pFrame->bytes_per_pixel
                        :   i_pFrame->width
                          * i_pFrame->height
                          * i_pFrame->bytes_per_pixel;
        std::memcpy(o_pFrame->data, i_pFrame->data, nSize);
    }

    return true;
}

//#############################################################################

//#############################################################################
Kinect2::Kinect2() :
    ePipelineType(CPU),
    nDeviceId(-1),
    strSerial(""),
    bRgb(false),
    bDepth(false),
    bIr(false),
    bUndistort(false),
    bRegister(false),
    bIrUndistort(false),
    bBigDepth(false),
    bSync(false),
    bOpen(false),
    bRunning(false),
    bNewRgb(false),
    bNewDepth(false),
    bNewIr(false),
    bNewUndistorted(false),
    bNewRegistered(false),
    bNewIrUndistorted(false),
    bNewBigDepth(false),
    p(new private_ns::Kinect2ImplHolder)
{}

//#############################################################################
Kinect2::~Kinect2()
{
    if (isOpen())
    {
        if (isRunning())
        {
            stop();
        }
        close();
    }
    //p->pRegistration.reset();
    delete p;
}

//#############################################################################
void Kinect2::fetchFrames()
{
    bNewRgb = false;
    bNewDepth = false;
    bNewIr = false;
    bNewUndistorted = false;
    bNewRegistered = false;
    bNewIrUndistorted = false;
    bNewBigDepth = false;

    if (bSync)
    {
        if (bRgb || bDepth || bUndistort || bBigDepth || bIr || bIrUndistort || bRegister)
        {
            if (p->pListenerSync->hasNewFrame())
            {
                p->pListenerSync->release(*p->pFrameMapSync);
                p->pListenerSync->waitForNewFrame(*p->pFrameMapSync);
                copyFramesSync();
                applyUndistortionOrRegistration();
            }
        }
    }
    else
    {
        bool bNewFrame = false;
        if (bRgb || bRegister || bBigDepth)
        {
            if (p->pListenerRgb->hasNewFrame())
            {
                p->pListenerRgb->release(*p->pFrameMapRgb);
                p->pListenerRgb->waitForNewFrame(*p->pFrameMapRgb);
                copyFramesRgb();
                bNewFrame = true;
            }
        }
        if (bDepth || bUndistort || bBigDepth || bIr || bIrUndistort || bRegister)
        {
            if (p->pListenerDepthIr->hasNewFrame())
            {
                p->pListenerDepthIr->release(*p->pFrameMapDepthIr);
                p->pListenerDepthIr->waitForNewFrame(*p->pFrameMapDepthIr);
                copyFramesDepthIr();
                bNewFrame = true;
            }
        }
        if (bNewFrame)
        {
            applyUndistortionOrRegistration();
        }
    }
}

//#############################################################################
Kinect2::tReturnCode Kinect2::open()
{
    tReturnCode eRet = setup();
    if (SUCCESS != eRet)
    {
        return eRet;
    }
    if (isOpen())
    {
        return SUCCESS;
    }
    p->pDevice = std::shared_ptr<libfreenect2::Freenect2Device>(
                p->pFreenect2->openDevice(strSerial, p->pPipeline));
    p->pDevice->setConfiguration(*p->pDeviceConfig);
    if (NULL == p->pDevice)
    {
        return FAILURE_OPEN;
    }
    bOpen = true;
    return SUCCESS;
}

//#######################################Frame######################################
Kinect2::tReturnCode Kinect2::close()
{
    if (!isOpen())
    {
        return SUCCESS;
    }
    if (!p->pDevice->close())
    {
        return FAILURE_CLOSE;
    }
    cleanListeners();
    bOpen = false;
    return SUCCESS;
}


//#############################################################################
bool Kinect2::isOpen() const
{
    return bOpen;
}

//#############################################################################
Kinect2::tReturnCode Kinect2::start()
{
    if (isRunning())
    {
        return SUCCESS;
    }
    installListeners();
    if (!isOpen() || !p->pDevice->startStreams(true, true))
    {
        return FAILURE_START;
    }
    p->pRegistration = std::make_shared<libfreenect2::Registration>(
                           p->pDevice->getIrCameraParams(),
                           p->pDevice->getColorCameraParams());
    waitForListeners();
    bRunning = true;
    return SUCCESS;
}

//#############################################################################
Kinect2::tReturnCode Kinect2::stop()
{
    if (!isRunning())
    {
        return SUCCESS;
    }
    if (!p->pDevice->stop())
    {
        return FAILURE_STOP;
    }
    bRunning = false;
    return SUCCESS;
}

//#############################################################################
Kinect2::tReturnCode Kinect2::restart()
{
    tReturnCode eRet;
    eRet = stop();
    if (SUCCESS != eRet)
    {
        return eRet;
    }
    eRet = start();
    if (SUCCESS != eRet)
    {
        return eRet;
    }
    return SUCCESS;
}

//#############################################################################
bool Kinect2::isRunning() const
{
    return bRunning;
}

//#############################################################################
Kinect2::tReturnCode Kinect2::update()
{
    if (isOpen() && isRunning())
    {
        fetchFrames();
        return SUCCESS;
    }
    return NOT_RUNNING;
}

//#############################################################################
Kinect2::tReturnCode Kinect2::setPipelineType(
        const tPipelineType i_ePipelineType)
{
    if (i_ePipelineType != ePipelineType)
    {
        ePipelineType = i_ePipelineType;
        const tReturnCode eRet = reopen();
        if (SUCCESS != eRet)
        {
            return eRet;
        }
    }
    return SUCCESS;
}

//#############################################################################
Kinect2::tReturnCode Kinect2::setDeviceId(const int i_nDeviceId)
{
    if (i_nDeviceId != nDeviceId)
    {
        nDeviceId = i_nDeviceId;
        const tReturnCode eRet = reopen();
        if (SUCCESS != eRet)
        {
            return eRet;
        }
    }
    return SUCCESS;
}

//#############################################################################
void Kinect2::setMinDepth(const float i_fMinDepth)
{
    p->pDeviceConfig->MinDepth = i_fMinDepth;
    if (NULL != p->pDevice)
    {
        const bool bWasRunning = isRunning();
        if (bWasRunning)
        {
            stop();
        }
        p->pDevice->setConfiguration(*p->pDeviceConfig);
        if (bWasRunning)
        {
            start();
        }
    }
}

//#############################################################################
void Kinect2::setMaxDepth(const float i_fMaxDepth)
{
    p->pDeviceConfig->MaxDepth = i_fMaxDepth;
    if (NULL != p->pDevice)
    {
        const bool bWasRunning = isRunning();
        if (bWasRunning)
        {
            stop();
        }
        p->pDevice->setConfiguration(*p->pDeviceConfig);
        if (bWasRunning)
        {
            start();
        }
    }
}

//#############################################################################
void Kinect2::setBilateralFilter(const bool i_bBilateralFilter)
{
    p->pDeviceConfig->EnableBilateralFilter = i_bBilateralFilter;
    if (NULL != p->pDevice)
    {
        const bool bWasRunning = isRunning();
        if (bWasRunning)
        {
            stop();
        }
        p->pDevice->setConfiguration(*p->pDeviceConfig);
        if (bWasRunning)
        {
            start();
        }
    }
}

//#############################################################################
void Kinect2::setEdgeAwareFilter(const bool i_bEdgeAwareFilter)
{
    p->pDeviceConfig->EnableEdgeAwareFilter = i_bEdgeAwareFilter;
    if (NULL != p->pDevice)
    {
        const bool bWasRunning = isRunning();
        if (bWasRunning)
        {
            stop();
        }
        p->pDevice->setConfiguration(*p->pDeviceConfig);
        if (bWasRunning)
        {
            start();
        }
    }
}

//#############################################################################
float Kinect2::getMinDepth() const
{
    return p->pDeviceConfig->MinDepth;
}

//#############################################################################
float Kinect2::getMaxDepth() const
{
    return p->pDeviceConfig->MaxDepth;
}

//#############################################################################
bool Kinect2::isBilateralFilter() const
{
    return p->pDeviceConfig->EnableBilateralFilter;
}

//#############################################################################
bool Kinect2::isEdgeAwareFilter() const
{
    return p->pDeviceConfig->EnableEdgeAwareFilter;
}

//#############################################################################
void Kinect2::setRgb(const bool i_bRgb)
{
    if (i_bRgb != bRgb)
    {
        bRgb = i_bRgb;
        if (isOpen())
        {
            installListeners();
        }
    }
}

//#############################################################################
void Kinect2::setDepth(const bool i_bDepth)
{
    if (i_bDepth != bDepth)
    {
        bDepth = i_bDepth;
        if (isOpen())
        {
            installListeners();
        }
    }
}

//#############################################################################
void Kinect2::setIr(const bool i_bIr)
{
    if (i_bIr != bIr)
    {
        bIr = i_bIr;
        if (isOpen())
        {
            installListeners();
        }
    }
}

//#############################################################################
void Kinect2::setSync(const bool i_bSync)
{
    if (i_bSync != bSync)
    {
        bSync = i_bSync;
        if (isOpen())
        {
            installListeners();
        }
    }
}

//#############################################################################
void Kinect2::setUndistort(const bool i_bUndistort)
{
    if (i_bUndistort != bUndistort)
    {
        bUndistort = i_bUndistort;
        if (isOpen())
        {
            installListeners();
        }
    }
}

//#############################################################################
void Kinect2::setRegister(const bool i_bRegister)
{
    if (i_bRegister != bRegister)
    {
        bRegister = i_bRegister;
        if (isOpen())
        {
            installListeners();
        }
    }
}

//#############################################################################
void Kinect2::setIrUndistort(const bool i_bIrUndistort)
{
    if (i_bIrUndistort != bIrUndistort)
    {
        bIrUndistort = i_bIrUndistort;
        if (isOpen())
        {
            installListeners();
        }
    }

}

//#############################################################################
void Kinect2::setBigDepth(const bool i_bBigDepth)
{
    if (i_bBigDepth != bBigDepth)
    {
        bBigDepth = i_bBigDepth;
        if (isOpen())
        {
            installListeners();
        }
    }
}

//#############################################################################
int Kinect2::getDeviceId() const
{
    return nDeviceId;
}

//#############################################################################
const std::string& Kinect2::getDeviceSerial() const
{
    return strSerial;
}

//#############################################################################
Kinect2::tPipelineType Kinect2::getPipelineType() const
{
    return ePipelineType;
}

//#############################################################################
Kinect2::tColorCameraParams Kinect2::getColorCameraParameters() const
{
    if (NULL == p->pDevice)
    {
        return tColorCameraParams();
    }
    const libfreenect2::Freenect2Device::ColorCameraParams sParamsFN2 =
            p->pDevice->getColorCameraParams();
    tColorCameraParams sParams;
    sParams.cx = sParamsFN2.cx;
    sParams.cy = sParamsFN2.cy;
    sParams.fx = sParamsFN2.fx;
    sParams.fy = sParamsFN2.fy;
    sParams.mx_x0y0 = sParamsFN2.mx_x0y0;
    sParams.mx_x0y1 = sParamsFN2.mx_x0y1;
    sParams.mx_x0y2 = sParamsFN2.mx_x0y2;
    sParams.mx_x0y3 = sParamsFN2.mx_x0y3;
    sParams.mx_x1y0 = sParamsFN2.mx_x1y0;
    sParams.mx_x1y1 = sParamsFN2.mx_x1y1;
    sParams.mx_x1y2 = sParamsFN2.mx_x1y2;
    sParams.mx_x2y0 = sParamsFN2.mx_x2y0;
    sParams.mx_x2y1 = sParamsFN2.mx_x2y1;
    sParams.mx_x3y0 = sParamsFN2.mx_x3y0;
    sParams.shift_d = sParamsFN2.shift_d;
    sParams.shift_m = sParamsFN2.shift_m;
    return sParams;
}

//#############################################################################
Kinect2::tIrCameraParams Kinect2::getIrCameraParameters() const
{
    if (NULL == p->pDevice)
    {
        return tIrCameraParams();
    }
    const libfreenect2::Freenect2Device::IrCameraParams sParamsFN2 =
            p->pDevice->getIrCameraParams();
    tIrCameraParams sParams;
    sParams.cx = sParamsFN2.cx;
    sParams.cy = sParamsFN2.cy;
    sParams.fx = sParamsFN2.fx;
    sParams.fy = sParamsFN2.fy;
    sParams.k1 = sParamsFN2.k1;
    sParams.k2 = sParamsFN2.k2;
    sParams.k3 = sParamsFN2.k3;
    sParams.p1 = sParamsFN2.p1;
    sParams.p2 = sParamsFN2.p2;
    return sParams;
}

//#############################################################################
bool Kinect2::isSync() const
{
    return bSync;
}

//#############################################################################
bool Kinect2::isRgb() const
{
    return bRgb;
}

//#############################################################################
bool Kinect2::isDepth() const
{
    return bDepth;
}

//#############################################################################
bool Kinect2::isIr() const
{
    return bIr;
}

//#############################################################################
bool Kinect2::isUndistort() const
{
    return bUndistort;
}

//#############################################################################
bool Kinect2::isRegister() const
{
    return bRegister;
}

//#############################################################################
bool Kinect2::isIrUndistort() const
{
    return bIrUndistort;
}

//#############################################################################
bool Kinect2::isBigDepth() const
{
    return bBigDepth;
}

//#############################################################################
int Kinect2::hasNewFrame() const
{
    int nTypes = 0;
    if (bNewRgb)
    {
        nTypes |= Frame::Color;
    }
    if (bNewDepth)
    {
        nTypes |= Frame::Depth;
    }
    if (bNewIr)
    {
        nTypes |= Frame::Ir;
    }
    if (bNewUndistorted)
    {
        nTypes |= Frame::Undistorted;
    }
    if (bNewRegistered)
    {
        nTypes |= Frame::Registered;
    }
    if (bNewIrUndistorted)
    {
        nTypes |= Frame::IrUndistorted;
    }
    if (bNewBigDepth)
    {
        nTypes |= Frame::BigDepth;
    }
    return nTypes;
}

//#############################################################################
void Kinect2::getFrames(const int i_nTypes, tFrameMap* i_pFrameMap) const
{
    if (i_nTypes & Frame::Color)
    {
        tFramePtr pRgb(p->pFrameRgb);
        i_pFrameMap->insert(std::make_pair(Frame::Color, pRgb));
    }
    if (i_nTypes & Frame::Depth)
    {
        tFramePtr pDepth(p->pFrameDepth);
        i_pFrameMap->insert(std::make_pair(Frame::Depth, pDepth));
    }
    if (i_nTypes & Frame::Ir)
    {
        tFramePtr pIr(p->pFrameIr);
        i_pFrameMap->insert(std::make_pair(Frame::Ir, pIr));
    }
    if (i_nTypes & Frame::Undistorted)
    {
        tFramePtr pUndistorted(p->pFrameUndistorted);
        i_pFrameMap->insert(std::make_pair(Frame::Undistorted, pUndistorted));
    }
    if (i_nTypes & Frame::Registered)
    {
        tFramePtr pRegistered(p->pFrameRegistered);
        i_pFrameMap->insert(std::make_pair(Frame::Registered, pRegistered));
    }
    if (i_nTypes & Frame::IrUndistorted)
    {
        tFramePtr pIrUndistorted(p->pFrameIrUndistorted);
        i_pFrameMap->insert(std::make_pair(Frame::IrUndistorted,
                                           pIrUndistorted));
    }
    if (i_nTypes & Frame::BigDepth)
    {
        tFramePtr pBigDepth(p->pFrameBigDepth);
        i_pFrameMap->insert(std::make_pair(Frame::BigDepth, pBigDepth));
    }
}

//#############################################################################
bool Kinect2::hasNewFrameRgb() const
{
    return bNewRgb;
}

//#############################################################################
const tFramePtr& Kinect2::getFrameRgb() const
{
    return p->pFrameRgb;
}

//#############################################################################
bool Kinect2::hasNewFrameDepth() const
{
    return bNewDepth;
}

//#############################################################################
const tFramePtr& Kinect2::getFrameDepth() const
{
    return p->pFrameDepth;
}

//#############################################################################
bool Kinect2::hasNewFrameIr() const
{
    return bNewIr;
}

//#############################################################################
const tFramePtr& Kinect2::getFrameIr() const
{
    return p->pFrameIr;
}

//#############################################################################
bool Kinect2::hasNewFrameUndistorted() const
{
    return bNewUndistorted;
}

//#############################################################################
const tFramePtr& Kinect2::getFrameUndistorted() const
{
    return p->pFrameUndistorted;
}

//#############################################################################
bool Kinect2::hasNewFrameRegistered() const
{
    return bNewRegistered;
}

//#############################################################################
const tFramePtr& Kinect2::getFrameRegistered() const
{
    return p->pFrameRegistered;
}

//#############################################################################
bool Kinect2::hasNewFrameIrUndistorted() const
{
    return bNewIrUndistorted;
}

//#############################################################################
const tFramePtr& Kinect2::getFrameIrUndistorted() const
{
    return p->pFrameIrUndistorted;
}

//#############################################################################
bool Kinect2::hasNewFrameBigDepth() const
{
    return bNewBigDepth;
}

//#############################################################################
const tFramePtr& Kinect2::getFrameBigDepth() const
{
    return p->pFrameBigDepth;
}

//#############################################################################
Kinect2::tReturnCode Kinect2::setup()
{
    const int nNumDevices = p->pFreenect2->enumerateDevices();
    if (0 == nNumDevices)
    {
        return NO_DEVICES_CONNECTED;
    }
    if (nDeviceId >= nNumDevices)
    {
        return UNKNOWN_DEVICE_ID;
    }

    if (-1 == nDeviceId)
    {
        strSerial = p->pFreenect2->getDefaultDeviceSerialNumber();
    }
    else
    {
        strSerial = p->pFreenect2->getDeviceSerialNumber(nDeviceId);
    }

    switch (ePipelineType)
    {
        case CPU:
            p->pPipeline = new libfreenect2::CpuPacketPipeline;
            break;
        case GL:
            p->pPipeline = new libfreenect2::OpenGLPacketPipeline;
            break;
        case CL:
            p->pPipeline = new libfreenect2::OpenCLPacketPipeline(nDeviceId);
            break;
        case CLKDE:
            p->pPipeline = new libfreenect2::OpenCLKdePacketPipeline(nDeviceId);
            break;
        case CUDA:
            p->pPipeline = new libfreenect2::CudaPacketPipeline(nDeviceId);
            break;
        case CUDAKDE:
            p->pPipeline = new libfreenect2::CudaKdePacketPipeline(nDeviceId);
            break;
    }


    return SUCCESS;
}


//#############################################################################
Kinect2::tReturnCode Kinect2::reopen()
{
    bool bWasOpen = false;
    bool bWasRunning = false;
    tReturnCode eRet = SUCCESS;
    if (isOpen())
    {
        if (isRunning())
        {
            eRet = stop();
            if (SUCCESS != eRet)
            {
                return eRet;
            }
            bWasRunning = true;
        }
        eRet = close();
        if (SUCCESS != eRet)
        {
            return eRet;
        }
        bWasOpen = true;
    }
    if (bWasOpen)
    {
        eRet = setup();
        if (SUCCESS != eRet)
        {
            return eRet;
        }
        eRet = open();
        if (SUCCESS != eRet)
        {
            return eRet;
        }
        if (bWasRunning)
        {
            eRet = start();
            if (SUCCESS != eRet)
            {
                return eRet;
            }
        }
    }
    return SUCCESS;
}

//#############################################################################
void Kinect2::installListeners()
{
    cleanListeners();
    if (isSync())
    {
        int nTypes = 0;
        if (bRgb || bRegister || bBigDepth)
        {
            nTypes |= libfreenect2::Frame::Color;
        }
        if (bDepth || bUndistort || bRegister || bBigDepth)
        {
            nTypes |= libfreenect2::Frame::Depth;
        }
        if (bIr || bIrUndistort)
        {
            nTypes |= libfreenect2::Frame::Ir;
        }
        p->pListenerSync =
                std::make_shared<libfreenect2::SyncMultiFrameListener>(nTypes);

        p->pDevice->setColorFrameListener(p->pListenerSync.get());

        p->pDevice->setIrAndDepthFrameListener(p->pListenerSync.get());
    }
    else
    {
        if (bRgb || bRegister || bBigDepth)
        {
            p->pListenerRgb =
                    std::make_shared<libfreenect2::SyncMultiFrameListener>(
                        libfreenect2::Frame::Color);
            p->pDevice->setColorFrameListener(p->pListenerRgb.get());
        }
        if (bDepth || bUndistort || bRegister || bBigDepth || bIr || bIrUndistort)
        {
            int nTypes = 0;
            if (bDepth || bUndistort || bRegister || bBigDepth)
            {
                nTypes |= libfreenect2::Frame::Depth;
            }
            if (bIr || bIrUndistort)
            {
                nTypes |= libfreenect2::Frame::Ir;
            }
            p->pListenerDepthIr =
                    std::make_shared<libfreenect2::SyncMultiFrameListener>(
                        nTypes);
            p->pDevice->setIrAndDepthFrameListener(p->pListenerDepthIr.get());
        }
    }
}

//#############################################################################
void Kinect2::cleanListeners()
{
    p->pDevice->setColorFrameListener(p->pDummyListener.get());
    p->pDevice->setIrAndDepthFrameListener(p->pDummyListener.get());
    p->pListenerSync.reset();
    p->pListenerRgb.reset();
    p->pListenerDepthIr.reset();
}

//#############################################################################
void Kinect2::waitForListeners()
{
    if (isSync())
    {
        if (bRgb || bDepth || bUndistort || bBigDepth || bIr || bIrUndistort || bRegister)
        {
            p->pListenerSync->release(*p->pFrameMapSync);
            p->pListenerSync->waitForNewFrame(*p->pFrameMapSync);
            copyFramesSync();
        }
    }
    else
    {
        if (bRgb || bRegister || bBigDepth)
        {
            p->pListenerRgb->release(*p->pFrameMapRgb);
            p->pListenerRgb->waitForNewFrame(*p->pFrameMapRgb);
            copyFramesRgb();
        }
        if (bDepth || bUndistort || bBigDepth || bIr || bIrUndistort || bRegister)
        {
            p->pListenerDepthIr->release(*p->pFrameMapDepthIr);
            p->pListenerDepthIr->waitForNewFrame(*p->pFrameMapDepthIr);
            copyFramesDepthIr();
        }
    }
    applyUndistortionOrRegistration();
}

//#############################################################################
void Kinect2::copyFramesSync()
{
    bool bRetRgb = false;
    bool bRetDepth = false;
    bool bRetIr = false;

    if (bRgb || bRegister || bBigDepth)
    {
        const libfreenect2::Frame* pRgb =
                (*p->pFrameMapSync)[libfreenect2::Frame::Color];
        bRetRgb = copyFrame(pRgb, p->pFrameRgbBuf.get(), true);
        bRetRgb = copyFrame(p->pFrameRgbBuf.get(), p->pFrameRgb.get(),
                            Frame::Color, false) && bRetRgb;
    }
    if (bDepth || bUndistort || bRegister || bBigDepth)
    {
        const libfreenect2::Frame* pDepth =
                (*p->pFrameMapSync)[libfreenect2::Frame::Depth];
        bRetDepth = copyFrame(pDepth, p->pFrameDepthBuf.get(), true);
        bRetDepth = copyFrame(p->pFrameDepthBuf.get(), p->pFrameDepth.get(),
                              Frame::Depth, false) && bRetDepth;
    }
    if (bIr || bIrUndistort)
    {
        const libfreenect2::Frame* pIr =
                (*p->pFrameMapSync)[libfreenect2::Frame::Ir];
        bRetIr = copyFrame(pIr, p->pFrameIrBuf.get(), true);
        bRetIr = copyFrame(p->pFrameIrBuf.get(), p->pFrameIr.get(),
                           Frame::Ir, false) && bRetIr;
    }
    p->pListenerSync->release(*p->pFrameMapSync);
    if (bRetRgb && bRgb)
    {
        bNewRgb = true;
    }
    if (bRetDepth && bDepth)
    {
        bNewDepth = true;
    }
    if (bRetIr && bIr)
    {
        bNewIr = true;
    }
}

//#############################################################################
void Kinect2::copyFramesRgb()
{
    bool bRetRgb = false;

    if (bRgb || bRegister || bBigDepth)
    {
        bRetRgb = copyFrame((*p->pFrameMapRgb)[libfreenect2::Frame::Color],
                            p->pFrameRgbBuf.get(), true);
        bRetRgb = copyFrame(p->pFrameRgbBuf.get(), p->pFrameRgb.get(),
                            Frame::Color, false) && bRetRgb;
    }
    p->pListenerRgb->release(*p->pFrameMapRgb);
    if (bRetRgb && bRgb)
    {
        bNewRgb = true;
    }

}

//#############################################################################
void Kinect2::copyFramesDepthIr()
{
    bool bRetDepth = false;
    bool bRetIr = false;
    if (bDepth || bUndistort || bRegister || bBigDepth)
    {
        bRetDepth = copyFrame(
                (*p->pFrameMapDepthIr)[libfreenect2::Frame::Depth],
                p->pFrameDepthBuf.get(), true);
        bRetDepth = copyFrame(p->pFrameDepthBuf.get(), p->pFrameDepth.get(),
                              Frame::Depth, false) && bRetDepth;
    }
    if (bIr || bIrUndistort)
    {
        bRetIr = copyFrame((*p->pFrameMapDepthIr)[libfreenect2::Frame::Ir],
                           p->pFrameIrBuf.get(), true);
        bRetIr = copyFrame(p->pFrameIrBuf.get(), p->pFrameIr.get(),
                           Frame::Ir, false) && bRetIr;
    }
    p->pListenerDepthIr->release(*p->pFrameMapDepthIr);
    if (bRetDepth && bDepth)
    {
        bNewDepth = true;
    }
    if (bRetIr && bIr)
    {
        bNewIr = true;
    }
}

//#############################################################################
void Kinect2::applyRegistration()
{
    bool bRetUndistort = false;
    bool bRetRegister = false;
    bool bRetBigDepth = false;

    if (bRegister || bBigDepth || bRegister)
    {
        libfreenect2::Frame* pBigDepth = (bBigDepth)
                                         ? p->pFrameBigDepthBuf.get()
                                         : NULL;
        p->pRegistration->apply(p->pFrameRgbBuf.get(),
                                p->pFrameDepthBuf.get(),
                                p->pFrameUndistortedBuf.get(),
                                p->pFrameRegisteredBuf.get(),
                                true,
                                pBigDepth);
        if (bUndistort)
        {
            bRetUndistort = copyFrame(p->pFrameUndistortedBuf.get(),
                                      p->pFrameUndistorted.get(),
                                      Frame::Undistorted, false);
        }
        if (bRegister)
        {
            bRetRegister = copyFrame(p->pFrameRegisteredBuf.get(),
                                     p->pFrameRegistered.get(),
                                     Frame::Registered, false);
        }
        if (bBigDepth)
        {
            bRetBigDepth = copyFrame(p->pFrameBigDepthBuf.get(),
                                     p->pFrameBigDepth.get(),
                                     Frame::BigDepth, false);
        }
    }

    if (bRetBigDepth && bBigDepth)
    {
        bNewBigDepth = true;
    }
    if (bRetUndistort && bUndistort)
    {
        bNewUndistorted = true;
    }
    if (bRetRegister && bRetRegister)
    {
        bNewRegistered = true;
    }
}

//#############################################################################
void Kinect2::applyUndistortion()
{
    bool bRetUndistort = false;
    if (bUndistort)
    {
        p->pRegistration->undistortDepth(p->pFrameDepthBuf.get(),
                                         p->pFrameUndistortedBuf.get());
        bRetUndistort = copyFrame(p->pFrameUndistortedBuf.get(),
                                  p->pFrameUndistorted.get(),
                                  Frame::Undistorted, false);
    }
    if (bRetUndistort && bUndistort)
    {
        bNewUndistorted = true;
    }
}

//#############################################################################
void Kinect2::applyUndistortionOrRegistration()
{
    if (bRegister || bBigDepth)
    {
        applyRegistration();
    }
    else if (bUndistort)
    {
        applyUndistortion();
    }
    if (bIrUndistort)
    {
        applyIrUndistortion();
    }
}

//#############################################################################
void Kinect2::applyIrUndistortion()
{
    bool bRetIrUndistort = false;
    if (bIrUndistort)
    {
        p->pRegistration->undistortDepth(p->pFrameIrBuf.get(),
                                         p->pFrameIrUndistortedBuf.get());
        bRetIrUndistort = copyFrame(p->pFrameIrUndistortedBuf.get(),
                                    p->pFrameIrUndistorted.get(),
                                    Frame::IrUndistorted, false);
    }
    if (bRetIrUndistort && bIrUndistort)
    {
        bNewIrUndistorted = true;
    }
}

//#############################################################################

//#############################################################################

}   // ns kinect2

//#############################################################################

