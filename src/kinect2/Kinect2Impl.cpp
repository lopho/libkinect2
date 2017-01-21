//# INCLUDES ##################################################################
#include <cstring>

#include <kinect2/Kinect2Impl.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>

#include <iostream>

//# TYPEDEFS / CONSTANTS ######################################################


//# DECLARATION ###############################################################

namespace kinect2
{
namespace private_ns
{

//#############################################################################
//#############################################################################

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
        pDummyListener(new libfreenect2::SyncMultiFrameListener(
                             libfreenect2::Frame::Color
                           | libfreenect2::Frame::Depth
                           | libfreenect2::Frame::Ir))
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
    const std::shared_ptr<Frame> pFrameRgb;
    const std::shared_ptr<libfreenect2::Frame> pFrameDepthBuf;
    const std::shared_ptr<Frame> pFrameDepth;
    const std::shared_ptr<libfreenect2::Frame> pFrameIrBuf;
    const std::shared_ptr<Frame> pFrameIr;
    const std::shared_ptr<libfreenect2::Frame> pFrameUndistortedBuf;
    const std::shared_ptr<Frame> pFrameUndistorted;
    const std::shared_ptr<libfreenect2::Frame> pFrameRegisteredBuf;
    const std::shared_ptr<Frame> pFrameRegistered;
    const std::shared_ptr<libfreenect2::Frame> pFrameIrUndistortedBuf;
    const std::shared_ptr<Frame> pFrameIrUndistorted;
    const std::shared_ptr<libfreenect2::Frame> pFrameBigDepthBuf;
    const std::shared_ptr<Frame> pFrameBigDepth;

    const std::shared_ptr<libfreenect2::SyncMultiFrameListener> pDummyListener;
};

//#############################################################################
//#############################################################################
kinect2::Frame::tPixelFormat convertPixelFormat(
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
Kinect2Impl::Kinect2Impl() :
    ePipelineType(Kinect2::CPU),
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
    p(new Kinect2ImplHolder)
{}

//#############################################################################
Kinect2Impl::~Kinect2Impl()
{
    if (isOpen())
    {
        if (isRunning())
        {
            stop();
        }
        close();
    }
    p->pRegistration.reset();
    delete p;
}

//#############################################################################
void Kinect2Impl::fetchFrames()
{
    bNewRgb = false;
    bNewDepth = false;
    bNewIr = false;
    bNewUndistorted = false;
    bNewRegistered = false;
    if (bSync)
    {
        if (p->pListenerSync->hasNewFrame())
        {
            p->pListenerSync->release(*p->pFrameMapSync);
            p->pListenerSync->waitForNewFrame(*p->pFrameMapSync);
            copyFramesSync();
            applyUndistortionOrRegistration();
        }
    }
    else
    {
        bool bNewFrame = false;
        if (bRgb)
        {
            if (p->pListenerRgb->hasNewFrame())
            {
                p->pListenerRgb->release(*p->pFrameMapRgb);
                p->pListenerRgb->waitForNewFrame(*p->pFrameMapRgb);
                copyFramesRgb();
                bNewFrame = true;
            }
        }
        if (bDepth || bIr)
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
Kinect2::tReturnCode Kinect2Impl::open()
{
    Kinect2::tReturnCode eRet = setup();
    if (Kinect2::SUCCESS != eRet)
    {
        return eRet;
    }
    if (isOpen())
    {
        return Kinect2::SUCCESS;
    }
    p->pDevice = std::shared_ptr<libfreenect2::Freenect2Device>(
                p->pFreenect2->openDevice(strSerial, p->pPipeline));
    p->pDevice->setConfiguration(*p->pDeviceConfig);
    if (NULL == p->pDevice)
    {
        return Kinect2::FAILURE_OPEN;
    }
    bOpen = true;
    return Kinect2::SUCCESS;
}

//#######################################Frame######################################
Kinect2::tReturnCode Kinect2Impl::close()
{
    if (!isOpen())
    {
        return Kinect2::SUCCESS;
    }
    if (!p->pDevice->close())
    {
        return Kinect2::FAILURE_CLOSE;
    }
    cleanListeners();
    bOpen = false;
    return Kinect2::SUCCESS;
}


//#############################################################################
bool Kinect2Impl::isOpen() const
{
    return bOpen;
}

//#############################################################################
Kinect2::tReturnCode Kinect2Impl::start()
{
    if (isRunning())
    {
        return Kinect2::SUCCESS;
    }
    installListeners();
    if (!isOpen() || !p->pDevice->startStreams(true, true))
    {
        return Kinect2::FAILURE_START;
    }
    p->pRegistration = std::make_shared<libfreenect2::Registration>(
                           p->pDevice->getIrCameraParams(),
                           p->pDevice->getColorCameraParams());
    waitForListeners();
    bRunning = true;
    return Kinect2::SUCCESS;
}

//#############################################################################
Kinect2::tReturnCode Kinect2Impl::stop()
{
    if (!isRunning())
    {
        return Kinect2::SUCCESS;
    }
    if (!p->pDevice->stop())
    {
        return Kinect2::FAILURE_STOP;
    }
    bRunning = false;
    return Kinect2::SUCCESS;
}

//#############################################################################
Kinect2::tReturnCode Kinect2Impl::restart()
{
    Kinect2::tReturnCode eRet;
    eRet = stop();
    if (Kinect2::SUCCESS != eRet)
    {
        return eRet;
    }
    eRet = start();
    if (Kinect2::SUCCESS != eRet)
    {
        return eRet;
    }
    return Kinect2::SUCCESS;
}

//#############################################################################
bool Kinect2Impl::isRunning() const
{
    return bRunning;
}

//#############################################################################
Kinect2::tReturnCode Kinect2Impl::update()
{
    if (isOpen() && isRunning())
    {
        fetchFrames();
        return Kinect2::SUCCESS;
    }
    return Kinect2::NOT_RUNNING;
}

//#############################################################################
Kinect2::tReturnCode Kinect2Impl::setPipelineType(
        const Kinect2::tPipelineType i_ePipelineType)
{
    if (i_ePipelineType != ePipelineType)
    {
        ePipelineType = i_ePipelineType;
        Kinect2::tReturnCode eRet = reopen();
        if (Kinect2::SUCCESS != eRet)
        {
            return eRet;
        }
    }
    return Kinect2::SUCCESS;
}

//#############################################################################
Kinect2::tReturnCode Kinect2Impl::setDeviceId(const int i_nDeviceId)
{
    if (i_nDeviceId != nDeviceId)
    {
        nDeviceId = i_nDeviceId;
        Kinect2::tReturnCode eRet = reopen();
        if (Kinect2::SUCCESS != eRet)
        {
            return eRet;
        }
    }
    return Kinect2::SUCCESS;
}

//#############################################################################
void Kinect2Impl::setMinDepth(const float i_fMinDepth)
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
void Kinect2Impl::setMaxDepth(const float i_fMaxDepth)
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
void Kinect2Impl::setBilateralFilter(const bool i_bBilateralFilter)
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
void Kinect2Impl::setEdgeAwareFilter(const bool i_bEdgeAwareFilter)
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
float Kinect2Impl::getMinDepth() const
{
    return p->pDeviceConfig->MinDepth;
}

//#############################################################################
float Kinect2Impl::getMaxDepth() const
{
    return p->pDeviceConfig->MaxDepth;
}

//#############################################################################
bool Kinect2Impl::isBilateralFilter() const
{
    return p->pDeviceConfig->EnableBilateralFilter;
}

//#############################################################################
bool Kinect2Impl::isEdgeAwareFilter() const
{
    return p->pDeviceConfig->EnableEdgeAwareFilter;
}

//#############################################################################
void Kinect2Impl::setRgb(const bool i_bRgb)
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
void Kinect2Impl::setDepth(const bool i_bDepth)
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
void Kinect2Impl::setIr(const bool i_bIr)
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
void Kinect2Impl::setSync(const bool i_bSync)
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
void Kinect2Impl::setUndistort(const bool i_bUndistort)
{
    bUndistort = i_bUndistort;
}

//#############################################################################
void Kinect2Impl::setRegister(const bool i_bRegister)
{
    bRegister = i_bRegister;
}

//#############################################################################
void Kinect2Impl::setIrUndistort(const bool i_bIrUndistort)
{
    bIrUndistort = i_bIrUndistort;
}

//#############################################################################
void Kinect2Impl::setBigDepth(const bool i_bBigDepth)
{
    bBigDepth = i_bBigDepth;
}

//#############################################################################
int Kinect2Impl::getDeviceId() const
{
    return nDeviceId;
}

//#############################################################################
const std::string& Kinect2Impl::getDeviceSerial() const
{
    return strSerial;
}

//#############################################################################
Kinect2::tPipelineType Kinect2Impl::getPipelineType() const
{
    return ePipelineType;
}

//#############################################################################
Kinect2::tColorCameraParams Kinect2Impl::getColorCameraParameters() const
{
    if (NULL == p->pDevice)
    {
        return Kinect2::tColorCameraParams();
    }
    const libfreenect2::Freenect2Device::ColorCameraParams sParamsFN2 =
            p->pDevice->getColorCameraParams();
    Kinect2::tColorCameraParams sParams;
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
Kinect2::tIrCameraParams Kinect2Impl::getIrCameraParameters() const
{
    if (NULL == p->pDevice)
    {
        return Kinect2::tIrCameraParams();
    }
    const libfreenect2::Freenect2Device::IrCameraParams sParamsFN2 =
            p->pDevice->getIrCameraParams();
    Kinect2::tIrCameraParams sParams;
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
bool Kinect2Impl::isSync() const
{
    return bSync;
}

//#############################################################################
bool Kinect2Impl::isRgb() const
{
    return bRgb;
}

//#############################################################################
bool Kinect2Impl::isDepth() const
{
    return bDepth;
}

//#############################################################################
bool Kinect2Impl::isIr() const
{
    return bIr;
}

//#############################################################################
bool Kinect2Impl::isUndistort() const
{
    return bUndistort;
}

//#############################################################################
bool Kinect2Impl::isRegister() const
{
    return bRegister;
}

//#############################################################################
bool Kinect2Impl::isIrUndistort() const
{
    return bIrUndistort;
}

//#############################################################################
bool Kinect2Impl::isBigDepth() const
{
    return bBigDepth;
}

//#############################################################################
int Kinect2Impl::hasNewFrame() const
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
void Kinect2Impl::getFrames(const int i_nTypes, tFrameMap* i_pFrameMap) const
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
bool Kinect2Impl::hasNewFrameRgb() const
{
    return bNewRgb;
}

//#############################################################################
const std::shared_ptr<Frame>& Kinect2Impl::getFrameRgb() const
{
    return p->pFrameRgb;
}

//#############################################################################
bool Kinect2Impl::hasNewFrameDepth() const
{
    return bNewDepth;
}

//#############################################################################
const std::shared_ptr<Frame>& Kinect2Impl::getFrameDepth() const
{
    return p->pFrameDepth;
}

//#############################################################################
bool Kinect2Impl::hasNewFrameIr() const
{
    return bNewIr;
}

//#############################################################################
const std::shared_ptr<Frame>& Kinect2Impl::getFrameIr() const
{
    return p->pFrameIr;
}

//#############################################################################
bool Kinect2Impl::hasNewFrameUndistorted() const
{
    return bNewUndistorted;
}

//#############################################################################
const std::shared_ptr<Frame>& Kinect2Impl::getFrameUndistorted() const
{
    return p->pFrameUndistorted;
}

//#############################################################################
bool Kinect2Impl::hasNewFrameRegistered() const
{
    return bNewRegistered;
}

//#############################################################################
const std::shared_ptr<Frame>& Kinect2Impl::getFrameRegistered() const
{
    return p->pFrameRegistered;
}

//#############################################################################
bool Kinect2Impl::hasNewFrameIrUndistorted() const
{
    return bNewIrUndistorted;
}

//#############################################################################
const std::shared_ptr<Frame>& Kinect2Impl::getFrameIrUndistorted() const
{
    return p->pFrameIrUndistorted;
}

//#############################################################################
bool Kinect2Impl::hasNewFrameBigDepth() const
{
    return bNewBigDepth;
}

//#############################################################################
const std::shared_ptr<Frame>& Kinect2Impl::getFrameBigDepth() const
{
    return p->pFrameBigDepth;
}

//#############################################################################
Kinect2::tReturnCode Kinect2Impl::setup()
{
    const int nNumDevices = p->pFreenect2->enumerateDevices();
    if (0 == nNumDevices)
    {
        return Kinect2::NO_DEVICES_CONNECTED;
    }
    if (nDeviceId >= nNumDevices)
    {
        return Kinect2::UNKNOWN_DEVICE_ID;
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
        case kinect2::Kinect2::CPU:
            p->pPipeline = new libfreenect2::CpuPacketPipeline;
            break;
        case kinect2::Kinect2::GL:
            p->pPipeline = new libfreenect2::OpenGLPacketPipeline;
            break;
        case kinect2::Kinect2::CL:
            p->pPipeline = new libfreenect2::OpenCLPacketPipeline(nDeviceId);
            break;
        case kinect2::Kinect2::CLKDE:
            p->pPipeline = new libfreenect2::OpenCLKdePacketPipeline(nDeviceId);
            break;
        case kinect2::Kinect2::CUDA:
            p->pPipeline = new libfreenect2::CudaPacketPipeline(nDeviceId);
            break;
        case kinect2::Kinect2::CUDAKDE:
            p->pPipeline = new libfreenect2::CudaKdePacketPipeline(nDeviceId);
            break;
    }


    return Kinect2::SUCCESS;
}


//#############################################################################
Kinect2::tReturnCode Kinect2Impl::reopen()
{
    bool bWasOpen = false;
    bool bWasRunning = false;
    Kinect2::tReturnCode eRet = Kinect2::SUCCESS;
    if (isOpen())
    {
        if (isRunning())
        {
            eRet = stop();
            if (Kinect2::SUCCESS != eRet)
            {
                return eRet;
            }
            bWasRunning = true;
        }
        eRet = close();
        if (Kinect2::SUCCESS != eRet)
        {
            return eRet;
        }
        bWasOpen = true;
    }
    if (bWasOpen)
    {
        eRet = setup();
        if (Kinect2::SUCCESS != eRet)
        {
            return eRet;
        }
        eRet = open();
        if (Kinect2::SUCCESS != eRet)
        {
            return eRet;
        }
        if (bWasRunning)
        {
            eRet = start();
            if (Kinect2::SUCCESS != eRet)
            {
                return eRet;
            }
        }
    }
    return Kinect2::SUCCESS;
}

//#############################################################################
void Kinect2Impl::installListeners()
{
    cleanListeners();
    if (isSync())
    {
        int nTypes = 0;
        if (bRgb)
        {
            nTypes |= libfreenect2::Frame::Color;
        }
        if (bDepth)
        {
            nTypes |= libfreenect2::Frame::Depth;
        }
        if (bIr)
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
        if (bRgb)
        {
            p->pListenerRgb =
                    std::make_shared<libfreenect2::SyncMultiFrameListener>(
                        libfreenect2::Frame::Color);
            p->pDevice->setColorFrameListener(p->pListenerRgb.get());
        }
        if (bDepth || bIr)
        {
            int nTypes = 0;
            if (bDepth)
            {
                nTypes |= libfreenect2::Frame::Depth;
            }
            if (bIr)
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
void Kinect2Impl::cleanListeners()
{
    p->pDevice->setColorFrameListener(p->pDummyListener.get());
    p->pDevice->setIrAndDepthFrameListener(p->pDummyListener.get());
    p->pListenerSync.reset();
    p->pListenerRgb.reset();
    p->pListenerDepthIr.reset();
}

//#############################################################################
void Kinect2Impl::waitForListeners()
{
    if (isSync())
    {
        p->pListenerSync->release(*p->pFrameMapSync);
        p->pListenerSync->waitForNewFrame(*p->pFrameMapSync);
        copyFramesSync();
    }
    else
    {
        if (bRgb)
        {
            p->pListenerRgb->release(*p->pFrameMapRgb);
            p->pListenerRgb->waitForNewFrame(*p->pFrameMapRgb);
            copyFramesRgb();
        }
        if (bDepth || bIr)
        {
            p->pListenerDepthIr->release(*p->pFrameMapDepthIr);
            p->pListenerDepthIr->waitForNewFrame(*p->pFrameMapDepthIr);
            copyFramesDepthIr();
        }
    }
    applyUndistortionOrRegistration();
}

//#############################################################################
void Kinect2Impl::copyFramesSync()
{
    const libfreenect2::Frame* pRgb =
            (*p->pFrameMapSync)[libfreenect2::Frame::Color];
    const libfreenect2::Frame* pDepth =
            (*p->pFrameMapSync)[libfreenect2::Frame::Depth];
    const libfreenect2::Frame* pIr =
            (*p->pFrameMapSync)[libfreenect2::Frame::Ir];
    bool bRetRgb = false;
    if (bRgb)
    {
        bRetRgb = copyFrame(pRgb, p->pFrameRgbBuf.get(), true);
        bRetRgb = copyFrame(p->pFrameRgbBuf.get(), p->pFrameRgb.get(),
                            Frame::Color, false) && bRetRgb;
    }
    bool bRetDepth = false;
    if (bDepth)
    {
        bRetDepth = copyFrame(pDepth, p->pFrameDepthBuf.get(), true);
        bRetDepth = copyFrame(p->pFrameDepthBuf.get(), p->pFrameDepth.get(),
                              Frame::Depth, false) && bRetDepth;
    }
    bool bRetIr = false;
    if (bIr)
    {
        bRetIr = copyFrame(pIr, p->pFrameIrBuf.get(), true);
        bRetIr = copyFrame(p->pFrameIrBuf.get(), p->pFrameIr.get(),
                           Frame::Ir, false) && bRetIr;
    }
    p->pListenerSync->release(*p->pFrameMapSync);
    if (bRetRgb)
    {
        bNewRgb = true;
    }
    if (bRetDepth)
    {
        bNewDepth = true;
    }
    if (bRetIr)
    {
        bNewIr = true;
    }
}

//#############################################################################
void Kinect2Impl::copyFramesRgb()
{
    bool bRetRgb = copyFrame(
            (*p->pFrameMapRgb)[libfreenect2::Frame::Color],
            p->pFrameRgbBuf.get(), true);
    bRetRgb = copyFrame(p->pFrameRgbBuf.get(), p->pFrameRgb.get(),
                        Frame::Color, false) && bRetRgb;

    p->pListenerRgb->release(*p->pFrameMapRgb);
    if (bRetRgb)
    {
        bNewRgb = true;
    }

}

//#############################################################################
void Kinect2Impl::copyFramesDepthIr()
{
    bool bRetDepth = copyFrame(
            (*p->pFrameMapDepthIr)[libfreenect2::Frame::Depth],
            p->pFrameDepthBuf.get(), true);
    bRetDepth = copyFrame(p->pFrameDepthBuf.get(), p->pFrameDepth.get(),
                          Frame::Depth, false) && bRetDepth;

    bool bRetIr = copyFrame(
            (*p->pFrameMapDepthIr)[libfreenect2::Frame::Ir],
            p->pFrameIrBuf.get(), true);
    bRetIr = copyFrame(p->pFrameIrBuf.get(), p->pFrameIr.get(),
                       Frame::Ir, false) && bRetIr;

    p->pListenerDepthIr->release(*p->pFrameMapDepthIr);
    if (bRetDepth)
    {
        bNewDepth = true;
    }
    if (bRetIr)
    {
        bNewIr = true;
    }
}

//#############################################################################
void Kinect2Impl::applyRegistration()
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
    const bool bRetUndistort = copyFrame(p->pFrameUndistortedBuf.get(),
                                         p->pFrameUndistorted.get(),
                                         Frame::Undistorted, false);
    const bool bRetRegister = copyFrame(p->pFrameRegisteredBuf.get(),
                                        p->pFrameRegistered.get(),
                                        Frame::Registered, false);
    if (bBigDepth)
    {
        const bool bRetBigDepth = copyFrame(p->pFrameBigDepthBuf.get(),
                                            p->pFrameBigDepth.get(),
                                            Frame::BigDepth, false);
        if (bRetBigDepth)
        {
            bNewBigDepth = true;
        }
    }
    if (bRetUndistort)
    {
        bNewUndistorted = true;
    }
    if (bRetRegister)
    {
        bNewRegistered = true;
    }
}

//#############################################################################
void Kinect2Impl::applyUndistortion()
{
    p->pRegistration->undistortDepth(p->pFrameDepthBuf.get(),
                                     p->pFrameUndistortedBuf.get());
    const bool bRetUndistort = copyFrame(p->pFrameUndistortedBuf.get(),
                                         p->pFrameUndistorted.get(),
                                         Frame::Undistorted, false);
    if (bRetUndistort)
    {
        bNewUndistorted = true;
    }
}

//#############################################################################
void Kinect2Impl::applyUndistortionOrRegistration(const bool i_bForce)
{
    if (i_bForce || (bRgb && bRegister && bDepth))
    {
        applyRegistration();
    }
    else if (bDepth && bUndistort)
    {
        applyUndistortion();
    }
    if (bIr && bIrUndistort)
    {
        applyIrUndistortion();
    }
}

//#############################################################################
void Kinect2Impl::applyIrUndistortion()
{
    p->pRegistration->undistortDepth(p->pFrameIrBuf.get(),
                                     p->pFrameIrUndistortedBuf.get());
    const bool bRetIrUndistort = copyFrame(p->pFrameIrUndistortedBuf.get(),
                                           p->pFrameIrUndistorted.get(),
                                           Frame::IrUndistorted, false);
    if (bRetIrUndistort)
    {
        bNewIrUndistorted = true;
    }
}

//#############################################################################

//#############################################################################

}   // ns private_ns
}   // ns kinect2

//#############################################################################

