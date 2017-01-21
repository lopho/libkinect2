#ifndef KINECT2_KINECT2FRAME_H
#define KINECT2_KINECT2FRAME_H
//# INCLUDES ##################################################################
#include <cstddef>
#include <cinttypes>

//# TYPEDEFS / CONSTANTS ######################################################


//# DECLARATION ###############################################################

namespace kinect2
{

class Frame
{
public:
    typedef enum tFrameType
    {
        Unknown = 0,
        Color = 1,
        Ir = 2,
        Depth = 4,
        Undistorted = 8,
        Registered = 16,
        IrUndistorted = 32,
        BigDepth = 64
    } tFrameType;

    typedef enum tPixelFormat
    {
        Invalid = 1,
        Raw = 2,
        Float = 4,
        BGRX = 8,
        RGBX = 16,
        Gray = 32
    } tPixelFormat;

    Frame(const std::size_t i_nWidth,
          const std::size_t i_nHeight,
          const std::size_t i_nBytesPerPixel,
          unsigned char* i_pData = NULL);
    virtual ~Frame();

public:
    std::size_t nWidth;
    std::size_t nHeight;
    std::size_t nBytesPerPixel;
    unsigned char* pData;
    std::uint32_t nTimestamp;
    std::uint32_t nSequence;
    float fExposure;
    float fGain;
    float fGamma;
    std::uint32_t nStatus;
    tPixelFormat eFormat;
    tFrameType eType;

private:
    bool bExternalData;
    unsigned char* pRawData;

};

//#############################################################################

}   // ns kinect2

//#############################################################################

#endif // KINECT2_KINECT2FRAME_H
