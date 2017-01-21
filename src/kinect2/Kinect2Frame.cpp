//# INCLUDES ##################################################################
#include <kinect2/Kinect2Frame.h>

//# TYPEDEFS / CONSTANTS ######################################################


//# DECLARATION ###############################################################

namespace kinect2
{

//#############################################################################
Frame::Frame(const std::size_t i_nWidth,
             const std::size_t i_nHeight,
             const std::size_t i_nBytesPerPixel,
             unsigned char* i_pData) :
    nWidth(i_nWidth),
    nHeight(i_nHeight),
    nBytesPerPixel(i_nBytesPerPixel),
    pData(i_pData),
    nTimestamp(0),
    nSequence(0),
    fExposure(0.0f),
    fGain(0.0f),
    fGamma(0.0f),
    nStatus(0),
    eFormat(Frame::Invalid),
    eType(Frame::Unknown),
    bExternalData(false)
{
    if (NULL != pData)
    {
        bExternalData = true;
        return;
    }
    pData = new unsigned char[nWidth * nHeight * nBytesPerPixel];
}

//#############################################################################
Frame::~Frame()
{
    if (!bExternalData)
    {
        delete[] pData;
    }
}

//#############################################################################

}   // ns kinect2

//#############################################################################


