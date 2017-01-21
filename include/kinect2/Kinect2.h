#ifndef KINECT2_KINECT2_H
#define KINECT2_KINECT2_H
//# INCLUDES ##################################################################
#include <string>
#include <memory>
#include <map>
#include <kinect2/Kinect2Frame.h>

//# TYPEDEFS / CONSTANTS ######################################################

//# DECLARATION ###############################################################

namespace kinect2
{

typedef std::shared_ptr<Frame> tFramePtr;
typedef std::map<Frame::tFrameType, tFramePtr> tFrameMap;

namespace private_ns
{
    class Kinect2ImplHolder;
}

//#############################################################################

class Kinect2
{
public:
    typedef enum tReturnCode
    {
        SUCCESS = 0,
        NO_DEVICES_CONNECTED = 1,
        UNKNOWN_DEVICE_ID = 2,
        FAILURE_OPEN = 4,
        FAILURE_CLOSE = 8,
        FAILURE_START = 16,
        FAILURE_STOP = 32,
        NOT_RUNNING = 64
    } tReturnCode;

    typedef enum tPipelineType
    {
        CPU = 1,
        GL = 2,
        CL = 4,
        CLKDE = 8,
        CUDA = 16,
        CUDAKDE = 32
    } tPipelineType;

    typedef struct tColorCameraParams
    {
      /** @name Intrinsic parameters */
      ///@{
      float fx; ///< Focal length x (pixel)
      float fy; ///< Focal length y (pixel)
      float cx; ///< Principal point x (pixel)
      float cy; ///< Principal point y (pixel)
      ///@}

      /** @name Extrinsic parameters
       * These parameters are used in [a formula](https://github.com/OpenKinect/libfreenect2/issues/41#issuecomment-72022111) to map coordinates in the
       * depth camera to the color camera.
       *
       * They cannot be used for matrix transformation.
       */
      ///@{
      float shift_d, shift_m;

      float mx_x3y0; // xxx
      float mx_x0y3; // yyy
      float mx_x2y1; // xxy
      float mx_x1y2; // yyx
      float mx_x2y0; // xx
      float mx_x0y2; // yy
      float mx_x1y1; // xy
      float mx_x1y0; // x
      float mx_x0y1; // y
      float mx_x0y0; // 1

      float my_x3y0; // xxx
      float my_x0y3; // yyy
      float my_x2y1; // xxy
      float my_x1y2; // yyx
      float my_x2y0; // xx
      float my_x0y2; // yy
      float my_x1y1; // xy
      float my_x1y0; // x
      float my_x0y1; // y
      float my_x0y0; // 1
      ///@}
    } tColorCameraParams;

    /** IR camera intrinsic calibration parameters.
     * Kinect v2 includes factory preset values for these parameters. They are used in depth image decoding, and Registration.
     */
    typedef struct tIrCameraParams
    {
      float fx; ///< Focal length x (pixel)
      float fy; ///< Focal length y (pixel)
      float cx; ///< Principal point x (pixel)
      float cy; ///< Principal point y (pixel)
      float k1; ///< Radial distortion coefficient, 1st-order
      float k2; ///< Radial distortion coefficient, 2nd-order
      float k3; ///< Radial distortion coefficient, 3rd-order
      float p1; ///< Tangential distortion coefficient
      float p2; ///< Tangential distortion coefficient
    } tIrCameraParams;


    Kinect2();
    virtual ~Kinect2();

    tReturnCode open();
    tReturnCode close();
    bool isOpen() const;

    tReturnCode start();
    tReturnCode stop();
    tReturnCode restart();
    bool isRunning() const;

    tReturnCode update();

    tReturnCode setPipelineType(
            const tPipelineType i_ePipelineType);
    tReturnCode setDeviceId(const int i_nDeviceId);

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
    tPipelineType getPipelineType() const;

    tColorCameraParams getColorCameraParameters() const;
    tIrCameraParams getIrCameraParameters() const;

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
    const tFramePtr& getFrameRgb() const;
    bool hasNewFrameDepth() const;
    const tFramePtr& getFrameDepth() const;
    bool hasNewFrameIr() const;
    const tFramePtr& getFrameIr() const;
    bool hasNewFrameUndistorted() const;
    const tFramePtr& getFrameUndistorted() const;
    bool hasNewFrameRegistered() const;
    const tFramePtr& getFrameRegistered() const;
    bool hasNewFrameIrUndistorted() const;
    const tFramePtr& getFrameIrUndistorted() const;
    bool hasNewFrameBigDepth() const;
    const tFramePtr& getFrameBigDepth() const;

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
    void applyUndistortionOrRegistration();
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

    private_ns::Kinect2ImplHolder* p;
};

//#############################################################################

}   // ns kinect2

//#############################################################################

#endif // KINECT2_KINECT2_H
