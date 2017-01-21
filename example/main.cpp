#include <cstdlib>
#include <kinect2/Kinect2.h>
#include <iostream>

int main(int /*argc*/, char** /*argv*/)
{
    kinect2::Kinect2* pKinect2 = new kinect2::Kinect2;
    pKinect2->setDepth(true);
    pKinect2->setPipelineType(kinect2::Kinect2::CUDAKDE);
    pKinect2->open();
    pKinect2->start();
    kinect2::tFrameMap map;

    for (int i = 0; i < 10000; ++i)
    {
        while (0 == pKinect2->hasNewFrame())
        {
            std::cout << pKinect2->update();
        }
        pKinect2->getFrames(63, &map);
        for (kinect2::tFrameMap::iterator it = map.begin();
             map.end() != it;
             ++it)
        {
            std::cout << &it->second->pData;
        }
    }
    std::cout << "hello" << std::endl;
    pKinect2->setDepth(false);
    for (int i = 0; i < 10000; ++i)
    {
        while (0 == pKinect2->hasNewFrame())
        {
            std::cout << pKinect2->update();
        }
        pKinect2->getFrames(63, &map);
        for (kinect2::tFrameMap::iterator it = map.begin();
             map.end() != it;
             ++it)
        {
            std::cout << &it->second->pData;
        }
    }
    std::cout << "hello" << std::endl;
    pKinect2->update();
    delete pKinect2;
    pKinect2 = NULL;
    return 0;
}
