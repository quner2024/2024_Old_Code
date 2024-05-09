#ifndef qunerCAM_H
#define qunerCAM_H

#include <DxImageProc.h>
#include <GxIAPI.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

class Camera {
public:
    Camera();
    ~Camera();
    bool open();
    bool close();
    bool startCapture();
    bool setExposureTime(uint32_t exposureTime);
    bool setWhiteBalanceMode(uint8_t whiteBalanceMode);
    bool stopCapture();
    static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);

private:
    GX_DEV_HANDLE hDevice = nullptr;
    static Camera* instance;
};

#endif