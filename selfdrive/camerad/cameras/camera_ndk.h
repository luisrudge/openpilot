#pragma once

#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

#include "selfdrive/camerad/cameras/camera_common.h"
#include "selfdrive/camerad/cameras/Native_Camera.h"

#define FRAME_BUF_COUNT 16

struct Image_Reader;

typedef struct CameraState {
  CameraInfo ci;
  int camera_num;
  int fps;
  float digital_gain;
  CameraBuf buf;

  // Camera variables
  Native_Camera* m_native_camera;

  camera_type m_selected_camera_type = BACK_CAMERA; // Default

  // Image Reader
  ImageFormat m_view{0, 0, 0};
  Image_Reader* m_image_reader;
  AImage* m_image;

  volatile bool m_camera_ready;
  int width, height;
} CameraState;


typedef struct MultiCameraState {
  CameraState road_cam;
  CameraState driver_cam;

  SubMaster *sm;
  PubMaster *pm;
} MultiCameraState;
