#include "selfdrive/camerad/cameras/camera_ndk.h"

#include <unistd.h>

#include <cassert>
#include <cstring>

#include "selfdrive/common/clutil.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"
#include "selfdrive/camerad/cameras/Image_Reader.h"

// id of the video capturing device
const int ROAD_CAMERA_ID = util::getenv("ROADCAM_ID", 1);
const int DRIVER_CAMERA_ID = util::getenv("DRIVERCAM_ID", 2);

#define FRAME_WIDTH  4032
#define FRAME_HEIGHT 3024
#define FRAME_WIDTH_FRONT  3280
#define FRAME_HEIGHT_FRONT 2464

extern ExitHandler do_exit;

namespace {

CameraInfo cameras_supported[CAMERA_ID_MAX] = {
  // road facing
  [CAMERA_ID_LGC920] = {
      .frame_width = FRAME_WIDTH,
      .frame_height = FRAME_HEIGHT,
      .frame_stride = FRAME_WIDTH*3,
      .bayer = false,
      .bayer_flip = false,
  },
  // driver facing
  [CAMERA_ID_LGC615] = {
      .frame_width = FRAME_WIDTH_FRONT,
      .frame_height = FRAME_HEIGHT_FRONT,
      .frame_stride = FRAME_WIDTH_FRONT*3,
      .bayer = false,
      .bayer_flip = false,
  },
};

void camera_open(CameraState *s, bool rear) {
  // empty
}

void camera_close(CameraState *s) {
  // empty
}

void camera_init(VisionIpcServer * v, CameraState *s, int camera_id, unsigned int fps, cl_device_id device_id, cl_context ctx, VisionStreamType rgb_type, VisionStreamType yuv_type, camera_type t) {
  assert(camera_id < std::size(cameras_supported));
  s->ci = cameras_supported[camera_id];
  assert(s->ci.frame_width != 0);

  s->camera_num = camera_id;
  s->fps = fps;
  s->buf.init(device_id, ctx, s, v, FRAME_BUF_COUNT, rgb_type, yuv_type);
  s->m_selected_camera_type = t;
}

void run_camera(CameraState *s) {
  uint32_t frame_id = 0;
  size_t buf_idx = 0;

  s->m_native_camera = new Native_Camera(s->m_selected_camera_type);
  s->m_native_camera->MatchCaptureSizeRequest(&s->m_view, s->width, s->height);
  assert(s->m_view.width && s->m_view.height);
  LOGD("matched w: %d, h: %d", s->m_view.width, s->m_view.height);

  s->m_image_reader = new Image_Reader(&s->m_view, AIMAGE_FORMAT_RAW10);
  s->m_image_reader->SetPresentRotation(s->m_native_camera->GetOrientation());

  ANativeWindow* image_reader_window = s->m_image_reader->GetNativeWindow();
  s->m_camera_ready = s->m_native_camera->CreateCaptureSession(image_reader_window);
  while (!do_exit && s->m_camera_ready) {
/*
    cv::Mat frame_mat, transformed_mat;
    video_cap >> frame_mat;
    if (frame_mat.empty()) continue;

    cv::warpPerspective(frame_mat, transformed_mat, transform, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

    s->buf.camera_bufs_metadata[buf_idx] = {.frame_id = frame_id};

    auto &buf = s->buf.camera_bufs[buf_idx];
    int transformed_size = transformed_mat.total() * transformed_mat.elemSize();
    CL_CHECK(clEnqueueWriteBuffer(buf.copy_q, buf.buf_cl, CL_TRUE, 0, transformed_size, transformed_mat.data, 0, NULL, NULL));
*/
    AImage *image = s->m_image_reader->GetLatestImage();
    if (image == nullptr) { 
      sleep(1);
      continue;
    }
    int planeCount;
    int32_t format;
    media_status_t status = AImageReader_getFormat(s->m_image_reader->reader(), &format);
    assert(status == AMEDIA_OK);
    LOGD("Format: %d", format);
    status = AImage_getNumberOfPlanes(image, &planeCount);
    assert(status == AMEDIA_OK && planeCount == 1);
    uint8_t *data = nullptr;
    int len = 0;
    AImage_getPlaneData(image, 0, &data, &len);
    LOGD("Image len:: %d", len);
    AImage_delete(image);
    //buffer.bits;
    s->buf.queue(buf_idx);

    ++frame_id;
    buf_idx = (buf_idx + 1) % FRAME_BUF_COUNT;
  }
}

static void road_camera_thread(CameraState *s) {
  util::set_thread_name("webcam_road_camera_thread");
  run_camera(s);
}

void driver_camera_thread(CameraState *s) {
  run_camera(s);
}

}  // namespace

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {
  camera_init(v, &s->road_cam, CAMERA_ID_LGC920, 20, device_id, ctx,
              VISION_STREAM_RGB_ROAD, VISION_STREAM_ROAD, BACK_CAMERA);
  s->road_cam.width = FRAME_WIDTH;
  s->road_cam.height = FRAME_HEIGHT;

  camera_init(v, &s->driver_cam, CAMERA_ID_LGC615, 10, device_id, ctx,
              VISION_STREAM_RGB_DRIVER, VISION_STREAM_DRIVER, FRONT_CAMERA);
  s->driver_cam.width = FRAME_WIDTH_FRONT;
  s->driver_cam.height = FRAME_HEIGHT_FRONT;
  s->pm = new PubMaster({"roadCameraState", "driverCameraState", "thumbnail"});
}

void camera_autoexposure(CameraState *s, float grey_frac) {}

void cameras_open(MultiCameraState *s) {
  // LOG("*** open driver camera ***");
  camera_open(&s->driver_cam, false);
  // LOG("*** open road camera ***");
  camera_open(&s->road_cam, true);
}

void cameras_close(MultiCameraState *s) {
  camera_close(&s->road_cam);
  camera_close(&s->driver_cam);
  delete s->pm;
}

void process_driver_camera(MultiCameraState *s, CameraState *c, int cnt) {
  MessageBuilder msg;
  auto framed = msg.initEvent().initDriverCameraState();
  framed.setFrameType(cereal::FrameData::FrameType::FRONT);
  fill_frame_data(framed, c->buf.cur_frame_data);
  s->pm->send("driverCameraState", msg);
}

void process_road_camera(MultiCameraState *s, CameraState *c, int cnt) {
  const CameraBuf *b = &c->buf;
  MessageBuilder msg;
  auto framed = msg.initEvent().initRoadCameraState();
  fill_frame_data(framed, b->cur_frame_data);
  framed.setImage(kj::arrayPtr((const uint8_t *)b->cur_yuv_buf->addr, b->cur_yuv_buf->len));
  framed.setTransform(b->yuv_transform.v);
  s->pm->send("roadCameraState", msg);
}

void cameras_run(MultiCameraState *s) {
  std::vector<std::thread> threads;
  threads.push_back(start_process_thread(s, &s->road_cam, process_road_camera));
  threads.push_back(start_process_thread(s, &s->driver_cam, process_driver_camera));

  std::thread t_rear = std::thread(road_camera_thread, &s->road_cam);
  util::set_thread_name("webcam_thread");
  driver_camera_thread(&s->driver_cam);

  t_rear.join();

  for (auto &t : threads) t.join();

  cameras_close(s);
}
