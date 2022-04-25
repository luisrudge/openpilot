/*
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "selfdrive/camerad/cameras/Image_Reader.h"
#include <string>
#include <cassert>
#include "selfdrive/camerad/cameras/Util.h"
#include "selfdrive/common/swaglog.h"

/**
 * MAX_BUF_COUNT:
 *   Max buffers in this ImageReader.
 */
#define MAX_BUF_COUNT 2

/**
 * ImageReader listener: called by AImageReader for every frame captured
 * We pass the event to ImageReader class, so it could do some housekeeping
 * about
 * the loaded queue. For example, we could keep a counter to track how many
 * buffers are full and idle in the queue. If camera almost has no buffer to
 * capture
 * we could release ( skip ) some frames by AImageReader_getNextImage() and
 * AImageReader_delete().
 */
void OnImageCallback(void *ctx, AImageReader *reader) {
  reinterpret_cast<Image_Reader *>(ctx)->ImageCallback(reader);
}

/**
 * Constructor
 */
Image_Reader::Image_Reader(ImageFormat *res, enum AIMAGE_FORMATS format)
    : reader_(nullptr) {
  media_status_t status = AImageReader_new(res->width, res->height, format,
                                           MAX_BUF_COUNT, &reader_);
  assert(reader_ && status == AMEDIA_OK);

#if false
  AImageReader_ImageListener listener{
      .context = this,
      .onImageAvailable = OnImageCallback,
  };
  AImageReader_setImageListener(reader_, &listener);
#endif
}

Image_Reader::~Image_Reader() {
  AImageReader_delete(reader_);
}

void Image_Reader::ImageCallback(AImageReader *reader) {
  int32_t format;
  media_status_t status = AImageReader_getFormat(reader, &format);
  assert(status == AMEDIA_OK);
  LOGD("format: 0x%X", format);
  if (format == AIMAGE_FORMAT_JPEG) {
    // Create a thread and write out the jpeg files
    AImage *image = nullptr;
    status = AImageReader_acquireNextImage(reader, &image);
    assert(status == AMEDIA_OK && image);

    int planeCount;
    status = AImage_getNumberOfPlanes(image, &planeCount);
    assert(status == AMEDIA_OK && planeCount == 1);
    uint8_t *data = nullptr;
    int len = 0;
    AImage_getPlaneData(image, 0, &data, &len);

    AImage_delete(image);
  }
}

ANativeWindow *Image_Reader::GetNativeWindow(void) {
  if (!reader_) return nullptr;
  ANativeWindow *nativeWindow;
  media_status_t status = AImageReader_getWindow(reader_, &nativeWindow);
  assert(status == AMEDIA_OK);

  return nativeWindow;
}

/**
 * GetNextImage()
 *   Retrieve the next image in Image_Reader's bufferQueue, NOT the last image
 * so
 * no image is
 *   skipped
 */
AImage *Image_Reader::GetNextImage(void) {
  AImage *image;
  media_status_t status = AImageReader_acquireNextImage(reader_, &image);
  if (status != AMEDIA_OK) {
    return nullptr;
  }
  return image;
}

/**
 *   Retrieve the Last image in Image_Reader's bufferQueue, images may be
 * skipped
 */
AImage *Image_Reader::GetLatestImage(void) {
  AImage *image;
  media_status_t status = AImageReader_acquireLatestImage(reader_, &image);
  if (status != AMEDIA_OK) {
    return nullptr;
  }

  return image;
}

/**
 *   Shows max image buffer
 */
int32_t Image_Reader::GetMaxImage(void) {
  int32_t image_count;
  media_status_t status = AImageReader_getMaxImages(reader_, &image_count);
  if (status != AMEDIA_OK) {
    return -1;
  }
  return image_count;
}

/**
 * Delete Image
 * @param image {@link AImage} instance to be deleted
 */
void Image_Reader::DeleteImage(AImage *image) {
  if (image != nullptr) {
    AImage_delete(image);
    image = nullptr;
  }
}
