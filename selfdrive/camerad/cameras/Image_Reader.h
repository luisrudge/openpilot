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

#ifndef OPENCV_NDK_IMAGE_READER_H
#define OPENCV_NDK_IMAGE_READER_H
#include "Util.h"
#include <media/NdkImageReader.h>

class Image_Reader {
 public:
  explicit Image_Reader(ImageFormat* res, enum AIMAGE_FORMATS format);

  ~Image_Reader();

  /**
   * Report cached ANativeWindow, which was used to create camera's capture
   * session output.
   */
  ANativeWindow* GetNativeWindow(void);

  /**
   * Retrieve Image on the top of Reader's queue
   */
  AImage* GetNextImage(void);

  /**
  * Retrieve Image on the bottom of Reader's queue
  */
  AImage* GetLatestImage(void);

  int32_t GetMaxImage(void);

  /**
   * Delete Image
   * @param image {@link AImage} instance to be deleted
   */
  void DeleteImage(AImage* image);

  /**
   * AImageReader callback handler. Called by AImageReader when a frame is
   * captured
   * (Internal function, not to be called by clients)
   */
  void ImageCallback(AImageReader* reader);

  inline const AImageReader* reader() { return reader_; }
 private:
  AImageReader* reader_;
};

#endif  // OPENCV_NDK_IMAGE_READER_H
