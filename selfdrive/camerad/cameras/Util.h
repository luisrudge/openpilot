//
// Created by fricke on 8/20/17.
//

#ifndef OPENCV_NDK_UTIL_H
#define OPENCV_NDK_UTIL_H

#include <unistd.h>

// A Data Structure to communicate resolution between camera and ImageReader
struct ImageFormat {
  int32_t width;
  int32_t height;
  int32_t format;  // ex) YUV_420
};

/**
 * A helper class to assist image size comparison, by comparing the absolute
 * size
 * regardless of the portrait or landscape mode.
 */
class Display_Dimension {
 public:
  Display_Dimension(int32_t w, int32_t h) : w_(w), h_(h), portrait_(false) {
    if (h > w) {
      // make it landscape
      w_ = h;
      h_ = w;
      portrait_ = true;
    }
  }
  Display_Dimension(const Display_Dimension& other) {
    w_ = other.w_;
    h_ = other.h_;
    portrait_ = other.portrait_;
  }

  Display_Dimension(void) {
    w_ = 0;
    h_ = 0;
    portrait_ = false;
  }
  Display_Dimension& operator=(const Display_Dimension& other) {
    w_ = other.w_;
    h_ = other.h_;
    portrait_ = other.portrait_;

    return (*this);
  }

  bool IsSameRatio(Display_Dimension& other) {
    return (w_ * other.h_ == h_ * other.w_);
  }
  bool operator>(Display_Dimension& other) {
    return (w_ >= other.w_ & h_ >= other.h_);
  }
  bool operator==(Display_Dimension& other) {
    return (w_ == other.w_ && h_ == other.h_ && portrait_ == other.portrait_);
  }
  Display_Dimension operator-(Display_Dimension& other) {
    Display_Dimension delta(w_ - other.w_, h_ - other.h_);
    return delta;
  }
  void Flip(void) { portrait_ = !portrait_; }
  bool IsPortrait(void) { return portrait_; }
  int32_t width(void) { return w_; }
  int32_t height(void) { return h_; }
  int32_t org_width(void) { return (portrait_ ? h_ : w_); }
  int32_t org_height(void) { return (portrait_ ? w_ : h_); }

 private:
  int32_t w_, h_;
  bool portrait_;

};
#endif  // OPENCV_NDK_UTIL_H
