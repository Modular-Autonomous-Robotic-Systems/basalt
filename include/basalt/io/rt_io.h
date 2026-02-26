#ifndef BASALT_RT_IO
#define BASALT_RT_IO
#include "basalt/io/dataset_io.h"
#include <opencv2/highgui/highgui.hpp>

namespace basalt {

// TODO move this to an independent file as the rest of the code here is data
// testing specific
inline ManagedImage<uint16_t>::Ptr readOpenCVImage(cv::Mat &img) {
    ManagedImage<uint16_t>::Ptr image =
        std::make_shared<ManagedImage<uint16_t>>(img.cols, img.rows);
    const uint8_t *data_in = img.ptr();
    uint16_t *data_out = image->ptr;

    size_t full_size = img.cols * img.rows;
    for (size_t i = 0; i < full_size; i++) {
        int val = data_in[i];
        val = val << 8;
        data_out[i] = val;
    }
    return image;
}

} // namespace basalt
#endif
