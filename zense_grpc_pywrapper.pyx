# distutils: language = c++
# distutils: sources = src/pico_zense_server_impl.cpp

from libc.string cimport memcpy
from libc.stdint cimport int32_t
from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp cimport bool
import toml

import base64
import numpy as np
cimport numpy as np

import os

include_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "include")

ctypedef enum DepthRange:
    Near,
    Mid,
    Far

ctypedef vector[DepthRange] WDRDepthRange

# Modules for cv::Mat->ndarray
# Refered from https://github.com/GothicAi/cython-global-matting
cdef extern from "opencv2/opencv.hpp":
    cdef int  CV_WINDOW_AUTOSIZE
    cdef int CV_8UC3
    cdef int CV_8UC1
    cdef int CV_32FC1
    cdef int CV_16UC1
    cdef int CV_8U
    cdef int CV_32F

cdef extern from "opencv2/opencv.hpp" namespace "cv":
    cdef cppclass Mat:
        Mat() except +
        void create(int, int, int)
        void * data
        int rows
        int cols
        int channels()
        int depth()
        size_t elemSize()

cdef extern from "Python.h":
    ctypedef struct PyObject
    object PyMemoryView_FromBuffer(Py_buffer * view)
    int PyBuffer_FillInfo(Py_buffer * view, PyObject * obj, void * buf, Py_ssize_t len, int readonly, int infoflags)
    enum:
        PyBUF_FULL_RO
        PyBUF_CONTIG

cdef object Mat2np(Mat m, bool is_UC16=False):
    # Create buffer to transfer data from m.data
    cdef Py_buffer buf_info
    # Define the size / len of data
    cdef size_t len = m.rows*m.cols*m.elemSize()

    # Fill buffer
    PyBuffer_FillInfo(& buf_info, NULL, m.data, len, False, PyBUF_FULL_RO)
    # Get Pyobject from buffer data
    Pydata  = PyMemoryView_FromBuffer(& buf_info)

    # Create ndarray with data
    # the dimension of the output array is 2 if the image is grayscale
    if m.channels() > 1:
        shape_array = (m.rows, m.cols, m.channels())
    else:
        shape_array = (m.rows, m.cols)

    if not is_UC16:
        pyary = np.frombuffer(Pydata.tobytes(), dtype=np.uint8).reshape(shape_array)    
    else:
        pyary = np.frombuffer(Pydata.tobytes(), dtype=np.uint16).reshape(shape_array)
    return pyary


cdef extern from "../include/pico_zense_server_impl.hpp" namespace "zense":
    cdef cppclass PicoZenseServerImpl:
        PicoZenseServerImpl() except +
        void setup(string cfgParamPath, string camKey, int32_t device_index_)
        bool update()
        bool is_rgb()
        bool is_ir()
        bool is_wdr()
        string getSerialNumber()
        vector[double] getCameraParameter()
        Mat getRGBImage()
        Mat getIRImage()
        Mat getDepthImage()
        vector[Mat] getWDRDepthImage()
        int getDepthRange()
        vector[int] getDepthRangeWDR()


cdef class PicoZenseGRPCServerImpl:
    cdef PicoZenseServerImpl * thisptr
    cdef object rgbImg_npy
    cdef object irImg_npy
    cdef vector[Mat] depthWDRImg
    cdef object depthImgRange1_npy
    cdef object depthImgRange2_npy

    def __cinit__(self, string cfgParamPath, string camKey, int32_t device_index_):
        self.thisptr = new PicoZenseServerImpl()
        self.thisptr.setup(cfgParamPath, camKey, device_index_)

    def __dealloc__(self):
        del self.thisptr

    def update(self):
        cdef Mat rgbImg
        cdef Mat irImg
        cdef Mat depthImg
        cdef bool status

        # ToDo: avoid infinite loop
        # ToDo: null data avoidance
        status = self.thisptr.update()
        if not status:
            "Zense Update process didn't finish correctory"
            return False

        if self.thisptr.is_rgb():
            if self.thisptr.is_ir():
                # RGBDIR case
                rgbImg = self.thisptr.getRGBImage()
                assert rgbImg.cols > 0
                self.rgbImg_npy = Mat2np(rgbImg)
                irImg = self.thisptr.getIRImage()
                self.irImg_npy = Mat2np(irImg, is_UC16=True)
                depthImg = self.thisptr.getDepthImage()
                self.depthImgRange1_npy = Mat2np(depthImg, is_UC16=True)

            else:
                # RGBD case
                rgbImg = self.thisptr.getRGBImage()
                self.rgbImg_npy = Mat2np(rgbImg)
                depthImg = self.thisptr.getDepthImage()
                self.depthImgRange1_npy = Mat2np(depthImg, is_UC16=True)

        else:
            if self.thisptr.is_wdr():
                # WDR case
                depthWDRImg = self.thisptr.getWDRDepthImage()
                self.depthImgRange1_npy = Mat2np(
                    depthWDRImg[0], is_UC16=True)
                self.depthImgRange2_npy = Mat2np(
                    depthWDRImg[1], is_UC16=True)
            else:
                # DepthIR case
                irImg = self.thisptr.getIRImage()
                self.irImg_npy = Mat2np(irImg)
                depthImg = self.thisptr.getDepthImage()
                self.depthImgRange1_npy = Mat2np(depthImg, is_UC16=True)

        # ToDo: check status carefully
        return status

    @property
    def is_rgb(self):
        return self.thisptr.is_rgb()

    @property
    def is_ir(self):
        return self.thisptr.is_ir()

    @property
    def is_wdr(self):
        return self.thisptr.is_wdr()

    @property
    def rgb_image(self):
        assert self.rgbImg_npy is not None
        return self.rgbImg_npy

    @property
    def ir_image(self):
        assert self.irImg_npy is not None
        return self.irImg_npy

    @property
    def depth_image_range1(self):
        assert self.depthImgRange1_npy is not None
        return self.depthImgRange1_npy

    @property
    def depth_image_range2(self):
        assert self.depthImgRange2_npy is not None
        return self.depthImgRange2_npy

    @property
    def depth_image_range1(self):
        assert self.depthImgRange1_npy is not None
        return self.depthImgRange1_npy

    @property
    def depth_image_range2(self):
        assert self.depthImgRange2_npy is not None
        return self.depthImgRange2_npy

    @property
    def get_depth_range(self):
        if self.thisptr.is_wdr():
            return self.thisptr.getDepthRangeWDR()
        else:
            return self.thisptr.getDepthRange()
