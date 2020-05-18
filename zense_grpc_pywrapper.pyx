# distutils: language = c++
# distutils: sources = src/pico_zense_server_impl.cpp

from libc.string cimport memcpy
from libc.stdint cimport int32_t
from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp cimport bool
import toml

import numpy as np
cimport numpy as np

#Modules for cv::Mat->ndarray
#Refered from https://github.com/GothicAi/cython-global-matting
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

cdef object Mat2np(Mat m, bool is_UC8):
    # Create buffer to transfer data from m.data
    cdef Py_buffer buf_info
    # Define the size / len of data
    cdef size_t len = m.rows*m.cols*m.elemSize()

    # Fill buffer
    PyBuffer_FillInfo( & buf_info, NULL, m.data, len, False, PyBUF_FULL_RO)
    # Get Pyobject from buffer data
    Pydata  = PyMemoryView_FromBuffer( & buf_info)

    # Create ndarray with data
    # the dimension of the output array is 2 if the image is grayscale
    if m.channels() > 1:
        shape_array = (m.rows, m.cols, m.channels())
    else:
        shape_array = (m.rows, m.cols)

    if is_UC8:
        pyary = np.asarray(Pydata, dtype=np.uint8).reshape(shape_array)
    else:
        pyary = np.frombuffer(Pydata.tobytes(), dtype=np.uint16).reshape(shape_array)
    return pyary


cdef extern from "include/pico_zense_server_impl.hpp" namespace "zense":
    cdef cppclass PicoZenseServerImpl:
        PicoZenseServerImpl() except +
        void setup(string cfgParamPath, string camKey, int32_t device_index__);
        bool update();

cdef class PicoZenseGRPCServer:
    cdef PicoZenseServerImpl * thisptr
    cdef object rgbImg_npy
    cdef object irImg_npy
    cdef object depthImg_npy

    def __cinit__(self, string cfgParamPath, string camKey, int32_t device_index__):
        self.thisptr = new PicoZenseServerImpl()
        self.thisptr.setup(cfgParamPath, camKey, device_index__)

    def __dealloc__(self):
        del self.thisptr

    def update(self):
        cdef bool status
        status = self.thisptr.update()
        return status
