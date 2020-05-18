from setuptools import setup, Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext
import numpy
import sys
import os
import glob
import pkgconfig

zense_cflags = pkgconfig.cflags('libpicozense')
zense_libs = pkgconfig.libs('libpicozense')

cvlib_folder = os.path.join(sys.prefix, 'local', 'lib')
lib_dirs = [cvlib_folder]

cvlibs = list()
for file in glob.glob(os.path.join(cvlib_folder, 'libopencv_*')):
    cvlibs.append(file.split('.')[0])
cvlibs = list(set(cvlibs))
cvlibs = [
    'opencv_{}'.format(lib.split(os.path.sep)[-1].split('libopencv_')[-1])
    for lib in cvlibs
]

setup(
    name="zense_grpc_pywrapper",
    ext_modules=cythonize([
        Extension(
            "zense_grpc_pywrapper",
            sources=[
                "zense_grpc_pywrapper.pyx", "src/pico_zense_server_impl.cpp",
                "src/common.cpp", "src/parameter_manager.cpp",
                "src/pico_zense_manager.cpp", "src/pico_zense_undistorter.cpp"
            ],
            extra_compile_args=[
                "-std=gnu++11",
                "-O3",
                zense_cflags,
                zense_libs,
                "-w",  #Make warning messages silent
            ],
            include_dirs=[
                numpy.get_include(),
                os.path.join(sys.prefix, 'include', 'opencv2'), 'include'
            ],
            library_dirs=lib_dirs,
            libraries=cvlibs + ["picozense_api"],
            language="c++",
        ),
    ]),
    cmdclass={'build_ext': build_ext},
)
