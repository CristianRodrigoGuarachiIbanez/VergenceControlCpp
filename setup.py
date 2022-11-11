from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from Cython.Build import cythonize
import numpy

import sys
import os
import glob

DIR = "/home/cristian/c++Project/iCub_Vergence/Vergence_Control/yarp/src/libYARP_sig/" # "/home/cristian/PycharmProjects/tEDRAM/diparity_maps/src/disparity_maps/iCub_Vergence/yarp/src/libYARP_sig/"
FOLDER = "/home/cristian/PycharmProjects/projektModul/robotology-superbuild/build" # "/home/cristian/PycharmProjects/projektModul/robotology-superbuild/build"

dir = os.path.join(DIR, 'src', "yarp")
lib_folder = os.path.join(FOLDER, 'install', "lib")

# Find opencv libraries in lib_folder

cvlibs = list()
for file in glob.glob(os.path.join(lib_folder, 'libYARP_*')):
    cvlibs.append(file.split('.')[0])

# cvlibs = pkgconfig.libs("opencv") #
# cvlibs = list(set(cvlibs))
print("LIBS", cvlibs)
cvlibs = ['-L{}'.format(lib_folder)] + ['YARP_{}'.format(lib.split(os.path.sep)[-1].split('libYARP_')[-1]) for lib in cvlibs]

print("cv libs", cvlibs)
print("dir", dir)
print("lib folder", lib_folder)


setup(
    cmdclass={'build_ext': build_ext},
    ext_modules=cythonize(Extension("PyVergenceControl",
                                    sources=["PyVergenceControl.pyx", "VergenceControl.cpp"],
                                    language="c++",
                                    include_dirs=[numpy.get_include(), dir ],
                                    library_dirs=[lib_folder],
                                    libraries=cvlibs,
                                    )
                          )
)
