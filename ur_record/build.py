from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

# python3 build.py build_ext --inplace
setup(
    ext_modules=cythonize([
        Extension("dual_arm_solver", sources=["./controllers/dual_arm_solver.py"]),
        Extension("arm_controller", sources=["./controllers/arm_controller.py"]),
        Extension("inspire_hand", sources=["./mmodules/inspire_hand.py"]),
        Extension("inspire_controller", sources=["./mmodules/inspire_controller.py"]),
    ])
)
