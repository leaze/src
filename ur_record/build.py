#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
'''
@File    :   build.py
@Time    :   2025/07/07 14:33:52
@Author  :   StarCheng
@Version :   1.0
@Site    :   https://star-cheng.github.io/Blog/
'''
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
