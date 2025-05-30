import os
import sys
import platform
import subprocess
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from pathlib import Path


class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = ""):
        super().__init__(name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            subprocess.check_output(["cmake", "--version"])
        except OSError:
            raise RuntimeError("CMake must be installed to build the extensions")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext: CMakeExtension):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cfg = "Release"
        build_temp = Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
        ]

        build_args = []

        # Use Ninja if available in the current environment, otherwise use Unix Makefiles
        use_ninja = False
        try:
            subprocess.check_output(["ninja", "--version"])
            use_ninja = True
        except (OSError, subprocess.CalledProcessError):
            use_ninja = False
        if platform.system() != "Windows":
            if use_ninja:
                cmake_args += ["-G", "Ninja"]
            else:
                cmake_args += ["-G", "Unix Makefiles"]

        subprocess.check_call(["cmake", ext.sourcedir] + cmake_args, cwd=build_temp)
        subprocess.check_call(["cmake", "--build", ".", "--config", cfg] + build_args, cwd=build_temp)


# Read long description from README.md if available
long_description = ""
if os.path.exists("README.md"):
    with open("README.md", "r", encoding="utf-8") as f:
        long_description = f.read()

if long_description:
    setup(
        name="dsf",
        version="0.1.0",
        author="Your Name",
        author_email="your.email@example.com",
        description="DSM C++ core with Python bindings via pybind11",
        long_description=long_description,
        long_description_content_type="text/markdown",
        ext_modules=[CMakeExtension("dsf")],
        cmdclass={"build_ext": CMakeBuild},
        zip_safe=False,
        python_requires=">=3.7",
    )
else:
    setup(
        name="dsf",
        version="0.1.0",
        author="Your Name",
        author_email="your.email@example.com",
        description="DSM C++ core with Python bindings via pybind11",
        ext_modules=[CMakeExtension("dsf")],
        cmdclass={"build_ext": CMakeBuild},
        zip_safe=False,
        python_requires=">=3.7",
    )
