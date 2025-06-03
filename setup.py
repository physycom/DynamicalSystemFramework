import os
import sys
import platform
import subprocess
import re
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from pathlib import Path


def get_version_from_header():
    """Extract version from C++ header file"""
    header_path = Path(__file__).parent / "src" / "dsf" / "dsf.hpp"
    try:
        with open(header_path, "r") as f:
            content = f.read()

        major_match = re.search(r"DSF_VERSION_MAJOR = (\d+)", content)
        minor_match = re.search(r"DSF_VERSION_MINOR = (\d+)", content)
        patch_match = re.search(r"DSF_VERSION_PATCH = (\d+)", content)

        if major_match and minor_match and patch_match:
            return (
                f"{major_match.group(1)}.{minor_match.group(1)}.{patch_match.group(1)}"
            )
        else:
            return "unknown"
    except (FileNotFoundError, AttributeError):
        # Fallback version if header can't be read
        return "unknown"


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
            "-DBUILD_PYTHON_BINDINGS=ON",
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
        subprocess.check_call(
            ["cmake", "--build", ".", "--config", cfg] + build_args, cwd=build_temp
        )


# Read long description from README.md if available
long_description = ""
if os.path.exists("README.md"):
    with open("README.md", "r", encoding="utf-8") as f:
        long_description = f.read()

# Get version from header file
project_version = get_version_from_header()

if long_description:
    setup(
        name="dsf",
        version=project_version,
        author="Grufoony",
        author_email="gregorio.berselli@studio.unibo.it",
        description="DSF C++ core with Python bindings via pybind11",
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
        version=project_version,
        author="Grufoony",
        author_email="gregorio.berselli@studio.unibo.it",
        description="DSF C++ core with Python bindings via pybind11",
        ext_modules=[CMakeExtension("dsf")],
        cmdclass={"build_ext": CMakeBuild},
        zip_safe=False,
        python_requires=">=3.7",
    )
