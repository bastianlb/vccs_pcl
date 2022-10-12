import os

from conans import ConanFile, CMake
from conans import tools


class ExportMKVConan(ConanFile):
    name = "supervoxel"
    version = "0.1.0"

    description = "supervoxel oversegmentation"
    url = "https://github.com/lennart7/supervoxel"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake", "virtualrunenv"

    options = {
        "shared": [True, False],
        "with_python": [True, False],
    }

    requires = (
        "opencv/4.5.0@camposs/stable",
        "pcl/1.12.1-r1@camposs/stable",
    )

    default_options = {
        "shared": True,
        "with_python": True,
        "opencv:shared": True,
        "Boost:shared": True,
        "pcl:shared": True,
        "pcl:with_cuda": False
    }

    # all sources are deployed with the package
    exports_sources = "cmake/*", "include/*", "src/*", "CMakeLists.txt"

    def requirements(self):
        if self.options.with_python:
            self.requires("pybind11/2.7.1@camposs/stable")

    def configure(self):
        if self.options.shared:
            self.options["bzip2"].shared = True
            self.options["zlib"].shared = True
            self.options["flann"].shared = True
        if self.settings.os == "Linux":
            self.options["opencv"].with_gtk = True

    def imports(self):
        self.copy(src="bin", pattern="*.dll", dst="./bin")  # Copies all dll files from packages bin folder to my "bin" folder
        self.copy(src="lib", pattern="*.dll", dst="./bin")  # Copies all dll files from packages bin folder to my "bin" folder
        self.copy(src="lib", pattern="*.dylib*", dst="./lib")  # Copies all dylib files from packages lib folder to my "lib" folder
        self.copy(src="lib", pattern="*.so*", dst="./lib")  # Copies all so files from packages lib folder to my "lib" folder self.copy(src="lib", pattern="*.a", dst="./lib")  # Copies all static libraries from packages lib folder to my "lib" folder
        self.copy(src="bin", pattern="*", dst="./bin")  # Copies all applications
        if self.options.with_python:
            with tools.run_environment(self):
                python_version = os.environ.get("PYTHON_VERSION", None) or "3.8"
                self.output.write("Collecting python modules in ./lib/python%s" % python_version)
                self.copy(src="lib/python%s" % python_version, pattern="*", dst="./lib/python%s" % python_version, keep_path=True) # Copies all python modules
                self.copy(src="lib/python", pattern="*", dst="./lib/python", keep_path=True)  # Copies all python modules

    def _cmake_configure(self):
        cmake = CMake(self)
        cmake.verbose = True
        cmake.configure()
        return cmake

    def build(self):
        cmake = self._cmake_configure()
        cmake.build()

    def package(self):
        cmake = self._cmake_configure()
        cmake.install()

