from conans import ConanFile, CMake, tools


class UbitrackCoreConan(ConanFile):
    name = "ubitrack_component_vision_aruco"
    version = "1.3.0"

    description = "Ubitrack Vision Components using OpenCV Aruco Tracking"
    url = "https://github.com/Ubitrack/component_vision_aruco.git"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"

    requires = (
        "ubitrack_core/%s@ubitrack/stable" % version,
        "ubitrack_vision/%s@ubitrack/stable" % version,
        "ubitrack_dataflow/%s@ubitrack/stable" % version,
        "ubitrack_component_core/%s@ubitrack/stable" % version,
       )

    default_options = (
        "ubitrack_core:shared=True",
        "ubitrack_vision:shared=True",
        "ubitrack_dataflow:shared=True",
        "opencv:with_aruco=True",
        )

    # all sources are deployed with the package
    exports_sources = "apps/*", "doc/*", "src/*", "CMakeLists.txt"

    # def imports(self):
    #     self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
    #     self.copy(pattern="*.dylib*", dst="lib", src="lib") 
    #     self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        pass

    def package_info(self):
        pass

    def package_id(self):
        self.info.requires["ubitrack_vision"].full_package_mode()
