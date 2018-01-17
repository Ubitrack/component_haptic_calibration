from conans import ConanFile, CMake


class UbitrackCoreConan(ConanFile):
    name = "ubitrack_component_hapticcalibration"
    version = "1.3.0"

    description = "Ubitrack Haptic Calibration Components"
    url = "https://github.com/Ubitrack/component_haptic_calibration.git"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"
    options = {"shared": [True, False],
               "enable_hapi": [True, False]}
    requires = (
        "ubitrack_core/%s@ubitrack/stable" % version,
        "ubitrack_hapticcalibration/%s@ubitrack/stable" % version,
        "ubitrack_dataflow/%s@ubitrack/stable" % version,
       )

    default_options = (
        "shared=True",
        "enable_hapi=False",
        )

    # all sources are deployed with the package
    exports_sources = "cmake/*", "doc/*", "src/*", "CMakeLists.txt"

    def configure(self):
        if self.settings.os == "Windows":
            self.options.enable_dotnet = True
            
        if self.options.shared:
            self.options['ubitrack_core'].shared = True
            self.options['ubitrack_hapticcalibration'].shared = True
            self.options['ubitrack_dataflow'].shared = True

    def imports(self):
        self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
        self.copy(pattern="*.dylib*", dst="lib", src="lib") 
        self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        cmake = CMake(self)
        cmake.definitions['BUILD_SHARED_LIBS'] = self.options.shared
        cmake.definitions['ENABLE_HAPI'] = self.options.enable_hapi
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        self.copy("*.h", dst="include", src="src", keep_path=True)

    def package_info(self):
        pass