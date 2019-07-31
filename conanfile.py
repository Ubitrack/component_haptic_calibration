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
               "enable_hapi": [True, False],
               "workspaceBuild" : [True, False],
               }
 

    default_options = {
        "shared" : True,
        "enable_hapi" : False,
        "workspaceBuild" : False,
        }

    # all sources are deployed with the package
    exports_sources = "cmake/*", "doc/*", "src/*", "CMakeLists.txt"

    def requirements(self):
        userChannel = "ubitrack/stable"
        if self.options.workspaceBuild:
            userChannel = "local/dev"

        self.requires("ubitrack_core/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_dataflow/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_hapticcalibration/%s@%s" % (self.version, userChannel))

    def configure(self):
        if self.options.shared:
            self.options['ubitrack_core'].shared = True
            self.options['ubitrack_hapticcalibration'].shared = True
            self.options['ubitrack_dataflow'].shared = True

    # def imports(self):
    #     self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
    #     self.copy(pattern="*.dylib*", dst="lib", src="lib") 
    #     self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        cmake = CMake(self)
        cmake.definitions['BUILD_SHARED_LIBS'] = self.options.shared
        cmake.definitions['ENABLE_HAPI'] = self.options.enable_hapi
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        pass

    def package_info(self):
        pass
