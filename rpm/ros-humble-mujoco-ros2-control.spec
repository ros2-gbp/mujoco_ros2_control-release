%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

%global __cmake_in_source_build 1

Name:           ros-humble-mujoco-ros2-control
Version:        0.1.0
Release:        2%{?dist}%{?release_suffix}
Summary:        ROS mujoco_ros2_control package

License:        Apache-2.0
URL:            https://index.ros.org/p/mujoco_ros2_control/#humble
Source0:        %{name}-%{version}.tar.gz

Requires:       eigen3-devel
Requires:       fmt-devel
Requires:       glfw-devel
Requires:       python%{python3_pkgversion}-numpy
Requires:       python3-importlib-resources
Requires:       python3-libs
Requires:       python3-pip
Requires:       python3-pykdl
Requires:       ros-humble-ament-index-cpp
Requires:       ros-humble-ament-index-python
Requires:       ros-humble-backward-ros
Requires:       ros-humble-control-toolbox
Requires:       ros-humble-controller-manager
Requires:       ros-humble-geometry-msgs
Requires:       ros-humble-hardware-interface
Requires:       ros-humble-mujoco-ros2-control-msgs
Requires:       ros-humble-mujoco-ros2-control-plugins
Requires:       ros-humble-mujoco-vendor
Requires:       ros-humble-nav-msgs
Requires:       ros-humble-pluginlib
Requires:       ros-humble-rclcpp
Requires:       ros-humble-rclcpp-lifecycle
Requires:       ros-humble-realtime-tools
Requires:       ros-humble-ros2pkg
Requires:       ros-humble-rosgraph-msgs
Requires:       ros-humble-sensor-msgs
Requires:       ros-humble-std-msgs
Requires:       ros-humble-tinyxml2-vendor
Requires:       ros-humble-transmission-interface
Requires:       ros-humble-urdfdom-py
Requires:       ros-humble-ros-workspace
BuildRequires:  eigen3-devel
BuildRequires:  fmt-devel
BuildRequires:  git
BuildRequires:  glfw-devel
BuildRequires:  ros-humble-ament-cmake
BuildRequires:  ros-humble-ament-cmake-python
BuildRequires:  ros-humble-ament-index-cpp
BuildRequires:  ros-humble-backward-ros
BuildRequires:  ros-humble-control-toolbox
BuildRequires:  ros-humble-controller-manager
BuildRequires:  ros-humble-geometry-msgs
BuildRequires:  ros-humble-hardware-interface
BuildRequires:  ros-humble-mujoco-ros2-control-msgs
BuildRequires:  ros-humble-mujoco-ros2-control-plugins
BuildRequires:  ros-humble-mujoco-vendor
BuildRequires:  ros-humble-nav-msgs
BuildRequires:  ros-humble-pluginlib
BuildRequires:  ros-humble-rclcpp
BuildRequires:  ros-humble-rclcpp-lifecycle
BuildRequires:  ros-humble-realtime-tools
BuildRequires:  ros-humble-ros2-control-cmake
BuildRequires:  ros-humble-ros2pkg
BuildRequires:  ros-humble-rosgraph-msgs
BuildRequires:  ros-humble-sensor-msgs
BuildRequires:  ros-humble-std-msgs
BuildRequires:  ros-humble-tinyxml2-vendor
BuildRequires:  ros-humble-transmission-interface
BuildRequires:  ros-humble-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-humble-ament-cmake-gtest
BuildRequires:  ros-humble-ament-cmake-pytest
%endif

%description
ros2_control wrapper for the MuJoCo Simulate application

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/humble" \
    -DAMENT_PREFIX_PATH="/opt/ros/humble" \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Wed Aug 19 2026 Nathan Dunkelberger <nathan.b.dunkelberger@nasa.gov> - 0.1.0-2
- Autogenerated by Bloom

* Tue Aug 18 2026 Nathan Dunkelberger <nathan.b.dunkelberger@nasa.gov> - 0.1.0-1
- Autogenerated by Bloom

* Fri May 01 2026 Nathan Dunkelberger <nathan.b.dunkelberger@nasa.gov> - 0.0.3-1
- Autogenerated by Bloom

* Tue Mar 17 2026 Nathan Dunkelberger <nathan.b.dunkelberger@nasa.gov> - 0.0.2-1
- Autogenerated by Bloom

* Mon Mar 16 2026 Nathan Dunkelberger <nathan.b.dunkelberger@nasa.gov> - 0.0.1-2
- Autogenerated by Bloom

* Tue Feb 24 2026 Nathan Dunkelberger <nathan.b.dunkelberger@nasa.gov> - 0.0.1-1
- Autogenerated by Bloom

