Name:           ros-kinetic-homer-mapping
Version:        0.1.15
Release:        2%{?dist}
Summary:        ROS homer_mapping package

Group:          Development/Libraries
License:        GPLv3
Source0:        %{name}-%{version}.tar.gz

Requires:       eigen3-devel
Requires:       qt5-qtbase
Requires:       qt5-qtbase-gui
Requires:       ros-kinetic-homer-mapnav-msgs
Requires:       ros-kinetic-homer-nav-libs
Requires:       ros-kinetic-nav-msgs
Requires:       ros-kinetic-roscpp
Requires:       ros-kinetic-roslib
Requires:       ros-kinetic-sensor-msgs
Requires:       ros-kinetic-std-msgs
Requires:       ros-kinetic-tf
BuildRequires:  eigen3-devel
BuildRequires:  qt5-qtbase
BuildRequires:  qt5-qtbase-devel
BuildRequires:  qt5-qtbase-gui
BuildRequires:  ros-kinetic-catkin
BuildRequires:  ros-kinetic-cmake-modules
BuildRequires:  ros-kinetic-homer-mapnav-msgs
BuildRequires:  ros-kinetic-homer-nav-libs
BuildRequires:  ros-kinetic-nav-msgs
BuildRequires:  ros-kinetic-roscpp
BuildRequires:  ros-kinetic-roslib
BuildRequires:  ros-kinetic-sensor-msgs
BuildRequires:  ros-kinetic-tf

%description
homer_mapping

%prep
%setup -q

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/kinetic/setup.sh" ]; then . "/opt/ros/kinetic/setup.sh"; fi
mkdir -p obj-%{_target_platform} && cd obj-%{_target_platform}
%cmake .. \
        -UINCLUDE_INSTALL_DIR \
        -ULIB_INSTALL_DIR \
        -USYSCONF_INSTALL_DIR \
        -USHARE_INSTALL_PREFIX \
        -ULIB_SUFFIX \
        -DCMAKE_INSTALL_LIBDIR="lib" \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/kinetic" \
        -DCMAKE_PREFIX_PATH="/opt/ros/kinetic" \
        -DSETUPTOOLS_DEB_LAYOUT=OFF \
        -DCATKIN_BUILD_BINARY_PACKAGE="1" \

make %{?_smp_mflags}

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/kinetic/setup.sh" ]; then . "/opt/ros/kinetic/setup.sh"; fi
cd obj-%{_target_platform}
make %{?_smp_mflags} install DESTDIR=%{buildroot}

%files
/opt/ros/kinetic

%changelog
* Thu Feb 23 2017 Viktor Seib <vseib@uni-koblenz.de> - 0.1.15-2
- Autogenerated by Bloom

* Fri Jan 27 2017 Viktor Seib <vseib@uni-koblenz.de> - 0.1.11-1
- Autogenerated by Bloom

