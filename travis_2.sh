#!/bin/bash
# -*- indent-tabs-mode: nil -*-

export ROS_CI_DIR=$(dirname ${BASH_SOURCE:-$0}) #path to directory running the current script
export REPOSITORY_NAME=$(basename $PWD) #name of repository to test
export ROS_WS=${ROS_WS:-/root/ros_ws} #location of workspace

ROS_CI_TRAVIS_TIMEOUT=${ROS_CI_TRAVIS_TIMEOUT:-47} #50 minutes minus safety margin

#Loading helper functions
source ${ROS_CI_DIR}/util.sh

#Colcon output handling
COLCON_EVENT_HANDLING="--event-handlers desktop_notification- status-"

function update_system() {
   travis_fold start update "Updating system packages"
   # Update the sources
   travis_run apt-get -qq update

   # Make sure the packages are up-to-date
   travis_run apt-get -qq dist-upgrade

   # Make sure autoconf is installed and python3-lxml for the tests
   travis_run apt-get -qq install -y autoconf python3-lxml 

   # Install Googletest and wstool
   travis_run apt-get -qq install -y googletest python-wstool

   # Install clang-tidy stuff if needed
   [[ "$TEST" == *clang-tidy* ]] && travis_run apt-get -qq install -y clang-tidy
   # run-clang-tidy is part of clang-tools in Bionic, but not in Xenial -> ignore failure
   [ "$TEST" == *clang-tidy-fix* ] && travis_run_true apt-get -qq install -y clang-tools
   # Install abi-compliance-checker if needed
   [[ "$TEST" == *abi* ]] && travis_run_true apt-get -qq install -y abi-dumper abi-compliance-checker links
   # Enable ccache
   travis_run apt-get -qq install ccache
   export PATH=/usr/lib/ccache:$PATH

   # Setup rosdep - note: "rosdep init" is already setup in base ROS Docker image
   travis_run rosdep update

   travis_fold end update
}

function prepare_or_run_early_tests() {
   # Check for different tests. clang-format and ament_lint will trigger an early exit
   # However, they can only run when $CI_SOURCE_PATH is already available. If not try later again.
   if ! [ -d "$CI_SOURCE_PATH" ] ; then return 0; fi

   # EARLY_RESULT="" -> no early exit, EARLY_RESULT=0 -> early success, otherwise early failure
   local EARLY_RESULT
   for t in $(unify_list " ,;" "$TEST") ; do
      case "$t" in
         clang-format)
            (source ${ROS_CI_DIR}/check_clang_format.sh) # run in subshell to not exit
            EARLY_RESULT=$(( ${EARLY_RESULT:-0} + $? ))
            ;;
         ament_lint)
            (source ${ROS_CI_DIR}/check_ament_lint.sh) # run in subshell to not exit
            EARLY_RESULT=$(( ${EARLY_RESULT:-0} + $? ))
            ;;
         clang-tidy-check)  # run clang-tidy along with compiler and report warning
            CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_CXX_CLANG_TIDY=clang-tidy"
            ;;
         clang-tidy-fix)  # run clang-tidy -fix and report code changes in the end
            CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
            ;;
         abi)  # abi-checker requires debug symbols
            CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo"
            ;;
         *)
            echo -e $(colorize RED "Unknown TEST: $t")
            EARLY_RESULT=$(( ${EARLY_RESULT:-0} + 1 ))
            ;;
      esac
   done
   test -n "$EARLY_RESULT" && exit $EARLY_RESULT
}


# Install and run xvfb to allow for X11-based unittests on DISPLAY :99
function run_xvfb() {
   travis_fold start xvfb "Starting virtual X11 server to allow for X11-based unit tests"
   travis_run apt-get -qq install xvfb mesa-utils
   travis_run "Xvfb -screen 0 640x480x24 :99 &"
   export DISPLAY=:99.0
   travis_run_true glxinfo -B
   travis_fold end xvfb
}

function prepare_ros_workspace() {
   travis_fold start ros.ws "Setting up ROS workspace"
   travis_run_simple mkdir -p $ROS_WS/src

   travis_run_simple cd $ROS_WS/src

     # Pull additional packages into the ros workspace
     travis_run wstool init .
     for item in $(unify_list " ,;" ${UPSTREAM_WORKSPACE:-debian}) ; do
        echo "Adding $item"
        case "$item" in
           debian)
              echo "Obtaining debian packages for all upstream dependencies."
              break ;;
           https://github.com/*) # clone from github
              # extract url and optional branch from item=<url>#<branch>
              item="${item}#"
              url=${item%%#*}
              branch=${item#*#}; branch=${branch%#}; branch=${branch:+--branch ${branch}}
              travis_run_true git clone -q --depth 1 $branch $url
              test $? -ne 0 && echo -e "$(colorize RED Failed clone repository. Aborting.)" && exit 2
              continue ;;
           http://* | https://* | file://*) ;; # use url as is
           *) item="file://$CI_SOURCE_PATH/$item" ;; # turn into proper url
        esac
        travis_run_true wstool merge -k $item
        test $? -ne 0 && echo -e "$(colorize RED Failed to find rosinstall file. Aborting.)" && exit 2
     done

     # Download upstream packages into workspace
     if [ -e .rosinstall ]; then
        # ensure that the to-be-tested package is not in .rosinstall
        travis_run_true wstool rm $REPOSITORY_NAME
        # perform shallow checkout: only possible with wstool init
        travis_run_simple mv .rosinstall rosinstall
        travis_run cat rosinstall
        travis_run wstool init --shallow . rosinstall
     fi

   # Link in the repo we are testing
   if [ "$(dirname $CI_SOURCE_PATH)" != $PWD ] ; then
      travis_run_simple --title "Symlinking to-be-tested repo $CI_SOURCE_PATH into ROS workspace" ln -s $CI_SOURCE_PATH .
   fi

   # Fetch clang-tidy configs
   if [ "$TEST" == clang-tidy-check ] ; then
      # clang-tidy-check essentially runs during the build process for *all* packages.
      # However, we only want to check one repository ($CI_SOURCE_PATH).
      # Thus, we provide a dummy .clang-tidy config file as a fallback for the whole workspace
      travis_run_simple --no-assert cp $ROS_CI_DIR/.dummy-clang-tidy $ROS_WS/src/.clang-tidy
   fi
   #if [[ "$TEST" == clang-tidy-* ]] ; then
      # Ensure a useful .clang-tidy config file is present in the to-be-tested repo ($CI_SOURCE_PATH)
   #   [ -f $CI_SOURCE_PATH/.clang-tidy ] || \
   #      travis_run --title "Fetching default clang-tidy config from MoveIt" \
   #                 wget -nv https://raw.githubusercontent.com/ros-planning/moveit2/moveit2-ci/.clang-tidy \
   #                      -O $CI_SOURCE_PATH/.clang-tidy
   #   travis_run --display "Applying the following clang-tidy checks:" cat $CI_SOURCE_PATH/.clang-tidy
   #fi

   # run BEFORE_SCRIPT, which might modify the workspace further
   run_script BEFORE_SCRIPT

   # For debugging: list the files in workspace's source folder
   travis_run_simple cd $ROS_WS/src
   travis_run --title "List files in ROS workspace's source folder" ls --color=auto -alhF

   # Install source-based package dependencies
   travis_run rosdep install -y -q -r -n --from-paths . --ignore-src --rosdistro $ROS_DISTRO #--skip-keys "moveit_msgs octomap_msgs object_recognition_msgs"

   # Change to base of workspace
   travis_run_simple cd $ROS_WS

   # Validate that we have some packages to build
   test -z "$(colcon list --names-only)" && echo -e "$(colorize RED Workspace $ROS_WS has no packages to build. Terminating.)" && exit 1

   travis_fold end ros.ws
}

function build_workspace() {
   echo -e $(colorize GREEN Building Workspace)

   # Console output fix for: "WARNING: Could not encode unicode characters"
   export PYTHONIOENCODING=UTF-8

   # Change to base of workspace
   travis_run_simple cd $ROS_WS

   # COLCON_IGNORE packages that cause the build to fail
   # TODO: review this
   #travis_run_simple touch $ROS_WS/src/image_common/camera_calibration_parsers/COLCON_IGNORE
   #travis_run_simple touch $ROS_WS/src/image_common/camera_info_manager/COLCON_IGNORE

   # For a command that doesn’t produce output for more than 10 minutes, prefix it with travis_run_wait
   travis_run_wait 60 --title "colcon build" colcon build --merge-install $COLCON_CMAKE_ARGS $COLCON_EVENT_HANDLING

   # Allow to verify ccache usage
   travis_run --title "ccache statistics" ccache -s
}

function test_workspace() {
   echo -e $(colorize GREEN Testing Workspace)
   travis_run_simple --title "Sourcing newly built install space" source install/setup.bash

   # Consider TEST_BLACKLIST
   TEST_BLACKLIST=$(unify_list " ,;" $TEST_BLACKLIST)
   echo -e $(colorize YELLOW Test blacklist: $(colorize THIN $TEST_BLACKLIST))

   # Also blacklist external packages
   all_pkgs=$(colcon list --topological-order --names-only --base-paths $ROS_WS 2> /dev/null)
   source_pkgs=$(colcon list --topological-order --names-only --base-paths $CI_SOURCE_PATH 2> /dev/null)
   blacklist_pkgs=$(filter_out "$source_pkgs" "$all_pkgs")

   # Run tests, suppressing the error output (confuses Travis display?)
   travis_run_wait --title "colcon test" "colcon test --packages-skip $TEST_BLACKLIST $blacklist $COLCON_EVENT_HANDLING --merge-install 2>/dev/null"

   # Show failed tests
   travis_fold start test.results "colcon test-results"
   for file in $(colcon test-result | grep "\.xml:" | cut -d ":" -f1); do
      travis_run --display "Test log of $file" cat $file
   done
   travis_fold end test.results

   # Show test results summary and throw error if necessary
   colcon test-result || exit 2
}

###########################################################################################################
# main program

# If we are here, we can assume we are inside a Docker container
echo "Inside Docker container"

# Prepend current dir if path is not yet absolute
[[ "$ROS_CI_DIR" != /* ]] && ROS_CI_DIR=$PWD/$ROS_CI_DIR
if [[ "$CI_SOURCE_PATH" != /* ]] ; then
   # If CI_SOURCE_PATH is not yet absolute
   if [ -d "$PWD/$CI_SOURCE_PATH" ] ; then
      CI_SOURCE_PATH=$PWD/$CI_SOURCE_PATH  # prepend with current dir, if that's feasible
   else
      # otherwise assume the folder will be created in $ROS_WS/src
      CI_SOURCE_PATH=$ROS_WS/src/$CI_SOURCE_PATH
   fi
fi

# normalize WARNINGS_OK to 0/1. Originally we accept true, yes, or 1 to allow warnings.
test ${WARNINGS_OK:=true} == true -o "$WARNINGS_OK" == 1 -o "$WARNINGS_OK" == yes && WARNINGS_OK=1 || WARNINGS_OK=0

# Define CC/CXX defaults and print compiler version info
travis_run --title "CXX compiler info" $CXX --version

update_system
prepare_or_run_early_tests
run_xvfb
prepare_ros_workspace
prepare_or_run_early_tests

build_workspace
#test_workspace


# Run all remaining tests
for t in $(unify_list " ,;" "$TEST") ; do
   case "$t" in
      clang-tidy-fix)
         (source ${ROS_CI_DIR}/check_clang_tidy.sh)
         test $? -eq 0 || result=$(( ${result:-0} + 1 ))
         ;;
      abi)
         (source ${ROS_CI_DIR}/check_abi.sh)
         test $? -eq 0 || result=$(( ${result:-0} + 1 ))
         ;;
   esac
done
# Run warnings check
#(source ${ROS_CI_DIR}/check_warnings.sh)
#test $? -eq 0 || result=$(( ${result:-0} + 1 ))

#exit ${result:-0}
