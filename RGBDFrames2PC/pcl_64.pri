INCLUDEPATH+=D:/PCL/PCL1.9.1/include/pcl-1.9
INCLUDEPATH+=D:/PCL/PCL1.9.1/3rdParty/Boost/include/boost-1_68
INCLUDEPATH+=D:/PCL/PCL1.9.1/3rdParty/Eigen/eigen3
INCLUDEPATH+=D:/PCL/PCL1.9.1/3rdParty/FLANN/include
INCLUDEPATH+=D:/PCL/PCL1.9.1/3rdParty/OpenNI2/Include
INCLUDEPATH+=D:/PCL/PCL1.9.1/3rdParty/Qhull/include
INCLUDEPATH+=D:/PCL/PCL1.9.1/3rdParty/VTK/include/vtk-8.1


win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_common_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_common_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_common_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_features_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_features_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_features_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_filters_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_filters_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_filters_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_io_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_io_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_io_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_io_ply_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_io_ply_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_io_ply_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_kdtree_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_kdtree_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_kdtree_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_keypoints_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_keypoints_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_keypoints_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_ml_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_ml_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_ml_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_octree_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_octree_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_octree_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_outofcore_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_outofcore_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_outofcore_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_people_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_people_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_people_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_recognition_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_recognition_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_recognition_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_registration_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_registration_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_registration_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_sample_consensus_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_sample_consensus_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_sample_consensus_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_search_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_search_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_search_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_segmentation_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_segmentation_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_segmentation_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_stereo_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_stereo_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_stereo_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_surface_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_surface_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_surface_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_tracking_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_tracking_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_tracking_release

win64:CONFIG(release, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_visualization_release
else:win64:CONFIG(debug, debug|release): LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_visualization_debug
else:unix: LIBS += -LD:/PCL/PCL1.9.1/lib/ -lpcl_visualization_release

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_atomic-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_atomic-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_atomic-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_bzip2-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_bzip2-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_bzip2-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_chrono-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_chrono-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_chrono-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_container-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_container-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_container-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_context-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_context-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_context-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_contract-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_contract-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_contract-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_coroutine-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_coroutine-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_coroutine-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_date_time-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_date_time-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_date_time-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_exception-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_exception-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_exception-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_fiber-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_fiber-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_fiber-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_filesystem-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_filesystem-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_filesystem-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_graph-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_graph-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_graph-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_graph_parallel-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_graph_parallel-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_graph_parallel-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_iostreams-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_iostreams-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_iostreams-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_locale-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_locale-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_locale-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_log-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_log-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_log-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_log_setup-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_log_setup-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_log_setup-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_c99-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_c99-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_c99-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_c99f-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_c99f-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_c99f-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_c99l-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_c99l-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_c99l-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_tr1-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_tr1-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_tr1-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_tr1f-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_tr1f-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_tr1f-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_tr1l-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_tr1l-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_math_tr1l-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_mpi-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_mpi-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_mpi-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_numpy27-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_numpy27-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_numpy-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_numpy37-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_numpy37-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_numpy3-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_prg_exec_monitor-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_prg_exec_monitor-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_prg_exec_monitor-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_program_options-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_program_options-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_program_options-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_python27-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_python27-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_python-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_python37-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_python37-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_python3-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_random-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_random-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_random-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_regex-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_regex-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_regex-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_serialization-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_serialization-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_serialization-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_signals-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_signals-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_signals-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_stacktrace_noop-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_stacktrace_noop-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_stacktrace_noop-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_stacktrace_windbg_cached-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_stacktrace_windbg_cached-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_stacktrace_windbg_cached-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_stacktrace_windbg-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_stacktrace_windbg-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_stacktrace_windbg-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_system-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_system-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_system-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_test_exec_monitor-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_test_exec_monitor-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_test_exec_monitor-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_thread-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_thread-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_thread-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_timer-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_timer-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_timer-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_type_erasure-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_type_erasure-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_type_erasure-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_unit_test_framework-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_unit_test_framework-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_unit_test_framework-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_wave-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_wave-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_wave-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_wserialization-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_wserialization-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_wserialization-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_zlib-vc141-mt-x64-1_68
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_zlib-vc141-mt-gd-x64-1_68
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_zlib-vc141-mt-x64-1_68

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann_cpp
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann_cpp-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann_cpp

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann_cpp_s
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann_cpp_s-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann_cpp_s

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann_s
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann_s-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/FLANN/lib/ -lflann_s

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhullcpp
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhullcpp_d
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhullcpp

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhullstatic
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhullstatic_d
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhullstatic

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhullstatic_r
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhullstatic_r_d
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhullstatic_r

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhull
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhull_d
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhull

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhull_p
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhull_p_d
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhull_p

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhull_r
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhull_r_d
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/Qhull/lib/ -lqhull_r

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkalglib-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkalglib-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkalglib-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkChartsCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkChartsCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkChartsCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonColor-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonColor-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonColor-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonComputationalGeometry-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonComputationalGeometry-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonComputationalGeometry-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonDataModel-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonDataModel-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonDataModel-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonExecutionModel-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonExecutionModel-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonExecutionModel-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonMath-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonMath-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonMath-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonMisc-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonMisc-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonMisc-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonSystem-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonSystem-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonSystem-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonTransforms-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonTransforms-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkCommonTransforms-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkDICOMParser-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkDICOMParser-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkDICOMParser-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkDomainsChemistry-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkDomainsChemistry-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkDomainsChemistry-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkexoIIc-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkexoIIc-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkexoIIc-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkexpat-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkexpat-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkexpat-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersAMR-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersAMR-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersAMR-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersExtraction-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersExtraction-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersExtraction-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersFlowPaths-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersFlowPaths-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersFlowPaths-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersGeneral-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersGeneral-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersGeneral-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersGeneric-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersGeneric-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersGeneric-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersGeometry-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersGeometry-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersGeometry-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersHybrid-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersHybrid-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersHybrid-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersHyperTree-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersHyperTree-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersHyperTree-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersImaging-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersImaging-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersImaging-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersModeling-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersModeling-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersModeling-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersParallel-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersParallel-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersParallel-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersParallelImaging-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersParallelImaging-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersParallelImaging-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersPoints-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersPoints-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersPoints-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersProgrammable-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersProgrammable-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersProgrammable-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersSelection-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersSelection-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersSelection-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersSMP-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersSMP-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersSMP-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersSources-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersSources-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersSources-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersStatistics-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersStatistics-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersStatistics-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersTexture-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersTexture-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersTexture-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersTopology-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersTopology-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersTopology-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersVerdict-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersVerdict-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkFiltersVerdict-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkfreetype-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkfreetype-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkfreetype-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkGeovisCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkGeovisCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkGeovisCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkgl2ps-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkgl2ps-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkgl2ps-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkhdf5-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkhdf5-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkhdf5-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkhdf5_hl-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkhdf5_hl-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkhdf5_hl-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingColor-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingColor-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingColor-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingFourier-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingFourier-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingFourier-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingGeneral-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingGeneral-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingGeneral-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingHybrid-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingHybrid-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingHybrid-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingMath-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingMath-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingMath-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingMorphological-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingMorphological-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingMorphological-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingSources-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingSources-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingSources-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingStatistics-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingStatistics-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingStatistics-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingStencil-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingStencil-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkImagingStencil-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInfovisCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInfovisCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInfovisCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInfovisLayout-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInfovisLayout-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInfovisLayout-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInteractionImage-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInteractionImage-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInteractionImage-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInteractionStyle-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInteractionStyle-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInteractionStyle-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInteractionWidgets-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInteractionWidgets-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkInteractionWidgets-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOAMR-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOAMR-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOAMR-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOEnSight-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOEnSight-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOEnSight-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOExodus-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOExodus-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOExodus-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOExport-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOExport-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOExport-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOExportOpenGL-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOExportOpenGL-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOExportOpenGL-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOGeometry-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOGeometry-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOGeometry-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOImage-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOImage-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOImage-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOImport-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOImport-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOImport-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOInfovis-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOInfovis-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOInfovis-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOLegacy-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOLegacy-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOLegacy-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOLSDyna-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOLSDyna-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOLSDyna-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOMINC-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOMINC-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOMINC-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOMovie-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOMovie-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOMovie-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIONetCDF-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIONetCDF-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIONetCDF-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOParallel-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOParallel-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOParallel-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOParallelXML-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOParallelXML-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOParallelXML-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOPLY-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOPLY-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOPLY-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOSQL-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOSQL-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOSQL-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOTecplotTable-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOTecplotTable-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOTecplotTable-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOVideo-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOVideo-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOVideo-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOXML-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOXML-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOXML-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOXMLParser-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOXMLParser-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkIOXMLParser-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkjpeg-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkjpeg-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkjpeg-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkjsoncpp-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkjsoncpp-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkjsoncpp-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtklibharu-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtklibharu-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtklibharu-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtklibxml2-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtklibxml2-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtklibxml2-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtklz4-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtklz4-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtklz4-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkmetaio-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkmetaio-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkmetaio-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkNetCDF-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkNetCDF-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkNetCDF-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtknetcdfcpp-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtknetcdfcpp-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtknetcdfcpp-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkoggtheora-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkoggtheora-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkoggtheora-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkParallelCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkParallelCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkParallelCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkpng-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkpng-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkpng-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkproj4-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkproj4-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkproj4-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingAnnotation-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingAnnotation-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingAnnotation-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingContext2D-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingContext2D-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingContext2D-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingContextOpenGL-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingContextOpenGL-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingContextOpenGL-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingFreeType-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingFreeType-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingFreeType-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingGL2PS-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingGL2PS-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingGL2PS-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingImage-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingImage-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingImage-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingLabel-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingLabel-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingLabel-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingLIC-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingLIC-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingLIC-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingLOD-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingLOD-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingLOD-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingOpenGL-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingOpenGL-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingOpenGL-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingVolume-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingVolume-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingVolume-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingVolumeOpenGL-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingVolumeOpenGL-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkRenderingVolumeOpenGL-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtksqlite-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtksqlite-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtksqlite-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtksys-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtksys-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtksys-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtktiff-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtktiff-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtktiff-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkverdict-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkverdict-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkverdict-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkViewsContext2D-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkViewsContext2D-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkViewsContext2D-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkViewsCore-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkViewsCore-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkViewsCore-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkViewsInfovis-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkViewsInfovis-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkViewsInfovis-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkzlib-8.1
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkzlib-8.1-gd
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/VTK/lib/ -lvtkzlib-8.1

win64: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/OpenNI2/Lib/ -lOpenNI2
win64:!win64-g++: PRE_TARGETDEPS += D:/PCL/PCL1.9.1/3rdParty/OpenNI2/Lib/ -lOpenNI2
else:unix: LIBS += -LD:/PCL/PCL1.9.1/3rdParty/OpenNI2/Lib/ -lOpenNI2



win64: LIBS += -L$$PWD/../../PCL/PCL1.9.1/3rdParty/Boost/lib/ -llibboost_thread-vc141-mt-gd-x64-1_68

INCLUDEPATH += $$PWD/../../PCL/PCL1.9.1/3rdParty/Boost/include/boost-1_68
DEPENDPATH += $$PWD/../../PCL/PCL1.9.1/3rdParty/Boost/include/boost-1_68

win64:!win64-g++: PRE_TARGETDEPS += $$PWD/../../PCL/PCL1.9.1/3rdParty/Boost/lib/libboost_thread-vc141-mt-gd-x64-1_68.lib
else:win64-g++: PRE_TARGETDEPS += $$PWD/../../PCL/PCL1.9.1/3rdParty/Boost/lib/liblibboost_thread-vc141-mt-gd-x64-1_68.a
