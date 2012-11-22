PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcd_viewer ${SUBSYS_NAME} pcd_viewer.cpp)
target_link_libraries(pcd_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)
    
PCL_ADD_EXECUTABLE_OPT_BUNDLE(timed_trigger_test ${SUBSYS_NAME} timed_trigger_test.cpp)
target_link_libraries(timed_trigger_test pcl_io pcl_common pcl_kdtree pcl_visualization)

find_library( OPENCV_LIB
  NAMES opencv_core opencv_imgproc opencv_highgui opencv_features2d
  PATHS /usr/lib /usr/lib64
)

if(OPENNI_FOUND AND BUILD_OPENNI)

    FIND_PACKAGE (OpenCV REQUIRED)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcd_grabber_viewer ${SUBSYS_NAME} pcd_grabber_viewer.cpp)
    target_link_libraries(pcd_grabber_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)
      
    #PCL_ADD_EXECUTABLE(openni_viewer_simple ${SUBSYS_NAME} openni_viewer_simple.cpp)
    #target_link_libraries(openni_viewer_simple pcl_common pcl_io pcl_kdtree pcl_visualization)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(oni_viewer ${SUBSYS_NAME} oni_viewer_simple.cpp)
    target_link_libraries(oni_viewer pcl_common pcl_io pcl_kdtree pcl_visualization)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(openni_viewer ${SUBSYS_NAME} openni_viewer.cpp)
    target_link_libraries(openni_viewer pcl_common pcl_io pcl_kdtree pcl_visualization gd_sonic_dog openal alut opencv_core opencv_imgproc opencv_highgui opencv_features2d)

    PCL_ADD_EXECUTABLE_OPT_BUNDLE(openni_image ${SUBSYS_NAME} openni_image.cpp)
    target_link_libraries(openni_image pcl_common pcl_io pcl_kdtree pcl_visualization)
endif()

